// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! CAN-to-UDS diagnostic transport bridge.
//!
//! [`DiagCanTransport`] integrates the ISO-TP codec (`bsw-docan`), the UDS
//! diagnostic router (`bsw-uds`), and a hardware CAN transceiver (`bsw-can`)
//! into a single, poll-driven state machine suitable for a bare-metal main
//! loop.
//!
//! # Architecture
//!
//! ```text
//!  CAN bus  <-->  CanTransceiver (HW)
//!                      |
//!           DiagCanTransport::poll()
//!                      |
//!           +----------+----------+
//!           |                     |
//!     RX path (ISO-TP)      TX path (ISO-TP)
//!      decode_frame()        encode_*_frame()
//!      RxProtocolHandler     TxProtocolHandler
//!           |                     |
//!           +------> DiagRouter --+
//!                   (UDS jobs)
//! ```
//!
//! # State machines
//!
//! ## RX states
//!
//! ```text
//! Idle ----[SF received]----> dispatch to UDS immediately
//! Idle ----[FF received]----> ReceivingMultiFrame
//!                              send FC(CTS) to tester
//!                              wait for consecutive frames
//! ReceivingMultiFrame ----[all CFs received]----> dispatch to UDS
//! ReceivingMultiFrame ----[timeout]----> Idle (abort)
//! ```
//!
//! ## TX states
//!
//! ```text
//! Idle ----[UDS response ready, <= 7 bytes]----> encode SF, write, Idle
//! Idle ----[UDS response ready, > 7 bytes]-----> encode FF, write, Sending
//! Sending ----[FC(CTS) received]----> encode CFs, write one per poll
//! Sending ----[all frames sent]----> Idle
//! Sending ----[timeout / error]----> Idle (abort)
//! ```
//!
//! # No-std / no-alloc
//!
//! All buffers are fixed-size stack arrays. No heap allocation occurs.
//! The maximum ISO-TP message size is 4095 bytes (standard addressing,
//! 12-bit length field).
//!
//! # Usage
//!
//! The caller must provide a concrete `CanTransceiver` implementation that
//! also implements [`CanReceiver`] (i.e., has a non-blocking `receive()`
//! method). Both [`BxCanTransceiver`](crate::can_bxcan::BxCanTransceiver)
//! and [`FdCanTransceiver`](crate::can_fdcan::FdCanTransceiver) fulfil this
//! contract.
//!
//! ```rust,ignore
//! use bsw_bsp_stm32::diag_can::{DiagCanTransport, CanReceiver};
//!
//! let mut transport = DiagCanTransport::new(
//!     transceiver,   // impl CanTransceiver + CanReceiver
//!     0x600,         // request CAN ID (tester -> ECU)
//!     0x601,         // response CAN ID (ECU -> tester)
//!     &router,       // DiagRouter with registered jobs
//! );
//!
//! // In main loop:
//! loop {
//!     transport.poll();
//!     // ...other tasks...
//! }
//! ```

use bsw_can::frame::CanFrame;
use bsw_can::can_id::CanId;
use bsw_can::transceiver::{CanTransceiver, TransceiverState};
use bsw_docan::codec::{
    self, CodecConfig, DecodedFrame, FirstFrame, SingleFrame, ConsecutiveFrame,
};
use bsw_docan::constants::FlowStatus;
use bsw_docan::rx_handler::RxProtocolHandler;
use bsw_docan::tx_handler::{TxProtocolHandler, TxState};
use bsw_lifecycle::TransitionResult;
use bsw_uds::diag_job::DiagRouter;
use bsw_uds::session::DiagSession;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Maximum ISO-TP message size (12-bit length field, standard addressing).
/// Maximum message length for ISO-TP reassembly/TX.
/// Kept at 256 to avoid stack overflow on Cortex-M4 (8 KB stack).
/// For messages >256 bytes, increase this and ensure sufficient stack.
const MAX_MSG_LEN: usize = 256;

/// CAN frame data length for classic CAN (8 bytes).
const CAN_DLC: usize = 8;

/// Payload bytes in a single frame (classic CAN, no address byte).
/// SF PCI = 1 byte, leaving 7 for data.
const SF_DATA_SIZE: usize = 7;

/// Payload bytes in the first frame (classic CAN, no address byte).
/// FF PCI = 2 bytes, leaving 6 for data.
const FF_DATA_SIZE: usize = 6;

/// Payload bytes in a consecutive frame (classic CAN, no address byte).
/// CF PCI = 1 byte, leaving 7 for data.
const CF_DATA_SIZE: usize = 7;

/// UDS negative response SID.
const NRC_SID: u8 = 0x7F;

/// Maximum WAIT flow-controls to tolerate on the TX side.
const MAX_FC_WAIT: u8 = 10;

// ---------------------------------------------------------------------------
// CanReceiver trait
// ---------------------------------------------------------------------------

/// Extension trait for non-blocking CAN frame reception.
///
/// The `CanTransceiver` trait in `bsw-can` covers the transmit path only
/// (mirroring the C++ `ICanTransceiver`). Platform BSPs that provide an
/// RX software queue must additionally implement this trait so that the
/// diagnostic transport can poll for incoming frames.
pub trait CanReceiver {
    /// Returns the next received CAN frame, or `None` if the RX queue is
    /// empty. Must be non-blocking.
    fn receive(&mut self) -> Option<CanFrame>;
}

// ---------------------------------------------------------------------------
// Transport RX/TX state enums
// ---------------------------------------------------------------------------

/// Receive-side state of the diagnostic transport.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RxPhase {
    /// No reception in progress. Ready for a new request.
    Idle,
    /// Receiving a multi-frame ISO-TP message (FF received, waiting for CFs).
    ReceivingMultiFrame,
}

/// Transmit-side state of the diagnostic transport.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TxPhase {
    /// No transmission in progress.
    Idle,
    /// Actively sending a multi-frame response (FF sent, sending CFs or
    /// waiting for FC from tester).
    Sending,
}

// ---------------------------------------------------------------------------
// DiagCanTransport
// ---------------------------------------------------------------------------

/// CAN-based diagnostic transport — bridges hardware CAN to UDS via ISO-TP.
///
/// Generic over `T` which must implement both [`CanTransceiver`] (for TX /
/// state management) and [`CanReceiver`] (for non-blocking RX). This allows
/// the same transport code to work with `BxCanTransceiver` (STM32F4) and
/// `FdCanTransceiver` (STM32G4).
///
/// # Lifetime `'a`
///
/// The `'a` lifetime borrows the [`DiagRouter`] and its registered
/// [`DiagJob`](bsw_uds::diag_job::DiagJob) references. The transport must
/// not outlive the router.
pub struct DiagCanTransport<'a, T: CanTransceiver + CanReceiver> {
    /// Hardware CAN transceiver (TX + RX).
    transceiver: T,

    /// ISO-TP codec configuration (PCI offset, filler byte).
    codec_config: CodecConfig,

    /// CAN arbitration ID for incoming diagnostic requests (tester -> ECU).
    request_id: u32,

    /// CAN arbitration ID for outgoing diagnostic responses (ECU -> tester).
    response_id: u32,

    // -- RX state ----------------------------------------------------------

    /// Current receive-side phase.
    rx_phase: RxPhase,

    /// ISO-TP RX protocol handler (active during multi-frame reception).
    #[allow(dead_code)]
    rx_handler: Option<RxProtocolHandler>,

    /// Reassembly buffer for incoming ISO-TP messages.
    rx_buf: [u8; MAX_MSG_LEN],

    /// Number of valid bytes currently in `rx_buf`.
    rx_len: usize,

    /// Total expected message length (from FF header).
    rx_expected_len: usize,

    /// Next expected ISO-TP sequence number (1..=15, wraps at 16).
    rx_seq: u8,

    // -- TX state ----------------------------------------------------------

    /// Current transmit-side phase.
    tx_phase: TxPhase,

    /// ISO-TP TX protocol handler (active during multi-frame transmission).
    tx_handler: Option<TxProtocolHandler>,

    /// Response buffer holding the complete UDS response to be sent.
    tx_buf: [u8; MAX_MSG_LEN],

    /// Total length of the UDS response in `tx_buf`.
    tx_len: usize,

    /// Number of response bytes already encoded and sent.
    tx_offset: usize,

    /// TX consecutive-frame sequence counter (1..=15, wraps).
    tx_seq: u8,

    // -- UDS ---------------------------------------------------------------

    /// Current diagnostic session.
    session: DiagSession,

    /// UDS job router.
    router: DiagRouter<'a>,

    // -- Timing (placeholder for future timeout enforcement) ---------------

    /// Timestamp (microseconds) of the last received frame.
    /// Reserved for N_Cr timeout enforcement in a future iteration.
    #[allow(dead_code)]
    last_rx_time_us: u32,
}

impl<'a, T: CanTransceiver + CanReceiver> DiagCanTransport<'a, T> {
    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    /// Creates a new diagnostic transport.
    ///
    /// - `transceiver`: the hardware CAN transceiver to use for TX/RX.
    /// - `request_id`: CAN ID on which diagnostic requests arrive (e.g. 0x600).
    /// - `response_id`: CAN ID for outgoing diagnostic responses (e.g. 0x601).
    /// - `router`: the UDS [`DiagRouter`] containing all registered jobs.
    pub fn new(
        transceiver: T,
        request_id: u32,
        response_id: u32,
        router: DiagRouter<'a>,
    ) -> Self {
        Self {
            transceiver,
            codec_config: CodecConfig {
                pci_offset: 0,
                filler_byte: 0xCC,
            },
            request_id,
            response_id,
            rx_phase: RxPhase::Idle,
            rx_handler: None,
            rx_buf: [0u8; MAX_MSG_LEN],
            rx_len: 0,
            rx_expected_len: 0,
            rx_seq: 1,
            tx_phase: TxPhase::Idle,
            tx_handler: None,
            tx_buf: [0u8; MAX_MSG_LEN],
            tx_len: 0,
            tx_offset: 0,
            tx_seq: 1,
            session: DiagSession::Default,
            router,
            last_rx_time_us: 0,
        }
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /// Returns the current diagnostic session.
    pub fn session(&self) -> DiagSession {
        self.session
    }

    /// Returns a reference to the underlying transceiver.
    pub fn transceiver(&self) -> &T {
        &self.transceiver
    }

    /// Returns a mutable reference to the underlying transceiver.
    pub fn transceiver_mut(&mut self) -> &mut T {
        &mut self.transceiver
    }

    // -----------------------------------------------------------------------
    // Main poll loop
    // -----------------------------------------------------------------------

    /// Perform one unit of work.
    ///
    /// Call this from the main loop (or a periodic RTOS task). Each
    /// invocation does at most one of the following:
    ///
    /// 1. Reads one CAN frame from the transceiver and processes it
    ///    (decodes ISO-TP, reassembles multi-frame messages).
    /// 2. If a complete request is available, dispatches it to the UDS
    ///    router and starts sending the response.
    /// 3. If a TX session is in progress, encodes and sends the next
    ///    consecutive frame.
    ///
    /// This design avoids blocking and allows the caller to interleave
    /// other work between polls.
    pub fn poll(&mut self) {
        // ── Step 0: Bus-off recovery ────────────────────────────────────
        // If the FDCAN went bus-off (TEC >= 256), it enters INIT mode
        // automatically on STM32G4. We detect this and re-open the bus.
        if self.transceiver.transceiver_state() == TransceiverState::BusOff {
            // Re-initialize: close → init → open
            self.transceiver.close();
            self.transceiver.init();
            self.transceiver.open();
            // Reset any in-progress RX/TX sessions
            self.rx_phase = RxPhase::Idle;
            self.tx_phase = TxPhase::Idle;
            self.rx_handler = None;
            self.tx_handler = None;
            return; // skip this poll cycle, let bus stabilize
        }

        // ── Step 1: Try to receive a CAN frame ──────────────────────────
        if let Some(frame) = self.transceiver.receive() {
            let frame_id = frame.id().raw_id();

            if frame_id == self.request_id {
                self.handle_rx_frame(&frame);
            } else if frame_id == self.response_id {
                // Flow control from tester arrives on the response ID in
                // some addressing schemes. Handle it for the TX path.
                self.handle_tx_flow_control(&frame);
            }
        }

        // ── Step 2: Drive TX if a multi-frame response is in progress ───
        if self.tx_phase == TxPhase::Sending {
            self.drive_tx();
        }
    }

    // -----------------------------------------------------------------------
    // RX path — frame handling
    // -----------------------------------------------------------------------

    /// Processes a single received CAN frame on the request ID.
    fn handle_rx_frame(&mut self, frame: &CanFrame) {
        let data = frame.payload();
        let decoded = match codec::decode_frame(data, &self.codec_config) {
            Ok(d) => d,
            Err(_) => return, // malformed frame — ignore silently
        };

        match decoded {
            DecodedFrame::Single(sf) => self.handle_single_frame(&sf),
            DecodedFrame::First(ff) => self.handle_first_frame(&ff),
            DecodedFrame::Consecutive(cf) => self.handle_consecutive_frame(&cf),
            DecodedFrame::FlowControl(_) => {
                // FC on request ID is unexpected in normal server mode.
                // Silently ignore.
            }
        }
    }

    /// Handles a complete single-frame request: copy to RX buffer and
    /// dispatch immediately.
    fn handle_single_frame(&mut self, sf: &SingleFrame<'_>) {
        let len = sf.data_length as usize;
        if len == 0 || len > SF_DATA_SIZE || len > MAX_MSG_LEN {
            return;
        }
        self.rx_buf[..len].copy_from_slice(sf.data);
        self.rx_len = len;
        self.rx_phase = RxPhase::Idle;

        // Dispatch to UDS and start response TX.
        self.dispatch_and_respond();
    }

    /// Handles the first frame of a multi-frame request: store initial
    /// payload, set up RX handler, and send FC(CTS) to the tester.
    fn handle_first_frame(&mut self, ff: &FirstFrame<'_>) {
        let msg_len = ff.message_length as usize;
        if msg_len < 8 || msg_len > MAX_MSG_LEN {
            return; // invalid or too large for our buffer
        }

        // Copy the data carried in the first frame.
        let first_data_len = ff.data.len().min(msg_len).min(FF_DATA_SIZE);
        self.rx_buf[..first_data_len].copy_from_slice(&ff.data[..first_data_len]);
        self.rx_len = first_data_len;
        self.rx_expected_len = msg_len;
        self.rx_seq = 1; // next expected SN

        // Calculate number of CFs expected.
        let remaining = msg_len - first_data_len;
        let cf_count = remaining.div_ceil(CF_DATA_SIZE);
        // cf_count bounded by MAX_MSG_LEN/1 <= 4095, fits in u32.
        #[allow(clippy::cast_possible_truncation)]
        let cf_count_u32 = cf_count as u32;

        self.rx_handler = Some(RxProtocolHandler::new_multi(cf_count_u32, 0, 0));

        // Immediately mark allocation as successful (we use static buffers).
        if let Some(ref mut handler) = self.rx_handler {
            let transition = handler.allocated(true);
            if transition.send_flow_control {
                send_fc_cts(
                    &mut self.transceiver,
                    self.response_id,
                    &self.codec_config,
                );
                // Notify handler that FC was sent successfully.
                handler.frame_sent(true);
            }
        }

        self.rx_phase = RxPhase::ReceivingMultiFrame;
    }

    /// Handles a consecutive frame during multi-frame reception.
    fn handle_consecutive_frame(&mut self, cf: &ConsecutiveFrame<'_>) {
        if self.rx_phase != RxPhase::ReceivingMultiFrame {
            return; // unexpected CF — ignore
        }

        // Verify sequence number.
        if cf.sequence_number != (self.rx_seq & 0x0F) {
            // Sequence error — abort reception.
            self.abort_rx();
            return;
        }
        self.rx_seq = self.rx_seq.wrapping_add(1);

        // Append payload bytes, clamping to the expected message length.
        let remaining = self.rx_expected_len.saturating_sub(self.rx_len);
        let copy_len = cf.data.len().min(remaining);
        if copy_len == 0 {
            return;
        }
        let dest_end = self.rx_len + copy_len;
        if dest_end > MAX_MSG_LEN {
            self.abort_rx();
            return;
        }
        self.rx_buf[self.rx_len..dest_end].copy_from_slice(&cf.data[..copy_len]);
        self.rx_len = dest_end;

        // Notify the protocol handler.
        if let Some(ref mut handler) = self.rx_handler {
            let transition = handler.consecutive_frame_received();

            // If handler requests another FC (block boundary), send it.
            if transition.send_flow_control {
                send_fc_cts(
                    &mut self.transceiver,
                    self.response_id,
                    &self.codec_config,
                );
                handler.frame_sent(true);
            }
        }

        // Check if message is complete.
        if self.rx_len >= self.rx_expected_len {
            self.rx_phase = RxPhase::Idle;
            self.rx_handler = None;
            self.dispatch_and_respond();
        }
    }

    /// Aborts an in-progress multi-frame reception and returns to idle.
    fn abort_rx(&mut self) {
        self.rx_phase = RxPhase::Idle;
        self.rx_handler = None;
        self.rx_len = 0;
        self.rx_expected_len = 0;
    }

    // -----------------------------------------------------------------------
    // UDS dispatch
    // -----------------------------------------------------------------------

    /// Dispatches the assembled request to the UDS router and starts
    /// transmitting the response.
    ///
    /// Uses a 256-byte local buffer (not 4095) to avoid stack overflow
    /// on Cortex-M4. UDS responses are always < 256 bytes in practice.
    fn dispatch_and_respond(&mut self) {
        let request = &self.rx_buf[..self.rx_len];
        let mut response_buf = [0u8; 256];

        match self.router.dispatch(request, self.session, &mut response_buf) {
            Ok(resp_len) => {
                self.start_tx(&response_buf[..resp_len]);
            }
            Err(nrc) => {
                let sid = if request.is_empty() { 0x00 } else { request[0] };
                let nrc_response = [NRC_SID, sid, nrc.as_byte()];
                self.start_tx(&nrc_response);
            }
        }
    }

    // -----------------------------------------------------------------------
    // TX path — response transmission
    // -----------------------------------------------------------------------

    /// Initiates transmission of a UDS response.
    ///
    /// For responses that fit in a single frame (<= 7 bytes), the SF is
    /// encoded and sent immediately. For longer responses, the first frame
    /// is sent and the transport transitions to `TxPhase::Sending` to
    /// continue with consecutive frames after receiving FC from the tester.
    fn start_tx(&mut self, response: &[u8]) {
        if response.is_empty() {
            return;
        }

        if response.len() <= SF_DATA_SIZE {
            // ── Single Frame ────────────────────────────────────────────
            let mut buf = [self.codec_config.filler_byte; CAN_DLC];
            if codec::encode_single_frame(&mut buf, response, &self.codec_config).is_ok() {
                let frame = CanFrame::with_data(
                    CanId::base(self.response_id as u16),
                    &buf,
                );
                let _ = self.transceiver.write(&frame);
            }
            self.tx_phase = TxPhase::Idle;
        } else {
            // ── Multi-Frame (FF + CFs) ──────────────────────────────────
            let msg_len = response.len();
            if msg_len > MAX_MSG_LEN {
                return; // too large, silently drop
            }

            // Copy response into TX buffer.
            self.tx_buf[..msg_len].copy_from_slice(response);
            self.tx_len = msg_len;

            // Encode and send the First Frame.
            let ff_data_len = FF_DATA_SIZE.min(msg_len);
            let mut buf = [self.codec_config.filler_byte; CAN_DLC];
            #[allow(clippy::cast_possible_truncation)]
            let msg_len_u32 = msg_len as u32;
            if codec::encode_first_frame(
                &mut buf,
                msg_len_u32,
                &response[..ff_data_len],
                &self.codec_config,
            )
            .is_ok()
            {
                let frame = CanFrame::with_data(
                    CanId::base(self.response_id as u16),
                    &buf,
                );
                let _ = self.transceiver.write(&frame);
            }
            self.tx_offset = ff_data_len;
            self.tx_seq = 1;

            // Set up TX protocol handler.
            let total_frames = codec::frame_count(msg_len_u32, FF_DATA_SIZE, CF_DATA_SIZE);
            let mut handler = TxProtocolHandler::new(total_frames, MAX_FC_WAIT);
            handler.start();
            handler.frame_sending(); // FF queued
            handler.frames_sent();   // FF confirmed -> Wait for FC
            self.tx_handler = Some(handler);
            self.tx_phase = TxPhase::Sending;
        }
    }

    /// Handles a Flow Control frame received from the tester on the
    /// response CAN ID (during multi-frame TX).
    fn handle_tx_flow_control(&mut self, frame: &CanFrame) {
        if self.tx_phase != TxPhase::Sending {
            return;
        }

        let data = frame.payload();
        let decoded = match codec::decode_frame(data, &self.codec_config) {
            Ok(d) => d,
            Err(_) => return,
        };

        if let DecodedFrame::FlowControl(fc) = decoded {
            if let Some(ref mut handler) = self.tx_handler {
                handler.handle_flow_control(fc.status, fc.block_size);
            }
        }
    }

    /// Drives the TX state machine: encodes and sends the next consecutive
    /// frame if the protocol handler is in the `Send` state.
    fn drive_tx(&mut self) {
        let handler_state = match self.tx_handler {
            Some(ref handler) => handler.state(),
            None => {
                self.tx_phase = TxPhase::Idle;
                return;
            }
        };

        match handler_state {
            TxState::Send => {
                if self.tx_offset >= self.tx_len {
                    // All data sent — mark complete.
                    self.finish_tx();
                    return;
                }

                // Determine how many bytes this CF carries.
                let remaining = self.tx_len - self.tx_offset;
                let cf_data_len = remaining.min(CF_DATA_SIZE);

                // Encode the consecutive frame.
                let mut buf = [self.codec_config.filler_byte; CAN_DLC];
                let cf_data = &self.tx_buf[self.tx_offset..self.tx_offset + cf_data_len];
                if codec::encode_consecutive_frame(
                    &mut buf,
                    self.tx_seq,
                    cf_data,
                    &self.codec_config,
                )
                .is_ok()
                {
                    let frame = CanFrame::with_data(
                        CanId::base(self.response_id as u16),
                        &buf,
                    );
                    let _ = self.transceiver.write(&frame);
                }

                self.tx_offset += cf_data_len;
                self.tx_seq = self.tx_seq.wrapping_add(1);

                // Notify the protocol handler.
                if let Some(ref mut handler) = self.tx_handler {
                    handler.frame_sending();
                    handler.frames_sent();
                }

                // Check for completion.
                if let Some(ref handler) = self.tx_handler {
                    if handler.is_complete() {
                        self.finish_tx();
                    }
                }
            }
            TxState::Wait => {
                // Waiting for FC from tester — nothing to do this cycle.
            }
            TxState::Success => {
                self.finish_tx();
            }
            TxState::Fail => {
                self.finish_tx();
            }
            TxState::Initialized => {
                // Should not happen after start(), but handle gracefully.
            }
        }
    }

    /// Cleans up TX state after a completed (or aborted) transmission.
    fn finish_tx(&mut self) {
        self.tx_phase = TxPhase::Idle;
        self.tx_handler = None;
        self.tx_len = 0;
        self.tx_offset = 0;
        self.tx_seq = 1;
    }
}

// ---------------------------------------------------------------------------
// Free-standing FC helper (avoids borrow-splitting issues)
// ---------------------------------------------------------------------------

/// Encodes and sends a FC(CTS, BS=0, STmin=0) frame.
///
/// Factored out of `DiagCanTransport` so the caller can hold a mutable
/// borrow on the protocol handler while sending the FC through the
/// transceiver — the two borrows are disjoint.
fn send_fc_cts<T: CanTransceiver>(
    transceiver: &mut T,
    response_id: u32,
    codec_config: &CodecConfig,
) {
    let mut buf = [0u8; CAN_DLC];
    let _ = codec::encode_flow_control(
        &mut buf,
        FlowStatus::ContinueToSend,
        0, // block_size = unlimited
        0, // STmin = 0
        codec_config,
    );
    #[allow(clippy::cast_possible_truncation)]
    let frame = CanFrame::with_data(CanId::base(response_id as u16), &buf);
    let _ = transceiver.write(&frame);
}

// ---------------------------------------------------------------------------
// LifecycleComponent implementation
// ---------------------------------------------------------------------------

impl<T: CanTransceiver + CanReceiver> bsw_lifecycle::LifecycleComponent
    for DiagCanTransport<'_, T>
{
    /// Initialises the CAN transceiver hardware and opens the bus.
    fn init(&mut self) -> TransitionResult {
        self.transceiver.init();
        self.transceiver.open();
        TransitionResult::Done
    }

    /// Executes one poll cycle of the diagnostic transport.
    fn run(&mut self) -> TransitionResult {
        self.poll();
        TransitionResult::Done
    }

    /// Closes the CAN transceiver.
    fn shutdown(&mut self) -> TransitionResult {
        self.transceiver.close();
        TransitionResult::Done
    }

    /// Human-readable component name.
    fn name(&self) -> &str {
        "DiagCanTransport"
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use bsw_can::transceiver::{ErrorCode, State, TransceiverState};
    use bsw_uds::nrc::Nrc;

    // -- Mock transceiver ---------------------------------------------------

    /// Minimal mock that implements both `CanTransceiver` and `CanReceiver`.
    struct MockTransceiver {
        state: State,
        rx_queue: [Option<CanFrame>; 4],
        rx_head: usize,
        tx_log: [Option<CanFrame>; 16],
        tx_count: usize,
    }

    impl MockTransceiver {
        fn new() -> Self {
            const NONE_FRAME: Option<CanFrame> = None;
            Self {
                state: State::Closed,
                rx_queue: [NONE_FRAME; 4],
                rx_head: 0,
                tx_log: [NONE_FRAME; 16],
                tx_count: 0,
            }
        }

        /// Enqueue a frame to be returned by the next `receive()` call.
        fn enqueue_rx(&mut self, frame: CanFrame) {
            for slot in self.rx_queue.iter_mut() {
                if slot.is_none() {
                    *slot = Some(frame);
                    return;
                }
            }
        }

        /// Returns the number of frames that were written (transmitted).
        fn tx_count(&self) -> usize {
            self.tx_count
        }

        /// Returns a reference to the Nth transmitted frame.
        fn tx_frame(&self, index: usize) -> Option<&CanFrame> {
            self.tx_log[index].as_ref()
        }
    }

    impl CanTransceiver for MockTransceiver {
        fn init(&mut self) -> ErrorCode {
            self.state = State::Initialized;
            ErrorCode::Ok
        }
        fn shutdown(&mut self) {
            self.state = State::Closed;
        }
        fn open(&mut self) -> ErrorCode {
            self.state = State::Open;
            ErrorCode::Ok
        }
        fn open_with_frame(&mut self, _frame: &CanFrame) -> ErrorCode {
            self.state = State::Open;
            ErrorCode::Ok
        }
        fn close(&mut self) -> ErrorCode {
            self.state = State::Closed;
            ErrorCode::Ok
        }
        fn mute(&mut self) -> ErrorCode {
            ErrorCode::Ok
        }
        fn unmute(&mut self) -> ErrorCode {
            ErrorCode::Ok
        }
        fn state(&self) -> State {
            self.state
        }
        fn baudrate(&self) -> u32 {
            500_000
        }
        fn hw_queue_timeout(&self) -> u16 {
            10
        }
        fn bus_id(&self) -> u8 {
            0
        }
        fn write(&mut self, frame: &CanFrame) -> ErrorCode {
            if self.tx_count < self.tx_log.len() {
                self.tx_log[self.tx_count] = Some(frame.clone());
                self.tx_count += 1;
            }
            ErrorCode::Ok
        }
        fn transceiver_state(&self) -> TransceiverState {
            TransceiverState::Active
        }
    }

    impl CanReceiver for MockTransceiver {
        fn receive(&mut self) -> Option<CanFrame> {
            if self.rx_head < self.rx_queue.len() {
                let frame = self.rx_queue[self.rx_head].take();
                if frame.is_some() {
                    self.rx_head += 1;
                }
                frame
            } else {
                None
            }
        }
    }

    // -- Mock UDS job -------------------------------------------------------

    use bsw_uds::diag_job::DiagJob;
    use bsw_uds::session::SessionMask;

    /// Echoes back a positive response: `[SID+0x40, <request bytes>]`.
    struct EchoJob;

    impl DiagJob for EchoJob {
        fn implemented_request(&self) -> &[u8] {
            &[0x3E] // TesterPresent
        }
        fn session_mask(&self) -> SessionMask {
            SessionMask::ALL
        }
        fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
            if response.len() < 2 {
                return Err(Nrc::ResponseTooLong);
            }
            response[0] = request[0] + 0x40;
            response[1] = request[1];
            Ok(2)
        }
    }

    // -- Helper: build transport with mock -----------------------------------

    fn make_transport<'a>(
        transceiver: MockTransceiver,
        router: DiagRouter<'a>,
    ) -> DiagCanTransport<'a, MockTransceiver> {
        DiagCanTransport::new(transceiver, 0x600, 0x601, router)
    }

    /// Build a raw CAN frame with a given ID and payload.
    fn rx_frame(id: u32, data: &[u8]) -> CanFrame {
        CanFrame::with_data(CanId::base(id as u16), data)
    }

    // -- Tests ---------------------------------------------------------------

    #[test]
    fn single_frame_request_gets_single_frame_response() {
        let echo = EchoJob;
        let jobs: &[&dyn DiagJob] = &[&echo];
        let router = DiagRouter::new(jobs);
        let mut trx = MockTransceiver::new();

        // TesterPresent request: SF with 2 bytes [0x3E, 0x00]
        let sf_data = [0x02, 0x3E, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
        trx.enqueue_rx(rx_frame(0x600, &sf_data));

        let mut transport = make_transport(trx, router);
        transport.poll();

        // Should have sent one response frame.
        assert_eq!(transport.transceiver().tx_count(), 1);

        // Verify the response frame: SF [0x02, 0x7E, 0x00, ...]
        let resp = transport.transceiver().tx_frame(0).unwrap();
        assert_eq!(resp.payload()[0], 0x02); // SF PCI: length=2
        assert_eq!(resp.payload()[1], 0x7E); // positive response SID
        assert_eq!(resp.payload()[2], 0x00); // sub-function echo
    }

    #[test]
    fn unknown_service_returns_negative_response() {
        let echo = EchoJob;
        let jobs: &[&dyn DiagJob] = &[&echo];
        let router = DiagRouter::new(jobs);
        let mut trx = MockTransceiver::new();

        // Request SID 0x85 which no job handles.
        let sf_data = [0x02, 0x85, 0x01, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
        trx.enqueue_rx(rx_frame(0x600, &sf_data));

        let mut transport = make_transport(trx, router);
        transport.poll();

        assert_eq!(transport.transceiver().tx_count(), 1);
        let resp = transport.transceiver().tx_frame(0).unwrap();
        // NRC response: [0x03, 0x7F, 0x85, 0x11, ...]
        assert_eq!(resp.payload()[0], 0x03); // SF PCI: length=3
        assert_eq!(resp.payload()[1], NRC_SID); // 0x7F
        assert_eq!(resp.payload()[2], 0x85); // echo of request SID
        assert_eq!(resp.payload()[3], Nrc::ServiceNotSupported.as_byte());
    }

    #[test]
    fn wrong_can_id_is_ignored() {
        let echo = EchoJob;
        let jobs: &[&dyn DiagJob] = &[&echo];
        let router = DiagRouter::new(jobs);
        let mut trx = MockTransceiver::new();

        // Frame on wrong ID (0x700, not 0x600).
        let sf_data = [0x02, 0x3E, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
        trx.enqueue_rx(rx_frame(0x700, &sf_data));

        let mut transport = make_transport(trx, router);
        transport.poll();

        // No response should be sent.
        assert_eq!(transport.transceiver().tx_count(), 0);
    }

    #[test]
    fn empty_rx_queue_does_nothing() {
        let echo = EchoJob;
        let jobs: &[&dyn DiagJob] = &[&echo];
        let router = DiagRouter::new(jobs);
        let trx = MockTransceiver::new();

        let mut transport = make_transport(trx, router);
        transport.poll();
        transport.poll();

        assert_eq!(transport.transceiver().tx_count(), 0);
    }

    #[test]
    fn lifecycle_init_opens_transceiver() {
        use bsw_lifecycle::LifecycleComponent;

        let echo = EchoJob;
        let jobs: &[&dyn DiagJob] = &[&echo];
        let router = DiagRouter::new(jobs);
        let trx = MockTransceiver::new();

        let mut transport = make_transport(trx, router);
        assert_eq!(transport.transceiver().state(), State::Closed);

        let result = transport.init();
        assert_eq!(result, TransitionResult::Done);
        assert_eq!(transport.transceiver().state(), State::Open);
    }

    #[test]
    fn lifecycle_shutdown_closes_transceiver() {
        use bsw_lifecycle::LifecycleComponent;

        let echo = EchoJob;
        let jobs: &[&dyn DiagJob] = &[&echo];
        let router = DiagRouter::new(jobs);
        let mut trx = MockTransceiver::new();
        trx.init();
        trx.open();

        let mut transport = make_transport(trx, router);
        let _ = transport.shutdown();
        assert_eq!(transport.transceiver().state(), State::Closed);
    }

    #[test]
    fn transport_name() {
        use bsw_lifecycle::LifecycleComponent;

        let echo = EchoJob;
        let jobs: &[&dyn DiagJob] = &[&echo];
        let router = DiagRouter::new(jobs);
        let trx = MockTransceiver::new();

        let transport = make_transport(trx, router);
        assert_eq!(transport.name(), "DiagCanTransport");
    }

    #[test]
    fn session_defaults_to_default() {
        let echo = EchoJob;
        let jobs: &[&dyn DiagJob] = &[&echo];
        let router = DiagRouter::new(jobs);
        let trx = MockTransceiver::new();

        let transport = make_transport(trx, router);
        assert_eq!(transport.session(), DiagSession::Default);
    }
}
