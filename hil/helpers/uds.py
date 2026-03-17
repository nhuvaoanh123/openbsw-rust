"""UDS (ISO 14229) request/response helpers."""

from typing import Optional
from .can_transport import send_recv_raw, send_recv_multi, REQUEST_ID, RESPONSE_ID, flush_bus
from .isotp import encode_sf, decode_sf, is_sf

# UDS Service IDs
SID_DIAG_SESSION_CTRL = 0x10
SID_ECU_RESET = 0x11
SID_CLEAR_DTC = 0x14
SID_READ_DTC_INFO = 0x19
SID_READ_DID = 0x22
SID_SECURITY_ACCESS = 0x27
SID_WRITE_DID = 0x2E
SID_ROUTINE_CTRL = 0x31
SID_TESTER_PRESENT = 0x3E
SID_CONTROL_DTC_SETTING = 0x85

# NRC codes
NRC_SERVICE_NOT_SUPPORTED = 0x11
NRC_SUB_FUNCTION_NOT_SUPPORTED = 0x12
NRC_INCORRECT_MSG_LENGTH = 0x13
NRC_CONDITIONS_NOT_CORRECT = 0x22
NRC_REQUEST_SEQUENCE_ERROR = 0x24
NRC_REQUEST_OUT_OF_RANGE = 0x31
NRC_SECURITY_ACCESS_DENIED = 0x33
NRC_INVALID_KEY = 0x35
NRC_EXCEEDED_ATTEMPTS = 0x36
NRC_GENERAL_PROGRAMMING_FAILURE = 0x72

# Known service SIDs (that the server handles)
KNOWN_SIDS = {0x10, 0x11, 0x14, 0x19, 0x22, 0x27, 0x2E, 0x31, 0x3E, 0x85}


def send_uds_sf(request: bytes, timeout: float = 2.0) -> Optional[bytes]:
    """Send UDS request as SF, receive response (auto-handles SF and multi-frame).
    Returns UDS payload or None."""
    sf_hex = encode_sf(request)
    resp_hex = send_recv_raw(REQUEST_ID, sf_hex, RESPONSE_ID, timeout)
    if resp_hex is None:
        return None
    try:
        return decode_sf(resp_hex)
    except ValueError:
        # Non-SF response — check if it's a First Frame and try multi-frame
        raw = bytes.fromhex(resp_hex)
        if len(raw) >= 2 and (raw[0] >> 4) == 1:
            # FF detected — retry with multi-frame handler
            return send_uds_multiframe(request, timeout=timeout + 1)
        return raw


def send_uds_multiframe(request: bytes, timeout: float = 3.0) -> Optional[bytes]:
    """Send UDS request as SF, receive multi-frame response. Returns reassembled payload."""
    sf_hex = encode_sf(request)
    resp_hex = send_recv_multi(REQUEST_ID, sf_hex, RESPONSE_ID, REQUEST_ID, timeout)
    if resp_hex is None:
        return None
    return bytes.fromhex(resp_hex)


def expect_positive(resp: Optional[bytes], sid: int) -> bool:
    """Check response is positive (SID + 0x40)."""
    if resp is None:
        return False
    return len(resp) >= 1 and resp[0] == (sid + 0x40)


def expect_nrc(resp: Optional[bytes], sid: int, nrc: int) -> bool:
    """Check response is NRC with specific code."""
    if resp is None:
        return False
    return (len(resp) >= 3 and resp[0] == 0x7F
            and resp[1] == sid and resp[2] == nrc)


def expect_any_nrc(resp: Optional[bytes], sid: int) -> bool:
    """Check response is any NRC for the given SID."""
    if resp is None:
        return False
    return len(resp) >= 3 and resp[0] == 0x7F and resp[1] == sid
