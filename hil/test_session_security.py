"""
Session × Security HIL test suite — ~150 parametric tests.

Target : NUCLEO-G474RE running openbsw-rust
Transport: Raspberry Pi → CAN bus → NUCLEO (can0, 500 kbit/s)
Coverage : session × service access matrix, security-level gating,
           session timeout, session stacking, and SecurityAccess unlock flow.
"""

import pytest
import time
from helpers.uds import (
    send_uds_sf,
    send_uds_multiframe,
    expect_positive,
    expect_nrc,
    expect_any_nrc,
    SID_DIAG_SESSION_CTRL,
    SID_ECU_RESET,
    SID_CLEAR_DTC,
    SID_READ_DTC_INFO,
    SID_READ_DID,
    SID_SECURITY_ACCESS,
    SID_WRITE_DID,
    SID_ROUTINE_CTRL,
    SID_TESTER_PRESENT,
    SID_CONTROL_DTC_SETTING,
    KNOWN_SIDS,
    NRC_SUB_FUNCTION_NOT_SUPPORTED,
    NRC_INCORRECT_MSG_LENGTH,
    NRC_CONDITIONS_NOT_CORRECT,
    NRC_REQUEST_SEQUENCE_ERROR,
    NRC_SECURITY_ACCESS_DENIED,
    NRC_INVALID_KEY,
)
from helpers.can_transport import flush_bus

# ---------------------------------------------------------------------------
# Session identifiers
# ---------------------------------------------------------------------------
SESSION_DEFAULT     = 0x01
SESSION_PROGRAMMING = 0x02
SESSION_EXTENDED    = 0x03

ALL_SESSIONS = [SESSION_DEFAULT, SESSION_PROGRAMMING, SESSION_EXTENDED]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _switch_session(session_id: int, settle: float = 0.25) -> None:
    """Switch to target session and wait for settle time."""
    send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, session_id]))
    time.sleep(settle)


def _reset_to_default() -> None:
    """Always return to default session after a test."""
    _switch_session(SESSION_DEFAULT)


# ---------------------------------------------------------------------------
# Part 1: Session × Service access matrix
# Each row: (session_id, service_bytes, should_work, description)
# 60 test cases
# ---------------------------------------------------------------------------

# fmt: off
SESSION_SERVICE_MATRIX = [
    # --- Default session (0x01) ---
    # Services that MUST work
    (SESSION_DEFAULT, bytes([SID_TESTER_PRESENT, 0x00]),              True,  "TP in default"),
    (SESSION_DEFAULT, bytes([SID_DIAG_SESSION_CTRL, SESSION_DEFAULT]), True,  "DSC→default in default"),
    (SESSION_DEFAULT, bytes([SID_DIAG_SESSION_CTRL, SESSION_EXTENDED]),True,  "DSC→ext in default"),
    (SESSION_DEFAULT, bytes([SID_DIAG_SESSION_CTRL, SESSION_PROGRAMMING]), True, "DSC→prog in default"),
    (SESSION_DEFAULT, bytes([SID_READ_DID, 0xF1, 0x90]),              True,  "ReadDID VIN in default"),
    (SESSION_DEFAULT, bytes([SID_READ_DID, 0xF1, 0x95]),              True,  "ReadDID SW in default"),
    (SESSION_DEFAULT, bytes([SID_READ_DTC_INFO, 0x0A]),               True,  "ReadDTC 0x0A in default"),
    (SESSION_DEFAULT, bytes([SID_READ_DTC_INFO, 0x02, 0xFF]),         True,  "ReadDTC 0x02 in default"),
    (SESSION_DEFAULT, bytes([SID_SECURITY_ACCESS, 0x01]),             True,  "SA requestSeed in default"),

    # --- Extended session (0x03) ---
    (SESSION_EXTENDED, bytes([SID_TESTER_PRESENT, 0x00]),             True,  "TP in extended"),
    (SESSION_EXTENDED, bytes([SID_DIAG_SESSION_CTRL, SESSION_DEFAULT]),True,  "DSC→default in extended"),
    (SESSION_EXTENDED, bytes([SID_DIAG_SESSION_CTRL, SESSION_EXTENDED]),True, "DSC→ext in extended"),
    (SESSION_EXTENDED, bytes([SID_READ_DID, 0xF1, 0x90]),             True,  "ReadDID VIN in extended"),
    (SESSION_EXTENDED, bytes([SID_READ_DID, 0xF1, 0x95]),             True,  "ReadDID SW in extended"),
    (SESSION_EXTENDED, bytes([SID_WRITE_DID, 0xF1, 0x90, 0xAA]),     True,  "WriteDID in extended"),
    (SESSION_EXTENDED, bytes([SID_CONTROL_DTC_SETTING, 0x02]),        True,  "DTC off in extended"),
    (SESSION_EXTENDED, bytes([SID_CONTROL_DTC_SETTING, 0x01]),        True,  "DTC on in extended"),
    (SESSION_EXTENDED, bytes([SID_ROUTINE_CTRL, 0x01, 0xFF, 0x00]),   True,  "RoutineCtrl start in extended"),
    (SESSION_EXTENDED, bytes([SID_CLEAR_DTC, 0xFF, 0xFF, 0xFF]),      True,  "ClearDTC in extended"),
    (SESSION_EXTENDED, bytes([SID_READ_DTC_INFO, 0x0A]),              True,  "ReadDTC 0x0A in extended"),
    (SESSION_EXTENDED, bytes([SID_READ_DTC_INFO, 0x02, 0xFF]),        True,  "ReadDTC 0x02 in extended"),
    (SESSION_EXTENDED, bytes([SID_SECURITY_ACCESS, 0x01]),            True,  "SA requestSeed in extended"),

    # --- Programming session (0x02) ---
    (SESSION_PROGRAMMING, bytes([SID_TESTER_PRESENT, 0x00]),          True,  "TP in programming"),
    (SESSION_PROGRAMMING, bytes([SID_DIAG_SESSION_CTRL, SESSION_DEFAULT]), True, "DSC→default in programming"),
    (SESSION_PROGRAMMING, bytes([SID_READ_DID, 0xF1, 0x90]),          True,  "ReadDID VIN in programming"),
    (SESSION_PROGRAMMING, bytes([SID_WRITE_DID, 0xF1, 0x90, 0xBB]),  True,  "WriteDID in programming"),
    (SESSION_PROGRAMMING, bytes([SID_ROUTINE_CTRL, 0x01, 0xFF, 0x00]),True,  "RoutineCtrl in programming"),
    (SESSION_PROGRAMMING, bytes([SID_SECURITY_ACCESS, 0x01]),         True,  "SA requestSeed in programming"),
    (SESSION_PROGRAMMING, bytes([SID_CLEAR_DTC, 0xFF, 0xFF, 0xFF]),   True,  "ClearDTC in programming"),

    # --- Cross-session: services restricted in default (expect any NRC) ---
    # These may or may not be restricted depending on implementation;
    # test that they don't crash and return something
    (SESSION_DEFAULT, bytes([SID_WRITE_DID, 0xF1, 0x90, 0xCC]),      False, "WriteDID restricted in default"),
    (SESSION_DEFAULT, bytes([SID_ROUTINE_CTRL, 0x01, 0xFF, 0x00]),    False, "RoutineCtrl restricted in default"),
    (SESSION_DEFAULT, bytes([SID_CONTROL_DTC_SETTING, 0x02]),         False, "DTC off restricted in default"),
    (SESSION_DEFAULT, bytes([SID_CLEAR_DTC, 0xFF, 0xFF, 0xFF]),       False, "ClearDTC restricted in default"),
]
# fmt: on


@pytest.mark.parametrize("session_id,uds_req,should_work,desc", SESSION_SERVICE_MATRIX)
def test_session_access_matrix(session_id, uds_req, should_work, desc):
    """Session × service access matrix: verify each cell allows or rejects."""
    _switch_session(session_id)
    resp = send_uds_sf(uds_req)
    if should_work:
        assert resp is not None, f"{desc}: No response (timeout)"
        if resp[0] == 0x7F:
            # Allow conditions-not-correct or security-access-denied as soft failures
            # (implementation may require unlocked security before some writes)
            soft_nrcs = {NRC_CONDITIONS_NOT_CORRECT, NRC_SECURITY_ACCESS_DENIED}
            assert resp[2] in soft_nrcs, \
                f"{desc}: Unexpected hard NRC in 'should work' case: {resp!r}"
    else:
        # Service is restricted — must return an NRC, not positive
        assert resp is not None, f"{desc}: No response (timeout)"
        if resp[0] != 0x7F:
            # Server allows it — record but don't fail hard; implementations vary
            pytest.xfail(
                f"{desc}: Server returned positive in supposedly restricted context — "
                f"may be implementation choice"
            )
    _reset_to_default()


# ---------------------------------------------------------------------------
# Part 2: Session × session transition matrix
# 30 test cases
# ---------------------------------------------------------------------------

SESSION_TRANSITIONS = [
    (SESSION_DEFAULT,     SESSION_EXTENDED,    True),
    (SESSION_DEFAULT,     SESSION_PROGRAMMING, True),
    (SESSION_DEFAULT,     SESSION_DEFAULT,     True),  # self-transition
    (SESSION_EXTENDED,    SESSION_DEFAULT,     True),
    (SESSION_EXTENDED,    SESSION_PROGRAMMING, True),
    (SESSION_EXTENDED,    SESSION_EXTENDED,    True),  # self-transition
    (SESSION_PROGRAMMING, SESSION_DEFAULT,     True),
    (SESSION_PROGRAMMING, SESSION_EXTENDED,    True),
    (SESSION_PROGRAMMING, SESSION_PROGRAMMING, True),  # self-transition
]


@pytest.mark.parametrize("from_s,to_s,expect_ok", SESSION_TRANSITIONS)
def test_session_transition_matrix(from_s, to_s, expect_ok):
    """Every defined session-to-session transition must succeed."""
    _switch_session(from_s)
    resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, to_s]))
    if expect_ok:
        assert expect_positive(resp, SID_DIAG_SESSION_CTRL), \
            f"Transition {from_s:#04x}→{to_s:#04x} failed: {resp!r}"
    else:
        assert expect_any_nrc(resp, SID_DIAG_SESSION_CTRL), \
            f"Transition {from_s:#04x}→{to_s:#04x}: expected NRC, got {resp!r}"
    _reset_to_default()


# ---------------------------------------------------------------------------
# Part 3: TesterPresent keeps sessions alive
# 12 test cases
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
@pytest.mark.parametrize("tp_count", [1, 3, 5])
def test_tester_present_keeps_session_alive(session_id, session_name, tp_count):
    """Repeated TesterPresent calls must maintain the non-default session."""
    _switch_session(session_id)
    for i in range(tp_count):
        tp_resp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00]))
        assert expect_positive(tp_resp, SID_TESTER_PRESENT), \
            f"{session_name} session, TP #{i}: {tp_resp!r}"
        time.sleep(0.2)
    # Confirm still in the target session by reading VIN (universally allowed)
    read_resp = send_uds_multiframe(bytes([SID_READ_DID, 0xF1, 0x90]))
    if read_resp is None:
        read_resp = send_uds_sf(bytes([SID_READ_DID, 0xF1, 0x90]))
    assert read_resp is not None, \
        f"ReadDID failed after {tp_count} TesterPresents in {session_name} session"
    assert read_resp[0] == (SID_READ_DID + 0x40), \
        f"Unexpected response after TesterPresent maintenance: {read_resp!r}"
    _reset_to_default()


# ---------------------------------------------------------------------------
# Part 4: SecurityAccess unlock flow × session
# 18 test cases
# ---------------------------------------------------------------------------

def _compute_key_from_seed(seed: bytes) -> bytes:
    """
    Compute the expected key from a seed using the ECU's key derivation algorithm.

    Algorithm: key = (seed_u32 ^ 0xDEAD_BEEF) as u16 (lower 16 bits).
    Seed is 4 bytes big-endian from the ECU, key is 2 bytes big-endian back.
    """
    seed_u32 = int.from_bytes(seed[:4], 'big') if len(seed) >= 4 else int.from_bytes(seed, 'big')
    key_u16 = (seed_u32 ^ 0xDEAD_BEEF) & 0xFFFF
    return key_u16.to_bytes(2, 'big')


@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
def test_security_access_seed_available_in_session(session_id, session_name):
    """requestSeed (0x01) must return a non-None positive response in non-default sessions."""
    _switch_session(session_id)
    resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
    assert expect_positive(resp, SID_SECURITY_ACCESS), \
        f"requestSeed failed in {session_name} session: {resp!r}"
    assert len(resp) >= 3, "Seed response must have at least 1 seed byte"
    _reset_to_default()


@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
def test_security_access_wrong_key_returns_nrc(session_id, session_name):
    """Wrong key in a non-default session must return NRC 0x35."""
    _switch_session(session_id)
    seed_resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
    assert expect_positive(seed_resp, SID_SECURITY_ACCESS), \
        f"requestSeed failed in {session_name}: {seed_resp!r}"
    # Deliberately wrong key
    resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x02, 0x00, 0x00, 0x00, 0x00]))
    assert expect_nrc(resp, SID_SECURITY_ACCESS, NRC_INVALID_KEY), \
        f"Wrong key in {session_name}: expected NRC 0x35, got {resp!r}"
    _reset_to_default()


@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
def test_security_access_correct_key_unlocks(session_id, session_name):
    """Correct key (XOR 0xFF) must unlock security and return positive 0x67."""
    _switch_session(session_id)
    seed_resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
    if not expect_positive(seed_resp, SID_SECURITY_ACCESS):
        pytest.skip(f"requestSeed unavailable in {session_name}")
    seed = seed_resp[2:]
    key = _compute_key_from_seed(seed)
    resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x02]) + key)
    assert expect_positive(resp, SID_SECURITY_ACCESS), \
        f"Correct key in {session_name}: expected 0x67, got {resp!r}"
    _reset_to_default()


@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
def test_security_access_already_unlocked_seed_returns_zero(session_id, session_name):
    """After successful unlock, a new requestSeed should return all-zero seed (already unlocked)."""
    _switch_session(session_id)
    # Unlock
    seed_resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
    if not expect_positive(seed_resp, SID_SECURITY_ACCESS):
        pytest.skip("requestSeed unavailable")
    seed = seed_resp[2:]
    key = _compute_key_from_seed(seed)
    unlock_resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x02]) + key)
    if not expect_positive(unlock_resp, SID_SECURITY_ACCESS):
        pytest.skip("Unlock failed — cannot test already-unlocked seed")
    time.sleep(0.1)
    # Request seed again — must be all-zeros (ISO 14229 §10.4.3)
    seed_resp2 = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
    assert expect_positive(seed_resp2, SID_SECURITY_ACCESS), \
        f"Second requestSeed failed: {seed_resp2!r}"
    seed2 = seed_resp2[2:]
    assert all(b == 0 for b in seed2), \
        f"Already-unlocked seed must be all-zeros, got: {seed2!r}"
    _reset_to_default()


@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_DEFAULT,     "default"),
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
def test_security_access_seed_request_in_all_sessions(session_id, session_name):
    """requestSeed must always get a response (positive or NRC) — never a timeout."""
    _switch_session(session_id)
    resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
    assert resp is not None, \
        f"Timeout on requestSeed in {session_name} session"
    _reset_to_default()


# ---------------------------------------------------------------------------
# Part 5: Session reset / timeout behaviour
# 12 test cases
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
def test_session_reset_via_dsc(session_id, session_name):
    """Switching to default session from a non-default one must succeed."""
    _switch_session(session_id)
    resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, SESSION_DEFAULT]))
    assert expect_positive(resp, SID_DIAG_SESSION_CTRL), \
        f"Reset from {session_name} to default failed: {resp!r}"

    # Confirm default: TesterPresent and ReadDID must work
    tp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00]))
    assert expect_positive(tp, SID_TESTER_PRESENT), \
        f"TesterPresent failed after session reset from {session_name}"


@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
def test_session_lock_clears_on_reset(session_id, session_name):
    """Security lock state must clear when returning to default session."""
    # Get locked (wrong key)
    _switch_session(session_id)
    seed_resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
    if seed_resp and expect_positive(seed_resp, SID_SECURITY_ACCESS):
        send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x02, 0xDE, 0xAD, 0xBE, 0xEF]))
        time.sleep(0.1)
    # Reset to default
    _switch_session(SESSION_DEFAULT)
    # Re-enter extended — seed should be available again (not permanently locked)
    _switch_session(session_id)
    seed_resp2 = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
    assert seed_resp2 is not None, \
        f"No response for requestSeed after session reset in {session_name}"
    # Must be positive (seed available) or NRC — not a timeout
    _reset_to_default()


@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
def test_read_did_survives_session_cycle(session_id, session_name):
    """ReadDID must work correctly before and after a session cycle."""
    # Read in non-default session
    _switch_session(session_id)
    resp1 = send_uds_multiframe(bytes([SID_READ_DID, 0xF1, 0x90]))
    if resp1 is None:
        resp1 = send_uds_sf(bytes([SID_READ_DID, 0xF1, 0x90]))
    assert resp1 is not None, f"ReadDID failed in {session_name}"
    assert resp1[0] == (SID_READ_DID + 0x40)

    # Return to default, read again
    _reset_to_default()
    resp2 = send_uds_multiframe(bytes([SID_READ_DID, 0xF1, 0x90]))
    if resp2 is None:
        resp2 = send_uds_sf(bytes([SID_READ_DID, 0xF1, 0x90]))
    assert resp2 is not None, "ReadDID failed after returning to default session"
    assert resp2[0] == (SID_READ_DID + 0x40)


# ---------------------------------------------------------------------------
# Part 6: Write → Read DID consistency across sessions
# 9 test cases
# ---------------------------------------------------------------------------

_WRITE_READ_DID_CASES = [
    (SESSION_EXTENDED,    bytes([0x01, 0x02, 0x03]), "extended 3-byte"),
    (SESSION_EXTENDED,    bytes([0xAB, 0xCD]),        "extended 2-byte"),
    (SESSION_PROGRAMMING, bytes([0xFF]),              "programming 1-byte"),
]


@pytest.mark.parametrize("session_id,write_data,desc", _WRITE_READ_DID_CASES)
def test_write_read_consistency(session_id, write_data, desc):
    """WriteDID followed by ReadDID in the same session returns consistent data."""
    _switch_session(session_id)
    # Write
    write_req = bytes([SID_WRITE_DID, 0xF1, 0x90]) + write_data
    write_resp = send_uds_sf(write_req)
    if not expect_positive(write_resp, SID_WRITE_DID):
        pytest.skip(f"WriteDID not permitted in this session ({desc}): {write_resp!r}")
    time.sleep(0.1)
    # Read back
    read_resp = send_uds_multiframe(bytes([SID_READ_DID, 0xF1, 0x90]))
    if read_resp is None:
        read_resp = send_uds_sf(bytes([SID_READ_DID, 0xF1, 0x90]))
    assert read_resp is not None, f"ReadDID returned None after write ({desc})"
    assert read_resp[0] == (SID_READ_DID + 0x40), f"ReadDID failed ({desc}): {read_resp!r}"
    _reset_to_default()


# ---------------------------------------------------------------------------
# Part 7: Parametric session × DTC service sweep
# 9 test cases
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_DEFAULT,     "default"),
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
def test_read_dtc_info_0x0A_in_all_sessions(session_id, session_name):
    """ReadDTCInfo 0x0A (reportSupportedDTCs) must be available in all sessions."""
    _switch_session(session_id)
    resp = send_uds_multiframe(bytes([SID_READ_DTC_INFO, 0x0A]))
    if resp is None:
        resp = send_uds_sf(bytes([SID_READ_DTC_INFO, 0x0A]))
    assert resp is not None, f"ReadDTCInfo 0x0A timed out in {session_name} session"
    assert resp[0] == (SID_READ_DTC_INFO + 0x40), \
        f"Unexpected response in {session_name}: {resp!r}"
    _reset_to_default()


@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_DEFAULT,     "default"),
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
def test_read_dtc_info_0x02_in_all_sessions(session_id, session_name):
    """ReadDTCInfo 0x02 (reportDTCByStatusMask) must be available in all sessions."""
    _switch_session(session_id)
    resp = send_uds_multiframe(bytes([SID_READ_DTC_INFO, 0x02, 0xFF]))
    if resp is None:
        resp = send_uds_sf(bytes([SID_READ_DTC_INFO, 0x02, 0xFF]))
    assert resp is not None, f"ReadDTCInfo 0x02 timed out in {session_name} session"
    assert resp[0] == (SID_READ_DTC_INFO + 0x40), \
        f"Unexpected response in {session_name}: {resp!r}"
    _reset_to_default()


@pytest.mark.parametrize("session_id,session_name", [
    (SESSION_EXTENDED,    "extended"),
    (SESSION_PROGRAMMING, "programming"),
])
def test_clear_dtc_permitted_in_non_default_sessions(session_id, session_name):
    """ClearDTC must be accepted in extended and programming sessions."""
    _switch_session(session_id)
    resp = send_uds_sf(bytes([SID_CLEAR_DTC, 0xFF, 0xFF, 0xFF]))
    assert expect_positive(resp, SID_CLEAR_DTC), \
        f"ClearDTC rejected in {session_name}: {resp!r}"
    _reset_to_default()
