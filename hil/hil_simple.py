#!/usr/bin/env python3
"""Simple HIL test runner using cansend/candump (no python-can dependency)."""

import subprocess
import sys
import time

IFACE = "can0"

def cansend(can_id, data_hex):
    """Send a CAN frame. data_hex like '023E00'."""
    frame = f"{can_id:03X}#{data_hex}"
    subprocess.run(["cansend", IFACE, frame], check=True, timeout=2)

def candump_one(timeout=2.0, filter_id=0x601):
    """Receive one frame matching filter_id. Returns data bytes as hex string or None."""
    try:
        result = subprocess.run(
            ["candump", f"{IFACE},{filter_id:03X}:7FF",
             "-n", "1", "-T", str(int(timeout * 1000))],
            capture_output=True, text=True, timeout=timeout + 1
        )
        # Parse candump output: "  can0  601   [3]  02 7E 00"
        for line in result.stdout.strip().split('\n'):
            if f"{filter_id:03X}" in line:
                # Extract data bytes after [N]
                parts = line.split(']')
                if len(parts) >= 2:
                    data = parts[1].strip().replace(' ', '')
                    return data
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
        pass
    return None

def send_recv(req_hex, timeout=2.0):
    """Send ISO-TP SF request, receive SF response. Returns data hex or None."""
    req_bytes = bytes.fromhex(req_hex)
    sf_data = f"{len(req_bytes):02X}{req_hex}"

    # Start candump FIRST (background), then send — ensures we catch the response
    dump_proc = subprocess.Popen(
        ["candump", f"{IFACE},601:7FF", "-n", "1", "-T", str(int(timeout * 1000))],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    time.sleep(0.05)  # brief settle for candump to attach

    cansend(0x600, sf_data)

    try:
        stdout, _ = dump_proc.communicate(timeout=timeout + 1)
        for line in stdout.strip().split('\n'):
            if "601" in line:
                parts = line.split(']')
                if len(parts) >= 2:
                    return parts[1].strip().replace(' ', '')
    except subprocess.TimeoutExpired:
        dump_proc.kill()
    return None

def send_recv_multiframe(req_hex, timeout=3.0):
    """Send ISO-TP SF request, receive multi-frame response (FF + FC + CFs).
    Returns reassembled data hex string or None."""
    req_bytes = bytes.fromhex(req_hex)
    sf_data = f"{len(req_bytes):02X}{req_hex}"

    # Capture multiple frames (FF + up to 4 CFs)
    dump_proc = subprocess.Popen(
        ["candump", f"{IFACE},601:7FF", "-n", "5", "-T", str(int(timeout * 1000))],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    time.sleep(0.05)

    # Send request
    cansend(0x600, sf_data)

    # Wait for FF
    time.sleep(0.1)

    # Send FC(CTS, BS=0, STmin=0) on request ID
    cansend(0x600, "300000")

    try:
        stdout, _ = dump_proc.communicate(timeout=timeout + 1)
    except subprocess.TimeoutExpired:
        dump_proc.kill()
        stdout, _ = dump_proc.communicate()

    # Parse all received frames
    frames = []
    for line in stdout.strip().split('\n'):
        if "601" in line and ']' in line:
            parts = line.split(']')
            if len(parts) >= 2:
                data_hex = parts[1].strip().replace(' ', '')
                frames.append(data_hex)

    if not frames:
        return None

    # Check first frame type
    first_pci = int(frames[0][:2], 16)
    frame_type = first_pci >> 4

    if frame_type == 0:
        # SF response
        length = first_pci & 0x0F
        return frames[0][2:2 + length * 2]

    if frame_type == 1:
        # FF response — reassemble
        msg_len = ((first_pci & 0x0F) << 8) | int(frames[0][2:4], 16)
        result = frames[0][4:]  # first 6 data bytes (12 hex chars)

        for f in frames[1:]:
            f_pci = int(f[:2], 16)
            if (f_pci >> 4) == 2:  # CF
                result += f[2:]  # up to 7 bytes (14 hex chars)

        # Trim to msg_len
        result = result[:msg_len * 2]
        return result

    return None

def run_test(name, req_hex, check_fn):
    """Run a single test. Returns (name, passed, detail)."""
    resp = send_recv(req_hex, timeout=2.0)
    time.sleep(0.2)
    if resp is None:
        return (name, False, "timeout")
    passed, detail = check_fn(resp)
    return (name, passed, detail)

def check_tester_present(resp):
    return resp.startswith("027E00"), f"resp={resp}"

def check_diag_session_default(resp):
    # Positive response: 0x50 0x01 + P2/P2* timing (may vary)
    return resp.upper().startswith("06500100") or resp.upper().startswith("065001"), f"resp={resp}"

def check_diag_session_extended(resp):
    return resp.upper().startswith("06500300") or resp.upper().startswith("065003"), f"resp={resp}"

def check_read_did_sw(resp):
    return "62F195" in resp.upper(), f"resp={resp}"

def check_unknown_service(resp):
    return resp.upper().startswith("037FAA11"), f"resp={resp}"

def check_unknown_did(resp):
    return resp.upper().startswith("037F2231"), f"resp={resp}"

def check_short_request(resp):
    return resp.upper().startswith("037F22"), f"resp={resp}"

def main():
    tests = [
        ("TesterPresent", "3E00", check_tester_present),
        ("DiagSessionDefault", "1001", check_diag_session_default),
        ("DiagSessionExtended", "1003", check_diag_session_extended),
        ("UnknownService_NRC", "AA", check_unknown_service),
        ("UnknownDID_NRC", "220001", check_unknown_did),
        ("ShortRequest_NRC", "22", check_short_request),
    ]

    # Multi-frame test — ReadDID VIN (19 bytes response)
    multiframe_tests = [
        ("ReadDID_VIN_multiframe", "22F190"),
    ]

    print(f"\n=== Rust BSW HIL Test Suite (simple) ===\n")

    passed = 0
    total = len(tests)

    for name, req, check in tests:
        ok, detail = False, "timeout"
        try:
            _, ok, detail = run_test(name, req, check)
        except Exception as e:
            detail = str(e)
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}] {name}: {detail}")
        if ok:
            passed += 1
        time.sleep(1.0)  # 1 second — bus-off recovery needs 128×11 recessive bits + reinit

    # Run multi-frame tests
    for name, req_hex in multiframe_tests:
        total += 1
        time.sleep(1.0)
        try:
            resp = send_recv_multiframe(req_hex, timeout=3.0)
            if resp and resp.upper().startswith("62F190"):
                # Decode VIN from hex
                vin_hex = resp[6:]  # skip 62F190
                vin = bytes.fromhex(vin_hex).decode('ascii', errors='replace')
                print(f"  [PASS] {name}: VIN='{vin}'")
                passed += 1
            else:
                print(f"  [FAIL] {name}: resp={resp}")
        except Exception as e:
            print(f"  [FAIL] {name}: {e}")

    print(f"\n=== RESULT: {passed}/{total} PASS ===")
    sys.exit(0 if passed == total else 1)

if __name__ == "__main__":
    main()
