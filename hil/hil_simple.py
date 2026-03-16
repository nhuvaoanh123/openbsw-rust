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
    return resp.startswith("0650010019"), f"resp={resp}"

def check_diag_session_extended(resp):
    return resp.startswith("0650030019"), f"resp={resp}"

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
        # ReadDID_SwVersion is multi-frame (12 bytes) — needs separate multi-frame test
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

    print(f"\n=== RESULT: {passed}/{total} PASS ===")
    sys.exit(0 if passed == total else 1)

if __name__ == "__main__":
    main()
