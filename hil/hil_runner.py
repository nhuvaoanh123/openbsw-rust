#!/usr/bin/env python3
"""HIL test runner for Rust BSW stack on STM32.

Runs on the Raspberry Pi connected to the CAN bus.
Sends UDS requests and verifies responses.

Usage:
    python3 hil_runner.py [--bus can0] [--scenario all|basic|security|multiframe|negative]
"""

import sys
import time
import argparse
from can_helpers import *

class TestResult:
    def __init__(self, name, passed, detail=""):
        self.name = name
        self.passed = passed
        self.detail = detail

def test_tester_present(bus) -> TestResult:
    """UDS 0x3E TesterPresent."""
    resp = send_recv_uds(bus, bytes([0x3E, 0x00]))
    if resp and resp[0] == 0x7E and resp[1] == 0x00:
        return TestResult("TesterPresent", True, f"resp={resp.hex()}")
    return TestResult("TesterPresent", False, f"resp={resp}")

def test_diag_session_default(bus) -> TestResult:
    """UDS 0x10 01 DiagSessionControl default."""
    resp = send_recv_uds(bus, bytes([0x10, 0x01]))
    if resp and resp[0] == 0x50 and resp[1] == 0x01:
        return TestResult("DiagSessionDefault", True, f"resp={resp.hex()}")
    return TestResult("DiagSessionDefault", False, f"resp={resp}")

def test_diag_session_extended(bus) -> TestResult:
    """UDS 0x10 03 DiagSessionControl extended."""
    resp = send_recv_uds(bus, bytes([0x10, 0x03]))
    if resp and resp[0] == 0x50 and resp[1] == 0x03:
        return TestResult("DiagSessionExtended", True, f"resp={resp.hex()}")
    return TestResult("DiagSessionExtended", False, f"resp={resp}")

def test_read_did_sw_version(bus) -> TestResult:
    """UDS 0x22 F195 ReadDID SW version (SF response)."""
    resp = send_recv_uds_multiframe(bus, bytes([0x22, 0xF1, 0x95]))
    if resp and resp[0] == 0x62 and resp[1] == 0xF1 and resp[2] == 0x95:
        sw_ver = resp[3:].decode('ascii', errors='replace')
        return TestResult("ReadDID_SwVersion", True, f"version='{sw_ver}'")
    return TestResult("ReadDID_SwVersion", False, f"resp={resp}")

def test_read_did_vin(bus) -> TestResult:
    """UDS 0x22 F190 ReadDID VIN (multi-frame response, 19 bytes)."""
    resp = send_recv_uds_multiframe(bus, bytes([0x22, 0xF1, 0x90]))
    if resp and len(resp) >= 19 and resp[0] == 0x62 and resp[1] == 0xF1 and resp[2] == 0x90:
        vin = resp[3:19].decode('ascii', errors='replace')
        return TestResult("ReadDID_VIN", True, f"VIN='{vin}'")
    return TestResult("ReadDID_VIN", False, f"resp={resp} len={len(resp) if resp else 0}")

def test_unknown_service(bus) -> TestResult:
    """Unknown SID -> NRC 0x11 serviceNotSupported."""
    resp = send_recv_uds(bus, bytes([0xAA]))
    if resp and resp[0] == 0x7F and resp[1] == 0xAA and resp[2] == 0x11:
        return TestResult("UnknownService_NRC", True)
    return TestResult("UnknownService_NRC", False, f"resp={resp}")

def test_unknown_did(bus) -> TestResult:
    """ReadDID unknown DID -> NRC 0x31 requestOutOfRange."""
    resp = send_recv_uds(bus, bytes([0x22, 0x00, 0x01]))
    if resp and resp[0] == 0x7F and resp[1] == 0x22 and resp[2] == 0x31:
        return TestResult("UnknownDID_NRC", True)
    return TestResult("UnknownDID_NRC", False, f"resp={resp}")

def test_short_request(bus) -> TestResult:
    """Too-short request -> NRC 0x13 incorrectMessageLength."""
    resp = send_recv_uds(bus, bytes([0x22]))  # ReadDID needs 3 bytes
    if resp and resp[0] == 0x7F:
        return TestResult("ShortRequest_NRC", True, f"nrc=0x{resp[2]:02X}")
    return TestResult("ShortRequest_NRC", False, f"resp={resp}")

def run_tests(bus, categories=None):
    """Run all test scenarios."""
    all_tests = [
        ("basic", test_tester_present),
        ("basic", test_diag_session_default),
        ("basic", test_diag_session_extended),
        ("basic", test_read_did_sw_version),
        ("multiframe", test_read_did_vin),
        ("negative", test_unknown_service),
        ("negative", test_unknown_did),
        ("negative", test_short_request),
    ]

    if categories:
        tests = [(cat, fn) for cat, fn in all_tests if cat in categories]
    else:
        tests = all_tests

    results = []
    for category, test_fn in tests:
        # Flush any stale frames before each test
        while bus.recv(timeout=0.05):
            pass
        try:
            result = test_fn(bus)
            results.append((category, result))
            status = "PASS" if result.passed else "FAIL"
            print(f"  [{status}] {result.name}: {result.detail}")
        except Exception as e:
            results.append((category, TestResult(test_fn.__name__, False, str(e))))
            print(f"  [FAIL] {test_fn.__name__}: {e}")
        time.sleep(0.3)  # longer pause — server needs time to process + avoid bus-off

    return results

def main():
    parser = argparse.ArgumentParser(description="HIL test runner for Rust BSW")
    parser.add_argument("--bus", default="can0", help="CAN interface (default: can0)")
    parser.add_argument("--category", nargs="*", choices=["basic", "security", "multiframe", "negative", "all"],
                        default=["all"])
    args = parser.parse_args()

    categories = None if "all" in args.category else set(args.category)

    print(f"\n=== Rust BSW HIL Test Suite ===")
    print(f"Bus: {args.bus}, Categories: {categories or 'all'}\n")

    bus = create_bus(channel=args.bus)

    # Flush stale frames (wait longer for bus to settle)
    while bus.recv(timeout=0.3):
        pass

    # Warm-up: send a TesterPresent and discard the response.
    # This ensures the server's CAN controller is synchronized to the bus.
    print("Warm-up: sending TesterPresent...")
    send_recv_uds(bus, bytes([0x3E, 0x00]), timeout=1.0)
    while bus.recv(timeout=0.2):
        pass
    time.sleep(0.3)
    print()

    results = run_tests(bus, categories)
    bus.shutdown()

    # Summary
    passed = sum(1 for _, r in results if r.passed)
    total = len(results)
    print(f"\n=== RESULT: {passed}/{total} PASS ===")

    if passed == total:
        print("All tests passed!")
        sys.exit(0)
    else:
        failed = [(cat, r) for cat, r in results if not r.passed]
        print("Failed tests:")
        for cat, r in failed:
            print(f"  [{cat}] {r.name}: {r.detail}")
        sys.exit(1)

if __name__ == "__main__":
    main()
