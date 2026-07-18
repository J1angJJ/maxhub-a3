#!/usr/bin/env python3
"""Check basic network reachability for the MAXHUB A3 controller.

This script does not import the robot SDK and does not send motion commands.
It only checks whether the web endpoint is reachable from the Ubuntu VM.
"""

import http.client
import socket
import sys
from urllib.parse import urlparse


ROBOT_HOST = "192.168.31.60"
ROBOT_URL = "http://192.168.31.60"
HTTP_TIMEOUT_SECONDS = 3


def check_tcp(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=HTTP_TIMEOUT_SECONDS):
            return True
    except OSError as exc:
        print(f"[FAIL] TCP {host}:{port} unreachable: {exc}")
        return False


def check_http(url: str) -> bool:
    parsed = urlparse(url)
    host = parsed.hostname
    port = parsed.port or 80
    path = parsed.path or "/"

    if not host:
        print(f"[FAIL] Invalid URL: {url}")
        return False

    try:
        conn = http.client.HTTPConnection(host, port, timeout=HTTP_TIMEOUT_SECONDS)
        conn.request("HEAD", path)
        response = conn.getresponse()
        print(f"[OK] HTTP HEAD {url} -> {response.status} {response.reason}")
        conn.close()
        return True
    except OSError as exc:
        print(f"[FAIL] HTTP {url} unreachable: {exc}")
        return False


def main() -> int:
    print("MAXHUB A3 network check")
    print(f"Robot host: {ROBOT_HOST}")
    print(f"Robot URL:  {ROBOT_URL}")
    print()

    tcp_ok = check_tcp(ROBOT_HOST, 80)
    if tcp_ok:
        print(f"[OK] TCP {ROBOT_HOST}:80 reachable")

    http_ok = check_http(ROBOT_URL)
    return 0 if tcp_ok and http_ok else 1


if __name__ == "__main__":
    sys.exit(main())
