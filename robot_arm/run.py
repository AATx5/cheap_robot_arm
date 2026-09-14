#!/usr/bin/env python3
"""
Launcher for the robotic arm control panel.

    python run.py                 # http://127.0.0.1:5000
    python run.py --lan           # also reachable from your phone
    python run.py --port 8080
    python run.py --no-browser
"""

import argparse
import socket
import threading
import webbrowser

from arm.server import create_app


def local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:  # noqa: BLE001
        return "127.0.0.1"
    finally:
        s.close()


def main() -> None:
    ap = argparse.ArgumentParser(description="Robotic arm control panel")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--lan", action="store_true",
                    help="bind 0.0.0.0 so other devices on your network can connect")
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    host = "0.0.0.0" if args.lan else "127.0.0.1"
    url = f"http://127.0.0.1:{args.port}"

    print("=" * 62)
    print("  Robotic Arm Control Panel")
    print("=" * 62)
    print(f"  Local:   {url}")
    if args.lan:
        print(f"  Network: http://{local_ip()}:{args.port}")
    print("  Pick 'SIM' as the port to try the UI with no hardware attached.")
    print("  Ctrl-C to stop.")
    print("=" * 62)

    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    app = create_app()
    # threaded=True matters: status polling must not block motion commands.
    app.run(host=host, port=args.port, debug=args.debug,
            threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
