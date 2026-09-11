"""Local HTTP mock of the official simulator (127.0.0.1:2026 by default).

It implements the documented status codes and the accepted / request_id rules,
so the robot program can be developed and debugged before the real simulator is
available. GET /truth is a mock-only endpoint used to score the strategy.
"""
from __future__ import annotations

import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from arena import MockArena, make_case

ARENA = None
LOCK = threading.Lock()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass

    def _send(self, code: int, payload: dict):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/truth":
            with LOCK:
                truth = {
                    "n": ARENA.case.n,
                    "cleared": sorted(ARENA.cleared),
                    "virtual_time_s": ARENA.virtual_time,
                    "sources": [
                        {
                            "channel": s.channel,
                            "x": s.x,
                            "y": s.y,
                            "eff_radius": s.eff_radius,
                            "directional": s.directional,
                        }
                        for s in ARENA.case.sources
                    ],
                }
            self._send(200, truth)
        else:
            self._send(404, {"error": "unknown path"})

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b""
        ctype = (self.headers.get("Content-Type") or "").lower()
        if ctype not in ("application/json", "application/json; charset=utf-8"):
            self._send(415, {"error": "unsupported content type"})
            return
        if len(raw) > 65536:
            self._send(413, {"error": "body too large"})
            return
        if self.path not in ("/enter", "/measure", "/clear", "/exit"):
            self._send(404, {"error": "unknown path"})
            return
        try:
            data = json.loads(raw.decode("utf-8"))
            if not isinstance(data, dict):
                raise ValueError
        except Exception:
            self._send(400, {"error": "bad json"})
            return
        missing = [k for k in ("arena_id", "robot_id", "request_id") if not data.get(k)]
        if missing:
            self._send(400, {"error": "missing " + ",".join(missing)})
            return
        if data["arena_id"] != "default":
            self._send(200, {"accepted": False, "virtual_time_s": 0})
            return
        if self.path == "/enter":
            with LOCK:
                self._send(200, ARENA.enter())
            return
        if self.path == "/exit":
            with LOCK:
                self._send(200, ARENA.exit())
            return
        pos = data.get("position")
        ch = data.get("channel")
        if not isinstance(pos, dict) or "x" not in pos or "y" not in pos:
            self._send(400, {"error": "missing position"})
            return
        if not isinstance(ch, int) or not 1 <= ch <= 20:
            self._send(400, {"error": "bad channel"})
            return
        x, y = float(pos["x"]), float(pos["y"])
        if abs(x) > 2.0e6 or abs(y) > 2.0e6:
            self._send(400, {"error": "position out of range"})
            return
        with LOCK:
            if self.path == "/measure":
                self._send(200, ARENA.measure(x, y, ch))
            else:
                self._send(200, ARENA.clear(x, y, ch))


def main():
    global ARENA
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=2026)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--sources", type=int, default=None)
    parser.add_argument("--directional", type=int, default=0)
    args = parser.parse_args()
    ARENA = MockArena(make_case(args.seed, args.sources, args.directional))
    print(f"mock simulator listening on http://127.0.0.1:{args.port} (seed={args.seed})")
    ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
