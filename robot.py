"""HTTP+JSON client for the official 机器狗 interface (B 题附件 2).

Protocol rules implemented here:

- four endpoints only: /enter /measure /clear /exit, POST, JSON body
- one action at a time, never concurrent
- a new request_id per new action; a retry of the same action reuses the exact
  same body and request_id
- every response is checked for both HTTP status and accepted == true
- remaining_real_duration_s decides the real-world budget, not a fixed 1200 s
"""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.request


class RobotError(RuntimeError):
    pass


class HttpRobot:
    def __init__(
        self,
        robot_id: str,
        base_url: str = "http://127.0.0.1:2026",
        timeout: float = 5.0,
        retries: int = 4,
        log_path: str | None = None,
        enter_wait: float = 0.0,
    ):
        self.robot_id = robot_id
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.retries = retries
        self.counter = 0
        self.virtual_time = 0.0
        self.remaining_real = None
        self.log_path = log_path
        self.enter_wait = enter_wait
        self.log_file = open(log_path, "a", encoding="utf-8") if log_path else None
        self.n_requests = 0

    def _next_id(self, prefix: str) -> str:
        self.counter += 1
        return f"{prefix}-{self.counter}"

    def _record(self, path: str, payload: dict, status, resp) -> None:
        if not self.log_file:
            return
        self.log_file.write(
            json.dumps(
                {
                    "path": path,
                    "request_id": payload.get("request_id"),
                    "payload": payload,
                    "http_status": status,
                    "response": resp,
                    "wall_time": time.time(),
                },
                ensure_ascii=False,
            )
            + "\n"
        )
        self.log_file.flush()

    def _post(self, path: str, payload: dict):
        """POST with retries; the payload and request_id are reused on retry."""
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        last_error = None
        for attempt in range(self.retries):
            req = urllib.request.Request(
                self.base_url + path,
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                    status = resp.status
                    data = resp.read().decode("utf-8")
                self.n_requests += 1
                try:
                    parsed = json.loads(data) if data else {}
                except json.JSONDecodeError:
                    parsed = {"_raw": data}
                self._record(path, payload, status, parsed)
                return status, parsed
            except urllib.error.HTTPError as exc:
                self.n_requests += 1
                data = exc.read().decode("utf-8", "replace")
                try:
                    parsed = json.loads(data) if data else {}
                except json.JSONDecodeError:
                    parsed = {"_raw": data}
                self._record(path, payload, exc.code, parsed)
                if 400 <= exc.code < 500 and exc.code != 429:
                    raise RobotError(f"{path} -> HTTP {exc.code}: {parsed}") from exc
                last_error = RobotError(f"{path} -> HTTP {exc.code}")
            except (urllib.error.URLError, OSError, TimeoutError) as exc:
                last_error = exc
            time.sleep(0.2 * (attempt + 1))
        raise RobotError(f"{path} failed after {self.retries} attempts: {last_error}")

    def _consume(self, status: int, resp: dict) -> dict:
        if status != 200 or not resp.get("accepted"):
            raise RobotError(f"not accepted (HTTP {status}): {resp}")
        vt = resp.get("virtual_time_s")
        if isinstance(vt, (int, float)) and vt:
            self.virtual_time = float(vt)
        if "remaining_real_duration_s" in resp:
            self.remaining_real = resp["remaining_real_duration_s"]
        return resp

    def _command(self, path: str, prefix: str, **fields):
        payload = {
            "arena_id": "default",
            "robot_id": self.robot_id,
            "request_id": self._next_id(prefix),
        }
        payload.update(fields)
        status, resp = self._post(path, payload)
        return self._consume(status, resp)

    def enter(self) -> dict:
        """Enter the arena, waiting for the interface to open if needed.

        The interface is closed during the countdown, so connection attempts can
        fail for a while. The same request_id is reused while waiting, which is
        what the protocol asks for.
        """
        payload = {
            "arena_id": "default",
            "robot_id": self.robot_id,
            "request_id": self._next_id("enter"),
        }
        deadline = time.time() + max(self.enter_wait, 0.0)
        last_error: Exception | None = None
        while True:
            try:
                status, resp = self._post("/enter", payload)
                if status == 200 and not resp.get("accepted"):
                    raise RobotError(f"/enter refused: {resp}")
                return self._consume(status, resp)
            except RobotError as exc:
                if "refused" in str(exc):
                    raise
                last_error = exc
            if time.time() >= deadline:
                raise RobotError(f"could not enter the arena: {last_error}")
            time.sleep(0.5)

    def exit(self) -> dict:
        return self._command("/exit", "exit")

    def measure(self, x: float, y: float, channel: int) -> dict:
        return self._command("/measure", "measure", position={"x": x, "y": y}, channel=channel)

    def clear(self, x: float, y: float, channel: int) -> dict:
        return self._command("/clear", "clear", position={"x": x, "y": y}, channel=channel)

    def close(self) -> None:
        if self.log_file:
            self.log_file.close()
            self.log_file = None
