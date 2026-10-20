#!/usr/bin/env python3

import ipaddress
import json
import logging
import os
import signal
import socket
import ssl
import subprocess
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


HOST = "0.0.0.0"
PORT = 8765
HTML_FILE = Path(__file__).with_name("index.html")
CERTIFICATE_DIR = Path.home() / ".local" / "share" / "rc-control"
CERTIFICATE_FILE = CERTIFICATE_DIR / "server.crt"
PRIVATE_KEY_FILE = CERTIFICATE_DIR / "server.key"
COMMANDS = frozenset({"rc", "rc_bag"})
jobs: dict[str, subprocess.Popen[bytes]] = {}
jobs_lock = threading.Lock()


class ControlHandler(BaseHTTPRequestHandler):
    server_version = "RCControl/1.0"

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            self._serve_page()
            return
        if self.path == "/api/status":
            with jobs_lock:
                status = {
                    name: {
                        "running": process.poll() is None,
                        "exit_code": process.poll(),
                    }
                    for name, process in jobs.items()
                }
            self._send_json(200, {"jobs": status})
            return
        self.send_error(404)

    def do_POST(self) -> None:
        if not self._same_origin():
            self.send_error(403, "Requests must come from this page's HTTPS origin.")
            return

        parts = self.path.strip("/").split("/")
        if (
            len(parts) != 3
            or parts[0] != "api"
            or parts[1] not in COMMANDS
            or parts[2] not in ("start", "stop")
        ):
            self.send_error(404)
            return

        alias_name, operation = parts[1], parts[2]
        if operation == "start":
            self._start_job(alias_name)
        else:
            self._stop_job(alias_name)

    def _serve_page(self) -> None:
        try:
            content = HTML_FILE.read_bytes()
        except OSError:
            logging.exception("Could not read the control page")
            self.send_error(500, "Could not read the control page.")
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(content)

    def _same_origin(self) -> bool:
        origin = self.headers.get("Origin")
        host = self.headers.get("Host")
        return bool(origin and host and origin == f"https://{host}")

    def _start_job(self, alias_name: str) -> None:
        with jobs_lock:
            process = jobs.get(alias_name)
            if process is not None and process.poll() is None:
                self._send_json(409, {"error": f"{alias_name} is already running."})
                return

            command = (
                f'type -t {alias_name} | grep -qx alias || '
                f'{{ echo "Alias {alias_name} is not defined in ~/.bashrc" >&2; exit 127; }}; '
                f"{alias_name}"
            )
            try:
                process = subprocess.Popen(
                    ["bash", "-ic", command],
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except OSError as error:
                logging.exception("Could not start alias %s", alias_name)
                self._send_json(500, {"error": f"Could not start {alias_name}: {error}"})
                return
            jobs[alias_name] = process

        time.sleep(0.15)
        exit_code = process.poll()
        if exit_code is not None:
            self._send_json(
                500,
                {"error": f"{alias_name} exited immediately with code {exit_code}."},
            )
            return
        self._send_json(200, {"message": f"Started {alias_name}.", "running": True})

    def _stop_job(self, alias_name: str) -> None:
        with jobs_lock:
            process = jobs.get(alias_name)
            if process is None or process.poll() is not None:
                self._send_json(409, {"error": f"{alias_name} is not running."})
                return

        # if alias_name == "rc":
        #     try:
        #         result = subprocess.run(
        #             [
        #                 "bash",
        #                 "-ic",
        #                 "ros2 topic pub -t 2 /json_msg std_msgs/msg/String 'data: \"{\"speaker\": {\"tts\": \"Stopping RoboColumbus\"}}\"'",
        #             ],
        #             stdin=subprocess.DEVNULL,
        #             capture_output=True,
        #             text=True,
        #             timeout=10,
        #             check=False,
        #         )
        #     except (OSError, subprocess.TimeoutExpired) as error:
        #         logging.exception("Could not publish the RC shutdown announcement")
        #         self._send_json(
        #             500,
        #             {"error": f"Could not announce RC shutdown; rc is still running: {error}"},
        #         )
        #         return
        #     if result.returncode != 0:
        #         details = result.stderr.strip() or result.stdout.strip()
        #         logging.error("RC shutdown announcement failed: %s", details)
        #         self._send_json(
        #             500,
        #             {
        #                 "error": "Could not announce RC shutdown; rc is still running."
        #                 + (f" {details}" if details else "")
        #             },
        #         )
        #         return
        #     time.sleep(1)

        with jobs_lock:
            if process.poll() is not None:
                self._send_json(409, {"error": f"{alias_name} is no longer running."})
                return
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                self._send_json(409, {"error": f"{alias_name} is no longer running."})
                return

        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()

        self._send_json(200, {"message": f"Stopped {alias_name}.", "running": False})

    def _send_json(self, status: int, payload: dict[str, object]) -> None:
        content = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format_string: str, *args: object) -> None:
        logging.info("%s - %s", self.address_string(), format_string % args)


def stop_jobs() -> None:
    with jobs_lock:
        active = [
            (name, process)
            for name, process in jobs.items()
            if process.poll() is None
        ]
    for name, process in active:
        logging.info("Stopping %s during server shutdown", name)
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            continue
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()


def exit_on_signal(signum: int, frame: object) -> None:
    raise SystemExit(0)


def ensure_certificate() -> None:
    if CERTIFICATE_FILE.exists() and PRIVATE_KEY_FILE.exists():
        return
    if CERTIFICATE_FILE.exists() or PRIVATE_KEY_FILE.exists():
        raise RuntimeError(
            f"Only one TLS file exists in {CERTIFICATE_DIR}; remove the incomplete "
            "certificate or restore its matching key."
        )

    hostname = socket.gethostname()
    addresses = {"127.0.0.1"}
    try:
        for result in socket.getaddrinfo(hostname, None, family=socket.AF_INET):
            addresses.add(str(ipaddress.ip_address(result[4][0])))
    except socket.gaierror:
        logging.warning("Could not resolve hostname %s for the TLS certificate", hostname)

    names = {hostname, f"{hostname}.local"}
    subject_alt_names = [f"DNS:{name}" for name in sorted(names)]
    subject_alt_names.extend(f"IP:{address}" for address in sorted(addresses))
    CERTIFICATE_DIR.mkdir(mode=0o700, parents=True, exist_ok=True)
    try:
        with tempfile.TemporaryDirectory(dir=CERTIFICATE_DIR) as temporary_dir:
            certificate = Path(temporary_dir) / "server.crt"
            private_key = Path(temporary_dir) / "server.key"
            result = subprocess.run(
                [
                    "openssl",
                    "req",
                    "-x509",
                    "-newkey",
                    "rsa:2048",
                    "-sha256",
                    "-nodes",
                    "-days",
                    "365",
                    "-keyout",
                    str(private_key),
                    "-out",
                    str(certificate),
                    "-subj",
                    f"/CN={hostname}",
                    "-addext",
                    f"subjectAltName={','.join(subject_alt_names)}",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode != 0:
                raise RuntimeError(f"Could not generate HTTPS certificate: {result.stderr.strip()}")
            private_key.chmod(0o600)
            private_key.replace(PRIVATE_KEY_FILE)
            certificate.replace(CERTIFICATE_FILE)
    except FileNotFoundError as error:
        raise RuntimeError("OpenSSL is required to generate the HTTPS certificate.") from error
    logging.info("Generated a self-signed HTTPS certificate in %s", CERTIFICATE_DIR)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    signal.signal(signal.SIGTERM, exit_on_signal)
    try:
        ensure_certificate()
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(CERTIFICATE_FILE, PRIVATE_KEY_FILE)
    except (OSError, RuntimeError) as error:
        raise SystemExit(str(error)) from error

    server = ThreadingHTTPServer((HOST, PORT), ControlHandler)
    server.daemon_threads = True
    server.socket = context.wrap_socket(server.socket, server_side=True)
    logging.info("RC control page listening on https://%s:%d", socket.gethostname(), PORT)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        stop_jobs()


if __name__ == "__main__":
    main()
