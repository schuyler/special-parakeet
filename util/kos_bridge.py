#!/usr/bin/env python3
"""
kOS Telnet Bridge

KSP's kOS mod serves a telnet terminal on port 5410. This exposes that terminal
as a pair of files, so a shell session can drive a running game:

    python3 kos_bridge.py &
    echo 'print ship:altitude.' > /tmp/kos_cmd
    cat /tmp/kos_out

The server speaks real telnet: it will not accept a single keystroke until the
client has answered its option negotiation and declared a terminal type. Until
then the banner reports "type = INITIAL_UNSET" and input is discarded. So the
negotiation below is not politeness, it is the price of admission.

The terminal repaints itself with cursor-control sequences rather than emitting
a clean transcript, so /tmp/kos_out is a lossy view -- fine for watching, poor
for parsing. To get data *out* of kOS reliably, have the script LOG to a file on
the archive volume (which is the directory this bridge is run from) and read that
file directly.

Usage:
    python3 kos_bridge.py [--port 5410] [--attach 1] [--log]
"""

import socket
import threading
import os
import sys
import argparse
import re
import time
from datetime import datetime

CMD_PIPE = "/tmp/kos_cmd"
OUT_FILE = "/tmp/kos_out"

# Telnet protocol bytes (RFC 854 and friends).
IAC = 255       # "interpret as command" -- introduces every escape below
DONT, DO, WONT, WILL = 254, 253, 252, 251
SB, SE = 250, 240   # start and end of a subnegotiation payload

OPT_ECHO = 1
OPT_SGA = 3         # suppress go-ahead: character-at-a-time, not line-at-a-time
OPT_TTYPE = 24      # terminal type
OPT_NAWS = 31       # negotiate about window size

TTYPE_IS, TTYPE_SEND = 0, 1

TERM_NAME = b"XTERM"

# Cursor moves, colour changes, and the like. Stripped from the output file.
ANSI_RE = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]|\x1b[()][A-Z0-9]|\x1b[=>]')
# kOS sends its own control codes in Unicode's private use area, U+E000-U+F8FF.
KOS_PRIVATE_RE = re.compile('[\ue000-\uf8ff]')


class KOSBridge:
    def __init__(self, host='localhost', port=5410, log_enabled=False, attach=None):
        self.host = host
        self.port = port
        self.log_enabled = log_enabled
        self.attach = attach
        self.sock = None
        self.running = False
        self.out_file = None
        self.lock = threading.Lock()
        self.pending = b''      # bytes received but not yet parsed
        self.attached = False
        # What each side has settled on, so a repeat request draws no reply.
        # remote_opts is the server's side (WILL/WONT), local_opts is ours
        # (DO/DONT). A missing key means "never discussed", which is distinct
        # from False and always earns an answer.
        self.remote_opts = {}
        self.local_opts = {}

    def log(self, msg):
        if self.log_enabled:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] {msg}", file=sys.stderr)

    def connect(self):
        while self.running:
            try:
                self.log(f"Connecting to {self.host}:{self.port}...")
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.host, self.port))
                self.sock.settimeout(0.5)
                self.pending = b''
                self.attached = False
                self.remote_opts = {}
                self.local_opts = {}
                self.log("Connected")
                self.write_output(f"--- Connected to kOS at {self.host}:{self.port} ---\n")
                return True
            except (socket.error, ConnectionRefusedError) as e:
                self.log(f"Connection failed: {e}. Retrying in 3s...")
                time.sleep(3)
        return False

    def write_output(self, text):
        with self.lock:
            if self.out_file:
                self.out_file.write(text)
                self.out_file.flush()

    def send_raw(self, data):
        if not self.sock:
            return
        try:
            self.sock.sendall(data)
        except socket.error as e:
            self.log(f"Send error: {e}")

    def send_option(self, verb, opt):
        self.log(f"SEND: IAC {verb} {opt}")
        self.send_raw(bytes([IAC, verb, opt]))

    def send_subneg(self, payload):
        self.log(f"SEND: IAC SB {list(payload)} IAC SE")
        self.send_raw(bytes([IAC, SB]) + payload + bytes([IAC, SE]))

    def handle_option(self, verb, opt):
        """Answer one WILL/WONT/DO/DONT from the server.

        Two rules, and both matter. Symmetry: a DO asks us to enable something
        on our side and must be met with WILL or WONT; a WILL announces the
        server's own side and must be met with DO or DONT. An unanswered option
        leaves the server waiting forever.

        Silence on no-change: an option already in the state being asked for
        draws no reply at all. Answering anyway is what turns two polite
        endpoints into an infinite exchange -- the server re-announces WILL
        ECHO, we re-approve it, and neither side ever runs out of things to say.
        """
        if verb in (WILL, WONT):
            # The server echoing our keystrokes and suppressing go-ahead are
            # both what an ordinary terminal expects; anything else we decline.
            wanted = verb == WILL and opt in (OPT_ECHO, OPT_SGA)
            if self.remote_opts.get(opt) == wanted:
                return
            self.remote_opts[opt] = wanted
            self.send_option(DO if wanted else DONT, opt)
        elif verb in (DO, DONT):
            # We answer DO TTYPE (the terminal type is the price of admission,
            # see the module docstring) but decline NAWS. Reporting a window
            # size makes kOS resize its in-game terminal GUI to match, which
            # disrupts the layout on screen; leaving NAWS unnegotiated lets kOS
            # keep whatever size it already has.
            wanted = verb == DO and opt == OPT_TTYPE
            if self.local_opts.get(opt) == wanted:
                return
            self.local_opts[opt] = wanted
            self.send_option(WILL if wanted else WONT, opt)

    def handle_subneg(self, payload):
        if len(payload) >= 2 and payload[0] == OPT_TTYPE and payload[1] == TTYPE_SEND:
            # This is the answer the server blocks on. Without it the terminal
            # stays INITIAL_UNSET and every command sent is discarded.
            self.send_subneg(bytes([OPT_TTYPE, TTYPE_IS]) + TERM_NAME)
            if self.attach and not self.attached:
                # The CPU menu only accepts a selection once the terminal is real.
                threading.Timer(1.0, self.do_attach).start()

    def do_attach(self):
        if self.attached:
            return
        self.attached = True
        self.log(f"Attaching to CPU {self.attach}")
        self.send_raw(f"{self.attach}\n".encode())

    def feed(self, data):
        """Split the stream into telnet commands, which are answered, and text,
        which is returned. A sequence straddling two recv() calls stays in
        self.pending and is reparsed once the rest of it arrives.
        """
        self.pending += data
        out = bytearray()
        i = 0
        buf = self.pending
        while i < len(buf):
            b = buf[i]
            if b != IAC:
                out.append(b)
                i += 1
                continue
            if i + 1 >= len(buf):
                break                       # incomplete: wait for more
            cmd = buf[i + 1]
            if cmd == IAC:                  # escaped literal 0xFF
                out.append(IAC)
                i += 2
            elif cmd in (DO, DONT, WILL, WONT):
                if i + 2 >= len(buf):
                    break
                opt = buf[i + 2]
                self.log(f"RECV: IAC {cmd} {opt}")
                self.handle_option(cmd, opt)
                i += 3
            elif cmd == SB:
                end = buf.find(bytes([IAC, SE]), i)
                if end < 0:
                    break
                payload = buf[i + 2:end]
                self.log(f"RECV: IAC SB {list(payload)} IAC SE")
                self.handle_subneg(payload)
                i = end + 2
            else:
                i += 2                      # other two-byte command; ignored
        self.pending = buf[i:]
        return bytes(out)

    def reader_thread(self):
        while self.running:
            if not self.sock:
                time.sleep(0.1)
                continue
            try:
                data = self.sock.recv(4096)
                if not data:
                    self.log("Connection closed by server")
                    self.write_output("--- Connection closed ---\n")
                    self.sock = None
                    self.connect()
                    continue
                text = self.feed(data).decode('utf-8', errors='replace')
                text = ANSI_RE.sub('', text)
                text = KOS_PRIVATE_RE.sub('', text)
                if text:
                    self.write_output(text)
            except socket.timeout:
                continue
            except socket.error as e:
                self.log(f"Socket error: {e}")
                self.sock = None
                self.connect()

    def cmd_thread(self):
        while self.running:
            try:
                with open(CMD_PIPE, 'r') as fifo:
                    for line in fifo:
                        cmd = line.rstrip('\n')
                        if not cmd:
                            continue
                        self.log(f"SEND: {cmd}")
                        self.send_raw((cmd + '\n').encode('utf-8'))
            except Exception as e:
                self.log(f"Command pipe error: {e}")
                time.sleep(0.1)

    def setup_pipes(self):
        if os.path.exists(CMD_PIPE):
            os.remove(CMD_PIPE)
        os.mkfifo(CMD_PIPE)
        self.out_file = open(OUT_FILE, 'w')
        self.write_output(f"--- kOS Bridge started at {datetime.now()} ---\n")

    def cleanup(self):
        self.running = False
        if self.sock:
            self.sock.close()
        if self.out_file:
            self.out_file.close()
        if os.path.exists(CMD_PIPE):
            os.remove(CMD_PIPE)

    def run(self):
        self.running = True
        self.setup_pipes()
        print("kOS Bridge running", file=sys.stderr)
        print(f"  Commands: echo 'print 1+1.' > {CMD_PIPE}", file=sys.stderr)
        print(f"  Output:   tail -f {OUT_FILE}", file=sys.stderr)
        reader = threading.Thread(target=self.reader_thread, daemon=True)
        reader.start()
        self.connect()
        try:
            self.cmd_thread()
        except KeyboardInterrupt:
            print("\nShutting down...", file=sys.stderr)
        finally:
            self.cleanup()


def main():
    parser = argparse.ArgumentParser(description='kOS Telnet Bridge')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=5410)
    parser.add_argument('--attach', help='CPU menu selection to attach to on connect')
    parser.add_argument('--log', action='store_true', help='Enable debug logging')
    args = parser.parse_args()
    KOSBridge(host=args.host, port=args.port,
              log_enabled=args.log, attach=args.attach).run()


if __name__ == '__main__':
    main()
