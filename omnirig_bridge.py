"""OmniRig -> "Yaesu FT-950" bridge for old Ham Radio Deluxe (v2).

HRD talks to one end of a com0com virtual COM pair believing it is an FT-950.
This script listens on the other end and answers HRD's FT-950 CAT commands using
live data from OmniRig (your real FT-991A). Frequency, mode and PTT go through
OmniRig directly. AF gain, RF gain, squelch, mic gain, power and the S-meter are
read from / written to the radio through OmniRig's "custom command" channel.

Setup:   pip install pyserial pywin32
Run:     python omnirig_bridge.py COM13 --baud 38400   (HRD connects to COM12, radio = FT-950)
Log:     omnirig_bridge_log.txt (next to this script) - send it to me if something misbehaves.
"""
import argparse
import re
import time
from collections import Counter, OrderedDict, deque
from datetime import datetime
from pathlib import Path

# OmniRig parameter constants (from the OmniRig type library)
PM_SPLITON, PM_RX, PM_TX = 0x8000, 0x200000, 0x400000
PM_VFOBA, PM_VFOBB, PM_VFOA, PM_VFOB = 0x200, 0x400, 0x800, 0x1000
PM_CW_U, PM_CW_L, PM_SSB_U, PM_SSB_L = 0x800000, 0x1000000, 0x2000000, 0x4000000
PM_DIG_U, PM_DIG_L, PM_AM, PM_FM = 0x8000000, 0x10000000, 0x20000000, 0x40000000

# OmniRig mode -> FT-950 mode character (Yaesu FT-950 CAT manual)
TO_FT950 = {PM_SSB_L: "1", PM_SSB_U: "2", PM_CW_U: "3", PM_FM: "4", PM_AM: "5",
            PM_CW_L: "7", PM_DIG_L: "8", PM_DIG_U: "C"}
FROM_FT950 = {"1": PM_SSB_L, "2": PM_SSB_U, "3": PM_CW_U, "4": PM_FM, "5": PM_AM, "6": PM_DIG_L,
              "7": PM_CW_L, "8": PM_DIG_L, "9": PM_DIG_U, "A": PM_FM, "B": PM_FM, "C": PM_DIG_U, "D": PM_AM}

# Read commands answered with fixed placeholder values
CANNED = {
    "ID": "ID0310", "AI": "AI0", "PS": "PS1", "RS": "RS0", "FR": "FR0", "VS": "VS0", "TS": "TS0",
    "LK": "LK0", "VX": "VX0", "KR": "KR0", "KS": "KS020", "CS": "CS0", "BI": "BI0", "FS": "FS0",
    "MG": "MG050", "PC": "PC050", "PL": "PL050", "PR": "PR0", "VG": "VG050", "VD": "VD0100",
    "SD": "SD0300", "XT": "XT0", "RT": "RT0", "UL": "UL0", "MS": "MS1", "BY": "BY00", "AC": "AC000",
    "SC": "SC0", "MX": "MX0", "KP": "KP04",
    "AG0": "AG0128", "RG0": "RG0255", "SQ0": "SQ0000", "SM0": "SM0000", "NB0": "NB00",
    "NR0": "NR00", "NA0": "NA00", "GT0": "GT04", "PA0": "PA00", "RA0": "RA00", "SH0": "SH000",
    "RF0": "RF03", "IS0": "IS0+0000", "NL0": "NL0000", "CT0": "CT00", "CN0": "CN000",
    "RL0": "RL001", "CO00": "CO000", "CO01": "CO0101",
    "AN0": "AN010", "BC0": "BC00", "ML0": "ML0000", "ML1": "ML1050", "RI0": "RI00", "RI1": "RI10",
    "BP00": "BP00000", "BP01": "BP01150",
}
CANNED.update({f"RM{i}": f"RM{i}000" for i in range(9)})
NO_REPLY_SHORT = {"AB", "SV", "UP", "DN", "QI", "QR", "QS", "RC", "VM", "AM", "MA", "EK"}
ALLOW_POWEROFF = False   # set by --allow-poweroff


def clamp(n, lo, hi):
    return max(lo, min(hi, n))


def _co(m):
    """FT-950 contour (on/off or level 1-30) -> FT-991A contour command."""
    val = int(m.group(2))
    if m.group(1) == "0":
        return "CO00" + ("0001" if val else "0000")
    return "CO01" + f"{clamp(val * 100, 10, 3200):04d}"   # approximate: level -> Hz


# FT-950 set command (regex) -> FT-991A command text sent to the real radio.
RULES = [
    (r"(PA0\d|NR0\d|BC0\d|GT0\d|NA0\d|BI\d|FS\d|LK\d|KR\d|VX\d|CS\d|BD0|BU0|AC\d{3}"
     r"|ML0\d{3}|SH0\d{2}|RL0\d{2}|BP0[01]\d{3})", lambda m: m.group(0)),
    (r"RA0(\d)", lambda m: "RA0" + ("0" if m.group(1) == "0" else "1")),        # FT-991A: one level only
    (r"NB0(\d)", lambda m: "NB0" + ("0" if m.group(1) == "0" else "1")),        # no "wide" NB
    (r"NL0(\d{3})", lambda m: f"NL0{clamp(round(int(m.group(1)) * 10 / 255), 0, 10):03d}"),
    (r"IS0([+-])(\d{4})", lambda m: f"IS0{m.group(1)}{clamp(round(int(m.group(2)) / 20) * 20, 0, 1200):04d}"),
    (r"CO0([01])(\d{2,3})", _co),
    (r"KP(\d{2})", lambda m: f"KP{clamp(int(m.group(1)) * 5, 0, 75):02d}"),
    (r"MA", lambda m: "VM"),   # HRD memory/VFO button -> FT-991A [V/M] key
    (r"CH[01]", lambda m: m.group(0)),   # memory channel up / down
    (r"MS[0-5]", lambda m: m.group(0)),   # meter select (COMP/ALC/PO/SWR/ID/VDD)
    (r"ML1(\d{3})", lambda m: f"ML1{clamp(round(int(m.group(1)) * 100 / 255), 0, 100):03d}"),
]


class Live:
    """Reads OmniRig state with a short cache so HRD's polling stays fast."""

    def __init__(self, rig, ttl=0.12):
        self.rig, self.ttl, self.t, self.v, self.ov = rig, ttl, 0.0, {}, {}

    def _refresh(self):
        now = time.time()
        if now - self.t > self.ttl or not self.v:
            r = self.rig
            self.v = {"fa": int(r.FreqA or 0), "fb": int(r.FreqB or 0), "mode": r.Mode,
                      "tx": r.Tx, "split": r.Split, "vfo": getattr(r, "Vfo", 0)}
            self.t = now
        for k, (val, exp) in list(self.ov.items()):
            if now < exp:
                self.v[k] = val
            else:
                del self.ov[k]

    def __getattr__(self, k):
        if k in ("fa", "fb", "mode", "tx", "split", "vfo"):
            self._refresh()
            return self.v[k]
        raise AttributeError(k)

    def set(self, key, val, hold=0.8):
        """Remember a value we just sent, so HRD doesn't see it bounce back."""
        self.ov[key] = (val, time.time() + hold)
        self.t = 0.0


class Radio:
    """Real-radio values (gains, S-meter...) via OmniRig custom commands, cached."""
    FAST, FAST_TX = ["SM0", "TX"], ["TX", "RM5", "RM6", "RM4", "RM3"]
    SLOW = ["AG0", "RG0", "SQ0", "MG", "PC", "RA0", "PA0", "NB0", "NR0", "BC0", "NA0", "RL0",
            "BI", "FS", "LK", "MX", "PR0", "PR1", "CO00", "CO01", "BP00", "BP01", "IS0", "SH0", "AC", "ML0", "MS", "KP", "VX", "GT0"]
    K2 = ("MG", "PC", "BI", "FS", "LK", "MX", "AC", "MS", "TX", "KP", "VX")
    K3 = ("SM0", "AG0", "RG0", "SQ0", "RA0", "PA0", "NB0", "NR0", "BC0", "NA0", "RL0", "PR0", "PR1", "SH0", "ML0", "GT0")
    K4 = ("CO00", "CO01", "BP00", "BP01")

    def __init__(self, rig, log):
        self.rig, self.log = rig, log
        self.cache, self.outbox = {}, OrderedDict()
        self.pending, self.enabled = None, True
        self.last_send = self.last_slow = 0.0
        self.fi = self.si = self.logged = 0
        self.recent = deque(maxlen=6)
        self.pr_order = []   # which of processor (PR0) / mic EQ (PR1) HRD turned on last
        self.transmitting = False
        self.rej_logged = set()

    def _send(self, text, reply_end):
        try:
            try:
                self.rig.SendCustomCommand(text, 0, reply_end)
            except Exception:
                self.rig.SendCustomCommand(text.encode("ascii"), 0, reply_end.encode("ascii"))
            self.last_send = time.time()
            return True
        except Exception as exc:
            self.enabled = False
            self.log(f"CUSTOM COMMANDS FAILED ({exc}) - gains and S-meter will use placeholders")
            return False

    def remember(self, text):
        """Store a radio value (from a poll answer or something we just sent)."""
        t = text.strip().rstrip(";").upper()
        try:
            if t[:4] in self.K4:
                self.cache[t[:4]] = int(t[4:])
            elif t[:3] == "IS0":
                self.cache["IS0"] = int(t[3:])          # signed, e.g. +0100
            elif t[:3] in self.K3:
                self.cache[t[:3]] = int(t[3:])
            elif t[:2] == "RM":
                self.cache["RM" + t[2]] = int(t[3:])
            elif t[:2] in self.K2:
                self.cache[t[:2]] = int(t[2:])
        except ValueError:
            pass

    def on_reply(self, text):
        up = text.strip().upper()
        if up.startswith("?"):
            if self.pending:
                key = self.pending[0]
                if key not in self.rej_logged:
                    self.rej_logged.add(key)
                    self.log(f"RADIO REJECTED reading {key} (normal if the radio's current mode doesn't support it)")
            else:
                self.log(f"RADIO REJECTED A COMMAND - recently sent: {list(self.recent)}")
        before = dict(self.cache)
        self.remember(text)
        for k, v in self.cache.items():
            if before.get(k) != v and not k.startswith(("SM", "RM")):
                self.log(f"radio value: {k} {before.get(k)} -> {v}")
        self.pending = None

    def queue_set(self, key, text, value=None):
        self.outbox[key] = text
        if value is not None:
            self.cache[key] = value

    def tick(self, transmitting):
        self.transmitting = transmitting
        if not self.enabled:
            return
        now = time.time()
        if self.pending and now - self.pending[1] > 0.6:
            self.pending = None
        if self.pending or now - self.last_send < 0.06:
            return
        if self.outbox:
            _, text = self.outbox.popitem(last=False)
            self.recent.append(text)
            self._send(text + ";", "")
            return
        if now - self.last_slow > 0.4:
            self.last_slow = now
            key = self.SLOW[self.si % len(self.SLOW)]
            self.si += 1
        else:
            fast = self.FAST_TX if transmitting else self.FAST
            key = fast[self.fi % len(fast)]
            self.fi += 1
        if self._send(key + ";", ";"):
            self.pending = (key, now)


def pr_value(radio):
    """HRD (FT-950) has ONE processor/EQ selector: 0 off, 1 processor, 2 mic EQ.
    The FT-991A can have both on, so report the one HRD turned on most recently."""
    c = radio.cache
    for key in reversed(radio.pr_order):
        if c.get(key) == 1:
            return 1 if key == "PR0" else 2
    if c.get("PR1") == 1:
        return 2
    return 1 if c.get("PR0") == 1 else 0


def radio_answer(cmd, radio):
    """FT-950-style answer built from real FT-991A values, or None if not known yet."""
    c = radio.cache
    if cmd in ("AG0", "RG0", "PC") and cmd in c:
        return f"{cmd}{c[cmd]:03d};"
    if cmd == "SM0" and "SM0" in c:
        v = c["SM0"]
        if radio.transmitting:      # HRD only ever reads SM0, so carry the selected TX meter (default: power) in it
            sel = {0: "RM3", 1: "RM4", 2: "RM5", 3: "RM6"}.get(c.get("MS"), "RM5")
            v = c.get(sel, 0)
        return f"SM0{v:03d};"
    if cmd == "SQ0" and cmd in c:
        return f"SQ0{round(c[cmd] * 2.55):03d};"   # FT-991A 0-100 -> FT-950 0-255
    if cmd == "MG" and cmd in c:
        return f"MG{round(c[cmd] * 2.55):03d};"
    if cmd == "RM1" and "SM0" in c:
        return f"RM1{c['SM0']:03d};"
    if cmd in ("RM3", "RM4", "RM5", "RM6", "RM7", "RM8"):          # COMP, ALC, PO, SWR, ID, VDD
        if cmd in c or not radio.transmitting:
            return f"{cmd}{(c.get(cmd, 0) if radio.transmitting else 0):03d};"
    if cmd in ("RM0", "RM2"):                                      # "the meter selected on the panel"
        sel = {0: "RM3", 1: "RM4", 2: "RM5", 3: "RM6", 4: "RM7", 5: "RM8"}.get(c.get("MS")) if cmd == "RM2" else "RM5"
        if radio.transmitting and sel in c:
            return f"{cmd}{c[sel]:03d};"
        if "SM0" in c:
            return f"{cmd}{c['SM0']:03d};"
    if cmd == "MS" and cmd in c:
        return f"MS{c[cmd]};"
    if cmd == "GT0" and cmd in c:                                   # AGC: FT-991A auto modes 4-6 -> FT-950 "auto"
        return f"GT0{min(c[cmd], 4)};"
    if cmd == "KP" and cmd in c:
        return f"KP{clamp(round(c[cmd] / 5), 0, 15):02d};"      # FT-991A 0-75 (10 Hz) -> FT-950 0-15 (50 Hz)
    if cmd in ("PA0", "NB0", "NR0", "BC0", "NA0", "BI", "FS", "LK", "MX", "VX") and cmd in c:
        return f"{cmd}{c[cmd]};"
    if cmd == "RA0" and cmd in c:
        return f"RA0{1 if c[cmd] else 0};"
    if cmd == "RL0" and cmd in c:
        return f"RL0{clamp(c[cmd], 1, 15):02d};"
    if cmd == "PR" and ("PR0" in c or "PR1" in c):
        return f"PR{pr_value(radio)};"
    if cmd == "AC" and cmd in c:
        return f"AC{c[cmd]:03d};"
    if cmd == "ML0" and cmd in c:
        return f"ML0{c[cmd]:03d};"
    if cmd == "IS0" and cmd in c:
        return f"IS0{'+' if c[cmd] >= 0 else '-'}{abs(c[cmd]):04d};"
    if cmd == "SH0" and cmd in c:
        return f"SH0{c[cmd]:02d};"
    if cmd == "CO00" and cmd in c:
        return f"CO00{'01' if c[cmd] else '00'};"
    if cmd == "CO01" and cmd in c:
        return f"CO01{clamp(round(c[cmd] / 100), 1, 30):02d};"
    if cmd in ("BP00", "BP01") and cmd in c:
        return f"{cmd}{c[cmd]:03d};"
    return None

def build_info(prefix, hz, mode_char):
    """27-character IF / OI answer: channel, freq, clarifier, mode, etc."""
    return f"{prefix}000{hz:08d}+000000{mode_char}00000;"


def handle(cmd, rig, live, radio=None):
    """Return (reply_or_None, note) for one command (without the ';')."""
    cmd = cmd.strip().upper()
    c2 = cmd[:2]

    if cmd == "FA":
        return f"FA{live.fa:08d};", ""
    if cmd == "FB":
        return f"FB{live.fb:08d};", ""
    if cmd == "IF":
        return build_info("IF", live.fa, TO_FT950.get(live.mode, "2")), ""
    if cmd == "OI":
        return build_info("OI", live.fb, TO_FT950.get(live.mode, "2")), ""
    if cmd == "MD0":
        return f"MD0{TO_FT950.get(live.mode, '2')};", ""
    if cmd == "TX":
        t = radio.cache.get("TX") if radio else None        # real TX state, even if keyed from the radio's PTT
        if t in (1, 2):
            return f"TX{t};", ""
        return ("TX2;" if live.tx == PM_TX or (radio and radio.cache.get("MX") == 1) else "TX0;"), ""
    if cmd == "FT":
        return ("FT1;" if live.split == PM_SPLITON else "FT0;"), ""
    if cmd == "VS":
        return ("VS1;" if live.vfo in (PM_VFOB, PM_VFOBA, PM_VFOBB) else "VS0;"), ""
    if radio and radio.enabled:
        ans = radio_answer(cmd, radio)
        if ans:
            return ans, ""
    if cmd in CANNED:
        return CANNED[cmd] + ";", "(placeholder answer)"

    # Set commands
    m = re.fullmatch(r"(AG0|RG0|SQ0|MG|PC)(\d{3})", cmd)
    if m:
        if not (radio and radio.enabled):
            return None, "(set ignored)"
        key, n = m.group(1), int(m.group(2))
        if key in ("AG0", "RG0"):
            v = clamp(n, 0, 255)
        elif key in ("SQ0", "MG"):
            v = clamp(round(n * 100 / 255), 0, 100)   # FT-950 0-255 -> FT-991A 0-100
        else:
            v = clamp(n, 5, 100)
        radio.queue_set(key, f"{key}{v:03d}", v)
        return None, f"SET {key} -> {v}"
    if c2 in ("FA", "FB") and len(cmd) > 2 and cmd[2:].isdigit():
        hz = int(cmd[2:])
        if c2 == "FA":
            rig.FreqA = hz
            live.set("fa", hz)
        else:
            rig.FreqB = hz
            live.set("fb", hz)
        return None, f"SET {c2} -> {hz} Hz"
    if c2 == "MD" and len(cmd) == 4 and cmd[3] in FROM_FT950:
        rig.Mode = FROM_FT950[cmd[3]]
        live.set("mode", rig.Mode)
        return None, f"SET mode -> {cmd[3]}"
    if c2 == "TX" and len(cmd) == 3:
        on = cmd[2] in "12"
        rig.Tx = PM_TX if on else PM_RX
        live.set("tx", PM_TX if on else PM_RX, hold=0.3)
        return None, f"SET PTT {'ON' if on else 'OFF'}"
    if cmd == "PS0":
        if ALLOW_POWEROFF and radio and radio.enabled:
            radio.queue_set("PS", "PS0")
            return None, "SEND PS0 (RADIO POWER OFF)"
        return None, "(power off ignored - start the bridge with --allow-poweroff to allow it)"
    if cmd in ("MX0", "MX1"):
        if radio and radio.enabled:
            radio.queue_set("MX", cmd, int(cmd[2]))
            return None, f"SEND {cmd} (MOX)"
        on = cmd == "MX1"
        rig.Tx = PM_TX if on else PM_RX
        live.set("tx", PM_TX if on else PM_RX, hold=0.3)
        return None, f"SET PTT {'ON' if on else 'OFF'} (MOX)"
    m = re.fullmatch(r"PR([012])", cmd)
    if m and radio and radio.enabled:
        n = int(m.group(1))
        if n == 0:                       # FT-991A: P2 0 = off, 1 = on; both can be on together
            cur = pr_value(radio)        # turn off the one HRD is showing as active
            if cur == 1:
                radio.queue_set("PR0", "PR00", 0)
            elif cur == 2:
                radio.queue_set("PR1", "PR10", 0)
            else:
                radio.queue_set("PR0", "PR00", 0)
                radio.queue_set("PR1", "PR10", 0)
        else:
            key = "PR0" if n == 1 else "PR1"
            radio.queue_set(key, f"{key}1", 1)
            radio.pr_order = [k for k in radio.pr_order if k != key] + [key]
        return None, f"SEND {cmd} (processor / mic EQ)"
    if cmd in ("VS0", "VS1"):
        want = PM_VFOB if cmd == "VS1" else PM_VFOA
        try:
            rig.Vfo = want
            live.set("vfo", want)
        except Exception as exc:
            return None, f"VFO select not accepted by OmniRig ({exc})"
        return None, f"SET VFO {'B' if cmd == 'VS1' else 'A'}"
    for pattern, fn in RULES:
        m = re.fullmatch(pattern, cmd)
        if m:
            if not (radio and radio.enabled):
                return None, "(set ignored - no radio link)"
            text = fn(m)
            if text[:2] in ("CH", "BD", "BU", "VM"):      # one-shot buttons: never merge repeated presses
                key = f"{text}#{time.time()}"
            else:
                key = text[:4] if text[:2] in ("CO", "BP") else text[:2]
            radio.queue_set(key, text)
            radio.remember(text)
            return None, f"SEND {cmd} -> {text}"
    if c2 in NO_REPLY_SHORT or len(cmd) > 2:
        return None, "(not handled - ignored)"
    return "?;", "UNKNOWN - replied ?;"


def to_text(v):
    if isinstance(v, str):
        return v
    try:
        return bytes(v).decode("ascii", "ignore")
    except Exception:
        return str(v)


def main():
    import pythoncom
    import serial
    import win32com.client

    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="virtual COM port the bridge listens on, e.g. COM13")
    ap.add_argument("--rig", type=int, default=1, help="OmniRig rig number (1 or 2)")
    ap.add_argument("--baud", type=int, default=4800)
    ap.add_argument("--allow-poweroff", action="store_true",
                    help="let HRD's power-off command switch the real radio off")
    args = ap.parse_args()
    globals()["ALLOW_POWEROFF"] = args.allow_poweroff

    log_file = open(Path(__file__).with_name("omnirig_bridge_log.txt"), "a", encoding="utf-8")

    def note(text, console=True):
        line = f"{datetime.now():%H:%M:%S.%f}"[:-3] + "  " + text
        log_file.write(line + "\n")
        log_file.flush()
        if console:
            print(line)

    holder = {}

    class Events:
        def OnCustomReply(self, RigNumber, Command, Reply):
            if holder.get("radio") and RigNumber == args.rig:
                holder["radio"].on_reply(to_text(Reply))

    omni = win32com.client.DispatchWithEvents("OmniRig.OmniRigX", Events)
    rig = omni.Rig1 if args.rig == 1 else omni.Rig2
    print(f"OmniRig rig {args.rig}: {rig.RigType} - {rig.StatusStr}")
    if "on-line" not in str(rig.StatusStr).lower():
        print("WARNING: OmniRig does not say On-line. Check its settings first.")

    live, radio = Live(rig), Radio(rig, note)
    holder["radio"] = radio
    seen = set()
    ser = serial.Serial(args.port, args.baud, timeout=0.01)
    print(f"Listening on {args.port}. Point HRD (FT-950) at the other port. Ctrl+C to stop.")
    buf, last_cmd = "", ""
    prev_tx, tx_counts = False, Counter()
    while True:
        pythoncom.PumpWaitingMessages()
        try:
            radio.tick(live.tx == PM_TX or radio.cache.get("MX") == 1 or radio.cache.get("TX") in (1, 2))
        except Exception as exc:
            note(f"ERROR in radio poll: {exc}")
        tx_now = bool(radio.transmitting)
        if tx_now and not prev_tx:
            tx_counts.clear()
            note("TX STARTED")
        if prev_tx and not tx_now:
            note("TX ENDED - HRD asked for: " + (", ".join(f"{k} x{v}" for k, v in tx_counts.most_common(25)) or "nothing"))
        prev_tx = tx_now
        data = ser.read(ser.in_waiting or 1)
        if data:
            buf += data.decode("ascii", "ignore")
        while ";" in buf:
            cmd, buf = buf.split(";", 1)
            cmd = cmd.strip()
            if not cmd:
                continue
            try:
                reply, info = handle(cmd, rig, live, radio)
            except Exception as exc:  # keep running if OmniRig hiccups
                reply, info = "?;", f"ERROR {exc}"
            if reply:
                ser.write(reply.encode("ascii"))
            up = cmd.upper()
            if tx_now:
                tx_counts[up] += 1
            first = up not in seen
            seen.add(up)
            dup = up == last_cmd
            last_cmd = up
            show = first or any(w in info for w in ("UNKNOWN", "SET", "SEND", "ERROR", "not accepted"))
            if not dup:
                note(f"HRD> {cmd};   BRIDGE> {reply or '-'}   {info}", show)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
