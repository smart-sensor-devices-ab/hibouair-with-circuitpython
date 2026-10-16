import time
import json
import binascii
import board
import usb_host
import usb.core
import microcontroller
import asyncio
import pwmio

# --- WHAT THE DEMO LEARNS ---------------------------------------------------
# The latest PM10 reading from a HibouAir PM sensor, in ug/m3. None until one
# has been heard. pm10_sensor is the sensor's id as the HibouAir app shows it
# ("22026A"), and pm10_time is time.monotonic() when it arrived, so a reading
# can be told apart from a stale one.
pm10 = None
pm10_sensor = None
pm10_time = None

# Every HibouAir sensor heard, by id, with everything its last beacon said.
sensors = {}

# Which PM sensor pm10 follows. None takes whichever PM sensor was heard last;
# set it to an id, e.g. "22026A", to follow one sensor only.
PM_SENSOR_ID = None

# --- THE SERVO DIAL -------------------------------------------------------------
# PM10 shown by a hobby servo on servo port GP0 (the MOTION 2350 Pro's servo
# ports are GP0 to GP7): 0 ug/m3 at 0 degrees, PM10_FULL_SCALE and above at 180.
# With 100 at full scale, the HibouAir app's limits for PM10 -- 20, 50 and 100
# ug/m3 between good, fair, poor and bad -- fall at 36, 90 and 180 degrees,
# which is where to draw the bands on a paper scale behind the arm.
SERVO_PIN = board.GP0
PM10_FULL_SCALE = 100.0
SERVO_MIN_US = 500         # pulse width at 0 degrees
SERVO_MAX_US = 2500        # and at 180; narrow these if the servo strains at the ends
SERVO_STEP_DEG = 2         # how far the arm moves per 20 ms, so it swings rather than jumps
SERVO_REST_S = 0.5         # once the arm has been still this long, its pulses stop
SERVO_DEADBAND_DEG = 1.0   # a reading must move the needle this far to wake it

# 1. Initialize the USB Host port using raw pins
try:
    print("Initializing USB Host via microcontroller pins...")
    pin_dp = microcontroller.pin.GPIO24
    pin_dm = microcontroller.pin.GPIO25
    uh = usb_host.Port(pin_dp, pin_dm)
    print("USB Host successfully initialized!")
except Exception as e:
    print("USB Host already running or initialization failed:", e)

# BleuIO USB Identification
BLEUIO_VID = 0x2DCF
BLEUIO_PID = 0x6002

print("Waiting for BleuIO dongle to be connected...")
device = None
while device is None:
    device = usb.core.find(idVendor=BLEUIO_VID, idProduct=BLEUIO_PID)
    if device is None:
        time.sleep(1)

print(f"BleuIO dongle found! ({hex(BLEUIO_VID)}:{hex(BLEUIO_PID)})")
device.set_configuration()

# 2. Enable CDC ACM Control Lines (DTR/RTS)
# This forces the dongle to stream serial data back over USB bulk endpoints.
try:
    device.ctrl_transfer(0x21, 0x22, 0x03, 0, None)
    print("CDC Control Lines (DTR/RTS) enabled.")
except Exception as e:
    print("Could not set control lines:", e)

# USB Bulk Endpoints for BleuIO
endpoint_out = 0x02
endpoint_in = 0x81


# --- HIBOUAIR ---------------------------------------------------------------
# HibouAir's company id is 0x075B, which appears in the advertisement as the
# bytes 5B 07 after the manufacturer-data type FF -- hence the scan filter
# FF5B07. The offsets below count from the company id, as the HibouAir beacon
# documentation does. Pressure, temperature, humidity and the particle counts
# are tenths; CO2 is the one big-endian field.
HIBOUAIR_FILTER = "FF5B07"
BEACON_SENSORS = 0x05      # the beacon that carries the readings
BOARD_PM = 0x03            # the PM sensor's board type


def manufacturer_data(adv):
    """The manufacturer-specific data in an advertisement, from the company id
    on, if it is HibouAir's; otherwise None. Walks the AD structures rather than
    trusting a fixed offset, so a beacon without the flags field still works."""
    i = 0
    while i + 1 < len(adv):
        n = adv[i]
        if n == 0 or i + 1 + n > len(adv):
            break
        if adv[i + 1] == 0xFF and n >= 3 and adv[i + 2] == 0x5B and adv[i + 3] == 0x07:
            return adv[i + 2:i + 1 + n]
        i += n + 1
    return None


def u16(p, i):
    return p[i] | (p[i + 1] << 8)


def s16(p, i):
    v = u16(p, i)
    return v - 0x10000 if v & 0x8000 else v


def decode_hibouair(p):
    """One sensor beacon as a dict, or None if it is some other beacon."""
    if len(p) < 26 or p[2] != BEACON_SENSORS:
        return None
    return {
        "id": "%02X%02X%02X" % (p[4], p[5], p[6]),
        "board": p[3],
        "pressure_hpa": u16(p, 9) / 10,
        "temp_c": s16(p, 11) / 10,
        "humidity_pct": u16(p, 13) / 10,
        "pm1": u16(p, 17) / 10,
        "pm25": u16(p, 19) / 10,
        "pm10": u16(p, 21) / 10,
        "co2_ppm": (p[23] << 8) | p[24],
    }


def take_scan_line(msg):
    """A {"SF": ...} line from AT+FINDSCANDATA: decode it, remember it, and
    update pm10 if it came from the PM sensor being followed."""
    global pm10, pm10_sensor, pm10_time
    data = msg.get("data")
    if not data:
        return
    try:
        adv = binascii.unhexlify(data)
    except Exception:
        return
    p = manufacturer_data(adv)
    if p is None:
        return
    s = decode_hibouair(p)
    if s is None:
        return
    s["rssi"] = msg.get("rssi")
    s["time"] = time.monotonic()
    sensors[s["id"]] = s

    if s["board"] == BOARD_PM and PM_SENSOR_ID in (None, s["id"]):
        pm10 = s["pm10"]
        pm10_sensor = s["id"]
        pm10_time = s["time"]
        print(f"[HibouAir {s['id']}] PM10 {pm10:.1f} ug/m3  "
              f"PM2.5 {s['pm25']:.1f}  PM1 {s['pm1']:.1f}  {s['temp_c']:.1f} C")
    else:
        print(f"[HibouAir {s['id']}] board {s['board']:#04x}, {s['temp_c']:.1f} C, "
              f"{s['humidity_pct']:.1f} %rh, CO2 {s['co2_ppm']} ppm")


# --- TALKING TO THE DONGLE ----------------------------------------------------
# Everything the dongle says arrives in 64-byte pieces that do not respect line
# ends, so the reader keeps what is left over and hands on whole lines only.
# Scan results go to take_scan_line; everything else is kept in `replies` for
# whoever is waiting for the answer to a command.
replies = []
last_scan_line = 0.0
last_rx = 0.0          # when the dongle last sent anything at all
discarding = False     # while True, what arrives is thrown away unread


def to_text(raw):
    """A line as text. What the dongle sends is ASCII, but a line cut short by
    Ctrl-C can end in half of something, and one bad byte must cost that line
    and nothing else -- so a line that will not decode keeps its printable
    characters."""
    try:
        return raw.decode("utf-8").strip()
    except Exception:
        return "".join(chr(c) for c in raw if 32 <= c < 127).strip()


def take_line(line):
    global last_scan_line
    if line.startswith("{"):
        try:
            msg = json.loads(line)
        except ValueError:
            msg = None
        if isinstance(msg, dict) and "SF" in msg:
            last_scan_line = time.monotonic()
            take_scan_line(msg)
            return
    replies.append(line)
    if len(replies) > 20:
        replies.pop(0)


async def read_from_dongle():
    """Asynchronous loop that continuously captures real-time data from BleuIO."""
    global last_rx
    buf = bytearray(64)
    print("Starting background USB reader task...")
    pending = b""

    while True:
        try:
            # Quick read with a short timeout to prevent blocking the async loop
            bytes_read = device.read(endpoint_in, buf, timeout=10)
            if bytes_read > 0:
                last_rx = time.monotonic()
                if discarding:
                    pending = b""
                    continue
                pending += bytes(buf[:bytes_read])
                # Every complete line, however many came in this piece. The
                # line leaves `pending` before anything is done with it, so a
                # line that cannot be understood is gone, not read again at
                # the head of every later piece.
                while True:
                    cut = -1
                    for i, c in enumerate(pending):
                        if c == 10 or c == 13:
                            cut = i
                            break
                    if cut < 0:
                        break
                    raw = pending[:cut]
                    pending = pending[cut + 1:]
                    line = to_text(raw)
                    if line:
                        take_line(line)
                # A line that never ends is not allowed to grow for ever.
                if len(pending) > 1024:
                    pending = b""
        except Exception:
            # Timeout or no data available; safe to ignore
            pass

        # Yield control back to the asyncio scheduler for 10 milliseconds
        await asyncio.sleep(0.01)


async def send_at_command_async(command):
    """Sends an AT command asynchronously."""
    print(f"[Sending]: {command}")
    full_cmd = command + "\r"  # Carriage Return termination
    try:
        device.write(endpoint_out, full_cmd.encode("utf-8"))
    except Exception as e:
        print(f"Error writing to USB: {e}")


async def stop_scan():
    """Ctrl-C ends whatever the dongle is doing -- a scan left running from
    before this program started, most often. Sent a few times, because a byte
    arriving in the middle of a burst of scan output can go unnoticed.

    Then the dongle's queue is flushed: what it still had to send arrives after
    the Ctrl-C, often a line cut off in the middle, and it is thrown away
    unread until the dongle has been quiet for 0.3 s (3 s at most). The command
    that comes next then starts from a clean line on both sides."""
    global discarding
    discarding = True
    for _ in range(3):
        try:
            device.write(endpoint_out, b"\x03")
        except Exception as e:
            print(f"Error writing to USB: {e}")
        await asyncio.sleep(0.15)
    end = time.monotonic() + 3.0
    while time.monotonic() < end and time.monotonic() - last_rx < 0.3:
        await asyncio.sleep(0.05)
    discarding = False
    replies.clear()


async def command(cmd, want, timeout=2.0, tries=3):
    """Send cmd and wait for a reply line containing one of the strings in
    want. The plain reply ("ECHO OFF") and the verbose one ('"err":0') are both
    accepted, because which one comes depends on the mode the dongle was left in."""
    for _ in range(tries):
        replies.clear()
        await send_at_command_async(cmd)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for line in replies:
                if any(w in line for w in want):
                    return True
            await asyncio.sleep(0.05)
        print(f"No answer to {cmd}; trying again")
    print(f"The dongle never answered {cmd}")
    return False


async def start_scan():
    await stop_scan()
    await command("ATE0", ("ECHO OFF", '"err":0'))
    await command("ATV1", ("VERBOSE ON", '"err":0'))
    await send_at_command_async("AT+FINDSCANDATA=" + HIBOUAIR_FILTER)


async def main_logic():
    """Main program routine handling initialization and triggering the scan."""
    global last_scan_line
    await asyncio.sleep(1)  # Allow USB stack to settle

    print("Starting the HibouAir scan...")
    await start_scan()
    last_scan_line = time.monotonic()

    # The scan runs until it is stopped. If nothing has come for a while it has
    # stopped anyway -- or the dongle was unplugged and put back -- so start it
    # again.
    while True:
        await asyncio.sleep(1)
        if time.monotonic() - last_scan_line > 15:
            print("Nothing heard for 15 s; starting the scan again")
            await start_scan()
            last_scan_line = time.monotonic()


async def servo_dial():
    """Keeps the servo arm pointing at the latest PM10 value. It sweeps the
    whole scale once at start, so a servo that is not wired up is noticed at
    once rather than taken for clean air.

    A servo that is told to hold a position holds it actively, and at an end
    stop -- 0 ug/m3 is the commonest reading there is -- it pushes against the
    stop and hums. So once the arm has been still for SERVO_REST_S, the pulses
    stop: an ordinary hobby servo then goes quiet and stays where it is. They
    start again when a reading moves the needle by SERVO_DEADBAND_DEG or more,
    so 0.0 and 0.1 ug/m3 taking turns does not wake it."""
    pwm = pwmio.PWMOut(SERVO_PIN, frequency=50, duty_cycle=0)

    def point(deg):
        us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180
        pwm.duty_cycle = int(us * 65535 / 20000)    # 20 ms period at 50 Hz

    angle = 0.0
    for target in (180, 0):
        while angle != target:
            angle += SERVO_STEP_DEG if target > angle else -SERVO_STEP_DEG
            angle = max(0, min(180, angle))
            point(angle)
            await asyncio.sleep(0.02)

    still_since = time.monotonic()
    resting = False
    while True:
        target = 0.0
        if pm10 is not None:
            target = 180 * min(pm10, PM10_FULL_SCALE) / PM10_FULL_SCALE
        if resting:
            if abs(target - angle) < SERVO_DEADBAND_DEG:
                await asyncio.sleep(0.02)
                continue
            resting = False
        if angle != target:
            if abs(target - angle) <= SERVO_STEP_DEG:
                angle = target
            else:
                angle += SERVO_STEP_DEG if target > angle else -SERVO_STEP_DEG
            still_since = time.monotonic()
        point(angle)
        if time.monotonic() - still_since >= SERVO_REST_S:
            pwm.duty_cycle = 0         # no pulses: the servo lets go and is quiet
            resting = True
        await asyncio.sleep(0.02)


async def main():
    # Create parallel tasks for the reader, the logic engine and the dial
    reader_task = asyncio.create_task(read_from_dongle())
    logic_task = asyncio.create_task(main_logic())
    dial_task = asyncio.create_task(servo_dial())

    # Gather tasks to execute concurrently
    await asyncio.gather(reader_task, logic_task, dial_task)


# Start the cooperative multitasking scheduler
asyncio.run(main())
