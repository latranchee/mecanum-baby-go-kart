"""Drive a mix stimulus and print RAW serial, flagging any mid-run reboot.

Used to catch supply brownout under load: if the rail collapses while 4
motors run, the ESP32 resets and reprints its boot banner mid-stream.
Sends the `k` keepalive while driving and stops the robot on every exit path,
so a crash or Ctrl-C can't leave the cart running (robot test-link watchdog).
"""
import argparse, contextlib, sys, time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tlm import KEEPALIVE_S  # type: ignore


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default='COM8')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--vx', type=int, default=0)
    p.add_argument('--vy', type=int, default=0)
    p.add_argument('--omega', type=int, default=400)
    p.add_argument('--secs', type=float, default=4.0)
    args = p.parse_args()

    s = serial.Serial(args.port, args.baud, timeout=0.1)
    reboot = False
    try:
        time.sleep(0.3)
        s.write(b's\n'); s.flush(); time.sleep(0.2)
        s.write(b'r\n'); s.flush(); time.sleep(0.2)
        s.write(f't {args.vx} {args.vy} {args.omega}\n'.encode()); s.flush()
        print(f'driving t {args.vx} {args.vy} {args.omega} for {args.secs:.0f}s')

        end = time.time() + args.secs
        next_k = 0.0
        while time.time() < end:
            if time.monotonic() >= next_k:
                s.write(b'k\n'); s.flush()
                next_k = time.monotonic() + KEEPALIVE_S
            line = s.readline().decode('utf-8', 'replace').rstrip()
            if not line:
                continue
            print(line)
            if 'rst:' in line or 'reset reason' in line or 'ets ' in line:
                reboot = True
    finally:
        with contextlib.suppress(serial.SerialException, OSError):
            s.write(b's\n'); s.flush(); time.sleep(0.1)
            s.write(b'x\n'); s.flush(); time.sleep(0.1)
        s.close()
    print('\n*** BROWNOUT/REBOOT detected mid-drive ***' if reboot
          else '\n(no reboot during drive)')


if __name__ == '__main__':
    main()
