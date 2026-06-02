"""Drive a mix stimulus and print RAW serial, flagging any mid-run reboot.

Used to catch supply brownout under load: if the rail collapses while 4
motors run, the ESP32 resets and reprints its boot banner mid-stream.
"""
import argparse, sys, time
import serial


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default='COM8')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--vx', type=int, default=0)
    p.add_argument('--vy', type=int, default=0)
    p.add_argument('--omega', type=int, default=400)
    p.add_argument('--secs', type=float, default=4.0)
    args = p.parse_args()

    s = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(0.3)
    s.write(b's\n'); s.flush(); time.sleep(0.2)
    s.write(b'r\n'); s.flush(); time.sleep(0.2)
    s.write(f't {args.vx} {args.vy} {args.omega}\n'.encode()); s.flush()
    print(f'driving t {args.vx} {args.vy} {args.omega} for {args.secs:.0f}s')

    reboot = False
    end = time.time() + args.secs
    while time.time() < end:
        line = s.readline().decode('utf-8', 'replace').rstrip()
        if not line:
            continue
        print(line)
        if 'rst:' in line or 'reset reason' in line or 'ets ' in line:
            reboot = True
    s.write(b's\n'); s.flush(); time.sleep(0.1)
    s.write(b'x\n'); s.flush(); time.sleep(0.1)
    s.close()
    print('\n*** BROWNOUT/REBOOT detected mid-drive ***' if reboot
          else '\n(no reboot during drive)')


if __name__ == '__main__':
    main()
