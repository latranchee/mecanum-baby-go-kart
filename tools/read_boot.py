"""Reset the robot and print its serial output for a few seconds.

Used to read the boot banner (reset reason, battery volts, MAC) and the
normal-mode telemetry stream. Read-only — sends no drive commands.
"""
import argparse, sys, time
import serial


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default='COM8')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--secs', type=float, default=5.0)
    args = p.parse_args()

    s = serial.Serial(args.port, args.baud, timeout=0.2)
    # Pulse RTS to reset the ESP32 (EN line) so we catch the boot banner.
    s.setDTR(False)
    s.setRTS(True); time.sleep(0.1); s.setRTS(False)
    time.sleep(0.3)

    end = time.time() + args.secs
    while time.time() < end:
        line = s.readline().decode('utf-8', 'replace').rstrip()
        if line:
            print(line)
    s.close()


if __name__ == '__main__':
    main()
