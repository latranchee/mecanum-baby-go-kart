"""Spin slot 1 (FR) at direct PWM and stream its raw_tps live.

Use to debug the dead FR encoder: jiggle the GPIO 16/4 connector while it
spins and watch raw_tps1 — nonzero = encoder counting again.
"""
import argparse, queue, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from joyctl import SerialIO, parse_tlm  # type: ignore


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default='COM8')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--slot', type=int, default=1)
    p.add_argument('--pwm', type=int, default=500)
    p.add_argument('--secs', type=float, default=12.0)
    args = p.parse_args()

    sio = SerialIO(args.port, args.baud)
    try:
        sio.send('s'); time.sleep(0.3)
        sio.send('r'); time.sleep(0.2)
        sio.send(f'm {args.slot} {args.pwm}')
        print(f'spinning slot{args.slot} pwm={args.pwm} for {args.secs:.0f}s — watch tps')
        deadline = time.monotonic() + args.secs
        last = 0.0
        while time.monotonic() < deadline:
            try:
                line = sio.lines.get(timeout=0.1)
            except queue.Empty:
                continue
            s = parse_tlm(line)
            if s is None:
                continue
            now = time.monotonic()
            if now - last >= 0.25:
                last = now
                t = s.raw_tps[args.slot]
                bar = '#' * min(40, int(abs(t) / 30))
                print(f'  cnt={s.cnt[args.slot]:>7}  raw_tps={t:>+8.1f} {bar}')
        sio.send('s'); time.sleep(0.2)
        sio.send('x'); time.sleep(0.2)
        print('stopped.')
    finally:
        sio.close()


if __name__ == '__main__':
    main()
