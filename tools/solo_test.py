"""Drive each motor slot in isolation via direct PWM (no kinematics, no PID).

For each slot 0..3: run +PWM for ~1.5s, capture mean raw_tps; then -PWM.
Print summary so user can fill in visual observation per slot.

Requires robot firmware with `m <slot> <pwm>` command (test-mode direct path).
"""
import argparse
import queue
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from joyctl import SerialIO, parse_tlm  # type: ignore


def drain(sio, secs):
    deadline = time.monotonic() + secs
    while time.monotonic() < deadline:
        try:
            sio.lines.get(timeout=0.02)
        except queue.Empty:
            pass


def collect(sio, secs):
    samples = []
    deadline = time.monotonic() + secs
    while time.monotonic() < deadline:
        try:
            line = sio.lines.get(timeout=0.05)
        except queue.Empty:
            continue
        s = parse_tlm(line)
        if s is not None:
            samples.append(s)
    return samples


def steady(samples, window_s=0.7):
    if not samples:
        return [0.0] * 4
    end = samples[-1].ms
    cutoff = end - int(window_s * 1000)
    recent = [s for s in samples if s.ms >= cutoff] or samples[-5:]
    return [statistics.mean(s.raw_tps[i] for s in recent) for i in range(4)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default='COM8')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--pwm', type=int, default=500, help='PWM magnitude (0..1023)')
    p.add_argument('--run-secs', type=float, default=1.5)
    args = p.parse_args()

    sio = SerialIO(args.port, args.baud)
    try:
        sio.send('s'); drain(sio, 0.3)
        results = []
        for slot in range(4):
            for sign in (+1, -1):
                pwm = sign * args.pwm
                label = f'slot{slot} pwm={pwm:+d}'
                print(f'-- {label}')
                sio.send('r'); drain(sio, 0.2)
                sio.send(f'm {slot} {pwm}')
                samples = collect(sio, args.run_secs)
                sio.send('s'); drain(sio, 0.3)
                tps = steady(samples)
                results.append((slot, pwm, tps))
                print(f'   raw_tps = [{tps[0]:+8.1f} {tps[1]:+8.1f} {tps[2]:+8.1f} {tps[3]:+8.1f}]')
        sio.send('x'); drain(sio, 0.2)

        print('\n=== solo PWM sweep summary ===')
        print(f"{'slot':<5} {'pwm':>6} {'tps0':>9} {'tps1':>9} {'tps2':>9} {'tps3':>9}")
        for slot, pwm, tps in results:
            print(f"{slot:<5} {pwm:>+6d} {tps[0]:>+9.1f} {tps[1]:>+9.1f} {tps[2]:>+9.1f} {tps[3]:>+9.1f}")
        print('\nFor each slot, watch which physical wheel rotates and which direction.')
        print('Expected: only the commanded slot moves (others should read ~0 tps).')
        print('Report per slot: (wheel = FL/FR/RL/RR, +PWM direction = forward/backward)')
    finally:
        sio.close()


if __name__ == '__main__':
    main()
