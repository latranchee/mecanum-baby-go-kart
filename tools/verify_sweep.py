import csv, statistics, sys
from pathlib import Path

ENC_SIGN = [-1, +1, -1, +1]  # mirrors src/robot/main.cpp encSign[]
EXPECTED = {
    'vx+':    [+1, +1, +1, +1],
    'vx-':    [-1, -1, -1, -1],
    'vy+':    [-1, +1, +1, -1],
    'vy-':    [+1, -1, -1, +1],
    'omega+': [-1, +1, -1, +1],
    'omega-': [+1, -1, +1, -1],
}
TARGET_MAG = 0.4 * 2100  # 840
MAG_TOL = 0.4

def steady_state(csv_path):
    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        return [0.0]*4
    end_ms = int(rows[-1]['ms'])
    recent = [r for r in rows if int(r['ms']) >= end_ms - 1000]
    if not recent:
        recent = rows[-5:]
    return [statistics.mean(float(r[f'raw_tps{i}']) for r in recent) for i in range(4)]

def find_latest_set(log_dir):
    files = sorted(Path(log_dir).glob('*.csv'))
    by_ts = {}
    for f in files:
        ts = f.name[:15]
        by_ts.setdefault(ts, []).append(f)
    for ts in sorted(by_ts.keys(), reverse=True):
        labels = {f.stem.split('-', 2)[2] for f in by_ts[ts]}
        if labels >= {'vx+','vx-','vy+','vy-','omega+','omega-'}:
            return ts, sorted(by_ts[ts])
    return None, []

def main():
    log_dir = Path(__file__).resolve().parent / 'logs'
    ts, files = find_latest_set(log_dir)
    if not files:
        print('no sweep CSVs found')
        return 2
    print(f'verifying sweep {ts}')
    all_ok = True
    rows_out = []
    for f in files:
        label = f.stem.split('-', 2)[2]
        if label not in EXPECTED:
            continue
        tps_signed = [ENC_SIGN[i] * v for i, v in enumerate(steady_state(f))]
        for i, t in enumerate(tps_signed):
            exp = EXPECTED[label][i]
            sign_ok = (t > 0 and exp > 0) or (t < 0 and exp < 0) or (abs(t) < 50 and exp == 0)
            mag_ok = abs(abs(t) - TARGET_MAG) / TARGET_MAG <= MAG_TOL
            if not sign_ok:
                all_ok = False
            rows_out.append((label, i, t, exp, sign_ok, mag_ok))
    print(f"{'stim':<8} {'slot':<5} {'tps':>10} {'exp':>5} {'sign':>6} {'mag':>5}")
    for r in rows_out:
        print(f"{r[0]:<8} {r[1]:<5} {r[2]:>10.1f} {r[3]:>5d} {'OK' if r[4] else 'FAIL':>6} {'OK' if r[5] else '~':>5}")
    vx_plus_s3  = next((r[2] for r in rows_out if r[0] == 'vx+' and r[1] == 3), 0)
    vx_minus_s3 = next((r[2] for r in rows_out if r[0] == 'vx-' and r[1] == 3), 0)
    if abs(vx_plus_s3) > 0 and abs(vx_minus_s3) < 0.5 * abs(vx_plus_s3):
        print(f"\nNOTE: KNOWN_HW_ASYMMETRY on slot 3 (RR): vx+ |{vx_plus_s3:.0f}| vs vx- |{vx_minus_s3:.0f}|")
    return 0 if all_ok else 1

sys.exit(main())
