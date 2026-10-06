"""CSV interface: python -m uwb_filter INPUT.csv --output OUTPUT.csv."""
import argparse
import csv
from pathlib import Path
import numpy as np
from .core import Config, filter_positions, rts_smooth


def main():
    parser = argparse.ArgumentParser(description='Filter UWB xy coordinates (metres, capture time in seconds).')
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sigma', type=float, default=0.3, help='per-axis LOS std, metres')
    parser.add_argument('--q', type=float, default=0.5, help='acceleration PSD, m^2/s^3')
    parser.add_argument('--bias', nargs=2, type=float, default=[0., 0.])
    parser.add_argument('--smooth', action='store_true', help='add OFFLINE RTS output (uses future data)')
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve():
        parser.error('input and output must differ')
    try:
        data = np.genfromtxt(args.input, delimiter=',', names=True, ndmin=1)
        if not {'timestamp', 'x', 'y'} <= set(data.dtype.names or ()):
            raise ValueError('CSV requires timestamp,x,y; optional sigma (per row)')
        t, xy = data['timestamp'], np.column_stack([data['x'], data['y']])
        sd = data['sigma'] if 'sigma' in data.dtype.names else None
        records = filter_positions(t, xy, Config(sigma=args.sigma, q=args.q), sigma=sd, bias=args.bias)
        smooth = rts_smooth(records)[0] if args.smooth else None
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fields = ['timestamp', 'raw_x', 'raw_y', 'x', 'y', 'vx', 'vy', 'sigma_x', 'sigma_y',
              'cov_xy', 'nis_before_gate', 'gate_score', 'r_scale', 'accepted', 'status', 'age_s', 'stale', 'segment']
    if smooth is not None:
        fields += ['offline_x', 'offline_y']
    with args.output.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        for i, r in enumerate(records):
            row = [r['t'], *xy[i], *r['x'], *np.sqrt(np.diag(r['P'])[:2]), r['P'][0,1],
                   r['nis'], r['gate_score'], r['r_scale'], r['accepted'], r['status'], r['age'], r['stale'], r['segment']]
            if smooth is not None:
                row += list(smooth[i, :2])
            writer.writerow(row)
    print(f'{len(records)} rows -> {args.output}; accepted {sum(r["accepted"] for r in records)}')


if __name__ == '__main__':
    main()
