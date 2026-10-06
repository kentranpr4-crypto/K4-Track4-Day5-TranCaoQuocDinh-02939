"""Reproducible synthetic benchmark; truth is used for scoring only."""
import csv
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from uwb_filter import Config, filter_positions, rts_smooth


def simulate(seed, scenario):
    rng = np.random.default_rng(seed)
    t = np.cumsum(rng.uniform(.075, .125, 600)); t -= t[0]
    if scenario == 'stationary':
        truth = np.tile([2., 3.], (len(t), 1))
    elif scenario == 'sharp_turns':
        knots = [0, 10, 20, 30, 40, 50, 60]
        points = np.array([[0,0],[8,0],[8,8],[8,8],[0,8],[0,0],[8,0]])
        truth = np.column_stack([np.interp(t, knots, points[:, j]) for j in range(2)])
    else:
        truth = np.column_stack([6*np.sin(.12*t), 3*np.sin(.24*t)])
    raw = truth + rng.normal(0, .3, truth.shape)
    if scenario in ('outliers', 'dropout'):
        mask = (rng.random(len(t)) < .05) | ((t > 24) & (t < 26))
        mask[:10] = False
        ang = rng.uniform(0, 2*np.pi, mask.sum())
        raw[mask] += rng.uniform(5, 10, (mask.sum(),1))*np.column_stack([np.cos(ang),np.sin(ang)])
    if scenario == 'dropout':
        raw[(t > 38) & (t < 41)] = np.nan
    if scenario == 'persistent_bias':
        raw[t > 25] += [2., -1.]
    return t, truth, raw


def metrics(x, truth, mask):
    error = np.linalg.norm(x[mask] - truth[mask], axis=1)
    return {'rmse_m': float(np.sqrt(np.mean(error**2))), 'p95_m': float(np.percentile(error,95))}


def evaluate(seed, scenario):
    t, truth, raw = simulate(seed, scenario)
    cfg = Config()
    causal = filter_positions(t, raw, cfg)
    naive = filter_positions(t, raw, Config(robust=False))
    online = np.array([r['x'][:2] for r in causal])
    smooth = rts_smooth(causal)[0][:,:2]
    ma = np.array([np.nanmean(raw[max(0,i-6):i+1], axis=0) if np.any(np.isfinite(raw[max(0,i-6):i+1])) else [np.nan]*2 for i in range(len(t))])
    tracks = {'raw':raw, 'moving_average_7':ma, 'kalman':np.array([r['x'][:2] for r in naive]), 'robust_online':online, 'rts_offline':smooth}
    # Identical timestamps for every method: exclude warmup and missing windows.
    common = (t >= 3) & np.all(np.isfinite(raw),axis=1) & np.all(np.isfinite(ma),axis=1)
    rows = [dict(seed=seed, scenario=scenario, method=name, **metrics(x,truth,common)) for name,x in tracks.items()]
    missing = (t >= 3) & ~np.all(np.isfinite(raw),axis=1)
    if missing.any():
        for name in ('kalman','robust_online','rts_offline'):
            rows.append(dict(seed=seed, scenario=scenario+'_missing_only', method=name, **metrics(tracks[name],truth,missing)))
    return rows, (t,truth,raw,tracks,causal)


def main():
    out = Path(__file__).resolve().parents[1] / 'reports'
    out.mkdir(exist_ok=True)
    scenarios = ['stationary','smooth','sharp_turns','outliers','dropout','persistent_bias']
    rows=[]
    for scenario in scenarios:
        for seed in range(10):
            result, _ = evaluate(seed, scenario)
            rows.extend(result)
    with (out/'benchmark.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    summary=[]
    for scenario in scenarios+['dropout_missing_only']:
        for method in ['raw','moving_average_7','kalman','robust_online','rts_offline']:
            selected=[r for r in rows if r['scenario']==scenario and r['method']==method]
            if selected:
                summary.append(dict(scenario=scenario,method=method,mean_rmse_m=float(np.mean([r['rmse_m'] for r in selected])),mean_p95_m=float(np.mean([r['p95_m'] for r in selected]))))
    (out/'benchmark_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    _, (t,truth,raw,tracks,records)=evaluate(42,'dropout')
    np.savetxt(out/'example_uwb.csv',np.column_stack([t,raw]),delimiter=',',header='timestamp,x,y',comments='')
    fig,axes=plt.subplots(2,2,figsize=(13,8))
    ax=axes[0,0]; ax.scatter(*raw.T,s=5,alpha=.25,color='#EA7B12',label='Raw UWB')
    ax.plot(*truth.T,color='#13233B',label='Truth')
    ax.plot(*tracks['robust_online'].T,color='#059669',label='Robust online')
    ax.plot(*tracks['rts_offline'].T,color='#2563EB',ls='--',label='RTS offline')
    ax.set(xlabel='x (m)',ylabel='y (m)',title='Synthetic UWB: spikes, burst, dropout',aspect='equal');ax.legend(fontsize=8)
    ax=axes[0,1]
    for name in ['raw','kalman','robust_online','rts_offline']:
        ax.plot(t,np.linalg.norm(tracks[name]-truth,axis=1),label=name,alpha=.8,lw=1)
    ax.set(xlabel='Time (s)',ylabel='Position error (m)',title='Accuracy, including prediction during dropout');ax.legend(fontsize=8)
    ax=axes[1,0];ax.plot(t,[r['nis'] for r in records],lw=.7,label='NIS');ax.plot(t,[r['gate_score'] for r in records],lw=.7,alpha=.6,label='Bounded gate score')
    ax.axhline(-2*np.log(.001),color='red',ls='--',label='Hard gate (99.9%)')
    ax.set(yscale='symlog',xlabel='Time (s)',ylabel='NIS before gate',title='Rejected readings remain visible');ax.legend(fontsize=8)
    ax=axes[1,1]
    ax.plot(t,[np.sqrt(np.trace(r['P'][:2,:2])) for r in records],color='#059669')
    ax.axvspan(38,41,color='gray',alpha=.2,label='Missing UWB')
    ax.set(xlabel='Time (s)',ylabel='sqrt(trace(P_position)) (m)',title='Uncertainty grows when measurements disappear');ax.legend(fontsize=8)
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'uwb_comparison.png',dpi=150);plt.close(fig)
    for row in summary:
        print(f"{row['scenario']:22} {row['method']:18} RMSE={row['mean_rmse_m']:.3f} P95={row['mean_p95_m']:.3f}")

if __name__=='__main__':
    main()
