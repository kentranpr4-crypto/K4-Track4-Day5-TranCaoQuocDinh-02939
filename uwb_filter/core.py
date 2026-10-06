"""Causal 2-D position tracking and optional offline RTS smoothing (metres/seconds)."""
from dataclasses import dataclass
import numpy as np


def make_F(dt):
    F = np.eye(4)
    F[0, 2] = F[1, 3] = dt
    return F


def make_Q(dt, q):
    """Continuous white acceleration PSD q, in m²/s³; valid for variable dt."""
    Q = np.zeros((4, 4))
    block = q * np.array([[dt**3 / 3, dt**2 / 2], [dt**2 / 2, dt]])
    Q[np.ix_([0, 2], [0, 2])] = block
    Q[np.ix_([1, 3], [1, 3])] = block
    return Q


def make_H():
    return np.eye(2, 4)


def symmetric(P):
    return (P + P.T) * 0.5


class KalmanFilter:
    def __init__(self, x0, P0):
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)

    def predict(self, F, Q):
        self.x = F @ self.x
        self.P = symmetric(F @ self.P @ F.T + Q)

    def update(self, z, H, R):
        y = np.asarray(z) - H @ self.x
        S = H @ self.P @ H.T + R
        K = np.linalg.solve(S, H @ self.P).T
        self.x += K @ y
        A = np.eye(len(self.x)) - K @ H
        self.P = symmetric(A @ self.P @ A.T + K @ R @ K.T)
        return y, S, K


@dataclass(frozen=True)
class Config:
    sigma: float = 0.3  # LOS position std PER AXIS (m), measured by calibration
    q: float = 0.5      # acceleration PSD (m²/s³)
    gate_probability: float = 0.999
    soft_probability: float = 0.95
    recovery_samples: int = 8
    recovery_after: float = 1.0
    max_speed: float = 15.0  # only used to validate recovery candidates (m/s)
    max_gate_sigma: float = 1.0  # cap prior position std ONLY for robust gating (m)
    stale_after: float = 1.0
    robust: bool = True

    def __post_init__(self):
        for name in ('sigma', 'q', 'recovery_after', 'max_speed', 'stale_after', 'max_gate_sigma'):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not 0 < self.soft_probability < self.gate_probability < 1:
            raise ValueError('require 0 < soft_probability < gate_probability < 1')
        if not isinstance(self.recovery_samples, int) or self.recovery_samples < 3:
            raise ValueError('recovery_samples must be an integer >= 3')


class UWBTracker:
    """One tracker per tag. Timestamps must strictly increase; NaN means missing.

    The first valid reading initializes the track, so an untrusted first fix
    needs an external prior. Recovery is flagged, never silently hidden.
    """
    def __init__(self, config=None, bias=(0.0, 0.0)):
        self.config = config or Config()
        self.bias = np.asarray(bias, float)
        if self.bias.shape != (2,) or not np.all(np.isfinite(self.bias)):
            raise ValueError('bias must contain two finite metre offsets')
        self.kf = None
        self.t = None
        self.last_accepted = None
        self.candidates = []
        self.segment = 0

    def step(self, timestamp, position, sigma=None):
        c = self.config
        t = float(timestamp)
        z = np.asarray(position, float)
        sd = c.sigma if sigma is None else float(sigma)
        if not np.isfinite(t) or (self.t is not None and t <= self.t):
            raise ValueError('capture timestamps must be finite and strictly increasing')
        if z.shape != (2,) or np.any(np.isinf(z)):
            raise ValueError('position must have two values; use NaN for missing fixes')
        if not np.isfinite(sd) or sd <= 0:
            raise ValueError('sigma must be finite and positive')
        valid = bool(np.all(np.isfinite(z)))
        z = z - self.bias
        F = make_F(0 if self.t is None else t - self.t)
        if self.kf is not None:
            self.kf.predict(F, make_Q(t - self.t, c.q))
        self.t = t
        nis, gate_score, scale, accepted, status = np.nan, np.nan, 1.0, False, 'missing'
        xp = np.full(4, np.nan) if self.kf is None else self.kf.x.copy()
        Pp = np.full((4, 4), np.nan) if self.kf is None else self.kf.P.copy()
        if valid:
            R = np.eye(2) * sd**2
            if self.kf is None:
                self.kf = KalmanFilter([*z, 0, 0], np.diag([sd**2]*2 + [c.max_speed**2]*2))
                status, accepted = 'initialized', True
            else:
                y = z - self.kf.x[:2]
                S = self.kf.P[:2, :2] + R
                nis = float(y @ np.linalg.solve(S, y))
                # For df=2 the chi-square quantile is exactly -2 log(1-p).
                hard = -2 * np.log1p(-c.gate_probability)
                soft = -2 * np.log1p(-c.soft_probability)
                # Bound the acceptance region when prediction uncertainty grows.
                # Keep the true P and NIS for diagnostics; this score is heuristic.
                vals, vecs = np.linalg.eigh(self.kf.P[:2, :2])
                gate_S = (vecs * np.minimum(vals, c.max_gate_sigma**2)) @ vecs.T + R
                gate_score = float(y @ np.linalg.solve(gate_S, y))
                if not c.robust or gate_score <= hard:
                    scale = max(1.0, nis / soft) if c.robust else 1.0
                    self.kf.update(z, make_H(), R * scale)
                    status, accepted = ('downweighted' if scale > 1 else 'accepted'), True
                else:
                    status = 'rejected'
                    self.candidates.append((t, z.copy(), sd))
                    self.candidates = self.candidates[-c.recovery_samples:]
                    # Require a sequence compatible with motion, not merely N rejects.
                    if len(self.candidates) == c.recovery_samples and t - self.last_accepted >= c.recovery_after:
                        times = np.array([v[0] for v in self.candidates])
                        values = np.array([v[1] for v in self.candidates])
                        sigmas = np.array([v[2] for v in self.candidates])
                        A = np.column_stack([np.ones(len(times)), times - t])
                        coef = np.linalg.lstsq(A / sigmas[:, None], values / sigmas[:, None], rcond=None)[0]
                        residual = np.sqrt(np.mean(((values - A @ coef) / sigmas[:, None])**2))
                        if (np.max(np.diff(times)) <= c.stale_after and residual < 2.0
                                and np.linalg.norm(coef[1]) <= c.max_speed):
                            self.kf = KalmanFilter(np.r_[coef[0], coef[1]], np.diag([4*sd**2]*2 + [c.max_speed**2]*2))
                            self.segment += 1
                            status, accepted = 'reinitialized', True
        else:
            self.candidates.clear()
        if accepted:
            self.last_accepted = t
            self.candidates.clear()
        age = np.inf if self.last_accepted is None else t - self.last_accepted
        return dict(t=t, x=np.full(4, np.nan) if self.kf is None else self.kf.x.copy(),
                    P=np.full((4, 4), np.nan) if self.kf is None else self.kf.P.copy(),
                    predicted_x=xp, predicted_P=Pp, F=F, nis=nis, gate_score=gate_score,
                    r_scale=scale, accepted=accepted, status=status, age=age,
                    stale=age > c.stale_after, segment=self.segment)


def filter_positions(t, xy, config=None, sigma=None, bias=(0.0, 0.0)):
    t, xy = np.asarray(t, float), np.asarray(xy, float)
    if t.ndim != 1 or xy.shape != (len(t), 2) or not len(t):
        raise ValueError('expected nonempty t (N,) and xy (N,2)')
    if not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError('timestamps must be finite and strictly increasing')
    if np.any(np.isinf(xy)):
        raise ValueError('infinite positions are invalid')
    sd = np.full(len(t), (config or Config()).sigma) if sigma is None else np.broadcast_to(sigma, t.shape)
    if not np.all(np.isfinite(sd)) or np.any(sd <= 0):
        raise ValueError('sigma must be finite and positive')
    tracker = UWBTracker(config, bias)
    return [tracker.step(ti, zi, si) for ti, zi, si in zip(t, xy, sd)]


def rts_smooth(records):
    """Offline only: uses future samples. Never smooth across recovery resets."""
    if not records:
        raise ValueError('records must not be empty')
    xs = np.array([r['x'] for r in records])
    Ps = np.array([r['P'] for r in records])
    for k in range(len(records) - 2, -1, -1):
        nxt = records[k+1]
        if records[k]['segment'] != nxt['segment'] or not np.all(np.isfinite(xs[k])):
            continue
        C = np.linalg.solve(nxt['predicted_P'], nxt['F'] @ records[k]['P']).T
        xs[k] += C @ (xs[k+1] - nxt['predicted_x'])
        Ps[k] = symmetric(Ps[k] + C @ (Ps[k+1] - nxt['predicted_P']) @ C.T)
    return xs, Ps


def calibrate_stationary(xy, reference=None):
    """Robust per-axis LOS std and optional absolute bias from a STATIC log.

    A known reference is required to determine bias; motion invalidates this
    estimate. The caller must select a static, representative calibration log.
    """
    xy = np.asarray(xy, float)
    if xy.ndim != 2 or xy.shape[1] != 2:
        raise ValueError('expected (N,2)')
    xy = xy[np.all(np.isfinite(xy), axis=1)]
    if len(xy) < 20:
        raise ValueError('need at least 20 valid static samples')
    center = np.median(xy, axis=0)
    sd = 1.4826 * np.median(np.abs(xy - center), axis=0)
    out = {'sigma_axes': sd.tolist(), 'sigma': float(max(np.sqrt(np.mean(sd**2)), 1e-3))}
    if reference is not None:
        reference = np.asarray(reference, float)
        if reference.shape != (2,) or not np.all(np.isfinite(reference)):
            raise ValueError('reference must contain two finite metre coordinates')
        out['bias'] = (center - reference).tolist()
    return out
