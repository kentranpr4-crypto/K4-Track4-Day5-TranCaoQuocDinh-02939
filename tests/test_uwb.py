import unittest
import numpy as np
from uwb_filter import Config, UWBTracker, filter_positions, rts_smooth, calibrate_stationary
from uwb_filter.core import KalmanFilter, make_F, make_Q, make_H


class TrackingTests(unittest.TestCase):
    def test_scalar_posterior(self):
        kf = KalmanFilter([0.], [[4.]])
        kf.update(np.array([10.]), np.array([[1.]]), np.array([[1.]]))
        np.testing.assert_allclose(kf.x, [8.])
        np.testing.assert_allclose(kf.P, [[.8]])

    def test_continuous_Q_composes(self):
        a,b=.12,.37
        F=make_F(b)
        np.testing.assert_allclose(make_Q(a+b,.5),F@make_Q(a,.5)@F.T+make_Q(b,.5),atol=1e-15)

    def test_causality_and_spike(self):
        t=np.arange(100)*.1
        xy=np.column_stack([t,t*.3])
        xy[50]+=[100,-100]
        records=filter_positions(t,xy)
        prefix=filter_positions(t[:60],xy[:60])
        np.testing.assert_allclose([r['x'] for r in records[:60]],[r['x'] for r in prefix])
        self.assertEqual(records[50]['status'],'rejected')
        np.testing.assert_allclose(records[50]['x'],records[50]['predicted_x'])
        self.assertLess(np.linalg.norm(records[-1]['x'][:2]-xy[-1]),.05)
        self.assertGreater(records[50]['nis'],100)

    def test_dropout_and_covariance(self):
        t=np.arange(100)*.1;xy=np.column_stack([t,np.zeros(100)])
        xy[40:60]=np.nan
        records=filter_positions(t,xy)
        self.assertTrue(records[59]['stale'])
        self.assertGreater(np.trace(records[59]['P']),np.trace(records[39]['P']))
        for r in records:
            np.testing.assert_allclose(r['P'],r['P'].T,atol=1e-12)
            self.assertGreater(np.linalg.eigvalsh(r['P']).min(),0)
        xs,Ps=rts_smooth(records)
        self.assertTrue(np.all(np.isfinite(xs)))
        self.assertGreaterEqual(np.linalg.eigvalsh(Ps).min(),-1e-10)
        self.assertTrue(np.all(np.trace(Ps,axis1=1,axis2=2)<=np.array([np.trace(r['P']) for r in records])+1e-10))

    def test_gate_does_not_expand_without_bound_during_outage(self):
        t=np.arange(100)*.1;xy=np.zeros((100,2))
        xy[50:80]=np.nan;xy[80]=[6,0]
        rec=filter_positions(t,xy)
        r=rec[80]
        # Ordinary NIS would accept this spike after the outage.
        self.assertLess(r['nis'],-2*np.log(.001))
        self.assertGreater(r['gate_score'],-2*np.log(.001))
        self.assertEqual(r['status'],'rejected')
        np.testing.assert_allclose(r['P'],r['predicted_P'])

    def test_initial_missing(self):
        rec=filter_positions([0,1,2,3],[[np.nan,np.nan],[np.nan,1],[2,3],[2,3]])
        xs,_=rts_smooth(rec)
        self.assertTrue(np.all(np.isnan(xs[:2])))
        self.assertTrue(np.all(np.isfinite(xs[2:])))
        self.assertEqual(rec[2]['status'],'initialized')
        self.assertEqual(filter_positions([0],[[np.nan,np.nan]])[0]['status'],'missing')

    def test_invalid_inputs_do_not_mutate_tracker(self):
        tr=UWBTracker();tr.step(1,[0,0]);x=tr.kf.x.copy()
        for t,xy in [(1,[1,1]),(.5,[1,1]),(np.nan,[0,0]),(2,[np.inf,0]),(2,[1])]:
            with self.assertRaises(ValueError):tr.step(t,xy)
            self.assertEqual(tr.t,1)
            np.testing.assert_array_equal(tr.kf.x,x)
        for t,xy in [([],[]),([0,0],[[0,0],[1,1]]),([2,1],[[0,0],[1,1]])]:
            with self.assertRaises(ValueError):filter_positions(t,xy)
        with self.assertRaises(ValueError):Config(q=-1)
        with self.assertRaises(ValueError):filter_positions([0],[[0,0]],sigma=0)

    def test_recovery_is_flagged_and_smoother_segmented(self):
        t=np.arange(100)*.1;xy=np.zeros((100,2));xy[50:]=[20,20]
        rec=filter_positions(t,xy)
        resets=[i for i,r in enumerate(rec) if r['status']=='reinitialized']
        self.assertEqual(len(resets),1)
        i=resets[0];xs,_=rts_smooth(rec)
        np.testing.assert_allclose(xs[i-1],rec[i-1]['x'])
        self.assertLess(np.linalg.norm(rec[-1]['x'][:2]-xy[-1]),.1)

    def test_random_burst_does_not_trigger_recovery(self):
        t=np.arange(100)*.1;xy=np.zeros((100,2))
        xy[40:65]=np.random.default_rng(1).uniform(30,100,(25,2))
        rec=filter_positions(t,xy)
        self.assertFalse(any(r['status']=='reinitialized' for r in rec))
        self.assertLess(np.linalg.norm(rec[-1]['x'][:2]),.1)

    def test_calibration_and_bias(self):
        xy=np.random.default_rng(2).normal([2,3],.3,(2000,2));xy[:20]+=100
        result=calibrate_stationary(xy,reference=[0,0])
        self.assertAlmostEqual(result['sigma'],.3,delta=.025)
        np.testing.assert_allclose(result['bias'],[2,3],atol=.04)
        rec=filter_positions([0],[[2,3]],bias=[2,3])
        np.testing.assert_array_equal(rec[0]['x'][:2],[0,0])

    def test_accuracy_on_independent_seeds(self):
        for seed in [101,202,303]:
            rng=np.random.default_rng(seed);t=np.arange(500)*.1
            truth=np.column_stack([4*np.sin(.12*t),2*np.cos(.24*t)])
            raw=truth+rng.normal(0,.3,truth.shape)
            raw[80:90]+=rng.uniform(5,10,(10,2))
            rec=filter_positions(t,raw);online=np.array([r['x'][:2] for r in rec]);smooth=rts_smooth(rec)[0][:,:2]
            mse=lambda x:np.mean(np.sum((x[30:]-truth[30:])**2,axis=1))
            self.assertLess(mse(online),mse(raw)*.25)
            self.assertLess(mse(smooth),mse(online))


if __name__=='__main__':unittest.main()
