"""Contracts independent of Isaac: frame conventions, recurrent reset, PD limits."""
import unittest
import numpy as np
from r2s2r.core.go2_moe_runtime import Go2MoeRuntime, DEFAULT_POS, TORQUE_LIMIT

class Go2MoeContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runtime = Go2MoeRuntime(pi_enabled=False)

    def test_observation_is_invariant_to_world_yaw(self):
        runtime = self.runtime
        runtime.reset()
        runtime.infer(DEFAULT_POS, np.zeros(12), [1,0,0,0],
                      [.2,-.1,0], [.1,.2,.3], [.4,0,.2], 2)
        obs, action = runtime.last_observation.copy(), runtime.action.copy()
        runtime.reset()
        half = np.sqrt(.5)
        runtime.infer(DEFAULT_POS, np.zeros(12), [half,0,0,half],
                      [.1,.2,0], [-.2,.1,.3], [.4,0,.2], 2)
        np.testing.assert_allclose(runtime.last_feedback, [.2,-.1,.3], atol=1e-7)
        np.testing.assert_allclose(runtime.last_observation, obs, atol=1e-7)
        np.testing.assert_allclose(runtime.action, action, atol=1e-6)

    def test_reset_restores_first_action_after_history_and_pi_use(self):
        runtime = self.runtime
        runtime.pi_enabled = True
        args = (DEFAULT_POS, np.zeros(12), [1,0,0,0], [0,0,0], [0,0,0], [.4,0,.4], 2)
        runtime.reset()
        runtime.infer(*args)
        expected = runtime.action.copy()
        for _ in range(30):
            runtime.infer(*args)
        runtime.reset()
        np.testing.assert_array_equal(runtime.action, np.zeros(12))
        runtime.infer(*args)
        np.testing.assert_allclose(runtime.action, expected, atol=0)
        runtime.pi_enabled = False

    def test_pd_limits_are_per_joint_and_symmetric(self):
        runtime = self.runtime
        runtime.reset()
        np.testing.assert_array_equal(runtime.effort(DEFAULT_POS-100, np.zeros(12)), TORQUE_LIMIT)
        np.testing.assert_array_equal(runtime.effort(DEFAULT_POS+100, np.zeros(12)), -TORQUE_LIMIT)
        np.testing.assert_array_equal(runtime.effort(DEFAULT_POS, np.zeros(12)), np.zeros(12))

    def test_invalid_sensor_data_rejected_before_inference(self):
        runtime = self.runtime
        runtime.reset()
        with self.assertRaises(ValueError):
            runtime.infer(DEFAULT_POS, np.zeros(12), [0,0,0,0],
                          [0,0,0], [0,0,0], [0,0,0], 2)
        with self.assertRaises(ValueError):
            runtime.infer(DEFAULT_POS, np.zeros(12), [1,0,0,0],
                          [0,0,0], [np.nan,0,0], [0,0,0], 2)

if __name__ == "__main__":
    unittest.main()
