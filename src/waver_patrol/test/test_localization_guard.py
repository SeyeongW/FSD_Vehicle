from waver_patrol.safety.localization_guard import LocalizationGuard


def test_covariance_stop():
    cov = [0.0] * 36
    cov[0] = 2.0
    cov[7] = 2.0
    decision = LocalizationGuard().evaluate(cov, pose_stamp=0.0, tf_stamp=0.0, now=0.1)
    assert decision.action == "STOP"


def test_pose_stale_stop():
    cov = [0.0] * 36
    decision = LocalizationGuard().evaluate(cov, pose_stamp=0.0, tf_stamp=0.0, now=2.0)
    assert decision.action == "STOP"
