"""Isolated commissioning checks: never authorize the live input controller."""
APP_VERSION = "pico-debug-43.0-split-roll"


def self_test():
    import numpy as np
    from .controller import TeleopController
    from .simulation import SimulationArm
    checks = []
    try:
        arm = SimulationArm()
        c = TeleopController(arm)
        q = arm.read().copy()
        p, _ = arm.kinematics.fk(q[:5])
        checks.append(dict(name="SO101 模型与正向运动学", ok=bool(np.isfinite(p).all())))
        solved = arm.kinematics.ik(p, seed_q=q[:5])
        actual, _ = arm.kinematics.fk(solved)
        checks.append(dict(name="逆运动学往返误差 < 2 mm", ok=bool(np.linalg.norm(actual-p)<.002)))
        c.attach("isolated-selftest")
        c.clock.bounds=(0,0);c.clock.updated_at=1.
        def frame(seq, now, grip=0., position=None):
            return dict(type="frame",session_id="isolated-selftest",seq=seq,client_time_ms=now*1000,
                        space_epoch=0,reference_space="local-floor",mode="synthetic",visible=True,
                        right=dict(valid=True,position=position or [.2,1.,-.3],orientation=[0,0,0,1],
                                   grip=grip,trigger=0.,precision=False,stop=False,arm=False))
        c.ingest(frame(1,1.),1.);c.arm(1.)
        c.ingest(frame(2,1.02,1.),1.02);c.tick(1.02,.02)
        checks.append(dict(name="首次握持保持原位",ok=bool(np.allclose(arm.read(),q))))
        c.ingest(frame(3,1.05,1.,[.2,1.,-.32]),1.05);c.tick(1.05,.03)
        checks.append(dict(name="合成手柄驱动仿真关节",ok=bool(np.linalg.norm(arm.read()-q)>1e-7)))
        held=arm.read().copy();c.tick(1.4,.03)
        checks.append(dict(name="输入超时撤权并保持",ok=bool(not c.armed and np.allclose(held,arm.read()))))
    except Exception as exc:
        checks.append(dict(name="自检异常",ok=False,message=str(exc)))
    return dict(ok=all(x["ok"] for x in checks),scope="simulation_only",checks=checks,
                message="仅验证独立仿真链路；PICO 追踪、按钮和头显三维显示须戴上头显实测。")
