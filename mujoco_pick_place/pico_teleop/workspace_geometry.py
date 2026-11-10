"""Offline joint-line entry into a Cartesian box; no arm or motor I/O."""
import mujoco
import numpy as np
from scipy.optimize import least_squares, minimize


_STEP_RAD = .002
_PATH_TOL_M = 1e-6


def nearest_entry_plan(kinematics, q, limits, work_min, work_max, margin=.025):
    """Find a nearby reachable inset-box point and verify its joint line.

    Cartesian distance has priority over joint displacement. Local numerical
    solves do not prove a global nearest reachable point. The returned
    projection_residual_m measures the miss from the ideal box projection;
    optimality_gap_m is the excess travel over that geometric lower bound.
    Verification samples the model, not physical motion or collisions.
    """
    try:
        q = np.asarray(q, dtype=float)
        limits = np.asarray(limits, dtype=float)
        low = np.asarray(work_min, dtype=float)
        high = np.asarray(work_max, dtype=float)
        margin_array = np.asarray(margin, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError('finite geometry inputs required') from error
    if (q.shape != (5,) or limits.shape != (5, 2)
            or low.shape != (3,) or high.shape != (3,)
            or margin_array.shape != () or not np.isfinite(margin_array)
            or margin_array < 0
            or not np.isfinite(np.r_[q, limits.ravel(), low, high]).all()
            or np.any(limits[:, 0] > limits[:, 1])
            or np.any(low >= high)):
        raise ValueError('invalid joint limits, workspace or margin')
    inner_low, inner_high = low + float(margin_array), high - float(margin_array)
    if np.any(inner_low >= inner_high):
        raise ValueError('margin leaves no workspace interior')
    if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
        raise ValueError('initial joints outside limits')
    try:
        model, site = kinematics.model, kinematics.site
        data = mujoco.MjData(model)
        if model.nq < 5 or model.nv < 5 or not 0 <= site < model.nsite:
            raise ValueError('five-joint site geometry required')
    except (AttributeError, TypeError) as error:
        raise ValueError('MuJoCo kinematics with a TCP site required') from error
    # Copy non-arm coordinates without modifying the caller's kinematics data.
    data.qpos[:] = kinematics.data.qpos
    jp = np.zeros((3, model.nv))
    jr = np.zeros_like(jp)

    def position(joints):
        data.qpos[:5] = joints
        mujoco.mj_kinematics(model, data)
        point = data.site_xpos[site].copy()
        if not np.isfinite(point).all():
            raise ValueError('nonfinite forward geometry')
        return point

    origin = position(q)
    projection = np.clip(origin, inner_low, inner_high)
    clearance = np.minimum(1e-7, (inner_high - inner_low) * .1)
    solver_low, solver_high = inner_low + clearance, inner_high - clearance
    solver_projection = np.clip(projection, solver_low, solver_high)

    def verified_plan(goal):
        if (not np.isfinite(goal).all() or np.any(goal < limits[:, 0])
                or np.any(goal > limits[:, 1])):
            return None
        # Euclidean spacing also bounds every individual joint increment.
        count = int(np.ceil(np.linalg.norm(goal - q) / _STEP_RAD))
        joint_path = np.linspace(q, goal, count + 1) if count else q[None, :].copy()
        tcp_path = np.array([position(row) for row in joint_path])
        endpoint = tcp_path[-1]
        if np.any(endpoint < inner_low - 1e-9) or np.any(endpoint > inner_high + 1e-9):
            return None
        overflow = np.maximum(np.maximum(low - tcp_path, tcp_path - high), 0.)
        # Compare with the historical minimum to prevent cumulative drift.
        best_overflow = np.minimum.accumulate(overflow, axis=0)
        if np.any(overflow > best_overflow + _PATH_TOL_M):
            return None
        distance = float(np.linalg.norm(endpoint - origin))
        return dict(goal_q=goal.tolist(), goal_m=endpoint.tolist(),
                    origin_m=origin.tolist(), q_path=joint_path.tolist(),
                    tcp_path=tcp_path.tolist(), max_step_rad=(
                        float(np.max(np.linalg.norm(np.diff(joint_path, axis=0), axis=1)))
                        if count else 0.),
                    projection_m=projection.tolist(),
                    projection_residual_m=float(np.linalg.norm(endpoint - projection)),
                    cartesian_distance_m=distance,
                    joint_distance_rad=float(np.linalg.norm(goal - q)),
                    optimality_gap_m=max(0., distance - float(np.linalg.norm(projection - origin))),
                    approximate=True)

    if np.array_equal(projection, origin):
        return verified_plan(q.copy())

    free = limits[:, 1] > limits[:, 0]
    if not np.any(free):
        raise ValueError('no reachable inset workspace goal')
    fixed = q.copy()
    bounds = list(zip(limits[free, 0], limits[free, 1]))

    def expand(x):
        joints = fixed.copy()
        joints[free] = x
        return joints

    def evaluate(x):
        point = position(expand(x))
        mujoco.mj_comPos(model, data)
        mujoco.mj_jacSite(model, data, jp, jr, site)
        return point, jp[:, :5][:, free].copy()

    def residual(x):
        point, _ = evaluate(x)
        return np.r_[(point - solver_projection) * 100., (x - q[free]) * .01]

    def residual_jac(x):
        _, jac = evaluate(x)
        return np.vstack([jac * 100., np.eye(np.sum(free)) * .01])

    def box(x):
        point, _ = evaluate(x)
        return np.r_[point - solver_low, solver_high - point] * 100.

    def box_jac(x):
        _, jac = evaluate(x)
        return np.vstack([jac, -jac]) * 100.

    def travel(x):
        point, _ = evaluate(x)
        return float(np.sum((point - origin) ** 2) * 1e4)

    def travel_jac(x):
        point, jac = evaluate(x)
        return 2e4 * jac.T @ (point - origin)

    def displacement(x):
        return float(np.sum((x - q[free]) ** 2))

    def displacement_jac(x):
        return 2. * (x - q[free])

    candidates = []
    reached_box = False
    middle = np.mean(limits, axis=1)
    # Nearby seed first; deterministic alternate seeds only resolve local IK.
    seeds = [q, .8 * q + .2 * middle, middle]
    for seed in seeds:
        fit = least_squares(residual, seed[free], jac=residual_jac,
                            bounds=(limits[free, 0], limits[free, 1]),
                            max_nfev=180, ftol=1e-12, xtol=1e-12, gtol=1e-12)
        nearest = minimize(travel, fit.x, jac=travel_jac, bounds=bounds,
                           constraints=[dict(type='ineq', fun=box, jac=box_jac)],
                           method='SLSQP', options=dict(maxiter=160, ftol=1e-12))
        point, _ = evaluate(nearest.x)
        if np.any(point < inner_low - 1e-9) or np.any(point > inner_high + 1e-9):
            continue
        reached_box = True
        best_travel = travel(nearest.x)
        # Minimize joint change without meaningfully increasing Cartesian travel.
        close = dict(type='ineq',
                     fun=lambda x: best_travel + 1e-8 - travel(x),
                     jac=lambda x: -travel_jac(x))
        refined = minimize(displacement, nearest.x, jac=displacement_jac,
                           bounds=bounds,
                           constraints=[dict(type='ineq', fun=box, jac=box_jac), close],
                           method='SLSQP', options=dict(maxiter=160, ftol=1e-12))
        for x in (refined.x, nearest.x):
            result = verified_plan(expand(x))
            if result is not None:
                candidates.append(result)
        if candidates and min(p['projection_residual_m'] for p in candidates) < 2e-6:
            break
    if not candidates:
        if reached_box:
            raise ValueError('entry path cannot be verified to stay inward')
        raise ValueError('no reachable inset workspace goal found')
    # Numerical ties within a micrometre prefer the smallest joint movement.
    best_distance = min(p['cartesian_distance_m'] for p in candidates)
    close_candidates = [p for p in candidates if p['cartesian_distance_m'] <= best_distance + 1e-6]
    return min(close_candidates, key=lambda p: p['joint_distance_rad'])
