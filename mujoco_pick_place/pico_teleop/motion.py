"""Coordinated, bounded physical commissioning profile; not a motor rating.

The performance profile completed full-arm local 60 mm/40 mm trajectories.
Experimental names are finite commissioning candidates, not validated motor
ratings. Software trajectory, tracking and feedback checks remain independent.
"""
from dataclasses import dataclass,replace
import os


@dataclass(frozen=True)
class PhysicalMotion:
    tcp_speed:float=.04
    joint_speed:float=.3
    wrist_roll_speed_rad_s:float=.3
    joint_acceleration:float=1.5
    gripper_speed_rad_s:float=.08
    raw_command_rate:int=200
    raw_motor_speed:int=300
    raw_motor_acceleration:int=10
    hand_filter_tau:float=.035
    hand_deadband:float=.003
    hand_scale:float=1.
    stable_input:bool=False
    orientation_tracking:bool=False
    orientation_weight:float=.1
    tracking_lead_counts:int=32
    tracking_brake_counts:int=28
    raw_tracking_limit_counts:int=40
    stationary_tolerance_counts:int=16
    tcp_planning_ratio:float=.9
    prediction_horizon:float=0.
    prediction_max_distance:float=.008
    prediction_velocity_tau:float=.035
    prediction_braking:bool=False
    tcp_lookahead:float=.025
    control_hz:int=30
    motor_period:float=.04


def profile_for(name):
    measured=PhysicalMotion()
    if name=='measured':return measured
    if name=='agile_fast':
        return replace(profile_for('agile'),tcp_speed=.45,joint_speed=2.5,joint_acceleration=24.,
                       wrist_roll_speed_rad_s=2.7,
                       gripper_speed_rad_s=2.4,
                       raw_command_rate=1800,raw_motor_speed=2600,tracking_brake_counts=176,
                       tracking_lead_counts=208,raw_tracking_limit_counts=256,tcp_lookahead=.08,prediction_braking=True,
                       orientation_tracking=True)
    if name=='agile':
        return replace(profile_for('performance_direct'),hand_filter_tau=.008,hand_deadband=.0015,
                       prediction_velocity_tau=.020,motor_period=.01,stable_input=True)
    if name=='performance_predict20':
        # Completed local trajectory; negligible gain over the 12 mm setting.
        return replace(profile_for('performance'),prediction_max_distance=.020)
    if name=='performance_direct':
        # Retained comparison profile; completed a local physical trajectory
        # with the bounded transient-temperature confirmation logic.
        return replace(profile_for('performance_predict20'),tcp_speed=.30)
    if name in ('commission_acc20','commission_acc40','commission_acc50'):
        return replace(profile_for('responsive'),raw_motor_acceleration=int(name.removeprefix('commission_acc')))
    if name=='performance':
        return replace(profile_for('responsive'),tcp_speed=.20,joint_speed=1.5,joint_acceleration=12.,
            gripper_speed_rad_s=1.6,
            raw_command_rate=1200,raw_motor_speed=1800,raw_motor_acceleration=50,
            tracking_brake_counts=112,tracking_lead_counts=128,raw_tracking_limit_counts=160,
            tcp_lookahead=.05,prediction_max_distance=.012,control_hz=50,motor_period=.02)
    if name=='responsive':
        # Prepared for supervised commissioning; never selected implicitly.
        return replace(measured,tcp_speed=.10,joint_speed=.75,joint_acceleration=4.,
            gripper_speed_rad_s=.8,
            raw_command_rate=600,raw_motor_speed=900,raw_motor_acceleration=20,
            tracking_brake_counts=56,tracking_lead_counts=64,raw_tracking_limit_counts=80,
            hand_filter_tau=.02,hand_deadband=.002,tcp_planning_ratio=.85,prediction_horizon=.15)
    raise ValueError('unknown PICO motion profile: '+str(name))


def bounded_prediction(velocity,horizon,maximum,acceleration=None):
    import numpy as np
    v=np.asarray(velocity,float);speed=float(np.linalg.norm(v))
    if v.shape!=(3,) or not np.isfinite(v).all() or not np.isfinite([horizon,maximum]).all() or horizon<0 or maximum<0:
        raise ValueError('invalid prediction input')
    a=None if acceleration is None else np.asarray(acceleration,float)
    if a is not None and (a.shape!=(3,) or not np.isfinite(a).all()):raise ValueError('invalid prediction acceleration')
    # Remove low-speed tracking noise continuously, rather than switching a
    # fixed offset on at a threshold. No future pose information is used.
    if speed<=.02:return np.zeros(3)
    distance=(speed-.02)*horizon
    if a is not None:
        deceleration=max(0.,-float(np.dot(a,v/speed)))
        if deceleration>1e-9:
            duration=min(horizon,(speed-.02)/deceleration)
            distance=(speed-.02)*duration-.5*deceleration*duration**2
    offset=v/speed*distance;length=float(np.linalg.norm(offset))
    return offset*min(1.,maximum/max(length,1e-12))


MOTION_PROFILE_NAME=os.environ.get('PICO_MOTION_PROFILE','measured')
PHYSICAL_MOTION=profile_for(MOTION_PROFILE_NAME)


def conditioned_trigger(value,previous=None):
    """Match the arming-neutral range and reject small analog button noise."""
    import math
    value=float(value)
    if not math.isfinite(value) or not 0<=value<=1:raise ValueError('invalid trigger')
    target=0. if value<=.05 else 1. if value>=.98 else (value-.05)/.93
    if previous is None or target in (0.,1.) or abs(target-previous)>.01:return target
    return previous
