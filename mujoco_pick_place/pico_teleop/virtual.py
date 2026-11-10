"""In-memory motor transport for the complete physical-control software path.

This module never constructs SerialBus, enumerates ports or operates ADB.
Its identity and calibration are deliberately unusable on the registered board.
"""
from collections import deque
from copy import deepcopy
import json
from pathlib import Path
from .timing import clock as time
import numpy as np

from .commission import RawCalibration
from .real import PhysicalArm
from .servo import PositionServoBank,ServoProfile
from .simulation import SimulationArm
from mujoco_pick_place.so101.kinematics import ROOT


class VirtualBus:
    is_virtual=True
    name='虚拟总线（不访问串口）'
    def __init__(self,data,initial):
        self.signature=data['signature'];self.torque=[0]*6;self.events=deque(maxlen=512)
        self.supply_version=data['supply_version'];self.voltage_raw=74 if self.supply_version=='7.4V' else 120
        self.tick=2*np.pi/4096;self.goals=np.array(initial,int)
        profile=ServoProfile(velocity_rad_s=(.15,)*6,acceleration_rad_s2=(.5,)*6)
        self.bank=PositionServoBank(self.goals*self.tick,np.tile([0,4095*self.tick],(6,1)),profile)
        self.last_time=time.monotonic()

    def advance(self,dt):self.bank.advance(dt);self.last_time=time.monotonic()
    def read(self):
        self.advance(max(.000001,time.monotonic()-self.last_time))
        raw=np.rint(self.bank.feedback()/self.tick).astype(int).tolist()
        return dict(raw=raw,voltage_raw=[self.voltage_raw]*6,temperature=[30]*6,torque=self.torque.copy())
    def goal(self,i,value):
        if not 1<=i<=6 or not 0<=value<=4095:raise ValueError('虚拟目标越界')
        self.goals[i-1]=int(value)
        self.events.append(dict(operation='goal',id=i,value=int(value),time=time.monotonic()))
        target=np.where(np.asarray(self.torque)==1,self.goals*self.tick,self.bank.actual)
        self.bank.command(target)
    def enable(self,i):
        self.torque[i-1]=1;self.events.append(dict(operation='enable',id=i,time=time.monotonic()))
    def disable(self,i):
        self.torque[i-1]=0;self.events.append(dict(operation='disable',id=i,time=time.monotonic()))
    def close(self):self.bank.hold()


def create_virtual_arm(output_dir=None):
    directory=Path(output_dir or ROOT/'artifacts/pico-teleop');directory.mkdir(parents=True,exist_ok=True)
    imported=ROOT/'artifacts/pico-teleop/imported-calibration.json'
    if imported.exists():data=deepcopy(json.loads(imported.read_text(encoding='utf-8')))
    else:
        data=dict(schema=1,signature=dict(board='virtual-so101',models=[777]*6,offsets=[0]*6),
                  zero_counts=[2048]*5,signs=[1]*5,raw_bounds=[[700,3400]]*5,
                  gripper_closed=1900,gripper_open=3000,pose_verified=True,directions_verified=True,
                  workcell_verified=True,supply_version='12V')
    data['signature']['board']='virtual-so101';data['is_virtual']=True
    data['source_file']='虚拟软件测试；输入映射来自 '+str(data.get('source_file','参考配置'))
    geometry=SimulationArm();calibration=RawCalibration(data,data['signature'],geometry.limits)
    initial=calibration.encode(geometry.read())
    path=directory/'virtual-calibration.json';path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    return PhysicalArm(path,bus=VirtualBus(data,initial))


class OfflineAssistant:
    """Preserve local diagnostics controls without USB enumeration or commands."""
    def __init__(self,*_):self.events=[]
    async def poll(self,**_):pass
    def status(self):
        return dict(enabled=False,devices=[],selected_serial=None,message='软件验证模式：无需连接串口或头显，可运行虚拟手柄演示。')
    def retry(self):pass
    def select(self,_):raise ValueError('软件模式不连接 USB 设备')
