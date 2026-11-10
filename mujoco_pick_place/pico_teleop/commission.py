"""Passive, user-verified mapping from this arm's encoder to model joints."""
import numpy as np

TICK=2*np.pi/4096


def import_lerobot(rows,signature,register_ranges,supply_version):
    """Import measured LeRobot calibration without rewriting motor memory.

    Matches the pinned SDK's DEGREES normalization exactly (range midpoint,
    360/4095). Present_Position already includes the motor's homing offset.
    """
    from mujoco_pick_place.so101.calibration import JOINT_NAMES
    if set(rows)!=set(JOINT_NAMES) or len(register_ranges)!=6 or len(signature.get('offsets',[]))!=6:
        raise ValueError('需要六个标准 SO101 关节的完整标定和硬件读回')
    bounds=[]
    for index,name in enumerate(JOINT_NAMES):
        row=rows[name]
        if row.get('id')!=index+1 or row.get('drive_mode')!=0:
            raise ValueError(f'{name} 编号不匹配或反向模式未经验证')
        low,high=row.get('range_min'),row.get('range_max');offset=row.get('homing_offset')
        if any(type(v) is not int for v in (low,high,offset)) or not 0<=low<high<=4095 or abs(offset)>2047:
            raise ValueError(f'{name} 标定数值不合法')
        encoded_offset=offset if offset>=0 else abs(offset)|(1<<11)
        if encoded_offset!=signature['offsets'][index] or [low,high]!=list(register_ranges[index]):
            raise ValueError(f'{name} 内存偏移/限位与标定文件不一致；禁止重复写入偏移')
        bounds.append([low,high])
    return dict(schema=1,signature=signature,zero_counts=[sum(b)/2 for b in bounds[:5]],
                signs=[1]*5,raw_bounds=bounds[:5],gripper_closed=bounds[5][0],gripper_open=bounds[5][1],
                ticks_per_revolution=4095,pose_verified=True,directions_verified=True,workcell_verified=True,
                supply_version=supply_version,verification_basis='user-supplied LeRobot calibration; pinned SDK DEGREES; standard new_calib CAD',
                source_calibration=rows)


def raw_vector(value):
    raw=np.asarray(value,float)
    if raw.shape!=(6,) or not np.isfinite(raw).all() or np.any(raw<0) or np.any(raw>4095):
        raise ValueError('需要六路有效单圈编码器反馈（0～4095）')
    return raw


class RawCalibration:
    feedback_margin_counts=4
    def __init__(self,data,signature,model_limits):
        if data.get('schema')!=1 or data.get('signature')!=signature:
            raise ValueError('标定与控制板、电机型号或内存偏移不匹配')
        if any(data.get(k) is not True for k in ('pose_verified','directions_verified','workcell_verified')):
            raise ValueError('必须核对零位、关节方向和工位')
        if data.get('supply_version') not in ('12V','7.4V'):raise ValueError('必须确认电机供电版本')
        self.data=data;self.zero=np.asarray(data['zero_counts'],float);self.signs=np.asarray(data['signs'],float)
        ticks=data.get('ticks_per_revolution',4096)
        if ticks not in (4095,4096):raise ValueError('不支持的编码器角度换算约定')
        self.tick=2*np.pi/ticks
        bounds=np.asarray(data['raw_bounds'],float)
        if (self.zero.shape!=(5,) or not np.isfinite(self.zero).all() or np.any(self.zero<0) or np.any(self.zero>4095)
                or self.signs.shape!=(5,) or not np.isin(self.signs,[-1,1]).all()
                or bounds.shape!=(5,2) or not np.isfinite(bounds).all() or np.any(bounds<0) or np.any(bounds>4095)
                or np.any(bounds[:,1]-bounds[:,0]<100) or np.any(self.zero<=bounds[:,0]) or np.any(self.zero>=bounds[:,1])):
            raise ValueError('每轴需在零位两侧记录足够的实际活动范围')
        mapped=(bounds-self.zero[:,None])*self.signs[:,None]*self.tick
        self.limits=np.sort(mapped,axis=1);self.limits[:,0]+=.01;self.limits[:,1]-=.01
        self.limits[:,0]=np.maximum(self.limits[:,0],np.asarray(model_limits)[:,0]+.01)
        self.limits[:,1]=np.minimum(self.limits[:,1],np.asarray(model_limits)[:,1]-.01)
        if np.any(self.limits[:,1]-self.limits[:,0]<.03):raise ValueError('可用关节范围过小')
        self.closed=float(data['gripper_closed']);self.open=float(data['gripper_open'])
        if not np.isfinite([self.closed,self.open]).all() or not 0<=min(self.closed,self.open)<max(self.closed,self.open)<=4095 or abs(self.open-self.closed)<100:
            raise ValueError('需要分开采集夹爪闭合和打开端点')
        mapped_raw=self.zero[:,None]+self.signs[:,None]*self.limits/self.tick
        self.command_raw_bounds=np.r_[np.column_stack((np.ceil(mapped_raw.min(axis=1)),np.floor(mapped_raw.max(axis=1)))),
            [[np.ceil(min(self.closed,self.open)),np.floor(max(self.closed,self.open))]]].astype(int)

    def validate_feedback(self,raw):
        raw=raw_vector(raw)
        if np.any(raw!=np.rint(raw)):raise ValueError('反馈编码器计数必须为整数')
        q=self.decode(raw,require_bounds=False)
        excess=np.maximum(self.limits[:,0]-q[:5],q[:5]-self.limits[:,1])/self.tick
        if np.any(excess>self.feedback_margin_counts+1e-9):
            axis=int(np.argmax(excess))
            raise ValueError(f'ID{axis+1}关节反馈超出软件范围{excess[axis]:.2f}计数（反馈容差{self.feedback_margin_counts}）')
        grip_excess=max(min(self.closed,self.open)-raw[5],raw[5]-max(self.closed,self.open))
        if grip_excess>self.feedback_margin_counts:
            raise ValueError(f'ID6夹爪反馈超出范围{grip_excess:.2f}计数')
        return q

    def project_command(self,q):
        """Feasible planning seed only; never modifies observed feedback."""
        q=np.asarray(q,float)
        if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('需要六路有效目标参考')
        target=np.r_[np.clip(q[:5],self.limits[:,0]+self.tick,self.limits[:,1]-self.tick),np.clip(q[5],0,1)]
        return self.decode(self.encode(target),require_bounds=False)

    def decode(self,raw,*,require_bounds=True):
        raw=raw_vector(raw);q=(raw[:5]-self.zero)*self.signs*self.tick
        # Feedback is observed, never clamped into the commissioned envelope.
        if require_bounds and (np.any(q<self.limits[:,0]-.02) or np.any(q>self.limits[:,1]+.02)):
            raise ValueError('反馈超出已标定范围；保持只读并重新标定/手动回到范围内')
        grip=(raw[5]-self.closed)/(self.open-self.closed)
        if not -.02<=grip<=1.02:raise ValueError('夹爪反馈超出标定端点')
        return np.r_[q,np.clip(grip,0,1)]

    def encode(self,q):
        q=np.asarray(q,float)
        if (q.shape!=(6,) or not np.isfinite(q).all() or not 0<=q[5]<=1
                or np.any(q[:5]<self.limits[:,0]) or np.any(q[:5]>self.limits[:,1])):
            raise ValueError('目标超出实测保守范围')
        raw=np.rint(np.r_[self.zero+self.signs*q[:5]/self.tick,self.closed+q[5]*(self.open-self.closed)]).astype(int)
        return np.clip(raw,self.command_raw_bounds[:,0],self.command_raw_bounds[:,1])


class Commissioning:
    def __init__(self):
        self.latest=None;self.zero=None;self.low=None;self.high=None;self.recording=False
        self.closed=None;self.open=None

    def observe(self,raw):
        raw=raw_vector(raw)
        if self.recording:
            if self.latest is not None and np.max(abs(raw-self.latest))>1024:
                self.recording=False;self.low=None;self.high=None
                raise ValueError('编码器出现大跳变/跨圈；已中止范围记录，请重新采集')
            self.low=np.minimum(self.low,raw);self.high=np.maximum(self.high,raw)
        self.latest=raw.copy()

    def capture_zero(self):
        if self.latest is None:raise ValueError('等待有效反馈')
        self.zero=self.latest[:5].copy();self.low=None;self.high=None;self.recording=False

    def start_range(self):
        if self.zero is None:raise ValueError('先按零位参考摆放并记录零位')
        self.low=self.latest.copy();self.high=self.latest.copy();self.recording=True

    def capture_gripper(self,endpoint):
        if self.latest is None:raise ValueError('等待有效反馈')
        if endpoint=='closed':self.closed=float(self.latest[5])
        elif endpoint=='open':self.open=float(self.latest[5])
        else:raise ValueError('未知夹爪端点')

    def build(self,signature,signs,pose,directions,workcell,supply):
        if self.zero is None or self.low is None or self.closed is None or self.open is None:
            raise ValueError('先采集零位、活动范围和夹爪两端点')
        data=dict(schema=1,signature=signature,zero_counts=self.zero.tolist(),signs=signs,
                  raw_bounds=np.stack([self.low[:5],self.high[:5]],axis=1).tolist(),
                  gripper_closed=self.closed,gripper_open=self.open,pose_verified=pose,
                  directions_verified=directions,workcell_verified=workcell,supply_version=supply)
        RawCalibration(data,signature,np.tile([-np.pi,np.pi],(5,1)))
        return data

    def status(self):
        return dict(zero_counts=None if self.zero is None else self.zero.tolist(),recording=self.recording,
                    range_counts=None if self.low is None else np.stack([self.low,self.high],axis=1).tolist(),
                    gripper_closed=self.closed,gripper_open=self.open)
