"""User control references, separate from motor calibration and motor I/O."""
import copy
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .orientation import XR_TO_ROBOT,TOOL_TO_GRIP


class ControlReference:
    def __init__(self,data,path=None):
        self.data=copy.deepcopy(data);self.path=None if path is None else Path(path);self.active=False
        try:
            r=np.asarray(data['robot_rotation'],float);g=float(data['gripper_neutral'])
            if (data.get('schema')!=1 or r.shape!=(3,3) or not np.isfinite(r).all()
                    or not np.allclose(r.T@r,np.eye(3),atol=1e-6) or abs(np.linalg.det(r)-1)>1e-6
                    or not np.isfinite(g) or not 0<=g<=1):raise ValueError('invalid reference')
            h=data.get('hand_reference')
            offset=np.asarray(data.get('wrist_offset_m',[0,0,0]),float)
            if offset.shape!=(3,) or not np.isfinite(offset).all() or np.linalg.norm(offset)>.15:
                raise ValueError('invalid wrist offset')
            if data.get('orientation_mapping','pose_reference') not in ('pose_reference','pointing_roll'):
                raise ValueError('invalid orientation mapping')
            if data.get('pose_policy','position_priority') not in ('position_priority','balanced','split_roll'):
                raise ValueError('invalid pose policy')
            if data.get('gripper_mode','analog') not in ('analog','toggle'):
                raise ValueError('invalid gripper mode')
            length=float(data.get('pose_orientation_length_m',.2))
            if not np.isfinite(length) or not .01<=length<=1:raise ValueError('invalid pose task length')
            if type(data.get('workspace_auto_entry',False)) is not bool:raise ValueError('invalid automatic entry policy')
            margin=float(data.get('workspace_margin_m',.015))
            if not np.isfinite(margin) or not .005<=margin<=.04:raise ValueError('invalid workspace margin')
            ready_margin=float(data.get('workspace_ready_margin_m',margin))
            entry_margin=float(data.get('workspace_entry_margin_m',max(margin+.010,ready_margin+.005)))
            if (not np.isfinite([ready_margin,entry_margin]).all() or not margin<=ready_margin<=.06
                    or not ready_margin+.005-1e-9<=entry_margin<=.10):raise ValueError('invalid preparation margins')
            lead=data.get('workspace_entry_lead_counts',24)
            if type(lead) is not int or not 24<=lead<=96:raise ValueError('invalid preparation lead')
            roll_adjustment=float(data.get('tool_roll_adjustment_deg',0.))
            if not np.isfinite(roll_adjustment) or abs(roll_adjustment)>180:raise ValueError('invalid tool roll adjustment')
            if h is not None:
                q=np.asarray(h['quaternion'],float)
                if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-5:raise ValueError('invalid hand reference')
        except (KeyError,TypeError,ValueError) as exc:raise ValueError('遥操作参考文件无效') from exc

    @classmethod
    def load(cls,path,signature,calibration_sha256):
        data=json.loads(Path(path).read_text(encoding='utf-8'))
        if data.get('signature')!=signature or data.get('calibration_sha256')!=calibration_sha256:
            raise ValueError('遥操作参考与控制板或舵机标定不匹配，请重新记录')
        return cls(data,path)

    @property
    def pending(self):return not self.active

    def deactivate(self):self.active=False

    def capture_hand(self,quaternion,reference_space,space_epoch):
        q=np.asarray(quaternion,float)
        if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-5:
            raise ValueError('竖直手柄参考需要有效单位四元数')
        candidate=copy.deepcopy(self.data)
        candidate['hand_reference']=dict(quaternion=q.tolist(),reference_space=reference_space,space_epoch=space_epoch)
        if self.path:
            try:
                self.path.parent.mkdir(parents=True,exist_ok=True)
                temporary=self.path.with_suffix(self.path.suffix+'.tmp')
                temporary.write_text(json.dumps(candidate,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
                temporary.replace(self.path)
            except OSError as exc:raise ValueError('参考保存失败，请检查配置文件写入权限：'+str(exc)) from exc
        self.data=candidate;self.active=True

    def alignment(self):
        if self.pending:raise ValueError('请先记录手柄竖直参考')
        hand=Rotation.from_quat(self.data['hand_reference']['quaternion']).as_matrix()
        roll_adjustment=Rotation.from_euler('x',float(self.data.get('tool_roll_adjustment_deg',0.)),degrees=True).as_matrix()
        if self.data.get('orientation_mapping')=='pointing_roll':
            # Capture roll alone. Full reference alignment would rotate the
            # forward axis away from the hand's -Z pointing direction.
            relative=(XR_TO_ROBOT@hand@TOOL_TO_GRIP).T@np.asarray(self.data['robot_rotation'],float)
            sine=relative[2,1]-relative[1,2];cosine=relative[1,1]+relative[2,2]
            theta=np.arctan2(sine,cosine) if np.hypot(sine,cosine)>1e-9 else 0.
            return TOOL_TO_GRIP@Rotation.from_rotvec([theta,0,0]).as_matrix()@roll_adjustment
        return (XR_TO_ROBOT@hand).T@np.asarray(self.data['robot_rotation'],float)@roll_adjustment

    @property
    def wrist_offset(self):return np.asarray(self.data.get('wrist_offset_m',[0,0,0]),float).copy()

    def gripper_target(self,trigger):
        if not np.isfinite(trigger) or not 0<=trigger<=1:raise ValueError('invalid trigger')
        return float(self.data['gripper_neutral'])*(1-trigger)

    def status(self):
        return dict(hand_required=self.pending,hand_recorded=self.data.get('hand_reference') is not None,
                    wrist_offset_m=self.wrist_offset.tolist(),orientation_mapping=self.data.get('orientation_mapping','pose_reference'),
                    pose_policy=self.data.get('pose_policy','position_priority'),
                    pose_orientation_length_m=float(self.data.get('pose_orientation_length_m',.2)),
                    tool_roll_adjustment_deg=float(self.data.get('tool_roll_adjustment_deg',0.)),
                    gripper_mode=self.data.get('gripper_mode','analog'),
                    gripper_neutral=float(self.data['gripper_neutral']),robot_captured_at=self.data.get('captured_at'))
