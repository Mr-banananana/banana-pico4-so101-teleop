"""PICO/SO101 commissioning: enumerate board, optionally read STS registers.

No SDK robot.connect(), torque change, configuration write or motion path.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import sys

from mujoco_pick_place.pico_teleop.hardware import inspect_motors,STSRegisterReader

ROOT=Path(__file__).resolve().parents[1]


def load_sdk():
    source=json.loads((ROOT/'vendor/ftservo/source.json').read_text(encoding='utf-8-sig'))
    for row in source['files']:
        if hashlib.sha256((ROOT/'vendor/ftservo'/row['path']).read_bytes()).hexdigest()!=row['sha256']:
            raise ValueError('pinned servo SDK modified')
    sys.path.insert(0,str(ROOT/'vendor/ftservo'))
    from scservo_sdk import PortHandler,sms_sts
    return PortHandler,lambda port:STSRegisterReader(sms_sts(port))


def _candidate_port(port):
    description=(getattr(port,'description','') or '').lower()
    manufacturer=(getattr(port,'manufacturer','') or '').lower()
    return (getattr(port,'vid',None) in (0x1A86,0x10C4,0x0403,0x067B)
            or any(mark in description+' '+manufacturer for mark in ('ch340','ch343','wch','ftdi','silicon labs','usb serial','usb-to-uart')))


def _identity_key(port):
    vid=int(getattr(port,'vid',None) or 0);pid=int(getattr(port,'pid',None) or 0)
    serial=getattr(port,'serial_number',None)
    location=getattr(port,'location',None)
    if serial:return f'{vid:04X}:{pid:04X}:serial:{serial}'
    if location:return f'{vid:04X}:{pid:04X}:location:{location}'
    return f'{vid:04X}:{pid:04X}:port:{port.device}'


def _supply_from_feedback(report):
    values=[]
    for motor in report['motors']:
        value=motor['registers'].get('present_voltage',{}).get('value_raw')
        if type(value) is not int:raise ValueError('六轴电压读数不完整，无法识别供电版本')
        values.append(value)
    matches=[]
    for version,(low,high) in (('12V',(90,126)),('7.4V',(40,74))):
        if all(low<=value<=high for value in values):matches.append(version)
    if len(matches)!=1:
        raise ValueError('六轴电压读数不一致或超出已支持范围；保持只读，请核对电机和电源')
    return matches[0]


def discover_motor_bus(ports=None):
    """Find exactly one complete six-servo bus using bounded read-only probes."""
    if ports is None:
        from serial.tools.list_ports import comports
        ports=list(comports())
    candidates=[p for p in ports if _candidate_port(p)]
    if not candidates:raise ValueError('未发现支持的 USB 串口控制板')
    PortHandler,Packet=load_sdk();found=[];failures=[]
    for board in candidates:
        port=PortHandler(board.device)
        try:
            if not port.openPort():
                failures.append(f'{board.device} 无法打开');continue
            report=inspect_motors(Packet(port))
            ids=report.get('responding_ids',[])
            if not report.get('feedback_complete') or ids!=[1,2,3,4,5,6]:
                failures.append(f'{board.device} 未读到完整 ID1～6');continue
            registers=[motor['registers'] for motor in report['motors']]
            required=('model','operating_mode','homing_offset','min_position','max_position')
            if any(any(not regs.get(name,{}).get('valid') for name in required) for regs in registers):
                failures.append(f'{board.device} 电机配置寄存器不完整');continue
            if any(regs['model']['value_raw']!=777 or regs['operating_mode']['value_raw']!=0 for regs in registers):
                failures.append(f'{board.device} 电机型号或位置模式不匹配');continue
            supply=_supply_from_feedback(report)
            signature=dict(board=_identity_key(board),models=[r['model']['value_raw'] for r in registers],
                           offsets=[r['homing_offset']['value_raw'] for r in registers])
            raw_limits=[[r['min_position']['value_raw'],r['max_position']['value_raw']] for r in registers]
            found.append(dict(port=board,signature=signature,raw_limits=raw_limits,
                              supply_version=supply,report=report))
        except Exception as exc:
            failures.append(f'{board.device}: {exc}')
        finally:
            if port.is_open:port.closePort()
    if len(found)>1:
        names=', '.join(item['port'].device for item in found)
        raise ValueError('发现多个完整的 SO101 电机总线（'+names+'）；请只连接目标机械臂后重试')
    if not found:
        detail='；'.join(failures[:4])
        raise ValueError('未找到唯一且完整的 SO101 六轴总线'+('：'+detail if detail else ''))
    return found[0]


def check(read_motors=False):
    from serial.tools.list_ports import comports
    report=dict(created_at=datetime.now(timezone.utc).isoformat(),read_only=True,
                board_connected=False,motors_checked=False,motion_ready=False,calibrated=False)
    ports=list(comports())
    report['ports']=[dict(device=p.device,description=p.description,vid=p.vid,pid=p.pid,
                          serial_number=p.serial_number) for p in ports]
    try:
        if not read_motors:
            adapters=[p.device for p in ports if _candidate_port(p)]
            report.update(board_connected=bool(adapters),candidate_ports=adapters)
            report['message']='已枚举 USB 串口；本次未读取电机。' if adapters else '未发现支持的 USB 串口控制板。'
            return report
        found=discover_motor_bus(ports)
        report.update(found['report'],board_connected=True,motors_checked=True,port=found['port'].device,
                      board_identity=found['signature']['board'],supply_version=found['supply_version'],
                      raw_limits=found['raw_limits'])
        report['message']='唯一六轴总线、型号、位置模式和供电版本已识别；仅完成只读检查，未启用运动。'
    except Exception as exc:
        report['message']='检查未完成：'+str(exc)
        report['error']=type(exc).__name__
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--read-motors',action='store_true',help='explicit bounded read of IDs1..6 at1Mbps; no writes')
    p.add_argument('--output',type=Path,default=ROOT/'artifacts/pico-teleop/hardware-check.json')
    args=p.parse_args();report=check(args.read_motors)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(report['message'])
    print('串口：'+str(report.get('port','未匹配')))
    print('回应 ID：'+str(report.get('responding_ids','未读取')))
    print('运动：未启用；标定：未完成')
    print('报告：'+str(args.output))
    return 0 if report.get('feedback_complete') or (not args.read_motors and report['board_connected']) else 2


if __name__=='__main__':raise SystemExit(main())
