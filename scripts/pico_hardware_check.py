"""PICO/SO101 commissioning: enumerate board, optionally read STS registers.

No SDK robot.connect(), torque change, configuration write or motion path.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import sys

from mujoco_pick_place.pico_teleop.hardware import select_board,inspect_motors,STSRegisterReader

ROOT=Path(__file__).resolve().parents[1]


def load_sdk():
    source=json.loads((ROOT/'vendor/ftservo/source.json').read_text(encoding='utf-8-sig'))
    for row in source['files']:
        if hashlib.sha256((ROOT/'vendor/ftservo'/row['path']).read_bytes()).hexdigest()!=row['sha256']:
            raise ValueError('pinned servo SDK modified')
    sys.path.insert(0,str(ROOT/'vendor/ftservo'))
    from scservo_sdk import PortHandler,sms_sts
    return PortHandler,lambda port:STSRegisterReader(sms_sts(port))


def check(read_motors=False):
    from serial.tools.list_ports import comports
    config=json.loads((ROOT/'configs/pico-hardware.local.json').read_text(encoding='utf-8-sig'))
    report=dict(created_at=datetime.now(timezone.utc).isoformat(),read_only=True,
                board_connected=False,motors_checked=False,motion_ready=False,calibrated=False,
                user_confirmed_connection=config['user_confirmed_connection'])
    ports=list(comports())
    report['ports']=[dict(device=p.device,description=p.description,vid=p.vid,pid=p.pid,
                          serial_number=p.serial_number) for p in ports]
    try:
        board=select_board(ports,config['board'])
        report.update(board_connected=True,port=board.device)
        if not read_motors:
            report['message']='控制板已识别；本次只枚举 USB，尚未检查电机。'
            return report
        PortHandler,Packet=load_sdk();port=PortHandler(board.device)
        try:
            if not port.openPort():raise RuntimeError('无法打开控制板串口')
            # PortHandler defaults to 1Mbps; openPort only sets host transport.
            report.update(inspect_motors(Packet(port)))
            report['motors_checked']=True
        finally:
            if port.is_open:port.closePort()
        if report['feedback_complete']:
            report['message']='六路基础反馈可读；仍需确认型号、关节映射、零位、方向与限位，未启用运动。'
        elif not report['responding_ids']:
            report['message']='控制板已连接，ID1～6 均未回应。仅接 USB 板时正常；接电机后核对电源、总线、跳线、ID 和波特率。'
        else:
            report['message']='仅部分反馈有效；检查离线 ID 或设备错误，未启用运动。'
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
