"""Launch PICO simulation or explicitly selected physical commissioning."""
import argparse
import asyncio
from contextlib import nullcontext
from datetime import datetime
import json
from pathlib import Path
import secrets
from mujoco_pick_place.pico_teleop.timing import clock as time
import webbrowser

from mujoco_pick_place.pico_teleop.server import TeleopService
from mujoco_pick_place.pico_teleop.debug import APP_VERSION
from mujoco_pick_place.pico_teleop.motion import MOTION_PROFILE_NAME
from mujoco_pick_place.pico_teleop.simulation import ServoSimulationArm,SimulationArm
from mujoco_pick_place.so101.kinematics import ROOT
from scripts.pico_debug import launcher_lock


async def run(args):
    if not 10<=args.hz<=50:raise ValueError('控制频率须为10～50Hz；尚未创建设备连接')
    if type(args.port) is not int or not 0<=args.port<=65535:
        raise ValueError('端口须为0～65535；尚未创建设备连接')
    out = ROOT / "artifacts/pico-teleop"
    out.mkdir(parents=True,exist_ok=True)
    # Hold ownership until all service/backend cleanup has finished. The
    # dashboard launch lock only serializes startup and direct CLI skips it.
    ownership=launcher_lock(out/'physical-service.lock',timeout=0,
        busy_message='已有真机服务正在运行；请从原页面结束调试后再启动。') if args.real else nullcontext()
    with ownership:
        await _run_service(args,out)


async def _create_backend(factory):
    # Cancelling to_thread alone abandons its result while the constructor
    # keeps running. Retain it and clean up before releasing service ownership.
    pending=asyncio.create_task(asyncio.to_thread(factory))
    try:return await asyncio.shield(pending)
    except asyncio.CancelledError:
        async def finish():
            try:backend=await pending
            except Exception:return
            await asyncio.to_thread(backend.close)
        cleanup=asyncio.create_task(finish())
        # Further cancellation requests must also wait for resource cleanup.
        while not cleanup.done():
            try:await asyncio.shield(cleanup)
            except asyncio.CancelledError:continue
        cleanup.result()
        raise


async def _run_service(args,out):
    trace = out / ("trace-"+datetime.now().strftime("%Y%m%d-%H%M%S")+"-"+secrets.token_hex(6)+".jsonl")
    if args.virtual_hardware:
        from mujoco_pick_place.pico_teleop.virtual import create_virtual_arm,OfflineAssistant
        backend=await _create_backend(create_virtual_arm)
        assistant_factory=OfflineAssistant
    elif args.real:
        from mujoco_pick_place.pico_teleop.real import PhysicalArm
        backend=await _create_backend(PhysicalArm)
    else:backend=SimulationArm() if args.kinematic else ServoSimulationArm()
    options=dict(backend=backend,trace_path=trace if not args.no_log else None,hz=args.hz,viewer=args.viewer,debug=args.debug,
                 start_stopped=bool(getattr(args,'start_stopped',False)))
    if args.virtual_hardware:options['assistant_factory']=assistant_factory
    try:service=TeleopService(**options)
    except Exception:
        if getattr(backend,'kind',None)=='so101_real':await asyncio.to_thread(backend.close)
        raise
    session_name='virtual-session' if args.virtual_hardware else 'real-session' if args.real else 'session'
    session_path=out/(session_name+'.json')
    try:
        await service.start(port=args.port)
        metadata = dict(**service.urls(),port=service.port,backend=service.backend_kind,
                        pid=__import__('os').getpid(),started_at=time.time(),trace=str(trace),
                        version=APP_VERSION,instance_id=service.instance_id,launch_id=args.launch_id)
        metadata['is_virtual']=bool(args.virtual_hardware)
        metadata['motion_profile']=MOTION_PROFILE_NAME
        temporary=out/(session_name+'.tmp')
        temporary.write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding="utf-8")
        temporary.replace(session_path)
        print('SO101 / PICO 4 — '+('VIRTUAL MOTORS: software only; no serial/ADB.' if args.virtual_hardware else
            'PHYSICAL: user-configured automatic workspace entry; XR still requires A authorization.' if args.real and getattr(backend,'workspace_entry',None) is not None else
            'PHYSICAL: READ-ONLY until calibrated and explicitly enabled.' if args.real else 'SIMULATION ONLY.'),flush=True)
        print("Desktop: "+metadata["desktop_url"],flush=True)
        print("Headset: "+metadata["headset_url"],flush=True)
        next_step = ("USB connection is managed by the dashboard." if args.debug else
                     "VIRTUAL MOTORS: open the headset URL in PICO Browser; no ADB or serial is used." if args.virtual_hardware else
                     "For ADB-assisted PICO connection, run scripts/connect_pico.ps1.")
        print(next_step,flush=True)
        print("Ctrl+C stops this service. Kinematic preview does not validate contact/grasp physics.",flush=True)
        if args.open:webbrowser.open(metadata["desktop_url"])
        await service.wait()
    finally:
        await service.close()
        try:
            current=json.loads(session_path.read_text(encoding='utf-8'))
            if isinstance(current,dict) and current.get('instance_id')==service.instance_id:
                session_path.unlink()
        except (OSError,ValueError):pass


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",type=int,default=8443)
    parser.add_argument("--hz",type=int,default=30)
    parser.add_argument("--viewer",action="store_true",help="open native MuJoCo simulation viewer")
    parser.add_argument("--open",action="store_true",help="open desktop diagnostics in the default browser")
    parser.add_argument("--no-log",action="store_true")
    parser.add_argument('--start-stopped',action='store_true',help='preserve an existing user stop during a maintenance restart')
    parser.add_argument("--debug",action="store_true",help="manage USB and commissioning from the dashboard")
    parser.add_argument("--launch-id",default=None,help=argparse.SUPPRESS)
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--kinematic',action='store_true',help='legacy instantaneous kinematic backend for diagnosis')
    modes.add_argument('--real',action='store_true',help='registered real board; automatic entry only when authorized in the bound local control reference')
    modes.add_argument('--virtual-hardware',action='store_true',help='physical-control software with in-memory motors; no serial/ADB')
    args=parser.parse_args(argv)
    try:asyncio.run(run(args))
    except KeyboardInterrupt:pass
    except Exception as exc:
        import re
        if args.launch_id and re.fullmatch('[0-9a-f]{24}',args.launch_id):
            out=ROOT/'artifacts/pico-teleop';out.mkdir(parents=True,exist_ok=True)
            message='串口被占用或无法打开；请关闭占用它的标定程序或旧真机服务，再重试。' if 'could not open port' in str(exc) else str(exc)
            (out/('startup-'+args.launch_id+'.json')).write_text(json.dumps(dict(message=message,error=type(exc).__name__),ensure_ascii=False),encoding='utf-8')
        raise


if __name__=="__main__":main()
