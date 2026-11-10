"""Idempotent desktop launcher for the PICO commissioning dashboard."""
import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import re
import secrets
import socket
import subprocess
import sys
from mujoco_pick_place.pico_teleop.timing import clock as time
from urllib.request import urlopen
import webbrowser

from mujoco_pick_place.pico_teleop.debug import APP_VERSION
from mujoco_pick_place.pico_teleop.motion import MOTION_PROFILE_NAME,PHYSICAL_MOTION
ROOT = Path(__file__).resolve().parents[1]


def health(port):
    with urlopen(f"http://127.0.0.1:{port}/health",timeout=1) as response:
        return json.load(response)


def reusable_session(path, fetch=health):
    try:
        meta=json.loads(Path(path).read_text(encoding="utf-8"))
        port=meta["port"]
        if type(port) is not int or not 1<=port<=65535:return None
        if not re.fullmatch(rf"http://localhost:{port}/\?token=[A-Za-z0-9_-]+&role=monitor",meta["desktop_url"]):return None
        live=fetch(port)
        if not isinstance(live,dict):return None
        if (meta.get("version")==APP_VERSION==live.get("version") and
            meta.get("instance_id") and meta["instance_id"]==live.get("instance_id") and
            meta.get('backend')==live.get('backend') and
            bool(meta.get('is_virtual'))==bool(live.get('is_virtual')) and
            meta.get('motion_profile','measured')==live.get('motion_profile','measured')==MOTION_PROFILE_NAME and
            "debug_assistant_v1" in live.get("features",[])):
            return meta
    except (OSError,ValueError,KeyError,TypeError):pass
    return None


def choose_port(first):
    for port in range(first,min(first+30,65536)):
        with socket.socket() as sock:
            try:sock.bind(("127.0.0.1",port))
            except OSError:continue
            return port
    raise RuntimeError("没有空闲调试端口，请查看 artifacts/pico-teleop/launcher.log")


def owned_physical_session(path,fetch=health):
    """Recognize an older live owner without stopping it or opening its port."""
    try:
        meta=json.loads(Path(path).read_text(encoding='utf-8'));port=meta['port']
        if type(port) is not int or not 1<=port<=65535:return None
        if not re.fullmatch(rf'http://localhost:{port}/\?token=[A-Za-z0-9_-]+&role=monitor',meta['desktop_url']):return None
        live=fetch(port)
        if not isinstance(live,dict):return None
        if (meta.get('instance_id') and meta['instance_id']==live.get('instance_id')
                and meta.get('backend')==live.get('backend')=='so101_real'
                and not meta.get('is_virtual') and not live.get('is_virtual')):return meta
    except (OSError,ValueError,KeyError,TypeError):pass
    return None


@contextmanager
def launcher_lock(path, *, timeout=15, busy_message="另一个启动器仍在启动，请稍后重试"):
    with Path(path).open("a+b") as stream:
        stream.seek(0);stream.write(b"0");stream.flush()
        deadline=time.monotonic()+timeout
        while True:
            try:
                stream.seek(0)
                if os.name=="nt":
                    import msvcrt
                    msvcrt.locking(stream.fileno(),msvcrt.LK_NBLCK,1)
                else:
                    import fcntl
                    fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
                break
            except OSError:
                if time.monotonic()>=deadline:raise RuntimeError(busy_message)
                time.sleep(.15)
        try:yield
        finally:
            stream.seek(0)
            if os.name=="nt":msvcrt.locking(stream.fileno(),msvcrt.LK_UNLCK,1)
            else:fcntl.flock(stream,fcntl.LOCK_UN)


def launch(*,port=8443,open_browser=True,viewer=False,no_log=False,real=False,virtual_hardware=False):
    if real and virtual_hardware:raise ValueError('真实电机与虚拟电机模式不能同时开启')
    out=ROOT/"artifacts/pico-teleop";out.mkdir(parents=True,exist_ok=True)
    with launcher_lock(out/"launcher.lock"):
        session_path=out/('virtual-session.json' if virtual_hardware else 'real-session.json' if real else 'session.json')
        meta=reusable_session(session_path)
        if meta and ((meta.get('backend')=='so101_real')!=(real or virtual_hardware)
                     or bool(meta.get('is_virtual'))!=virtual_hardware):meta=None
        if meta is None:
            owner=owned_physical_session(session_path) if real else None
            if owner:raise RuntimeError(f"已有真机服务占用设备（端口{owner['port']}）；请从原页面结束调试后再启动新版本。")
            port=choose_port(port)
            launch_id=secrets.token_hex(12)
            executable=Path(sys.executable).with_name("python.exe") if os.name=="nt" else Path(sys.executable)
            if real:executable=ROOT/'.venv-so101-hardware/Scripts/python.exe'
            command=[str(executable),"-m","scripts.pico_teleop","--port",str(port),"--debug","--launch-id",launch_id]
            command+=['--hz',str(PHYSICAL_MOTION.control_hz)]
            if real:command.append('--real')
            if virtual_hardware:command.append('--virtual-hardware')
            if viewer:command.append("--viewer")
            if no_log:command.append("--no-log")
            flags=subprocess.CREATE_NO_WINDOW if os.name=="nt" else 0
            with (out/"service.stdout.log").open("a",encoding="utf-8") as stdout, (out/"service.stderr.log").open("a",encoding="utf-8") as stderr:
                child=subprocess.Popen(command,cwd=ROOT,stdout=stdout,stderr=stderr,stdin=subprocess.DEVNULL,creationflags=flags)
            deadline=time.monotonic()+12
            while time.monotonic()<deadline:
                if child.poll() is not None:
                    error_file=out/('startup-'+launch_id+'.json')
                    try:reason=json.loads(error_file.read_text(encoding='utf-8'))['message']
                    except (OSError,ValueError,KeyError):reason='详情见 artifacts/pico-teleop/service.stderr.log'
                    raise RuntimeError('服务启动失败：'+reason)
                candidate=reusable_session(session_path)
                if candidate and candidate.get("launch_id")==launch_id:
                    meta=candidate;break
                time.sleep(.15)
            if meta is None:
                # Windows venv python.exe is a redirector; its Python child must
                # also exit. Target only the process tree spawned above.
                if child.poll() is None:
                    if os.name=="nt":
                        subprocess.run(["taskkill","/PID",str(child.pid),"/T","/F"],
                            stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
                            creationflags=subprocess.CREATE_NO_WINDOW,timeout=5,check=False)
                    else:child.terminate()
                    child.wait(timeout=3)
                raise RuntimeError("启动超时，详情见 artifacts/pico-teleop/service.stderr.log")
    if open_browser:webbrowser.open(meta["desktop_url"])
    return meta


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",type=int,default=8443)
    parser.add_argument("--no-browser",action="store_true")
    parser.add_argument("--viewer",action="store_true")
    parser.add_argument("--no-log",action="store_true")
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--real',action='store_true')
    modes.add_argument('--virtual-hardware',action='store_true')
    args=parser.parse_args()
    try:
        if not 1024<=args.port<=65535:raise ValueError("端口须为 1024..65535")
        port=8445 if args.virtual_hardware and args.port==8443 else 8444 if args.real and args.port==8443 else args.port
        result=launch(port=port,open_browser=not args.no_browser,viewer=args.viewer,no_log=args.no_log,real=args.real,virtual_hardware=args.virtual_hardware)
        if sys.stdout:print(result["desktop_url"])
    except Exception:
        import traceback
        out=ROOT/"artifacts/pico-teleop";out.mkdir(parents=True,exist_ok=True)
        log=out/"launcher.log";log.write_text(traceback.format_exc(),encoding="utf-8")
        if sys.stderr:traceback.print_exc()
        if os.name=="nt" and sys.stderr is None:
            import ctypes
            ctypes.windll.user32.MessageBoxW(0,f"启动失败。错误已保存到：\n{log}","PICO 调试",0x10)
        return 1
    return 0


if __name__=="__main__":raise SystemExit(main())
