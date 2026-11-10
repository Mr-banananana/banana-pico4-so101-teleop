"""Loopback XR service with separate simulation and commissioned physical I/O."""
import asyncio
import contextlib
from dataclasses import dataclass
from http import HTTPStatus
import json
import io
import platform
from pathlib import Path
import queue
import secrets
import threading
from .timing import clock as time,sleep_until
import zipfile
from urllib.parse import parse_qs, urlsplit

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Response
from websockets.datastructures import Headers

from mujoco_pick_place.so101.kinematics import ROOT
from .controller import TeleopController
from .simulation import SimulationArm
from .adb import AdbAssistant
from .debug import APP_VERSION, self_test
from .motion import MOTION_PROFILE_NAME


def encode(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def token_matches(candidate,expected):
    return isinstance(candidate,str) and candidate.isascii() and secrets.compare_digest(candidate,expected)


class TraceWriter:
    def __init__(self, path, *, max_bytes=20*1024*1024, backup_count=2):
        if type(max_bytes) is not int or max_bytes<512 or type(backup_count) is not int or not 1<=backup_count<=5:
            raise ValueError('日志容量须至少512字节，轮换保留1～5份')
        self.path = Path(path) if path else None
        self.max_bytes=max_bytes;self.backup_count=backup_count;self.rotations=0
        self.items = queue.Queue(maxsize=2048)
        self.dropped = 0
        self.error = None
        self.done = threading.Event()
        self.thread = None
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.thread = threading.Thread(target=self._write, daemon=True)
            self.thread.start()

    def add(self, record):
        if self.path and self.error is None and not self.done.is_set():
            try:self.items.put_nowait(record)
            except queue.Full:self.dropped += 1

    def _write(self):
        stream=None
        try:
            size=self.path.stat().st_size if self.path.exists() else 0
            if size>self.max_bytes:raise ValueError('已有日志超出容量上限，已保留原文件并停写；请使用新的日志文件名')
            stream=self.path.open('ab')
            while not self.done.is_set() or not self.items.empty():
                try:record = self.items.get(timeout=.1)
                except queue.Empty:continue
                line=(encode(record)+'\n').encode('utf-8')
                if len(line)>self.max_bytes:
                    self.dropped+=1;continue
                if size+len(line)>self.max_bytes:
                    stream.close()
                    for index in range(self.backup_count,1,-1):
                        previous=Path(str(self.path)+'.'+str(index-1))
                        if previous.exists():previous.replace(Path(str(self.path)+'.'+str(index)))
                    self.path.replace(Path(str(self.path)+'.1'))
                    stream=self.path.open('wb');size=0;self.rotations+=1
                stream.write(line);size+=len(line)
                if self.items.empty():stream.flush()
        except Exception as exc:self.error = str(exc)
        finally:
            if stream:
                with contextlib.suppress(OSError):stream.close()

    def close(self):
        self.done.set()
        if self.thread:self.thread.join(timeout=2)


@dataclass(eq=False)
class Peer:
    ws: object
    role: str
    updates: asyncio.Queue


class TeleopService:
    def __init__(self, *, backend=None, trace_path=None, hz=30, viewer=False,
                 debug=False, assistant_factory=AdbAssistant,start_stopped=False):
        if not 10 <= hz <= 50:
            raise ValueError("simulation frequency must be 10..50 Hz")
        arm=backend or SimulationArm()
        self.backend_kind=getattr(arm,'kind','kinematic_simulation')
        self.controller = TeleopController(arm,allow_emulated=isinstance(arm,SimulationArm))
        if start_stopped:self.controller.stop('更新前的停止状态已保留；长按 A 解除后才能准备和授权')
        self.controller.wall_clock = time.monotonic
        if self.controller.physical:
            arm.commit_guard=lambda:bool(self.controller.armed and not self.controller.stopped
                and self.controller.fresh(time.monotonic()) and self.controller.latest.valid
                and (not self.controller.require_passthrough or self.controller.latest.mode=='immersive-ar')
                and self.controller.latest.visible and not self.controller.latest.emulated and arm.ready)
        self.token = secrets.token_urlsafe(24)
        self.hz = hz
        self.peers = set()
        self.source = None
        self.server = None
        self.port = None
        self.worker = None
        self.view = None
        self.want_viewer = viewer
        self.trace = TraceWriter(trace_path)
        self.diagnostics = {}
        self.tick_ms = 0.
        self.instance_id = secrets.token_hex(12)
        self.debug = debug
        self.assistant_factory = assistant_factory
        self.assistant = None
        self.assistant_task = None
        self.shutdown = asyncio.Event()
        self.self_test_result = None
        self.self_test_busy = False
        self.demo_task=None;self.demo_result=None

    def _response(self, code, body, content_type="text/plain; charset=utf-8"):
        if isinstance(body, str):body = body.encode("utf-8")
        return Response(code, HTTPStatus(code).phrase,
                        Headers({"Content-Type":content_type,"Content-Length":str(len(body)),
                                 "Cache-Control":"no-store","X-Content-Type-Options":"nosniff"}), body)

    async def _http(self, connection, request):
        url = urlsplit(request.path)
        if url.path == "/ws":
            query = parse_qs(url.query)
            if not token_matches(query.get("token", [""])[0], self.token):
                return self._response(403, "Session token required")
            origin = request.headers.get("Origin")
            if origin:
                try:
                    parsed = urlsplit(origin)
                    allowed=(parsed.scheme=='http' and parsed.hostname in ('localhost','127.0.0.1') and parsed.port==self.port)
                except ValueError:allowed=False
                if not allowed:
                    return self._response(403, "Unexpected Origin")
            return None
        if url.path == "/health":
            return self._response(200, encode(dict(backend=self.backend_kind, state="ready",
                is_virtual=bool(getattr(getattr(self.controller.backend,'worker',None),'bus',None)
                                and getattr(self.controller.backend.worker.bus,'is_virtual',False)),
                version=APP_VERSION,instance_id=self.instance_id,motion_profile=MOTION_PROFILE_NAME,
                features=["debug_assistant_v1"] if self.debug else [])), "application/json")
        if url.path == "/debug-report":
            if not token_matches(parse_qs(url.query).get("token",[""])[0],self.token):
                return self._response(403,"Session token required")
            report=dict(version=APP_VERSION,backend=self.backend_kind,python=platform.python_version(),
                        platform=platform.platform(),status=self.controller.status(time.monotonic()),
                        debug=self.debug_status(),diagnostics=self.diagnostics,self_test=self.self_test_result,
                        events=list(self.assistant.events) if self.assistant else [],trace_error=self.trace.error)
            payload=json.dumps(report,ensure_ascii=False,indent=2).replace(self.token,"[REDACTED]")
            buffer=io.BytesIO()
            with zipfile.ZipFile(buffer,"w",zipfile.ZIP_DEFLATED) as archive:
                archive.writestr("diagnostics.json",payload)
            response=self._response(200,buffer.getvalue(),"application/zip")
            response.headers["Content-Disposition"]='attachment; filename="pico-debug-report.zip"'
            return response
        files = {"/":("index.html","text/html; charset=utf-8"),
                 "/client.js":("client.js","text/javascript; charset=utf-8"),
                 "/input.js":("input.js","text/javascript; charset=utf-8"),
                 "/hud.js":("hud.js","text/javascript; charset=utf-8"),
                 "/scene.js":("scene.js","text/javascript; charset=utf-8"),
                 "/hardware.js":("hardware.js","text/javascript; charset=utf-8"),
                 "/xr-preview.html":("xr-preview.html","text/html; charset=utf-8"),
                 "/preview.js":("preview.js","text/javascript; charset=utf-8"),
                 "/style.css":("style.css","text/css; charset=utf-8")}
        if url.path not in files:return self._response(404, "Not found")
        name, content_type = files[url.path]
        path = ROOT / "web/pico-teleop" / name
        if not path.is_file():return self._response(503, "Diagnostic UI is not installed")
        return self._response(200, path.read_bytes(), content_type)

    async def start(self, *, port=8443):
        self.server = await serve(self._handle, "127.0.0.1", port,
                                  process_request=self._http, max_size=16384, max_queue=8,
                                  compression=None, ping_interval=5, ping_timeout=5)
        self.port = self.server.sockets[0].getsockname()[1]
        if self.debug:
            options={};preference=ROOT/'configs/pico-client.local.json'
            if self.assistant_factory is AdbAssistant and preference.is_file():
                settings=json.loads(preference.read_text(encoding='utf-8-sig'))
                if not isinstance(settings,dict):raise ValueError('头显客户端配置必须是对象')
                options['browser_package']=settings.get('browser_package')
            self.assistant = self.assistant_factory(self.urls()["headset_url"],self.port,**options)
            self.assistant_task = asyncio.create_task(self._assist())
        if self.want_viewer:
            self.open_viewer()
        self.worker = asyncio.create_task(self._run(), name="pico-simulation-control")

    def urls(self):
        base = f"http://localhost:{self.port}/?token={self.token}"
        return dict(desktop_url=base+"&role=monitor",headset_url=base+"&role=input")

    async def _updates(self, peer):
        try:
            while True:
                status = await peer.updates.get()
                await asyncio.wait_for(peer.ws.send(encode(status)), .5)
        except (ConnectionClosed, TimeoutError):
            await peer.ws.close()
        except (TypeError,ValueError):
            self.controller.stop('操作状态无法更新，停止已锁存；请查看电脑诊断')
            await peer.ws.close(code=1011,reason='invalid control status')

    async def _handle(self, ws):
        query = parse_qs(urlsplit(ws.request.path).query)
        role = query.get("role", ["monitor"])[0]
        if role not in ("monitor", "input"):
            await ws.close(code=1008, reason="invalid role");return
        if role == "input" and self.source is not None:
            await ws.send(encode(dict(type="error",code="input_busy",message="另一台输入客户端已连接，请先断开它")))
            await ws.close(code=1013, reason="input already connected");return
        peer = Peer(ws, role, asyncio.Queue(maxsize=1))
        session_id = None
        if role == "input":
            self.source = peer
            session_id = secrets.token_hex(12)
            self.controller.attach(session_id)
            self.diagnostics = {}
        self.peers.add(peer)
        sender = asyncio.create_task(self._updates(peer))
        clock_task = None
        try:
            await ws.send(encode(dict(type="hello",role=role,session_id=session_id,
                                      backend=self.backend_kind,hz=self.hz)))
            if role == "input":
                clock_task = asyncio.create_task(self._clock_source(peer))
            async for raw in ws:
                try:
                    message = json.loads(raw)
                    if not isinstance(message, dict):raise ValueError("object required")
                    # Python's JSON decoder accepts NaN/Infinity and overflowing
                    # exponent literals. Reject them even in unused fields so
                    # accepted packets cannot poison status or the trace writer.
                    try:encode(message)
                    except (ValueError,TypeError,RecursionError):raise ValueError('输入消息包含非法数值或结构') from None
                    now = time.monotonic()
                    kind = message.get("type")
                    if kind == "control" and role == "monitor":
                        action = message.get("action")
                        if self.controller.physical and (str(action).startswith('cal_') or str(action).startswith('hardware_')):
                            if action=='hardware_disable':self.controller.stop('已退出真机接管，停止已锁存')
                            elif self.controller.armed:raise ValueError('先停止并撤销手柄授权')
                            self.controller.backend.control(action,message)
                        elif action == "arm":self.controller.arm(now)
                        elif action == "stop":self.controller.stop()
                        elif action == "reset_stop":self.controller.reset_stop()
                        elif action in ("position", "pose"):self.controller.set_mode(action)
                        elif self.debug and action in ("reconnect_pico","select_device"):
                            if action=="select_device":self.assistant.select(message.get("serial"))
                            self.controller.revoke("重新连接头显，需要重新授权")
                            if self.source:await self.source.ws.close(code=4000,reason="reopen headset page")
                            self.assistant.retry()
                        elif self.debug and action == "self_test":
                            result=await self.run_self_test()
                            await ws.send(encode(dict(type="self_test_result",**result)))
                        elif action=='virtual_demo':
                            bus=getattr(getattr(self.controller.backend,'worker',None),'bus',None)
                            if not getattr(bus,'is_virtual',False):raise ValueError('此演示仅允许虚拟舵机后端')
                            if self.source is not None or self.demo_task and not self.demo_task.done():raise ValueError('输入已被占用或演示正在运行')
                            self.demo_result=dict(running=True,scope='software_only')
                            self.demo_task=asyncio.create_task(self.run_virtual_demo())
                        elif self.debug and action == "open_viewer":self.open_viewer()
                        elif self.debug and action == "shutdown":
                            self.controller.stop("调试已结束")
                            await ws.send(encode(dict(type="control_result",ok=True,action=action)))
                            for other in tuple(self.peers):
                                if other is not peer:
                                    with contextlib.suppress(ConnectionClosed,TimeoutError):
                                        await asyncio.wait_for(other.ws.send(encode(dict(type="service_stopping"))),.2)
                            self.shutdown.set()
                            continue
                        else:raise ValueError("unknown control")
                        await ws.send(encode(dict(type="control_result",ok=True,action=action)))
                    elif role == "input" and self.source is peer:
                        if kind == "clock_reply":
                            self.controller.clock.accept(message, now)
                            await ws.send(encode(dict(type="clock_ok")))
                        elif kind == "frame":
                            self.controller.ingest(message, now)
                            await ws.send(encode(dict(type="frame_ok",seq=message["seq"])))
                            self.trace.add(dict(type="input",server_time=now,payload=message))
                        elif kind == "stop":self.controller.stop("头显客户端发出停止")
                        elif kind == "revoke":
                            reason={
                                'reference_reset':'头显坐标原点已重置：跟随已暂停；松开握持键和扳机，再短按 A 重新授权',
                                'xr_started':'已进入新的头显模式：松开握持键和扳机，再短按 A 授权',
                                'xr_ended':'已退出头显模式：跟随已暂停；重新进入后再短按 A 授权',
                                'xr_hidden':'头显应用暂时失去焦点：跟随已暂停；返回后松开按键，再短按 A 授权',
                                'page_hidden':'头显网页已切到后台：跟随已暂停；返回后松开按键，再短按 A 授权',
                                'page_left':'头显网页已关闭或跳转：跟随已暂停；重新进入后再授权',
                                'mapping_changed':'手柄按键映射已更改：松开按键，再短按 A 重新授权',
                                'render_failed':'头显三维绘制失败：跟随已暂停；请查看兼容性诊断',
                            }.get(str(message.get('reason','')),'头显输入已暂停：返回头显，松开握持键和扳机，再短按 A 授权')
                            self.controller.revoke(reason)
                            if message.get('reason')=='xr_started':
                                # The new XR session supplies its own first pose.
                                # Keep sequence/clock checks across re-entry, but
                                # do not compare this epoch with an ended session.
                                self.controller.latest=None
                        elif kind == "diagnostic":
                            details=message.get('details',{})
                            if not isinstance(details,dict):raise ValueError('诊断数据必须是对象')
                            self.diagnostics = details
                        else:raise ValueError("unknown input message")
                    else:raise ValueError("unsupported message")
                except (ValueError, TypeError, KeyError, RuntimeError) as exc:
                    if role == "input":self.controller.revoke(f"输入错误：{exc}")
                    await ws.send(encode(dict(type="error",code="rejected",message=str(exc))))
        except ConnectionClosed:pass
        finally:
            self.peers.discard(peer)
            if self.source is peer:
                self.source = None
                self.controller.detach(session_id)
            sender.cancel()
            with contextlib.suppress(asyncio.CancelledError, ConnectionClosed):await sender
            if clock_task:
                clock_task.cancel()
                with contextlib.suppress(asyncio.CancelledError, ConnectionClosed):await clock_task

    async def _clock_source(self, peer):
        # Network backpressure must never block or terminate the control task.
        while self.source is peer:
            try:
                challenge = self.controller.clock.challenge(time.monotonic())
                await asyncio.wait_for(peer.ws.send(encode(challenge)), .2)
            except (ConnectionClosed, TimeoutError):
                self.controller.revoke("时钟发送失败，输入已撤权；请重新连接")
                with contextlib.suppress(ConnectionClosed, TimeoutError):
                    await asyncio.wait_for(peer.ws.close(), .5)
                return
            await asyncio.sleep(2)

    async def wait(self):
        shutdown_task=asyncio.create_task(self.shutdown.wait())
        try:
            done,_=await asyncio.wait([self.worker,shutdown_task],return_when=asyncio.FIRST_COMPLETED)
            if self.worker in done:await self.worker
        except Exception:
            self.controller.stop("控制任务异常退出，服务已停止")
            raise
        finally:
            shutdown_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):await shutdown_task

    def open_viewer(self):
        if self.view is not None and self.view.is_running():return
        import mujoco.viewer
        arm=self.controller.backend
        self.view=mujoco.viewer.launch_passive(arm.model,arm.data)
        self.view.cam.lookat[:]=[.16,0,.13]
        self.view.cam.distance=.75;self.view.cam.azimuth=135;self.view.cam.elevation=-25

    async def _assist(self):
        while True:
            try:await self.assistant.poll(source_connected=self.source is not None)
            except Exception as exc:self.diagnostics["adb_error"]=str(exc)
            await asyncio.sleep(2)

    async def run_self_test(self):
        if self.self_test_busy:raise ValueError("自检正在运行，请稍候")
        self.self_test_busy=True
        try:
            self.self_test_result=await asyncio.to_thread(self_test)
            return self.self_test_result
        finally:self.self_test_busy=False

    async def run_virtual_demo(self):
        from .software_demo import run_demo
        try:self.demo_result=await run_demo(self.urls()['desktop_url'])
        except Exception as exc:self.demo_result=dict(ok=False,running=False,scope='software_only',message=str(exc))
        path=ROOT/'artifacts/pico-teleop/software-demo-latest.json'
        path.parent.mkdir(parents=True,exist_ok=True)
        await asyncio.to_thread(path.write_text,json.dumps(self.demo_result,ensure_ascii=False,indent=2),encoding='utf-8')
        return self.demo_result

    def debug_status(self):
        info=self.assistant.status() if self.assistant else dict(enabled=False)
        fresh=self.controller.fresh(time.monotonic())
        valid=bool(fresh and self.controller.latest.valid and self.controller.latest.visible)
        connected=self.source is not None
        info["steps"]=[dict(label="电脑服务",done=True),
                       dict(label="USB 已授权",done=any(d.get("state")=="device" and
                            d.get("serial")==info.get("selected_serial") for d in info.get("devices",[]))),
                       dict(label="头显页面",done=connected),dict(label="手柄输入",done=valid)]
        if connected:
            if valid and self.controller.require_passthrough and self.controller.latest.mode!='immersive-ar':
                info['message']='当前是不透明 VR；请退出并进入透视遥操作，先确认看得见真实机械臂。VR 预览不能授权真机跟随。'
            elif valid and self.controller.stopped:
                info["message"]="已停止：松开右握持键、扳机和 B，长按右 A 两秒解除；松开 A，再短按 A 授权。"
            elif valid and self.controller.armed:
                info["message"]="右手柄跟随已授权：握住 grip 移动，扳机控制夹爪，按住 A 精细操作，B 停止。"
            elif valid:
                info["message"]="只需右手柄：松开 A、握持键与扳机，再短按右 A 授权；握住 grip 移动。"
            else:
                info["message"]=("仿真模式：在头显页面点击「VR 仿真预览」，进入虚拟场景后检查右手柄追踪。"
                                  if not self.controller.require_passthrough else
                                  "真机模式：点击「进入透视遥操作」，确认看得见真实机械臂后再检查右手柄追踪。")
            if valid and self.controller.latest.emulated:
                info['message']+=' 当前使用浏览器估算位姿，仅允许仿真，真实机械臂禁用。'
        info["self_test_busy"]=self.self_test_busy
        if getattr(getattr(self.controller.backend,'worker',None),'bus',None) and getattr(self.controller.backend.worker.bus,'is_virtual',False):
            info['steps']=[dict(label='本地软件服务',done=True),dict(label='虚拟舵机反馈',done=self.controller.backend.feedback_valid),
                           dict(label='实际串口未打开',done=True),dict(label='虚拟手柄演示',done=bool(self.demo_result and self.demo_result.get('ok')))]
            info['message']='软件验证模式：无需机械臂或头显，点击「虚拟手柄演示」运行授权、跟随和停止。'
        return info

    async def _run(self):
        period = 1/self.hz
        last = time.monotonic()
        next_status = 0.
        status_period=1/min(self.hz,30)
        while True:
            now = time.monotonic()
            dt, last = now-last, now
            began = time.perf_counter()
            try:
                if self.controller.physical:self.controller.backend.poll()
                if self.controller.motor_response:
                    if (not self.controller.armed or not self.controller.engaged or self.controller.stopped
                            or not self.controller.fresh(now)):self.controller.backend.hold()
                    self.controller.backend.advance(max(dt,.001))
                status = self.controller.tick(now, max(dt, .001))
            except Exception as exc:
                self.controller.stop(f"控制异常，已停止：{exc}")
                status = self.controller.status(now)
            self.tick_ms = (time.perf_counter()-began)*1000
            status.update(tick_ms=round(self.tick_ms,2),hz=self.hz,diagnostics=self.diagnostics,
                          software_demo=None if self.demo_result is None else {k:v for k,v in self.demo_result.items() if k!='samples'},
                          trace_dropped=self.trace.dropped,trace_error=self.trace.error)
            if self.debug:status["debug"]=self.debug_status()
            self.trace.add(dict(type="tick",server_time=now,status=status))
            if self.view is not None:
                if self.view.is_running():self.view.sync()
                else:self.view.close();self.view=None
            if now>=next_status:
                for peer in tuple(self.peers):
                    if peer.updates.full():
                        with contextlib.suppress(asyncio.QueueEmpty):peer.updates.get_nowait()
                    peer.updates.put_nowait(status)
                next_status+=status_period
                if next_status<=now:next_status=now+status_period
            await sleep_until(now+period)

    async def close(self):
        self.controller.stop("服务关闭")
        if self.demo_task and not self.demo_task.done():
            self.demo_task.cancel()
            with contextlib.suppress(asyncio.CancelledError,Exception):await self.demo_task
        if self.assistant_task:
            self.assistant_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):await self.assistant_task
        if self.worker:
            self.worker.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):await self.worker
        if self.server:
            self.server.close();await self.server.wait_closed()
        if self.view:self.view.close();self.view=None
        await asyncio.to_thread(self.trace.close)
        if self.controller.physical:await asyncio.to_thread(self.controller.backend.close)
