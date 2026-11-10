"""Bounded, non-interactive USB assistant. Never grants device permissions."""
import asyncio
import contextlib
from collections import deque
import json
import os
import re
import shutil
import subprocess
from .timing import clock as time
from urllib.request import build_opener, ProxyHandler

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException


async def _debug_pages(port):
    def read():
        # Never send the local debug request through an environment proxy.
        with build_opener(ProxyHandler({})).open(f'http://127.0.0.1:{port}/json/list',timeout=2) as response:
            data=response.read(1_000_001)
        if len(data)>1_000_000:raise ValueError('browser target list too large')
        pages=json.loads(data)
        if not isinstance(pages,list):raise ValueError('invalid browser target list')
        return pages
    return await asyncio.to_thread(read)


async def _debug_request(ws,request_id,method,params):
    async with asyncio.timeout(2):
        await ws.send(json.dumps(dict(id=request_id,method=method,params=params)))
        while True:
            reply=json.loads(await ws.recv())
            if not isinstance(reply,dict):return None
            if reply.get('id')==request_id:
                result=reply.get('result')
                return result if not reply.get('error') and isinstance(result,dict) else None


async def _debug_page_location(port,target_id):
    # PICO's /json/list URL can remain stale after Page.navigate. Read only
    # live location/visibility, without reading DOM contents or changing the page.
    async with connect(f'ws://127.0.0.1:{port}/devtools/page/{target_id}',
                       open_timeout=2,close_timeout=.2,max_size=1_000_000,proxy=None) as ws:
        result=await _debug_request(ws,1,'Runtime.evaluate',dict(
            expression='({href:location.href,visible:document.visibilityState==="visible"})',returnByValue=True))
        remote=result.get('result') if isinstance(result,dict) and not result.get('exceptionDetails') else None
        value=remote.get('value') if isinstance(remote,dict) else None
        return value if isinstance(value,dict) and isinstance(value.get('href'),str) else None


async def _navigate_debug_page(port,target_id,url,*,guard=lambda:True,navigate=True):
    # Construct the local endpoint ourselves, never trust a returned debugger URL.
    async with connect(f'ws://127.0.0.1:{port}/devtools/page/{target_id}',
                       open_timeout=2,close_timeout=.2,max_size=1_000_000,proxy=None) as ws:
        if not guard():return False
        if navigate:
            result=await _debug_request(ws,1,'Page.navigate',dict(url=url))
            if not (result and isinstance(result.get('frameId'),str) and result['frameId']
                    and not result.get('errorText')):return False
        if not guard():return False
        # A successfully connected hidden tab is not the page the wearer sees.
        return await _debug_request(ws,2,'Page.bringToFront',{}) is not None


def parse_devices(output):
    result = []
    for line in output.splitlines():
        match = re.match(r'^([A-Za-z0-9._:-]+)\s+(device|unauthorized|offline)\b(.*)$', line.strip())
        if not match:
            continue
        serial, state, rest = match.groups()
        details = dict(re.findall(r'(\w+):([^\s]+)', rest))
        result.append(dict(serial=serial, state=state, model=details.get('model', ''),
                           transport_id=details.get('transport_id', '')))
    return result


class AdbAssistant:
    def __init__(self, headset_url, port, *, runner=None,browser_package=None):
        if not re.fullmatch(r'http://localhost:'+str(port)+r'/\?token=[A-Za-z0-9_-]+&role=input', headset_url):
            raise ValueError('invalid local headset URL')
        if browser_package not in (None,'com.igalia.wolvic','com.pico.browser'):raise ValueError('头显浏览器包名未经验证')
        self.browser_package=browser_package
        self.url, self.port = headset_url, port
        self.executable = shutil.which('adb')
        self.runner = runner or self._invoke
        self.available = runner is not None or self.executable is not None
        self.devices = []
        self.selected = None
        self.opened_for = None
        self.force_open = False
        self.configured_for = None
        self.generation = 0
        self.retry_at = 0.
        self.state = 'checking'
        self.message = '正在检查 USB，连接头显后会自动继续。'
        self.last_error = None
        self.events = deque(maxlen=30)
        self.changed_at = time.time()

    def _set(self, state, message, error=None):
        if state != self.state or message != self.message:
            self.events.append(dict(time=time.time(),state=state,message=message))
            self.changed_at = time.time()
        self.state, self.message, self.last_error = state, message, error

    def status(self):
        return dict(enabled=True,state=self.state,message=self.message,
                    devices=[dict(d) for d in self.devices],selected_serial=self.selected,
                    last_error=self.last_error,changed_at=self.changed_at,browser_package=self.browser_package)

    def select(self, serial):
        if serial not in [d['serial'] for d in self.devices]:
            raise ValueError('请选择当前列表中的头显')
        self.selected = serial
        self.retry()

    def retry(self):
        self.generation += 1
        self.opened_for = None
        self.force_open = True
        self.retry_at = 0.
        self._set('checking','正在重新检查，随后会重新打开头显页面。')

    async def _invoke(self, args):
        flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
        try:
            process = await asyncio.create_subprocess_exec(
                self.executable,*args,stdout=asyncio.subprocess.PIPE,stderr=asyncio.subprocess.STDOUT,
                creationflags=flags)
            try:
                output,_ = await asyncio.wait_for(process.communicate(),5)
            except (TimeoutError,asyncio.CancelledError):
                with contextlib.suppress(ProcessLookupError):process.kill()
                with contextlib.suppress(TimeoutError):await asyncio.wait_for(process.communicate(),1)
                if asyncio.current_task().cancelling():
                    raise
                return 124,'ADB 操作超时，请检查数据线；系统会自动重试。'
            return process.returncode,output.decode('utf-8',errors='replace').strip()
        except OSError as exc:
            return 127,str(exc)

    async def _refresh_native_page(self,serial,url,generation):
        """Repair PICO's retained old tab after VIEW is accepted but ignored.

        Only navigate this app's exact localhost input page on this service's
        port. Do not close tabs, inspect page contents, or change permissions.
        An unavailable debugging socket leaves the normal VIEW launch intact.
        """
        port=None
        try:
            code,output=await self.runner(['-s',serial,'forward','tcp:0','localabstract:chrome_devtools_remote'])
            if code or not re.fullmatch(r'\d{1,5}',output.strip()):return False
            port=int(output.strip())
            if not 1024<=port<=65535:port=None;return False
            pages=await _debug_pages(port)
            targets=[]
            is_input_url=lambda value:isinstance(value,str) and re.fullmatch(
                r'http://localhost:'+str(self.port)+r'/\?token=[A-Za-z0-9_-]+&role=input(?:&reopen=\d+)?',value)
            for page in pages:
                if not isinstance(page,dict) or page.get('type')!='page':continue
                target_id=page.get('id');old_url=page.get('url')
                if not isinstance(target_id,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',target_id):continue
                if not is_input_url(old_url):continue
                if generation!=self.generation:return False
                live=await _debug_page_location(port,target_id)
                if live is None:continue
                # A cached app URL may now point at another website or its
                # generic error page. Neither proves this is still our tab.
                if not is_input_url(live['href']):continue
                targets.append((target_id,live['href'],live.get('visible') is True))
            if not targets or generation!=self.generation:return False
            # VIEW may have opened a current tab while an older tab remains.
            # Prefer it over reviving an old tab and creating competing inputs.
            current=[t for t in targets if t[1].split('&reopen=',1)[0]==self.url]
            target_id,old_url,_=next((t for t in current if t[2]),current[0] if current else
                                     next((t for t in targets if t[2]),targets[0]))
            navigate=not (old_url==url or (not self.force_open and old_url.split('&reopen=',1)[0]==self.url))
            return await _navigate_debug_page(port,target_id,url,guard=lambda:generation==self.generation,navigate=navigate)
        except (OSError,ValueError,TimeoutError,WebSocketException):
            return False
        finally:
            if port is not None:
                await self.runner(['-s',serial,'forward','--remove',f'tcp:{port}'])

    async def poll(self, *, source_connected=False):
        generation=self.generation
        if not self.available:
            self._set('missing_adb','未找到 ADB。安装 Android Platform Tools 后重新启动调试台。')
            return
        code,output = await self.runner(['devices','-l'])
        if generation!=self.generation:return
        if code:
            self._set('error','USB 检查失败，系统会自动重试；也可点击“重新连接”。',output[:600])
            return
        self.devices = parse_devices(output)
        if not self.devices:
            self.opened_for = None
            self.configured_for = None
            self._set('waiting_usb','把 PICO 用 USB 数据线接到电脑即可；不用重启程序。')
            return
        chosen = next((d for d in self.devices if d['serial']==self.selected),None)
        if chosen is None:
            candidates = [d for d in self.devices if 'pico' in d['model'].lower()]
            if len(self.devices)==1:chosen=self.devices[0]
            elif len(candidates)==1:chosen=candidates[0]
            else:
                self.opened_for=None
                self._set('choose_device','检测到多台 Android 设备，请在下方选择 PICO。')
                return
            self.selected=chosen['serial']
        if chosen['state']!='device':
            self.opened_for=None
            self.configured_for=None
            if chosen['state']=='unauthorized':
                self._set('unauthorized','戴上 PICO，在 USB 调试弹窗点“允许”；确认后这里会自动继续。')
            else:self._set('offline','头显暂时离线。重新插拔 USB 数据线，程序会自动继续检查。')
            return
        fingerprint = (chosen['serial'],chosen['transport_id'])
        keep_page=source_connected and not self.force_open
        if keep_page and self.configured_for==fingerprint:
            self._set('page_connected','头显页面已连接；接下来检查 XR 和手柄输入。')
            return
        if not keep_page and self.opened_for==fingerprint:
            self._set('waiting_page','已请求打开头显页面。戴上 PICO 查看；未出现时点击“重新打开头显页面”。')
            return
        if time.monotonic()<self.retry_at:
            return
        self._set('connecting','正在连接并打开头显页面……')
        target_url=self.url+("&reopen="+str(time.monotonic_ns()) if self.force_open else "")
        commands = [['-s',chosen['serial'],'reverse',f'tcp:{self.port}',f'tcp:{self.port}']]
        for command in commands:
            code,output = await self.runner(command)
            if generation!=self.generation:return
            if code or re.search(r'(^|\n)\s*(Error:|Exception|Activity not started)',output):
                self.retry_at=time.monotonic()+5
                self._set('error','打开头显页面失败，5 秒后自动重试；也可点“重新连接”。',output[:600])
                return
        reused=False
        if not keep_page and self.browser_package=='com.pico.browser':
            reused=await self._refresh_native_page(chosen['serial'],target_url,generation)
            if generation!=self.generation:return
        if not keep_page and not reused:
            command=['-s',chosen['serial'],'shell','am','start','-a','android.intent.action.VIEW']
            if self.browser_package:command+=['-p',self.browser_package]
            code,output=await self.runner(command+['-d',"'"+target_url+"'"])
            if generation!=self.generation:return
            if code or re.search(r'(^|\n)\s*(Error:|Exception|Activity not started)',output):
                self.retry_at=time.monotonic()+5
                self._set('error','打开头显页面失败，5 秒后自动重试；也可点“重新连接”。',output[:600])
                return
            if self.browser_package=='com.pico.browser':
                await self._refresh_native_page(chosen['serial'],target_url,generation)
            if generation!=self.generation:return
        self.configured_for=fingerprint
        if keep_page:
            self._set('page_connected','头显页面已连接；接下来检查 XR 和手柄输入。')
            return
        self.opened_for=fingerprint
        self.force_open=False
        self._set('waiting_page','已请求打开头显页面。戴上 PICO 查看；未出现时点击“重新打开头显页面”。')
