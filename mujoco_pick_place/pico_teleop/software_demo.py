"""Bounded synthetic right controller through the real WebSocket protocol."""
import asyncio
import json
from .timing import clock as time
from urllib.parse import urlsplit,parse_qs
import numpy as np
from websockets.asyncio.client import connect


async def receive(ws,kind,timeout=3.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        item=json.loads(await asyncio.wait_for(ws.recv(),max(.001,deadline-time.monotonic())))
        if item.get('type')=='error':raise RuntimeError(item.get('message','演示协议错误'))
        if item.get('type')==kind:return item
    raise TimeoutError('演示等待 '+kind+' 超时')


async def run_demo(desktop_url):
    url=urlsplit(desktop_url);token=parse_qs(url.query)['token'][0]
    root=f'ws://{url.netloc}/ws?token={token}'
    async with connect(root+'&role=monitor') as monitor:
        await receive(monitor,'hello')
        status=await receive(monitor,'status')
        hardware=status.get('hardware')
        if not isinstance(hardware,dict) or hardware.get('is_virtual') is not True:
            raise ValueError('虚拟演示禁止连接实际电机，且只接受虚拟舵机后端')
        async with connect(root+'&role=input') as source:
            hello=await receive(source,'hello');challenge=await receive(source,'clock_ping')
            stamp=time.monotonic()*1000
            await source.send(json.dumps(dict(type='clock_reply',id=challenge['id'],client_recv_ms=stamp,client_send_ms=stamp)))
            await receive(source,'clock_ok');seq=0
            async def frame(*,arm=False,grip=0.,z=-.3):
                nonlocal seq
                seq+=1
                await source.send(json.dumps(dict(type='frame',session_id=hello['session_id'],seq=seq,
                    client_time_ms=time.monotonic()*1000,space_epoch=0,reference_space='local-floor',
                    mode='synthetic',visible=True,right=dict(valid=True,emulated=False,position=[.2,1.,z],
                    orientation=[0,0,0,1],grip=grip,trigger=0.,precision=False,stop=False,arm=arm))))
                await receive(source,'frame_ok')
            async def control(action,**extra):
                await monitor.send(json.dumps(dict(type='control',action=action,**extra)))
                return await receive(monitor,'control_result')
            try:
                await frame()
                await control('hardware_enable',workcell_verified=True)
                ready=False
                for _ in range(15):
                    await frame();await asyncio.sleep(.035)
                    status=await receive(monitor,'status')
                    if status['hardware']['ready']:ready=True;break
                if not ready:raise RuntimeError('虚拟电机未就绪：'+str(status['hardware'].get('fault')))
                if status['state']=='STOPPED':
                    await control('reset_stop');await frame()
                start=np.asarray(status['tcp_m']);await frame(arm=True);await frame(grip=1.)
                await asyncio.sleep(.05)
                samples=[]
                for i in range(36):
                    await frame(grip=1.,z=-.3-(i+1)*.00065)
                    await asyncio.sleep(1/30)
                    status=await receive(monitor,'status')
                    samples.append(dict(tcp=status['tcp_m'],command=status['command_tcp_m'],state=status['state'],
                                        reason=status['reason'],age_ms=status['age_ms']))
                    if status['hardware']['fault']:raise RuntimeError(status['hardware']['fault'])
                displacement=float(np.linalg.norm(np.asarray(status['tcp_m'])-start))
                await control('stop');await asyncio.sleep(.12)
                stopped=await receive(monitor,'status')
                if stopped['state']!='STOPPED':
                    stopped=await receive(monitor,'status')
                return dict(ok=displacement>.001 and stopped['state']=='STOPPED',scope='software_only',
                            displacement_m=round(displacement,6),frames=seq,stop_state=stopped['state'],samples=samples)
            finally:
                await control('stop')
