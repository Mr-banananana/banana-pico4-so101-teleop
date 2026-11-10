(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const params = new URLSearchParams(location.search);
  const token = params.get('token');
  const role = params.get('role') === 'input' ? 'input' : 'monitor';
  let socket, sessionId, xrSession, referenceSpace, gl, hud, worldScene, sceneAnchor;
  let sequence = 0, epoch = 0, currentStatus = null, lastDiagnostic = 0;
  let xrMode = null, ended = false, deviceOptions = '';
  let xrCapabilities=null, renderedFrames=0, xrLifecycle=null, pageDeparted=false,xrContextKey=null,xrInspectionGeneration=0;
  let softwareUiConfigured=false;
  const deviceReport=extra=>({userAgent:navigator.userAgent,control_scheme:'right_only',scene_version:'spatial-v3-servo',
    ...(xrCapabilities?{capabilities:xrCapabilities}:{}),...extra});
  const mapping = {trigger:0,grip:1,precision:4,stop:5,arm:4};
  $('role-label').textContent = role === 'input' ? '头显输入端' : '电脑观察端';
  $('arm').disabled = role !== 'monitor';
  $('stop').disabled = role !== 'monitor';
  $('unlock').disabled = role !== 'monitor';
  $('control-mode').disabled = role !== 'monitor';
  $('control-mode-panel').hidden = role !== 'monitor';
  if (!token) $('session-warning').hidden = false;
  function event(message) {
    const line = document.createElement('div');
    line.textContent = new Date().toLocaleTimeString() + '  ' + message;
    $('events').prepend(line);
    while ($('events').children.length > 30) $('events').lastChild.remove();
  }
  function send(message) {
    return PicoInput.boundedSend(socket,message,()=>event('发送缓冲堆积，断开并撤销跟随'));
  }
  function control(action) { send({type:'control',action}); }
  const hardwarePanel=typeof PicoHardwarePanel!=='undefined'?PicoHardwarePanel.create(send,role):null;
  $('arm').onclick = () => control('arm');
  $('stop').onclick = () => control('stop');
  $('unlock').onclick = () => control('reset_stop');
  $('control-mode').onchange = e => control(e.target.value);
  $('retry-pico').onclick=()=>control('reconnect_pico');
  $('self-test').onclick=()=>{control('self_test');$('self-test-result').textContent='正在独立仿真中检查模型、IK、输入与超时保持…';};
  $('open-viewer').onclick=()=>control('open_viewer');
  $('shutdown').onclick=()=>control('shutdown');
  $('device-select').onchange=e=>send({type:'control',action:'select_device',serial:e.target.value});
  $('debug-report').href='/debug-report?token='+encodeURIComponent(token || '');
  $('scene-preview').href='/xr-preview.html?token='+encodeURIComponent(token || '');
  for (const name of Object.keys(mapping)) {
    $('map-'+name).onchange = e => {
      const n = Number(e.target.value);
      if (Number.isInteger(n) && n >= 0 && n <= 15) {
        mapping[name] = n;send({type:'revoke',reason:'mapping_changed'});event('按钮映射已改变；请重新授权');
      }
    };
  }
  $('mapping-confirmed').onchange = () => send({type:'revoke',reason:'mapping_changed'});
  function connect() {
    if (!token || ended || pageDeparted) return;
    if(socket && (socket.readyState===0 || socket.readyState===1))return;
    const connection = new WebSocket(`ws://${location.host}/ws?token=${encodeURIComponent(token)}&role=${role}`);
    socket=connection;
    connection.onopen = () => {
      if(socket!==connection || ended || pageDeparted)return;
      $('connection').textContent = '本地服务已连接';$('connection-dot').classList.add('online');event('WebSocket 已连接');
    };
    connection.onmessage = e => {
      if(socket!==connection || ended || pageDeparted)return;
      const received = performance.now();
      let m;try {m=JSON.parse(e.data);} catch {return;}
      if (m.type === 'hello') {
        sessionId=m.session_id;sequence=0;
        if (role==='input') send({type:'diagnostic',details:deviceReport({})});
      } else if (m.type === 'clock_ping') {
        // A second quantized browser reading can invent a handling duration
        // longer than a short USB RTT. One receive sample leaves that handling
        // time inside the server-measured RTT, widening the offset interval.
        send({type:'clock_reply',id:m.id,client_recv_ms:received,client_send_ms:received});
      } else if (m.type === 'status') {
        currentStatus=m;renderStatus(m);inspectXR();
      } else if (m.type === 'error') {
        event(m.message);$('reason').textContent=m.message;
        if(hardwarePanel && hardwarePanel.isImportPending())hardwarePanel.failImport(m.message);
      } else if (m.type === 'self_test_result') {
        $('self-test-result').textContent=(m.ok?'自检通过。':'自检发现问题：')+
          m.checks.map(c=>(c.ok?'✓ ':'✕ ')+c.name).join('；')+'。'+m.message;
      } else if (m.type === 'control_result') {
        if(m.action==='cal_import' && m.ok && hardwarePanel)hardwarePanel.succeedImport();
        if(m.action==='shutdown' && m.ok)finishDebug();
        else event(m.ok ? '操作已完成' : m.message);
      } else if (m.type === 'service_stopping') finishDebug();
    };
    connection.onclose = e => {
      if(socket!==connection)return;
      sessionId=null;currentStatus=null;$('connection').textContent='连接已断开，跟随已撤销';$('connection-dot').classList.remove('online');
      stopXR();
      if(role==='input' && e.code===4000){
        ended=true;
        $('connection').textContent='此输入页已退出，请使用新打开的头显页面';
        $('xr-help').textContent='电脑正在重新打开输入页面。如未出现，请在电脑点击“重新打开头显页面”。';
        $('enter-ar').disabled=true;$('enter-vr').disabled=true;
        return;
      }
      if(pageDeparted)return;
      if(ended){$('connection').textContent='调试已结束；下次双击 PICO调试.cmd 即可';return;}
      event('断开；重连不会恢复授权');setTimeout(connect,2000);
    };
    connection.onerror = () => {if(socket===connection && !ended && !pageDeparted)$('connection').textContent='连接失败：检查服务和会话地址';};
  }
  function finishDebug(){
    ended=true;$('next-step').textContent=currentStatus && currentStatus.hardware && currentStatus.hardware.is_virtual?
      '软件验证已结束。下次双击 PICO软件验证.cmd。':'调试已结束。下次接上 USB，双击 PICO调试.cmd。';
    $('connection').textContent='调试已结束';
    $('connection-dot').classList.remove('online');
    for(const id of ['retry-pico','self-test','open-viewer','shutdown','arm','stop','unlock'])$(id).disabled=true;
    stopXR();syncXRButtons();
  }
  const fmt = (v,n=3) => Array.isArray(v) ? v.map(x=>Number(x).toFixed(n)).join('  ') : '—';
  function renderStatus(s) {
    if(ended)return;
    if(hardwarePanel)hardwarePanel.render(s);
    if(s.debug && role==='monitor') {
      $('debug-guide').hidden=false;
      $('next-step').textContent=s.debug.message;
      $('self-test').disabled=s.debug.self_test_busy;
      const steps=$('debug-steps');steps.replaceChildren(...s.debug.steps.map((step,i)=>{
        const li=document.createElement('li');li.className=step.done?'done':'';
        li.textContent=(step.done?'✓':String(i+1))+ '  '+step.label;return li;
      }));
      const devices=s.debug.devices || [], serialized=JSON.stringify(devices);
      $('device-picker').hidden=devices.length<2;
      if(serialized!==deviceOptions){
        deviceOptions=serialized;
        const empty=document.createElement('option');empty.value='';empty.textContent='请选择 PICO 设备';empty.disabled=true;
        $('device-select').replaceChildren(empty,...devices.map(d=>{
          const option=document.createElement('option');option.value=d.serial;
          option.textContent=(d.model?d.model+' · ':'')+d.serial+' · '+d.state;return option;
        }));
      }
      $('device-select').value=s.debug.selected_serial || '';
    }
    $('state').textContent=PicoInput.stateLabel(s.state);$('reason').textContent=s.reason;
    $('backend-badge').textContent=s.hardware?(s.hardware.is_virtual?'虚拟舵机 · 软件验证':s.hardware.ready?'真机 · 已接管':s.hardware.calibrated?'真机 · 反馈只读':'真机 · 未标定参考模型'):s.motor?'舵机响应仿真 · 未标定':'仅运动学仿真 · 未接真实电机';
    if(!s.hardware)document.querySelector('.subtitle').textContent='仿真模式：在 PICO 内进入 VR 仿真预览，用右手柄操控虚拟 SO101；不会连接真实电机。';
    if(s.hardware){
      $('arm').textContent=s.hardware.is_virtual?'授权虚拟跟随':'授权真机跟随';
      if(role==='monitor'){
        const inputPanel=document.querySelector('.input-panel');
        if($('hardware-panel').parentElement!==inputPanel.parentElement)inputPanel.before($('hardware-panel'));
        inputPanel.hidden=true;
      }
      document.querySelector('.stage h2').textContent=s.hardware.is_virtual?'虚拟舵机反馈 · 软件验证':s.hardware.pose_source==='feedback'?'SO101 真机反馈':s.hardware.pose_source==='zero_reference'?'零位参考 · 请对齐实物':'校准映射预览 · 尚未验证';
      document.querySelector('.stage .caption').textContent=s.hardware.is_virtual?'绿色为虚拟反馈，青点为目标；支持命令延迟、加减速和反馈量化，仅用于验证软件。':'绿色来自已标定的电机反馈，青点为目标；未标定时仅显示零位参考或映射预览。模型不反映真实障碍物。';
      document.querySelector('footer').textContent=s.hardware.is_virtual?'软件验证：内存虚拟舵机，共用真机控制软件；没有串口或 ADB 操作，不代表实机已验收。':'真机模式：启动只读，标定后显式接管。右 A 授权、grip 跟随、B 软件停止；电脑页面的「一键下电」关闭六轴扭矩使能。';
      if(s.hardware.is_virtual && !softwareUiConfigured){
        softwareUiConfigured=true;$('retry-pico').hidden=true;
        document.querySelector('#debug-guide h2').textContent='软件验证向导';
        document.querySelector('.subtitle').textContent='不连接机械臂或头显，验证手柄输入到舵机反馈的完整软件链路。';
        $('self-test-result').textContent='基础自检使用独立仿真；右侧虚拟手柄演示检查完整通信与控制链路。';
      }
    }
    $('control-mode').value=s.mode || 'position';
    const poseOption=Array.from($('control-mode').options || []).find(option=>option.value==='pose');
    if(poseOption)poseOption.textContent=s.pose_policy==='split_roll'?'坐标＋独立翻滚':'位置＋绝对旋转';
    $('control-mode').disabled=role!=='monitor' || ['ARMED','FOLLOWING'].includes(s.state);
    $('control-mode-help').textContent=s.mode==='pose'?'位置相对跟随，旋转绝对对齐：松手转动后，再握持会追到当前朝向。五轴优先保持位置，无法实现任意三轴朝向。切换前先停止并解除授权。':'仅跟随手柄位置，忽略手柄旋转。切换前先停止并解除授权。';
    if(s.control_reference_error)$('control-mode-help').textContent=s.control_reference_error;
    else if(s.control_reference)$('control-mode-help').textContent=s.mode==='position'?
      '仅位置模式：松开按键后短按 A 授权；不跟随手柄旋转，扳机使用已保存的夹爪默认开度。':s.control_reference.hand_required?
      '机械臂参考已保存。进入透视，把手柄竖直并松开握持键和扳机，短按 A 只记录竖直参考，不授权、不使能电机。':
      (s.pose_policy==='balanced'?'位置、翻滚、俯仰、偏航共同求解；五轴无法全部满足时允许位置与朝向折中。':
       '旋转与俯仰按已记录的竖直参考跟随，位置优先。')+'松握持不会重置朝向；'+(s.gripper_mode==='toggle'?'扣一下扳机抓取，松开仍保持，再扣一下张开；握持键只控制位置与朝向。':'扳机松开回到已保存开度，按到底闭合。');
    if(s.pose_policy==='split_roll' && s.mode==='pose' && !s.control_reference_error && !(s.control_reference && s.control_reference.hand_required))
      $('control-mode-help').textContent='坐标与翻滚分开跟随：手柄坐标控制末端位置，左右拧转单独控制夹爪自转；手柄俯仰、偏航不跟随，机械臂尽量保持夹爪前向水平。松握持不重置翻滚角。扣一下扳机抓取，再扣一下张开。';
    $('gripper-guide').textContent=s.gripper_mode==='toggle'?'扣一下扳机抓取，松开仍保持，再扣一下张开；无需按住握持键。':'握持时扣扳机控制夹爪，按住 A 为精细模式。';
    $('motor-info').textContent=s.hardware?`${s.hardware.is_virtual?'虚拟':'真机'}反馈年龄 ${s.hardware.feedback_age_ms}ms；关节轨迹限速${s.hardware.motion_profile?Number(s.hardware.motion_profile.joint_speed).toFixed(2):"—"}rad/s，实机响应以反馈为准`:s.motor?s.motor.name+'；参照 '+s.motor.reference_voltage+'V，实际供电未确认；指令/反馈各 '+Math.round(s.motor.command_delay_s*1000)+' / '+Math.round(s.motor.feedback_delay_s*1000)+'ms（假设）':'旧版瞬时关节更新';
    $('motor-error').textContent=s.motor?fmt(s.motor.target_rad.map((v,i)=>(v-s.motor.feedback_rad[i])*180/Math.PI),2):'—';
    $('tcp').textContent=fmt(s.tcp_m);$('requested').textContent=fmt(s.requested_m);
    if(s.hardware && s.hardware.is_virtual){
      if(s.state==='WAITING')$('reason').textContent='点击右侧「运行虚拟手柄演示」开始软件验证';
      if(s.state==='STOPPED' && s.software_demo && s.software_demo.ok)$('reason').textContent='虚拟演示已完成并停止；可再次运行';
    }
    $('joints').textContent=fmt(s.q_deg,1);$('gripper').textContent=(100*s.gripper).toFixed(1)+'%';
    $('timing').textContent=`${s.tick_ms} ms / ${s.hz} Hz`;
    $('freshness').textContent=`#${s.source_seq}  /  ${s.age_ms==null?'未同步':s.age_ms+' ms'}`;
    if (s.input) {
      $('position').textContent=fmt(s.input.position);$('orientation').textContent=fmt(s.input.orientation);
      for (const k of ['grip','trigger']) {$ (k).value=s.input[k];$(k+'-value').textContent=s.input[k].toFixed(2);}
      $('source-mode').textContent=s.input.valid ? (s.hardware && s.hardware.is_virtual?'虚拟手柄输入有效':s.input.emulated?'估算位姿 / 仅仿真':s.input.xr_mode==='immersive-ar'?'AR 输入有效':'VR 输入有效'):'右手柄位置追踪或映射无效';
    }
    if (role==='monitor' && s.diagnostics && Object.keys(s.diagnostics).length) {
      $('device-info').textContent=JSON.stringify(s.diagnostics,null,2);
    }
    drawArm(s);
  }
  function drawArm(s) {
    const c=$('arm-canvas'),ctx=c.getContext('2d'),w=c.width,h=c.height;
    ctx.clearRect(0,0,w,h);
    function project(p) {return [w*.42+(p[0]-p[1]) * 1000,h*.81-p[2]*1500+(p[0]+p[1]) * 260];}
    ctx.strokeStyle='#d9e1d3';ctx.lineWidth=1;
    for (let i=-3;i<=5;i++) {
      const a=project([i*.05,-.25,0]),b=project([i*.05,.25,0]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
      const d=project([0,i*.05,0]),e=project([.4,i*.05,0]);ctx.beginPath();ctx.moveTo(...d);ctx.lineTo(...e);ctx.stroke();
    }
    const points=s.points || [];
    ctx.lineCap='round';ctx.lineJoin='round';
    if (points.length) {
      ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(...project(p)):ctx.moveTo(...project(p)));
      ctx.strokeStyle='#c2cfb5';ctx.lineWidth=23;ctx.stroke();ctx.strokeStyle='#2a594b';ctx.lineWidth=10;ctx.stroke();
      for (const p of points) {ctx.beginPath();ctx.arc(...project(p),9,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();ctx.strokeStyle='#2a594b';ctx.lineWidth=4;ctx.stroke();}
    }
    if (s.requested_m) {const p=project(s.requested_m);ctx.strokeStyle='#cf813a';ctx.lineWidth=2;ctx.strokeRect(p[0]-9,p[1]-9,18,18);}
    if (s.motor && s.command_tcp_m) {const p=project(s.command_tcp_m);ctx.beginPath();ctx.arc(...p,6,0,Math.PI*2);ctx.fillStyle='#248ca8';ctx.fill();}
    ctx.fillStyle='#61745d';ctx.font='18px system-ui';ctx.fillText('SO101 / 关节运动学投影',28,38);
    ctx.font='15px system-ui';ctx.fillText('绿色：反馈关节  ·  橙色：末端请求  ·  青色：下发目标',28,h-23);
  }
  drawArm({points:[]});connect();

  async function inspectXR() {
    if (role!=='input') {
      const hardware=currentStatus && currentStatus.hardware;
      const realArm=!!(hardware && !hardware.is_virtual);
      $('enter-ar').hidden=true;$('enter-vr').hidden=true;$('xr-help').textContent=realArm?
        '头显页面点击「进入透视遥操作」，通过透视看真实机械臂；电脑查看反馈与连接。':
        '在头显页面点击「VR 仿真预览」，进入虚拟场景后用右手柄操作；电脑查看连接与关节反馈。';
      $('compatibility').textContent='等待 PICO 端回报浏览器、控制器 profile 和按钮数据。';return;
    }
    const hardware=currentStatus && currentStatus.hardware;
    const realArm=!!(hardware && !hardware.is_virtual);
    const key=`${currentStatus && currentStatus.backend || 'pending'}:${realArm?'real':'simulation'}`;
    if(key===xrContextKey)return;
    xrContextKey=key;const generation=++xrInspectionGeneration;
    if (!window.isSecureContext || !navigator.xr) {
      $('enter-vr').textContent='WebXR 不可用';$('xr-help').textContent='请使用 PICO 浏览器，并通过 USB localhost 地址访问。';
      $('compatibility').textContent=`安全上下文：${window.isSecureContext}；navigator.xr：${!!navigator.xr}`;return;
    }
    try {
      const vr=await navigator.xr.isSessionSupported('immersive-vr');
      let ar=false;
      try{ar=await navigator.xr.isSessionSupported('immersive-ar');}catch{/* Optional AR must not block VR. */}
      if(generation!==xrInspectionGeneration)return;
      $('enter-ar').classList.toggle('primary',realArm);$('enter-vr').classList.toggle('primary',!realArm);
      $('enter-ar').hidden=!realArm;$('enter-ar').disabled=!ar;$('enter-ar').textContent=ar?'进入透视遥操作':'当前浏览器未提供 AR 透视';
      $('enter-vr').hidden=realArm;$('enter-vr').disabled=!vr;$('enter-vr').textContent=vr?'VR 仿真预览':'浏览器不支持沉浸式 VR';
      $('compatibility').textContent=`VR：${vr?'支持':'不支持'}；AR 会话：${ar?'支持，透视需进入后检测':'不支持'}。VR 显示虚拟场景；AR 应看到头显周围实景，不是机器人相机回传。`;
      $('xr-help').textContent=realArm?
        (ar?'真机模式：点击「进入透视遥操作」，确认看到真实机械臂后再授权。':'当前浏览器不支持真机所需的透视模式。'):
        (vr?'仿真模式：点击「VR 仿真预览」进入虚拟场景，等待右手柄追踪有效后按 A 授权。不会连接或移动真机械臂。':'浏览器不支持 VR 仿真。');
      xrCapabilities={vr,ar};send({type:'diagnostic',details:deviceReport({})});
      syncXRButtons();
    } catch (e) {event('WebXR 检查失败：'+e.message);}
  }
  $('enter-ar').onclick=()=>startXR('immersive-ar');
  $('enter-vr').onclick=()=>startXR('immersive-vr');
  $('exit-xr').onclick=()=>stopXR();

  function syncXRButtons(){
    for(const [id,supported] of [['enter-ar',xrCapabilities && xrCapabilities.ar],['enter-vr',xrCapabilities && xrCapabilities.vr]])
      $(id).disabled=ended || pageDeparted || !!xrLifecycle || supported===false;
  }
  function stopXR(){
    const owner=xrLifecycle;if(!owner)return;
    owner.cleanup();
    if(owner.session)return owner.session.end().catch(()=>{});
  }

  function makeHUD() {
    const canvas=document.createElement('canvas');canvas.width=PicoHUD.layout.width;canvas.height=PicoHUD.layout.height;
    const shader=(type,source)=>{const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;};
    const program=gl.createProgram();gl.attachShader(program,shader(gl.VERTEX_SHADER,'attribute vec4 p; attribute vec2 t; varying vec2 uv; void main(){uv=t; gl_Position=p;}'));
    gl.attachShader(program,shader(gl.FRAGMENT_SHADER,'precision mediump float; varying vec2 uv; uniform sampler2D tex; void main(){gl_FragColor=texture2D(tex,uv);}'));gl.linkProgram(program);
    if (!gl.getProgramParameter(program,gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(program));
    const buffer=gl.createBuffer();
    const texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,texture);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.LINEAR);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.LINEAR);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
    return {canvas,program,buffer,texture,last:0,dispose(){gl.deleteBuffer(buffer);gl.deleteTexture(texture);gl.deleteProgram(program);}};
  }
  function drawHUD(frame,controllerPose,controls) {
    const layer=xrSession.renderState.baseLayer;gl.bindFramebuffer(gl.FRAMEBUFFER,layer.framebuffer);
    const transparent=xrMode==='immersive-ar' && ['alpha-blend','additive'].includes(xrSession.environmentBlendMode);
    gl.depthMask(true);gl.clearDepth(1);
    gl.clearColor(transparent?0:.04,transparent?0:.07,transparent?0:.08,transparent?0:1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
    const pose=frame.getViewerPose(referenceSpace);if(!pose)return;
    if(!sceneAnchor)sceneAnchor=PicoScene.createAnchor(pose.transform.matrix);
    if(xrMode!=='immersive-ar')worldScene.prepare(currentStatus,sceneAnchor,controllerPose,controls.valid && !controls.emulated);
    if (performance.now()-hud.last>100) {
      PicoHUD.paint(hud.canvas,currentStatus,controls,{passthrough:xrMode==='immersive-ar'});
      gl.bindTexture(gl.TEXTURE_2D,hud.texture);gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,gl.RGBA,gl.UNSIGNED_BYTE,hud.canvas);hud.last=performance.now();
    }
    for (const view of pose.views) {
      const v=layer.getViewport(view);gl.viewport(v.x,v.y,v.width,v.height);
      if(xrMode!=='immersive-ar')worldScene.draw(view);
      gl.useProgram(hud.program);gl.bindBuffer(gl.ARRAY_BUFFER,hud.buffer);
      gl.bufferData(gl.ARRAY_BUFFER,PicoHUD.vertices(view,pose.transform.matrix),gl.DYNAMIC_DRAW);
      const loc=gl.getAttribLocation(hud.program,'p'),texLoc=gl.getAttribLocation(hud.program,'t');
      gl.enableVertexAttribArray(loc);gl.vertexAttribPointer(loc,4,gl.FLOAT,false,24,0);
      gl.enableVertexAttribArray(texLoc);gl.vertexAttribPointer(texLoc,2,gl.FLOAT,false,24,16);
      gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,hud.texture);gl.uniform1i(gl.getUniformLocation(hud.program,'tex'),0);
      gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);gl.disable(gl.DEPTH_TEST);gl.depthMask(false);
      gl.drawArrays(gl.TRIANGLES,0,6);gl.disableVertexAttribArray(loc);gl.disableVertexAttribArray(texLoc);
    }
  }
  async function startXR(mode) {
    if (role!=='input' || ended || pageDeparted || xrLifecycle || !sessionId) {event('请等待输入服务连接完成，或退出当前头显模式');return;}
    const owner={session:null,hud:null,scene:null,space:null,frameId:null};
    xrLifecycle=owner;syncXRButtons();
    const active=()=>xrLifecycle===owner;
    const reset=()=>{if(!active())return;epoch++;sceneAnchor=null;send({type:'revoke',reason:'reference_reset'});event('头显坐标原点已重置，跟随已暂停；松开握持键和扳机，再短按 A 授权');};
    const visibility=()=>{if(active() && owner.session.visibilityState!=='visible')send({type:'revoke',reason:'xr_hidden'});};
    owner.cleanup=()=>{
      if(!active())return;
      // Invalidate callbacks before releasing resources or awaiting session.end().
      xrLifecycle=null;
      if(owner.session){
        if(owner.frameId!==null && owner.session.cancelAnimationFrame)owner.session.cancelAnimationFrame(owner.frameId);
        if(owner.session.removeEventListener){owner.session.removeEventListener('end',owner.cleanup);owner.session.removeEventListener('visibilitychange',visibility);}
        send({type:'revoke',reason:'xr_ended'});
      }
      if(owner.space && owner.space.removeEventListener)owner.space.removeEventListener('reset',reset);
      if(owner.scene)owner.scene.dispose();if(owner.hud)owner.hud.dispose();
      worldScene=null;sceneAnchor=null;xrSession=null;referenceSpace=null;hud=null;xrMode=null;
      $('exit-xr').hidden=true;$('xr-label').textContent='头显模式已退出';syncXRButtons();
      event('头显模式已退出，重新进入后需要再授权');
    };
    let created;
    try {
      created=await navigator.xr.requestSession(mode,{optionalFeatures:['local-floor']});
      if(!active()){await created.end().catch(()=>{});return;}
      owner.session=created;xrSession=created;
      created.addEventListener('end',owner.cleanup);
      created.addEventListener('visibilitychange',visibility);
      if(mode==='immersive-ar' && !['alpha-blend','additive'].includes(created.environmentBlendMode)){
        const blend=created.environmentBlendMode || 'unknown';
        send({type:'diagnostic',details:deviceReport({xr_mode:mode,blend_mode:blend,ar_passthrough:false})});
        owner.cleanup();
        $('xr-label').textContent='AR 未提供透视';
        $('xr-help').textContent='当前浏览器未提供 AR 透视（'+blend+'），已退出。需要适配 PICO 原生透视客户端；VR 只供仿真预览，不能替代看到真实机械臂。';
        event($('xr-help').textContent);await created.end().catch(()=>{});return;
      }
      xrMode=mode;epoch++;
      send({type:'revoke',reason:'xr_started'});
      gl=$('xr-canvas').getContext('webgl',{xrCompatible:true,alpha:true});if(!gl)throw Error('WebGL 不可用');
      await gl.makeXRCompatible();
      if(!active())return;
      created.updateRenderState({baseLayer:new XRWebGLLayer(created,gl,{alpha:true,depth:true}),depthNear:.02,depthFar:20});
      let spaceName='local-floor';
      let space;
      try {space=await created.requestReferenceSpace(spaceName);} catch {
        if(!active())return;
        spaceName='local';space=await created.requestReferenceSpace(spaceName);
      }
      if(!active())return;
      referenceSpace=space;owner.space=space;space.addEventListener('reset',reset);
      hud=owner.hud=makeHUD();
      if(mode!=='immersive-ar')worldScene=owner.scene=PicoScene.createRenderer(gl);
      sceneAnchor=null;renderedFrames=0;lastDiagnostic=-Infinity;
      $('exit-xr').hidden=false;$('xr-label').textContent=mode==='immersive-ar'?'AR 实景透视 · 仅操作提示':'VR 仿真预览 · 无实景';
      $('xr-help').textContent=mode==='immersive-ar'?'请确认能看见真实机械臂与桌面；仅叠加操作提示，不画虚拟机械臂。透视图像由头显本机显示，不传到电脑。':'VR 只供仿真预览，看不到现场实物；操控真实机械臂请退出并进入透视遥操作。';
      event(mode+' 已进入；参考空间 '+spaceName);
      function onFrame(t, frame) {
        if (!active()) return;
        let right=null;
        let pose,controls;
        try{
          for(const source of created.inputSources){if(source.handedness==='right')right=source;}
          const stopDown=!!(right && right.gamepad && PicoInput.buttonValue(right.gamepad.buttons[mapping.stop])>.5);
          if(stopDown && !owner.stopDown)send({type:'stop'});
          owner.stopDown=stopDown;
          pose=null;if(right && right.gripSpace)pose=frame.getPose(right.gripSpace,referenceSpace);
          controls=PicoInput.snapshot(right,pose,mapping,$('mapping-confirmed').checked,
            {allowEmulated:!!(currentStatus && currentStatus.allow_emulated_simulation)});
          drawHUD(frame,pose,controls);renderedFrames++;
        }catch(error){
          event('三维绘制失败：'+error.message);$('xr-help').textContent='三维绘制失败：'+error.message;
          send({type:'diagnostic',details:deviceReport({render_error:error.message,rendered_frames:renderedFrames})});
          send({type:'revoke',reason:'render_failed'});stopXR();return;
        }
        if(!active())return;
        owner.frameId=created.requestAnimationFrame(onFrame);
        // Publish each native XR sample: a 16 ms gate halves 72/90 Hz input.
        // The transport still rejects backlog; the host consumes latest state.
        const now=performance.now();if(!sessionId)return;
        send({type:'frame',session_id:sessionId,seq:++sequence,client_time_ms:now,space_epoch:epoch,reference_space:spaceName,mode,visible:xrSession.visibilityState==='visible',right:controls});
        if(now-lastDiagnostic>500){
          const describe=s=>s?{profiles:s.profiles,mapping:s.gamepad?s.gamepad.mapping:null,gripSpace:!!s.gripSpace,buttons:s.gamepad?s.gamepad.buttons.map(b=>({value:b.value,pressed:b.pressed})):[]}:null;
          const details=deviceReport({xr_mode:mode,blend_mode:xrSession.environmentBlendMode,rendered_frames:renderedFrames,
            ar_passthrough:mode==='immersive-ar',scene_content:mode==='immersive-ar'?'passthrough_hud_only':'virtual_robot',
            right:describe(right),emulatedPosition:pose?!!pose.emulatedPosition:null,
            raw_right_position:pose?[pose.transform.position.x,pose.transform.position.y,pose.transform.position.z]:null,
            raw_right_orientation:pose?[pose.transform.orientation.x,pose.transform.orientation.y,pose.transform.orientation.z,pose.transform.orientation.w]:null,
            estimated_simulation:!!(controls.valid && controls.emulated)});
          $('device-info').textContent=JSON.stringify(details,null,2);send({type:'diagnostic',details});lastDiagnostic=now;
        }
      }
      owner.frameId=created.requestAnimationFrame(onFrame);
    } catch(e){
      if(active()){
        event('XR 启动失败：'+e.message);send({type:'diagnostic',details:deviceReport({render_error:e.message})});
        owner.cleanup();if(created)await created.end().catch(()=>{});
      }
    }
  }
  document.addEventListener('visibilitychange',()=>{if(document.hidden && role==='input')send({type:'revoke',reason:'page_hidden'});});
  window.addEventListener('pagehide',()=>{pageDeparted=true;if(role==='input')send({type:'revoke',reason:'page_left'});stopXR();syncXRButtons();if(socket)socket.close();});
  window.addEventListener('pageshow',()=>{if(!pageDeparted || ended)return;pageDeparted=false;syncXRButtons();connect();});
  inspectXR();
})();
