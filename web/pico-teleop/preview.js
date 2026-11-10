(() => {
  const canvas=document.getElementById('preview'),label=document.getElementById('state');
  const gl=canvas.getContext('webgl',{antialias:true});
  if(!gl){label.textContent='WebGL 不可用';return;}
  const scene=PicoScene.createRenderer(gl);
  const anchor=PicoScene.createAnchor([1,0,0,0,0,1,0,0,0,0,1,0,0,1.6,0,1]);
  let status=null,angle=-.35,connected=false,fresh=false,prepared=null;
  const params=new URLSearchParams(location.search),demo=params.get('demo')==='1';
  const hudCanvas=document.getElementById('headset-hud');
  function showDemo(state){
    const following=state==='FOLLOWING';
    status={state,reason:state==='READY'?'松开握持键和扳机，再短按 A 授权':'',gripper:.5,unlock_hold_s:0,
      points:[[0,0,0],[.04,0,.06],[.08,0,.16],[.18,0,.24],[.26,0,.2],[.29,0,.19]],tcp_m:[.29,0,.19],
      input:{valid:true,emulated:false,grip:following?1:0,trigger:0,arm:false,stop:false}};
  }
  if(demo){
    showDemo('READY');document.getElementById('demo-controls').hidden=false;
    for(const state of ['READY','FOLLOWING','STOPPED'])document.getElementById('demo-'+state.toLowerCase()).onclick=()=>showDemo(state);
    document.getElementById('preview-help').textContent='无设备布局演示：使用示例机械臂与输入状态，不连接服务、头显或电机。示例按钮只切换提示内容。';
  }
  document.getElementById('front').onclick=()=>angle=0;
  document.getElementById('side').onclick=()=>angle=-1.;
  const sub=(a,b)=>a.map((v,i)=>v-b[i]),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
  const unit=a=>a.map(v=>v/Math.hypot(...a));
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  function camera(){
    const target=[0,1.6,-.83],eye=[Math.sin(angle)*.9,1.6,-.83+Math.cos(angle)*.9];
    const z=unit(sub(eye,target)),x=unit(cross([0,1,0],z)),y=cross(z,x);
    return [x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-dot(x,eye),-dot(y,eye),-dot(z,eye),1];
  }
  const token=params.get('token');
  function connect(){
    if(demo)return;
    if(!token){label.textContent='请从调试台的三维预览入口打开';return;}
    const ws=new WebSocket(`ws://${location.host}/ws?role=monitor&token=${encodeURIComponent(token)}`);
    ws.onopen=()=>{connected=true;fresh=false;label.textContent='服务已连接，等待新状态';};
    ws.onmessage=e=>{const m=JSON.parse(e.data);if(m.type==='status'){status=m;fresh=true;}};
    ws.onclose=()=>{connected=false;fresh=false;label.textContent='连接断开，显示最后状态';setTimeout(connect,2000);};
  }
  function draw(){
    requestAnimationFrame(draw);
    const dpr=Math.min(devicePixelRatio||1,2),w=Math.round(canvas.clientWidth*dpr),h=Math.round(canvas.clientHeight*dpr);
    if(canvas.width!==w || canvas.height!==h){canvas.width=w;canvas.height=h;}
    gl.viewport(0,0,w,h);gl.depthMask(true);gl.clearColor(.045,.075,.1,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
    if(prepared!==status || prepared===null){scene.prepare(status,anchor,null,false);prepared=status;}
    const f=1/Math.tan(Math.PI/6),near=.02,far=20;
    const projection=[f/(w/h),0,0,0,0,f,0,0,0,0,(far+near)/(near-far),-1,0,0,2*far*near/(near-far),0];
    scene.draw({projectionMatrix:projection,transform:{inverse:{matrix:camera()}}});
    const error=gl.getError();
    if(error)label.textContent='WebGL 错误 '+error;
    else if(demo)label.textContent='无设备布局演示 · '+PicoInput.stateLabel(status.state);
    else if(status && connected && fresh)label.textContent=(status.hardware?(status.hardware.is_virtual?'虚拟舵机软件验证':status.hardware.feedback_valid?'真机反馈':'零位参考 / 标定预览'):status.motor?'舵机响应（未标定）':'三维绘制正常')+' · '+PicoInput.stateLabel(status.state)+' · 夹爪 '+Math.round(status.gripper*100)+'%';
    const options={disconnected:!demo && (!connected || !fresh)};
    const controls=!options.disconnected && status && status.input || {valid:false,grip:0,trigger:0,arm:false,stop:false};
    PicoHUD.paint(hudCanvas,status,controls,options);
    hudCanvas.setAttribute('aria-label',PicoInput.hudLines(status,controls,options).join('；'));
  }
  connect();draw();
})();
