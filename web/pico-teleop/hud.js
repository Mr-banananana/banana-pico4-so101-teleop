/* One head-relative panel shared by both eyes; angles are initial UI design values. */
(() => {
  const Input=typeof module!=='undefined' && module.exports?require('./input.js'):globalThis.PicoInput;
  const layout=Object.freeze({width:1024,height:384,distance:1.25,widthDegrees:40,elevationDegrees:9});
  const rad=d=>d*Math.PI/180;
  function transform(m,p){return [0,1,2,3].map(row=>m[row]*p[0]+m[row+4]*p[1]+m[row+8]*p[2]+m[row+12]*p[3]);}
  function vertices(view,headMatrix){
    const halfWidth=layout.distance*Math.tan(rad(layout.widthDegrees/2));
    const halfHeight=halfWidth*layout.height/layout.width;
    const centerY=layout.distance*Math.tan(rad(layout.elevationDegrees));
    const result=[];
    for(const [x,y] of [[-1,-1],[1,-1],[-1,1],[-1,1],[1,-1],[1,1]]){
      const world=transform(headMatrix,[x*halfWidth,centerY+y*halfHeight,-layout.distance,1]);
      const eye=transform(view.transform.inverse.matrix,world);
      result.push(...transform(view.projectionMatrix,eye),(x+1)/2,(1-y)/2);
    }
    return new Float32Array(result);
  }
  function wrap(ctx,text,maxWidth,maxLines){
    const rows=[];let line='';
    for(const char of String(text || '')){
      if(char==='\n' || (line && ctx.measureText(line+char).width>maxWidth)){
        rows.push(line);line=char==='\n'?'':char;
      }else line+=char;
    }
    if(line)rows.push(line);
    if(rows.length<=maxLines)return rows;
    rows.length=maxLines;
    let tail=rows[maxLines-1];
    while(tail && ctx.measureText(tail+'…').width>maxWidth)tail=tail.slice(0,-1);
    rows[maxLines-1]=tail+'…';return rows;
  }
  function paint(canvas,status,controls,options={}){
    const ctx=canvas.getContext('2d'),lines=Input.hudLines(status,controls,options),h=status && status.hardware;
    const stopped=status && status.state==='STOPPED';
    const prep=status && status.workspace_preparation;
    const compact=!options.disconnected && status && status.state==='FOLLOWING' && !(prep && (prep.running || ['enabling','stabilizing','latching'].includes(prep.state))) && !status.workspace_limited && !status.constraint_pause && !status.thermal_pause && !status.tracking_limited && !status.joint_limit_limited && controls.valid && !controls.emulated && (!h || h.ready && !h.fault);
    const height=compact?272:layout.height;
    const accent=stopped?'#ffb3a8':controls.emulated || !controls.valid || status && status.tracking_limited?'#ffd393':'#a4e5c5';
    ctx.clearRect(0,0,layout.width,layout.height);
    ctx.fillStyle='rgba(12,29,25,.94)';ctx.fillRect(0,0,layout.width,height);
    ctx.strokeStyle='#446257';ctx.lineWidth=2;ctx.strokeRect(1,1,layout.width-2,height-2);
    ctx.fillStyle=accent;ctx.fillRect(0,0,layout.width,5);
    function text(value,y,size,color,maxLines=1,lineHeight=size+8,bold=false){
      ctx.font=`${bold?'600 ':''}${size}px system-ui, "Microsoft YaHei", sans-serif`;
      ctx.fillStyle=color;
      wrap(ctx,value,944,maxLines).forEach((row,i)=>ctx.fillText(row,40,y+i*lineHeight));
    }
    const backend=h?(h.is_virtual?'虚拟舵机':'真机'):status && status.motor?'舵机仿真':'仿真';
    text('SO101 · '+(options.passthrough?'透视 · ':'')+backend+' · '+(options.disconnected?'连接已断开':Input.stateLabel(status && status.state)),44,28,accent);
    text(compact?(status.mode==='pose'?(status.pose_policy==='split_roll'?'坐标＋独立翻滚 · 松开即可保持':'位置＋绝对旋转 · 松开即可保持'):'握持跟随中 · 松开即可保持'):lines[1],96,44,'#f1f7f3',compact?1:2,50,true);
    const normalize=s=>String(s).replace(/[\s，,。；;：:]/g,'');
    const detail=normalize(lines[1])===normalize(lines[2])?'握持键控制跟随 · 扳机控制夹爪 · B 停止':lines[2];
    const compactHint=status && status.gripper_mode==='toggle'?(status.grasp_closing?'抓取保持 · 再扣扳机张开 · B停止':'扣一下抓取 · 再扣张开 · B停止'):
      status && status.mode==='pose'?'姿态绝对对齐 · 扳机开合 · A精细 · B停止':'扳机调夹爪 · A 精细 · B 停止';
    text(compact?compactHint:detail,compact?144:192,30,'#cfdfdf',compact?1:2,38);
    const keys=`A ${controls.arm?'按下':'松开'}   B ${controls.stop?'按下':'松开'}   握持 ${Math.round(controls.grip*100)}%   扳机 ${Math.round(controls.trigger*100)}%`;
    text(keys,compact?190:284,30,'#b6d0c4');
    text(lines[4],compact?230:330,26,controls.emulated?'#ffd393':'#b6d0c4',2,32);
  }
  const api={layout,vertices,wrap,paint};
  if(typeof module!=='undefined' && module.exports)module.exports=api;
  else globalThis.PicoHUD=api;
})();
