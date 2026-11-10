/* Spatial kinematic visualization shared by WebXR and desktop verification. */
(() => {
  'use strict';
  const add=(a,b)=>a.map((v,i)=>v+b[i]);
  const sub=(a,b)=>a.map((v,i)=>v-b[i]);
  const mul=(a,s)=>a.map(v=>v*s);
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const unit=a=>mul(a,1/(Math.hypot(...a)||1));
  function createAnchor(matrix) {
    const forward=unit([-matrix[8],0,-matrix[10]]);
    if(Math.hypot(...forward)<.5)forward.splice(0,3,0,0,-1);
    return Object.freeze({origin:Object.freeze([matrix[12]+forward[0]*.55,matrix[13]-.45,matrix[14]+forward[2]*.55]),
      // Heading places the display only. Robot axes stay in reference space,
      // matching controller.XR_TO_ROBOT even if the user enters facing sideways.
      forward:Object.freeze([0,0,-1]),left:Object.freeze([-1,0,0]),scale:1.5});
  }
  function robotPoint(anchor,p){
    return anchor.origin.map((v,i)=>v+anchor.scale*(anchor.forward[i]*p[0]+anchor.left[i]*p[1]+(i===1?p[2]:0)));
  }
  function clipPoint(matrix,p){
    const q=[...p,1];return [0,1,2,3].map(row=>q.reduce((v,x,col)=>v+matrix[col*4+row]*x,0));
  }
  function buildGeometry(status,anchor,controllerPose,controllerValid){
    const mesh=[],lines=[],vertex=(out,p,c)=>out.push(...p,...c);
    const triangle=(a,b,c,color)=>{vertex(mesh,a,color);vertex(mesh,b,color);vertex(mesh,c,color);};
    const line=(a,b,color)=>{vertex(lines,a,color);vertex(lines,b,color);};
    function tube(a,b,r,color){
      const delta=sub(b,a);if(Math.hypot(...delta)<1e-7)return;
      const axis=unit(delta),u=unit(cross(axis,Math.abs(axis[1])<.8?[0,1,0]:[1,0,0])),v=cross(axis,u);
      const circle=t=>mul(add(mul(u,Math.cos(t)),mul(v,Math.sin(t))),r);
      for(let i=0;i<10;i++){
        const ra=circle(i*Math.PI/5),rb=circle((i+1)*Math.PI/5);
        const p=add(a,ra),q=add(a,rb),s=add(b,ra),t=add(b,rb);
        const shade=.65+.3*(Math.sin((i+.5)*Math.PI/5)*.7+Math.cos((i+.5)*Math.PI/5)*.3);
        const c=color.map(x=>x*shade);
        triangle(p,q,s,c);triangle(s,q,t,c);triangle(a,q,p,c);triangle(b,s,t,c);
      }
    }
    function ball(center,r,color){
      const point=(lat,lon)=>add(center,[r*Math.cos(lat)*Math.cos(lon),r*Math.sin(lat),r*Math.cos(lat)*Math.sin(lon)]);
      for(let j=0;j<5;j++)for(let i=0;i<10;i++){
        const a=-Math.PI/2+j*Math.PI/5,b=a+Math.PI/5,c=i*Math.PI/5,d=c+Math.PI/5;
        const shade=.6+.35*(j+1)/5,col=color.map(x=>x*shade);
        triangle(point(a,c),point(a,d),point(b,c),col);triangle(point(b,c),point(a,d),point(b,d),col);
      }
    }
    const world=p=>robotPoint(anchor,p),green=[.27,.82,.61],orange=[1,.65,.22];
    for(let i=0;i<=10;i++){
      const x=-.1+i*.05,y=-.25+i*.05;
      line(world([x,-.25,0]),world([x,.25,0]),[.22,.35,.4]);
      line(world([-.1,y,0]),world([.4,y,0]),[.22,.35,.4]);
    }
    const origin=world([0,0,0]);
    for(const [point,color] of [[[.12,0,0],[.9,.25,.25]],[[0,.12,0],[.3,.85,.4]],[[0,0,.12],[.3,.5,1]]])
      tube(origin,world(point),.002,color);
    tube(add(origin,[0,-.025,0]),origin,.055,[.3,.42,.46]);
    const points=status && status.points || [];
    for(let i=1;i<points.length;i++)tube(world(points[i-1]),world(points[i]),.013,green);
    for(const point of points)ball(world(point),.019,[.75,.88,.86]);
    let tcpWorld=null,fingers=[];
    if(status && status.tcp_m){
      const tcp=status.tcp_m,R=status.tcp_rotation || [[1,0,0],[0,1,0],[0,0,1]];
      tcpWorld=world(tcp);ball(tcpWorld,.012,orange);
      const forward=R.map(row=>row[0]),side=R.map(row=>row[2]);
      const opening=.012+.025*Math.max(0,Math.min(1,status.gripper||0));
      const ends=[-1,1].map(sign=>add(add(tcp,mul(forward,-.025)),mul(side,sign*opening)));
      tube(world(ends[0]),world(ends[1]),.007,orange);
      for(const end of ends){const tip=add(end,mul(forward,.055));tube(world(end),world(tip),.007,orange);fingers.push(world(tip));}
    }
    if(status && status.requested_m)ball(world(status.requested_m),.008,[1,.35,.25]);
    if(status && (status.motor || status.hardware && status.hardware.feedback_valid) && status.command_tcp_m){
      ball(world(status.command_tcp_m),.009,[.25,.8,1]);
      if(tcpWorld)line(tcpWorld,world(status.command_tcp_m),[.25,.7,.85]);
    }
    if(controllerPose && controllerPose.transform){
      const m=controllerPose.transform.matrix,p=[m[12],m[13],m[14]],color=controllerValid?[.2,.75,1]:[1,.5,.1];
      ball(p,.017,color);tube(p,add(p,[-m[8]*.13,-m[9]*.13,-m[10]*.13]),.004,color);
    }
    return {mesh:new Float32Array(mesh),lines:new Float32Array(lines),tcpWorld,fingers};
  }
  function createRenderer(gl){
    function shader(type,source){const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);
      if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;}
    const program=gl.createProgram();
    gl.attachShader(program,shader(gl.VERTEX_SHADER,'attribute vec3 p; attribute vec3 color; uniform mat4 projection; uniform mat4 eye; varying vec3 c; void main(){c=color;gl_Position=projection*eye*vec4(p,1.);}'));
    gl.attachShader(program,shader(gl.FRAGMENT_SHADER,'precision mediump float; varying vec3 c;void main(){gl_FragColor=vec4(c,1.);}'));
    gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));
    const mesh=gl.createBuffer(),lines=gl.createBuffer(),p=gl.getAttribLocation(program,'p'),color=gl.getAttribLocation(program,'color');
    const projection=gl.getUniformLocation(program,'projection'),eye=gl.getUniformLocation(program,'eye');
    let meshCount=0,lineCount=0;
    return {
      prepare(status,anchor,controllerPose,valid){
        const data=buildGeometry(status,anchor,controllerPose,valid);meshCount=data.mesh.length/6;lineCount=data.lines.length/6;
        gl.bindBuffer(gl.ARRAY_BUFFER,mesh);gl.bufferData(gl.ARRAY_BUFFER,data.mesh,gl.DYNAMIC_DRAW);
        gl.bindBuffer(gl.ARRAY_BUFFER,lines);gl.bufferData(gl.ARRAY_BUFFER,data.lines,gl.DYNAMIC_DRAW);
      },
      draw(view){
        gl.useProgram(program);gl.enable(gl.DEPTH_TEST);gl.depthMask(true);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);
        gl.uniformMatrix4fv(projection,false,view.projectionMatrix);gl.uniformMatrix4fv(eye,false,view.transform.inverse.matrix);
        gl.enableVertexAttribArray(p);gl.enableVertexAttribArray(color);
        function draw(buffer,mode,count){gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.vertexAttribPointer(p,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(color,3,gl.FLOAT,false,24,12);gl.drawArrays(mode,0,count);}
        draw(lines,gl.LINES,lineCount);draw(mesh,gl.TRIANGLES,meshCount);
        gl.disableVertexAttribArray(p);gl.disableVertexAttribArray(color);
      },
      dispose(){gl.deleteBuffer(mesh);gl.deleteBuffer(lines);gl.deleteProgram(program);}
    };
  }
  const api={createAnchor,robotPoint,clipPoint,buildGeometry,createRenderer};
  if(typeof module!=='undefined' && module.exports)module.exports=api;
  else globalThis.PicoScene=api;
})();
