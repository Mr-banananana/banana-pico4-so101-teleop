/* Pure input normalization, shared by browser and offline Node checks. */
(() => {
  function buttonValue(button) {
    if (!button || !Number.isFinite(button.value)) return 0;
    return Math.max(0, Math.min(1, button.value || (button.pressed ? 1 : 0)));
  }
  function snapshot(source, pose, mapping, confirmed, options={}) {
    const gp = source && source.gamepad;
    const mapped = gp && (gp.mapping === 'xr-standard' || confirmed);
    const buttons = gp ? gp.buttons : [];
    const emulated=!!(pose && pose.emulatedPosition);
    const valid = !!(source && source.gripSpace && pose && (!emulated || options.allowEmulated===true) && mapped
      && buttons[mapping.trigger] && buttons[mapping.grip]);
    const p = valid ? pose.transform.position : null;
    const q = valid ? pose.transform.orientation : null;
    const read = i => buttonValue(buttons[i]);
    return {
      valid,
      emulated,
      position: p ? [p.x, p.y, p.z] : null,
      orientation: q ? [q.x, q.y, q.z, q.w] : null,
      grip: read(mapping.grip), trigger: read(mapping.trigger),
      precision: read(mapping.precision) > .5 && read(mapping.grip) >= .35,
      stop: read(mapping.stop) > .5,
      // Keep A's raw state in every context: masking it by grip would invent
      // a new authorization edge when grip is released with A still held.
      arm: read(mapping.arm) > .5
    };
  }
  function stateLabel(state){return ({WAITING:'等待输入',READY:'待授权',ARMED:'已授权',FOLLOWING:'跟随中',STOPPED:'已停止'})[state] || '正在连接';}
  function hudLines(status,controls,options={}){
    if(options.disconnected)return ['SO101 预览｜连接已断开','服务连接中断或尚无新状态，请检查电脑连接',
      '重新连接后等待新反馈；不会自动恢复授权','当前按键与位置状态不可用','模型保留最后状态，仅供参考，不代表实时反馈'];
    const state=status && status.state,stopped=state==='STOPPED',armed=state==='ARMED' || state==='FOLLOWING';
    const motor=!!(status && status.motor);
    const hardware=status && status.hardware;
    const preparation=status && (status.workspace_preparation || hardware && hardware.workspace_preparation);
    const preparing=preparation && (['enabling','stabilizing','moving','latching','blocked'].includes(preparation.state) || preparation.running);
    const handRequired=status && status.mode==='pose' && status.control_reference && status.control_reference.hand_required;
    const title=(options.passthrough?'SO101 实景透视｜':'SO101 VR｜')+(hardware?(hardware.is_virtual?'虚拟舵机软件验证｜':'真机｜'):motor?'舵机响应，仅仿真未标定｜':'仅仿真｜')+stateLabel(state);
    const moving=motor && status.motor.velocity_rad_s.some(v=>Math.abs(v)>.05);
    const main=status && status.control_reference_error?status.control_reference_error:
      hardware && hardware.fault?hardware.fault:
      stopped?'松开握持键/扳机/B，长按A解除：'+Number(status.unlock_hold_s||0).toFixed(1)+' / 2秒':
      preparing && !stopped?preparation.message:
      handRequired && !stopped?'手柄竖直、松开握持和扳机，短按 A 只记录参考':
      hardware && !hardware.ready?(hardware.fault || (hardware.is_virtual?'先在电脑启动虚拟演示或接管虚拟舵机':!hardware.calibrated?'电脑尚未加载有效标定，请检查接管向导':!hardware.feedback_valid?'电机反馈无效，请在电脑检查电源与连接':'已加载标定：在电脑确认工位并按当前姿态接管电机')):
      !controls.valid?'右手柄位置未就绪，请拿到头显前方唤醒':
      !armed && moving?'指令已保持，舵机正在减速；稳定后再授权':
      !armed?'松开握持键和扳机，再短按A授权':
      status.thermal_pause?'温度单点待核验，保持中；有效反馈恢复后自动继续':
      status.constraint_pause?'目标暂受限，已保持授权；正在自动恢复':
      status.workspace_limited?'到达工作区边缘：沿边滑动或反向移动即可继续':
      status.joint_limit_limited?'到达当前可达边缘：不可达位移已吸收；反向移动即可继续':
      status.tracking_limited?(status.reason || '电机正在追赶，已自动减速；请放慢手部移动'):
      state==='FOLLOWING'?(status.mode==='pose'?(status.pose_policy==='split_roll'?'位置＋翻滚；夹爪尽量水平；松开握持保持，B停止':status.pose_policy==='balanced'?'位置＋三轴朝向协同；五轴受限时自动折中':'位置＋绝对旋转：松开握持键保持，B停止'):'正在跟随右手：松开握持键保持，B停止'):
      status && status.mode==='pose'?'已授权：握持后移动、拧转手柄控制夹爪':'已授权：按住握持键，移动右手控制机械臂';
    const hint=preparing && !stopped?'自动准备完成后恢复手柄控制｜B 软件停止':
      handRequired && !stopped?'本次 A 不会使能或移动电机；记录后松开 A，接管后再短按 A 授权':
      stopped?'解除后松开A，再短按A授权；不会自动恢复运动':
      !armed && status && status.reason?status.reason:
      status && status.gripper_mode==='toggle'?(status.grasp_closing?'已锁存抓取｜再扣扳机张开｜B停止':'扣一下扳机抓取｜松开仍保持｜B停止'):
      (status && status.mode==='pose'?'再握持追到当前朝向｜扳机开合｜A精细｜B软件停止':
      '扳机控制夹爪｜握持时按A精细｜B'+(hardware?'软件停止':motor?'减速停止':'立即停止'));
    const keys='按键：A'+(controls.arm?'按下':'松开')+'  B'+(controls.stop?'按下':'松开')+
      '  握持'+Math.round(controls.grip*100)+'%  扳机'+Math.round(controls.trigger*100)+'%';
    const quality=controls.emulated?'浏览器估算位姿｜仅仿真允许，真实机械臂禁用':
      hardware?(hardware.is_virtual?'显示虚拟反馈｜没有真实电机连接':hardware.feedback_valid?(options.passthrough?'电机反馈有效｜请以眼前实物为准':'显示电机反馈｜模型无法显示现场障碍物'):'当前无有效电机反馈，暂不可操作'):
      options.passthrough?'实景透视输入｜当前控制仿真，未接管真实电机':
      controls.valid?'位置追踪有效｜当前为三维仿真，未接真实电机':'等待位置追踪｜当前为三维仿真，未接真实电机';
    return [title,main,hint,keys,quality];
  }
  function boundedSend(socket, message, onBacklog) {
    if (!socket || socket.readyState !== 1) return false;
    if (socket.bufferedAmount > 32768) {
      onBacklog();
      // Browser callers may use 1000 or application codes 3000..4999 only.
      socket.close(4001, 'input backlog');return false;
    }
    socket.send(JSON.stringify(message));return true;
  }
  const api = {buttonValue, snapshot, boundedSend,hudLines,stateLabel};
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else globalThis.PicoInput = api;
})();
