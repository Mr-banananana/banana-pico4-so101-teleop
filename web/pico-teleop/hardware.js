/* Passive commissioning controls. Never mark a measurement verified by default. */
(() => {
  function create(send,role){
    let supplyRestored=false,virtual=false,lastCalibrated=null,lastHardware=null;
    const $=id=>document.getElementById(id),names=['肩部旋转','肩部俯仰','肘部','腕部俯仰','腕部旋转'];
    const jointNames={shoulder_pan:names[0],shoulder_lift:names[1],elbow_flex:names[2],wrist_flex:names[3],wrist_roll:names[4]};
    const signs=names.map((name,i)=>{
      const label=document.createElement('label');label.textContent=`ID${i+1} ${name} `;
      const select=document.createElement('select');
      for(const value of [1,-1]){const option=document.createElement('option');option.value=String(value);option.textContent=value===1?'正向':'反向';select.appendChild(option);}
      label.appendChild(select);$('cal-signs').appendChild(label);return select;
    });
    const payload=()=>({signs:signs.map(s=>Number(s.value)),pose_verified:$('cal-pose').checked,
      directions_verified:$('cal-directions').checked,workcell_verified:virtual || $('cal-workcell').checked,
      supply_version:$('cal-supply').value});
    function updateEnable(){
      const h=lastHardware;if(!h)return;
      $('hardware-enable').disabled=role!=='monitor' || !h.calibrated || !h.feedback_valid || h.enabled || !!(h.outside_joints && h.outside_joints.length) || (!virtual && !$('cal-workcell').checked);
    }
    $('cal-workcell').onchange=updateEnable;
    for(const [id,action] of Object.entries({'cal-reference':'cal_reference','cal-zero':'cal_zero','cal-range':'cal_range',
      'cal-closed':'cal_closed','cal-open':'cal_open','cal-save':'cal_save','hardware-enable':'hardware_enable','hardware-disable':'hardware_disable'})){
      $(id).onclick=()=>send({type:'control',action,...payload()});
      $(id).disabled=role!=='monitor';
    }
    for(const select of signs)select.onchange=()=>send({type:'control',action:'cal_signs',...payload()});
    $('virtual-demo').onclick=()=>send({type:'control',action:'virtual_demo'});
    return {render(s){
      const h=s.hardware;$('hardware-panel').hidden=!h || role!=='monitor';if(!h)return;
      lastHardware=h;
      virtual=!!h.is_virtual;$('software-guide').hidden=!virtual;$('passive-calibration').hidden=virtual;
      $('handover-steps').hidden=virtual;$('workcell-confirmation').hidden=virtual;
      if(lastCalibrated!==h.calibrated){$('passive-calibration').open=!h.calibrated;lastCalibrated=h.calibrated;}
      const source=String(h.calibration_source || '已保存的标定').split(/[\\/]/).pop();
      $('calibration-summary').textContent=virtual?'虚拟模式使用独立标定，只验证软件链路。':h.calibrated?
        `已加载 ${source}，无需重新标定。它将电机读数换算成关节角度；仍需核对模型与实物姿态一致。`:
        '尚未加载有效标定，暂不能接管。标定用于把电机读数换算成关节角度；请检查文件或展开下方手动标定。';
      $('hardware-heading').textContent=virtual?'软件链路验证':'真机接管向导';
      $('handover-feedback').textContent=!h.calibrated?'缺少有效标定，尚不能换算关节反馈；先检查标定文件。':
        !h.feedback_valid?'电机反馈无效或过期：先恢复六路反馈，当前不能接管。':'已加载标定并收到有效电机反馈。';
      $('virtual-demo').disabled=!virtual || role!=='monitor' || !!(s.software_demo && s.software_demo.running);
      const demo=s.software_demo;
      $('software-result').textContent=!demo?'点击演示：自动同步时钟、授权、握持跟随、停止并断开':demo.running?'正在运行虚拟手柄演示…':
        demo.ok?`软件验证通过：末端移动 ${(demo.displacement_m*1000).toFixed(1)}mm；已停止锁存`:'软件验证未通过：'+(demo.message || '请导出诊断');
      if(!supplyRestored && h.confirmed_supply){$('cal-supply').value=h.confirmed_supply;supplyRestored=true;}
      $('hardware-state').textContent=virtual?(h.ready?'虚拟舵机已接管':'虚拟舵机就绪 · 软件模式'):h.ready?'电机已接管':h.calibrated?'已标定 · 未接管':'只读标定 · 未使能';
      $('hardware-message').textContent=h.fault || h.calibration_error || (!h.calibrated?'请先检查标定文件；没有有效文件时，再按高级手动标定说明采集并保存':!h.feedback_valid?'电机反馈无效或过期：请检查机械臂电源、USB 与总线连接，恢复后再接管':h.outside_joints && h.outside_joints.length?'当前关节超出保守限位：'+h.outside_joints.map(name=>jointNames[name] || name).join('、')+'；保持未接管，托住手动移回中间':h.ready?'现在可在头显用 A 授权；握持后跟随手柄的位置与当前朝向':
        h.calibrated?'核对当前工位后，点击按当前姿态接管电机':'请先采集零位、活动范围与夹爪端点，再逐轴核对方向');
      if(virtual && !h.fault && !h.calibration_error)$('hardware-message').textContent='点击虚拟手柄演示，检查时钟、授权、跟随、反馈和停止；全部在软件内运行。';
      if(h.workspace_preparation && !h.fault)$('hardware-message').textContent=h.workspace_preparation.message;
      if(virtual)$('commission-help').textContent='演示可重复运行，结束后保持停止锁存。虚拟响应只用于检查软件流程，不代表实机速度或负载表现。';
      else $('commission-help').textContent='接管会使电机维持当前姿态，不会自动回零。需要新鲜反馈、有效标定和工位确认；折叠在桌面或超出限位时不能授权跟随。退出接管保持已使能电机，不自动卸力；总线失联时软件无法保证保持。';
      if(h.workspace_preparation)$('commission-help').textContent='已开启自动进入工作区：启动后接管并进入最近可达的区内位置，完成后可用 A 授权。遥操作保留边界缓冲；B 停止会取消自动准备，不会自行重新启动。';
      $('hardware-data').textContent=[`${h.port} ｜反馈年龄 ${h.feedback_age_ms==null?'无反馈':h.feedback_age_ms+'ms'}`,
        '原始位置：'+JSON.stringify(h.raw),'供电原始值：'+JSON.stringify(h.voltage_raw),
        '已核验温度 / ℃：'+JSON.stringify(h.validated_temperature),
        '温度核验年龄 / ms：'+JSON.stringify(h.temperature_age_ms),
        '本次温度测量 / ℃：'+JSON.stringify(h.temperature),
        '温度批量原始值：'+JSON.stringify(h.readback && h.readback.bulk_temperature_raw),
        '温度独立复读：'+JSON.stringify(h.readback && h.readback.temperature_confirm_raw),
        '最近温度复核：'+JSON.stringify(h.transport && h.transport.temperature_confirmation &&
          {count:h.transport.temperature_confirmation.count,source:h.transport.temperature_confirmation.last && h.transport.temperature_confirmation.last.source,
            recovered:h.transport.temperature_confirmation.last && h.transport.temperature_confirmation.last.recovered}),
        '扭矩使能：'+JSON.stringify(h.torque),
        '零位：'+JSON.stringify(h.commission.zero_counts),'记录中：'+h.commission.recording,
        '已记录范围：'+JSON.stringify(h.commission.range_counts),
        '夹爪闭合 / 打开：'+h.commission.gripper_closed+' / '+h.commission.gripper_open].join('\n');
      if(h.calibration_source)$('hardware-data').textContent+='\n标定来源：'+h.calibration_source;
      $('hardware-enable').textContent=virtual?'接管虚拟舵机':'按当前姿态接管电机';
      $('cal-save').hidden=virtual;
      updateEnable();
      for(const id of ['cal-reference','cal-zero','cal-range','cal-closed','cal-open','cal-save'])$(id).disabled=role!=='monitor' || h.enabled;
    }};
  }
  globalThis.PicoHardwarePanel={create};
})();
