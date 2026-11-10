"""Read-only commissioning evidence; importing this module opens no device.

Raw STS register layout follows the pinned FTServo SDK. Model IDs, voltage
variants, encoder conventions and joint assignments are not inferred here.
"""
from .timing import clock as time


def select_board(ports, identity):
    matches=[p for p in ports if p.vid==identity['vid'] and p.pid==identity['pid']
             and p.serial_number==identity['serial_number']]
    if len(matches)!=1:
        raise ValueError('已登记控制板未连接或身份不唯一；不会改选其他串口')
    return matches[0]


# name, address, byte width. Values stay raw until the motor variant and its
# register units are confirmed. Reading a register never changes its value.
REGISTERS=(('min_position',9,2),('max_position',11,2),
           ('p_coefficient',21,1),('d_coefficient',22,1),('i_coefficient',23,1),
           ('minimum_startup_force',24,2),('cw_dead_zone',26,1),('ccw_dead_zone',27,1),
           ('homing_offset',31,2),('operating_mode',33,1),
           ('torque_enabled',40,1),('acceleration',41,1),
           ('goal_position',42,2),('goal_speed',46,2),
           ('present_position',56,2),('present_speed',58,2),
           ('present_load',60,2),('present_voltage',62,1),
           ('present_temperature',63,1),('present_current',69,2),
           ('maximum_velocity_limit',84,1),('maximum_acceleration',85,1))


def transact(packet,request):
    """Release the pinned SDK's busy flag if this transaction raises.

    Serial ownership remains with one worker. An already-busy transaction is
    never unlocked here; the SDK must report its normal COMM_PORT_BUSY.
    """
    port=getattr(packet,'portHandler',None)
    if port is None or port.is_using:return packet.txRxPacket(request)
    try:return packet.txRxPacket(request)
    finally:port.is_using=False


class STSRegisterReader:
    """Restricted SDK facade validating status size before slicing parameters.

    The pinned SDK readTxRx slices a short packet through its checksum. Use its
    packet transport instead and validate the complete frame, without changing
    the pinned source or exposing any write operations to the inspector.
    """
    def __init__(self, packet):self._packet=packet

    def readTxRx(self,motor_id,address,length):
        allowed={(3,2),(56,15),(40,8),(40,31)}|{(a,n) for _,a,n in REGISTERS}
        if motor_id not in range(1,7) or (address,length) not in allowed:
            raise ValueError('仅允许 ID1～6 的已列明只读寄存器')
        request=[0,0,motor_id,4,2,address,length,0]  # INST_READ only
        response,communication,error=transact(self._packet,request)
        if communication!=0:return [],communication,error
        if (response is None or len(response)<6 or response[:2]!=[255,255]
                or response[2]!=motor_id or (sum(response[2:])&255)!=255):
            return [],-7,error
        if error:return [],0,error
        if len(response)!=length+6 or response[3]!=length+2:
            return [],-7,0
        return response[5:-1],0,0


def _read(packet, motor_id, address, length):
    start=time.monotonic()
    data,communication,error=packet.readTxRx(motor_id,address,length)
    valid=(communication==0 and error==0 and len(data)==length)
    return dict(address=address,length=length,value_raw=int.from_bytes(bytes(data),'little') if valid else None,
                communication_result=int(communication),device_error=int(error),valid=valid,
                elapsed_ms=round((time.monotonic()-start)*1000,3))


def inspect_motors(packet):
    start=time.monotonic();rows=[];responding=[]
    for motor_id in range(1,7):
        model=_read(packet,motor_id,3,2)
        row=dict(id=motor_id,registers=dict(model=model))
        if model['communication_result']==0:responding.append(motor_id)
        if model['valid']:
            for name,address,length in REGISTERS:
                row['registers'][name]=_read(packet,motor_id,address,length)
        row['feedback_ok']=all(row['registers'].get(name,{}).get('valid',False)
                               for name in ('model','present_position','present_voltage','present_temperature'))
        rows.append(row)
    return dict(read_only=True,protocol='FTServo STS register layout (motor variant unverified)',
                responding_ids=responding,motors=rows,feedback_complete=all(r['feedback_ok'] for r in rows),
                motion_ready=False,calibrated=False,elapsed_ms=round((time.monotonic()-start)*1000,3),
                note='仅原始寄存器；未确认电机型号/供电版本/关节映射，未发位置、使能、ID 或 EEPROM 写入。')
