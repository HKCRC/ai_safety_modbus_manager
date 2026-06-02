import logging
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
from pymodbus.server.sync import StartTcpServer

# 开启日志，方便观察
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.INFO)

def run_simulator():
    """
    配置各个设备的模拟寄存器。
    为防止 C++ 端请求未初始化的块报错，这次把 co (Coils) 和 di (Discrete Inputs) 全部初始化。
    """
    
    # 辅助函数：快速创建全 0 的数据块
    def make_block(size=100):
        return ModbusSequentialDataBlock(0, [0]*size)

    # ==========================================
    # 1. 小车 (Trolley)
    # ==========================================
    trolley_di = make_block()
    trolley_co = make_block()  
    trolley_ir = make_block()
    trolley_hr = make_block()
    
    # 预设 Coils (16 - 28 是状态)
    trolley_co.setValues(20, [0])  # COIL_STATUS_POWER_SAVING_MODE
    trolley_co.setValues(21, [1])  # COIL_STATUS_BMS_CHARGING (充电中)
    trolley_co.setValues(22, [0])  # COIL_STATUS_BRIDGE_PING_FAIL
    trolley_co.setValues(23, [0])  # COIL_STATUS_BMS_READ_FAIL
    trolley_co.setValues(24, [0])  # COIL_STATUS_MPPT_READ_FAIL
    trolley_co.setValues(25, [0])  # COIL_STATUS_LASER_1_READ_FAIL
    trolley_co.setValues(26, [0])  # COIL_STATUS_LASER_2_READ_FAIL
    trolley_co.setValues(27, [0])  # COIL_STATUS_CCTV_PING_FAIL
    trolley_co.setValues(28, [0])  # COIL_STATUS_SLEEP_MODE
    
    # 预设 Holding Registers
    trolley_hr.setValues(0, [1])       # HREG_WORK_MODE
    trolley_hr.setValues(6, [1])       # HREG_DEVICE_STATUS (1: active)

    # 预设 Input Registers
    trolley_ir.setValues(17, [8500])   # IREG_BATTERY_LEVEL (8500 -> 85%)
    trolley_ir.setValues(18, [120])    # IREG_DISCHARGE_TIME (120 min)
    trolley_ir.setValues(19, [0xFFFF]) # IREG_CHARGE_TIME (无效)
    trolley_ir.setValues(20, [1250])   # IREG_BATTERY_VOLTAGE (12.5V)
    trolley_ir.setValues(21, [500])    # IREG_BATTERY_CURRENT (5A)
    trolley_ir.setValues(25, [1])      # IREG_MPPT_CHARGE_STATUS (1: Charging)
    trolley_ir.setValues(32, [1500])   # IREG_LASER_DISTANCE_1 (1500mm)
    trolley_ir.setValues(33, [1550])   # IREG_LASER_DISTANCE_2 (1550mm)


    # ==========================================
    # 2. 吊钩 (Hook)
    # ==========================================
    hook_di = make_block(150)
    hook_co = make_block(150)
    hook_ir = make_block(150)
    hook_hr = make_block(150)
    
    hook_hr.setValues(0, [0])          # Control strobe
    hook_hr.setValues(100, [1])        # Strobe status
    hook_hr.setValues(101, [0])        # Speaker status
    hook_hr.setValues(102, [8500])     # Battery level 85%


    # ==========================================
    # 3. 编码器 (Encoder)
    # ==========================================
    encoder_di = make_block(10)
    encoder_co = make_block(10)
    encoder_ir = make_block(10)
    encoder_hr = make_block(10)
    
    encoder_hr.setValues(0, [0x0001, 0x1234]) # 模拟 32 位数据


    # ==========================================
    # 4. 组装 Slave Context
    # ==========================================
    trolley_context = ModbusSlaveContext(di=trolley_di, co=trolley_co, ir=trolley_ir, hr=trolley_hr, zero_mode=True)
    hook_context    = ModbusSlaveContext(di=hook_di,    co=hook_co,    ir=hook_ir,    hr=hook_hr, zero_mode=True)
    encoder_context = ModbusSlaveContext(di=encoder_di, co=encoder_co, ir=encoder_ir, hr=encoder_hr, zero_mode=True)

    # 映射 Slave ID
    slaves = {
        1: trolley_context,
        2: hook_context,
        3: encoder_context
    }
    context = ModbusServerContext(slaves=slaves, single=False)

    print("🚀 启动 AI Safety Modbus TCP 模拟器... 正在监听 0.0.0.0:5020")
    
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'AI Safety'
    identity.ModelName = 'Crane Simulator'

    StartTcpServer(
        context=context,
        identity=identity,
        address=("0.0.0.0", 5020),
    )


if __name__ == "__main__":
    run_simulator()