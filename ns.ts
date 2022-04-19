

/*****************************************************************************************************************************************
 *    纳深类 *****************************************************************************************************************************
 ****************************************************************************************************************************************/
//% color="#de7372" weight=21 icon="\uf185"
namespace AISTEAM
{
    //设备类型
    const Port_A = 1  //
    const Port_B = 2  //
    const Port_C = 3  //
    const Port_D = 4  //
    const Port_E = 5  //
    const MOTOR_TYPE = 6  //
    const ONEWIRE = 8  //
    const SERVO5V = 9  //
    const RS485_1 = 10 // 
    const RS485_2 =11  //
    
    //设备类型
    const DEVICEYTPE_ANALOGIN = 1  //模拟量输入类
    const DEVICEYTPE_DIGITALIN = 2 //数字量输入类
    const DEVICEYTPE_DIGITALOUT = 3//数字量输出类
    const DEVICEYTPE_BUS = 4       //总线类
    //设备型号(模拟量输入)
    const SENSOR_PHOTOSENSITIVE = 1//光敏传感器
    const SENSOR_VOICE = 2         //声音传感器
    const SENSOR_IRDISTANCE = 3    //红外避障
    //设备型号(数字量输入)
    const SENSOR_FLAM = 1 //火焰传感器
    const SENSOR_TRACK = 2//单路循迹传感器
    const SENSOR_CRASH = 3//碰撞开关传感器
    //设备型号(数字量输出)
    const DEVIDE_BEEP = 1        //蜂鸣器
    const DEVIDE_LED = 2         //单个LED
    const DEVIDE_DIGITALTUBE = 3 //数码管
    const DEVIDE_OPENMOTOR = 4   //开环减速电机
    const DEVIDE_FAN = 5         //风扇
    const DEVIDE_FULLCOLORLED = 6//全彩LED
    const DEVIDE_COLSEMOTOR = 7  //闭环减速电机
    const DEVIDE_ANALOGSERVO = 8 //模拟舵机
    //设备类型(总线类)
    const SENSOR_TH = 1        //温湿度传感器
    const SENSOR_ULTRASONIC = 2//超声波传感器    
    const SENSOR_FIVEGRAY = 3    //集成灰度
    const DIGITALSERVO = 4     //数字舵机
    const EXPANSIONMOTOR = 5   //拓展电机模块
    const SENSOR_IR = 6        //红外接收器  

    //端口
    const PORT_EXPANSIONMOTOR = 8//单总线口
    const PORT_ANALOGSERVO = 9   //模拟舵机口
    //帧头
    const FRAMEHEADER_BYTE = 0X5A//帧头字节
    //操作类型
    const OPERATION_READ = 1; //读操作
    const OPERATION_WRITE = 2;//写操作
    //SPI初始化标志
    let spi_initialized = false//true@已初始化,false@未初始化
    //拓展电机控制器相关开始
    const HEADER_1BYTE = 0X55//帧头第一字节
    const HEADER_2BYTE = 0XAA//帧头第二字节
    //读取
    const SPI_READDUMMY  = 0xff
    //数据量
    const DATALEN_SET_CAR_SPEED_ANGLE_TIME = 8   //设置麦克纳姆轮速度、角度、时间
    const DATALEN_SET_CAR_SPEED_ANGLE = 4        //设置麦克纳姆轮速度、角度
    const DATALEN_SET_ROTAT_SPEED_TIME = 6       //设置麦克纳姆轮旋转速度、时间
    const DATALEN_SET_ROTAT_SPEED = 2            //设置麦克纳姆轮旋转速度
    const DATALEN_SET_EXPANDMOTOE_SPEED = 9      //设置拓展模块电机速度
    const DATALEN_SET_SINGLEMOTOR_SPEED = 3      //设置单个电机速度
    const DATALEN_SET_SINGLEMOTOR_SPEED_TIME = 7 //设置单个电机速度、时间
    const DATALEN_SET_FOUR_MOTOR_COEF = 17//设置四个电机功率
    const DATALEN_STOP_SINGLEMOTOR = 2           //停止单个电机
    const DATALEN_STOP_ALLMOTOR = 2              //停止所有电机
    const DATALEN_CLEAR_SINGLEMOTOR_ENCODER = 2  //清除单个电机编码器的值
    const DATALEN_GET_SINGLEMOTOR_ENCODER = 2    //获取单个电机编码器的值
    //命令码
    const CMD_SET_CAR_SPEED_ANGLE_TIME = 1    //设置麦克纳姆轮速度、角度、时间
    const CMD_SET_CAR_SPEED_ANGLE = 2         //设置麦克纳姆轮速度、角度
    const CMD_SET_ROTAT_SPEED_TIME = 3        //设置麦克纳姆轮旋转速度、时间
    const CMD_SET_ROTAT_SPEED = 4             //设置麦克纳姆轮旋转速度
    const CMD_SET_EXPANDMOTOE_SPEED = 5       //设置拓展模块电机速度
    const CMD_SET_SINGLEMOTOR_SPEED = 6       //设置单个电机速度
    const CMD_SET_SINGLEMOTOR_SPEED_TIME = 7  //设置单个电机速度、时间
    const CMD_SET_FOUR_MOTOR_COEF = 8         //设置马达功率
    const CMD_STOP_SINGLEMOTOR = 9            //停止电机
    const CMD_STOP_ALLMOTOR = 10            //停止所有电机
    const CMD_CLEAR_SINGLEMOTOR_ENCODER = 11  //清除单个电机编码器的值
    const CMD_GET_SINGLEMOTOR_ENCODER = 12    //获取单个电机编码器的值
 
  




  //电机拓展模块id
  export enum  MOTOR_EXT

  {
      //% blockId="MOTOR_EXT1" block="1"
      MOTOR_EXT1 = 1,
      //% blockId="MOTOR_EXT2" block="2"
      MOTOR_EXT2 = 2,
      //% blockId="MOTOR_EXT3" block="3"
      MOTOR_EXT3 = 3,
      //% blockId="MOTOR_EXT4" block="4"
      MOTOR_EXT4 = 4,
 
  }

    //模拟输入口
    export enum AnalogInputPort
    {
        //% blockId="AnalogPort_IO1" block="A1"
        AnalogPort_IO1 = 1,
        //% blockId="AnalogPort_IO2" block="A2"
        AnalogPort_IO2 = 2,
        //% blockId="AnalogPort_IO3" block="A3"
        AnalogPort_IO3 = 3,
        //% blockId="AnalogPort_IO4" block="A4"
        AnalogPort_IO4 = 4,
        //% blockId="AnalogPort_IO5" block="A5"
        AnalogPort_IO5 = 5
    }
    //数字量输入口
    export enum DigitalInputPort
    {
        //% blockId="DigitalInputPort_IO1" block="A1"
        DigitalInputPort_IO1 = 1,
        //% blockId="DigitalInputPort_IO2" block="A2"
        DigitalInputPort_IO2 = 2,
        //% blockId="DigitalInputPort_IO3" block="A3"
        DigitalInputPort_IO3 = 3,
        //% blockId="DigitalInputPort_IO4" block="A4"
        DigitalInputPort_IO4 = 4,
        //% blockId="DigitalInputPort_IO5" block="A5"
        DigitalInputPort_IO5 = 5
    }
    //数字量输出口
    export enum DigitalOutputPort
    {
        //% blockId="DigitalOutputPort_IO1" block="A1"
        DigitalOutputPort_IO1 = 1,
        //% blockId="DigitalOutputPort_IO2" block="A2"
        DigitalOutputPort_IO2 = 2,
        //% blockId="DigitalOutputPort_IO3" block="A3"
        DigitalOutputPort_IO3 = 3,
        //% blockId="DigitalOutputPort_IO4" block="A4"
        DigitalOutputPort_IO4 = 4,
        //% blockId="DigitalOutputPort_IO5" block="A5"
        DigitalOutputPort_IO5 = 5
    }

    //风扇端口
    export enum OutputPort
    {
        //% blockId="OutputPort_A1" block="A1"
        OutputPort_A1 = 1,
        //% blockId="OutputPort_A2" block="A2"
        OutputPort_A2 = 2,
        //% blockId="OutputPort_A3" block="A3"
        OutputPort_A3 = 3,
        //% blockId="OutputPort_A4" block="A4"
        OutputPort_A4 = 4,
        //% blockId="OutputPort_A5" block="A5"
        OutputPort_A5 = 5,
        //% blockId="OutputPort_IOALL" block="全部"
        OutputPort_ALL = 6
    }

    //有无火焰
    export enum ns_enFlame
    {
        //% blockId="ns_Flame" block="有火焰"
        ns_Flame = 1,
        //% blockId="ns_NoVFlame" block="无火焰"
        ns_NoFlame = 2
    }
    //白线或黑线
    export enum ns_enPatrolLine
    {
        //% blockId="ns_WhiteLine" block="白线"
        ns_WhiteLine = 1,
        //% blockId="ns_BlackLine" block="黑线"
        ns_BlackLine = 2
    }
    //有无障碍物
    export enum ns_Barriers
    {
        //% blockId="Barriers" block="有障碍物"
        Barriers = 1,
        //% blockId="NoBarriers" block="无障碍物"
        NoBarriers = 2
    }


    //红外遥控值
    export enum ns_IrValue
    {
        //% blockId="Irzero" block="0"
        IrValue_Zero = 0,
        //% blockId="IrONE" block="1"
        IrValue_One = 1,
        //% blockId="IrTWO" block="2"
        IrValue_Two = 2,
        //% blockId="IrTHREE" block="3"
        IrValue_Three = 3,
        //% blockId="IrFOUR" block="4"
        IrValue_Four = 4,
        //% blockId="IrSix" block="5"
        IrValue_Six = 5,
        //% blockId="IrSeven" block="6"
        IrValue_Seven = 6,
        //% blockId="IrEight" block="7"
        IrValue_Eight = 7,
        //% blockId="IrNine" block="8"
        IrValue_Nine = 8,
        //% blockId="IrTen" block="9"
        IrValue_Ten = 9,
        //% blockId="IrEleven" block="*"
        IrValue_Eleven = 10,
        //% blockId="Irtwelve" block="#"
        IrValue_twelve = 11,
        //% blockId="Irthirteen" block="▲"
        IrValue_thirteen = 12,
        //% blockId="Irfourteen" block="▼"
        IrValue_fourteen = 13,
        //% blockId="Irfifteen" block="◀"
        IrValue_fifteen = 14,
        //% blockId="Irsixteen" block="▶"
        IrValue_sixteen = 15,
        //% blockId="Irseventeen" block="OK"
        IrValue_seventeen = 16,
        

    }



    //蜂鸣器状态
    export enum BeepStatus
    {
        //% blockId="BEEP_ON" block="响"
        BEEP_ON = 1,
        //% blockId="BEEP_OFF" block="不响"
        BEEP_OFF = 2,
        //% blockId="BEEP_1" block="频谱1"
        BEEP_1 = 3,
        //% blockId="BEEP_2" block="频谱2"
        BEEP_2 = 4,
        //% blockId="BEEP_3" block="频谱3"
        BEEP_3 = 5,
        //% blockId="BEEP_4" block="频谱4"
        BEEP_4 = 6,
        //% blockId="BEEP_5" block="频谱5"
        BEEP_5 = 7,
    }




    //单个LED状态
    export enum LEDStatus
    {
        //% blockId="LED_ON" block="亮"
        LED_ON = 1,
        //% blockId="LED_OFF" block="灭"
        LED_OFF = 2
    }
    //风扇状态
    export enum FanStatus
    {
        //% blockId="FAN_ON" block="运转"
        FAN_ON = 1,
        //% blockId="FAN_OFF" block="停止"
        FAN_OFF = 2
    }

    export enum ToneStatus
    {
        //% blockId="Toneport_C1" block="C1"
        Toneport_C1 = 1,
        //% blockId="Toneport_C2" block="C2"
        Toneport_C2 = 2,
        //% blockId="Toneport_C3" block="C3"
        Toneport_C3 = 3,
        //% blockId="Toneport_C4" block="C4"
        Toneport_C4 = 4,
    }
    export enum BeatStatus
    {
        //% blockId="Beat_C1" block="1/2节拍"
        Beat_C1 = 1,
        //% blockId="Beat_C2" block="1/4节拍"
        Beat_C2 = 2,
        //% blockId="Beat_C3" block="1/8节拍"
        Beat_C3 = 3,
        //% blockId="Beat_C4" block="1/16节拍"
        Beat_C4 = 4,
    }

    export enum humtiyStatus
    {
        //% blockId="humtiy_C1" block="温度"
        humtiy_C1 = 1,
        //% blockId="humtiy_C2" block="湿度"
        humtiy_C2 = 2,
       
    }


    //电机口(开环电机)
    export enum MotorPort
    {
        //% blockId="MotorPort_M1" block="M1"
        MotorPort_M1 = 1,
        //% blockId="MotorPort_M2" block="M2"
        MotorPort_M2 = 2,

    }

    //停止电机口(开环电机)
    export enum MotorStopPort
    {
        //% blockId="MotorPort_M1" block="M1"
        MotorPort_M1 = 1,
        //% blockId="MotorPort_M2" block="M2"
        MotorPort_M2 = 2,
        //% blockId="MotorPort_M1M2" block="全部"
        MotorPort_M1M2 = 3,
    }
    //拓展电机口(闭环电机)
    export enum ExpandMotorPort
    {
        //% blockId="ExpandMotor_M3" block="M3"
        M3 = 1,
        //% blockId="ExpandMotor_M4" block="M4"
        M4 = 2,
        //% blockId="ExpandMotor_M3" block="M5"
        M5 = 3,
        //% blockId="ExpandMotor_M4" block="M6"
        M6 = 4, 
    }
    //拓展电机口(闭环电机)
    export enum ExpandStopMotorPort
    {
        //% blockId="ExpandMotor_M3" block="M3"
        M3 = 1,
        //% blockId="ExpandMotor_M4" block="M4"
        M4 = 2,
        //% blockId="ExpandMotor_M3" block="M5"
        M5 = 3,
        //% blockId="ExpandMotor_M4" block="M6"
        M6 = 4,
        //% blockId="ExpandMotor_M_ALL" block="全部"
        M_ALL = 5, 
    }

   //集成灰度位置
   export enum GRAY_ID
   {
       //% blockId="GRAY_ID1" block="1"
       GRAY_ID1 = 1,
       //% blockId="GRAY_ID2" block="2"
       GRAY_ID2 = 2,
       //% blockId="GRAY_ID3" block="3"
       GRAY_ID3 = 3,
       //% blockId="GRAY_ID4" block="4"
       GRAY_ID4 = 4,
       //% blockId="GRAY_ID5" block="5"
       GRAY_ID5 = 5, 
   }


   export enum ToneNote {
    //% blockId="C_low" block="低 C"
       C3 = 262,
    //% blockId="D_low" block="低 D"
       D3 = 294,
    //% blockId="E_low" block="低 E"
       E3 = 330,
    //% blockId="F_low" block="低 F"
       F3 = 349,
    //% blockId="G_low" block="低 G"
       G3 = 392,
    //% blockId="A_low" block="低 A"
       A3 = 440,
    //% blockId="B_low" block="低 B"
       B3 = 494,
    //% blockId="C_mid" block="中 C"
       C4 = 523,
    //% blockId="D_mid" block="中 D"
       D4 = 587,
    //% blockId="E_mid" block="中 E"
       E4 = 659,
    //% blockId="F_mid" block="中 F"
       F4 = 698,
    //% blockId="G_mid" block="中 G"
       G4 = 784,
    //% blockId="A_mid" block="中 A"
       A4 = 880,
    //% blockId="B_mid" block="中 B"
       B4 = 988,
    //% blockId="C_high" block="高 C"
       C5 = 1046,
    //% blockId="D_high" block="高 D"
       D5 = 1175,
    //% blockId="E_high" block="高 E"
       E5 = 1318,
    //% blockId="F_high" block="高 F"
       F5 = 1397,
    //% blockId="G_high" block="高 G"
       G5 = 1568,
    //% blockId="A_high" block="高 A"
       A5 = 1760,
    //% blockId="B_high" block="高 B"
       B5 = 1967,
   } 
    


            
    let OneWireSendFrameHeader = [
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_SET_CAR_SPEED_ANGLE_TIME, CMD_SET_CAR_SPEED_ANGLE_TIME],      //设置麦克纳姆轮速度、角度、时间
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_SET_CAR_SPEED_ANGLE, CMD_SET_CAR_SPEED_ANGLE],                //设置麦克纳姆轮速度、角度
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_SET_ROTAT_SPEED_TIME, CMD_SET_ROTAT_SPEED_TIME],              //设置麦克纳姆轮旋转速度、角度
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_SET_ROTAT_SPEED, CMD_SET_ROTAT_SPEED],                        //设置麦克纳姆轮旋转速度
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_SET_EXPANDMOTOE_SPEED, CMD_SET_EXPANDMOTOE_SPEED],            //设置拓展模块电机速度
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_SET_SINGLEMOTOR_SPEED, CMD_SET_SINGLEMOTOR_SPEED],            //设置单个电机速度
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_SET_SINGLEMOTOR_SPEED_TIME, CMD_SET_SINGLEMOTOR_SPEED_TIME],  //设置单个电机速度、时间
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_SET_FOUR_MOTOR_COEF, CMD_SET_FOUR_MOTOR_COEF],                //设置四个电机功率
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_STOP_SINGLEMOTOR, CMD_STOP_SINGLEMOTOR],                      //停止单个电机
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_STOP_ALLMOTOR, CMD_STOP_ALLMOTOR],              //停止所有电机
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_CLEAR_SINGLEMOTOR_ENCODER, CMD_CLEAR_SINGLEMOTOR_ENCODER],    //清除单个电机编码器的值
        [HEADER_1BYTE, HEADER_2BYTE, DATALEN_GET_SINGLEMOTOR_ENCODER, CMD_GET_SINGLEMOTOR_ENCODER],        //获取单个电机编码器的值
    ]
    let SPISendFrameHeaderForExpandMotor = [
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_SET_CAR_SPEED_ANGLE_TIME, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR],   //设置麦克纳姆轮速度、角度、时间
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_SET_CAR_SPEED_ANGLE, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR],        //设置麦克纳姆轮速度、角度
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_SET_ROTAT_SPEED_TIME, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR],       //设置麦克纳姆轮旋转速度、角度
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_SET_ROTAT_SPEED, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR],            //设置麦克纳姆轮旋转速度
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_SET_EXPANDMOTOE_SPEED, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR],      //设置拓展模块电机速度
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_SET_SINGLEMOTOR_SPEED, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR],      //设置单个电机速度
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_SET_SINGLEMOTOR_SPEED_TIME, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR], //设置单个电机速度、时间
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_SET_FOUR_MOTOR_COEF, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR],//设置单个电机速度、角度
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_STOP_SINGLEMOTOR, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR],           //停止单个电机
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_STOP_ALLMOTOR, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR],//停止所有电机
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_CLEAR_SINGLEMOTOR_ENCODER, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR],  //清除单个电机编码器的值
        [FRAMEHEADER_BYTE, OPERATION_WRITE, 8 + DATALEN_GET_SINGLEMOTOR_ENCODER, PORT_EXPANSIONMOTOR, DEVICEYTPE_BUS, EXPANSIONMOTOR],    //获取单个电机编码器的值
    ] 
    
    let SendBuf = pins.createBuffer(32); //发送缓冲区
    
    //拓展电机控制相关结束

    //为发送缓冲区分配空间
    function SendBufCreate(): void {
        //SendBuf = pins.createBuffer(20)//发送缓冲区
    }
    //SPI发送数据
    function SPIBUSSendByte(len: number): void
    {
        for (let i: number = 0; i < len; i++) {
            pins.spiWrite(SendBuf[i]);
        }
    }
    
    
    


   



    //拓展电机模块控制SPI前端部分字节处理
    function SPIHeaderForExpandMotor(index: number): void
    {
        index -= 1;
        SendBuf[0] = SPISendFrameHeaderForExpandMotor[index][0];//帧头
        SendBuf[1] = SPISendFrameHeaderForExpandMotor[index][1];//写操作
        SendBuf[2] = SPISendFrameHeaderForExpandMotor[index][2];//数据量
        SendBuf[3] = SPISendFrameHeaderForExpandMotor[index][3];//端口
        SendBuf[4] = SPISendFrameHeaderForExpandMotor[index][4];//设备类型
        SendBuf[5] = SPISendFrameHeaderForExpandMotor[index][5];//设备型号
    }
    //帧前端部分数据进行赋值操作
    function FrameHeaderData(extid:MOTOR_EXT,index: number): void
    {
        index -= 1;
        SendBuf[6] = OneWireSendFrameHeader[index][0];//帧头第一字节
        SendBuf[7] = OneWireSendFrameHeader[index][1];//帧头第二字节
        SendBuf[8] = extid;
        SendBuf[9] = OneWireSendFrameHeader[index][2];//数据量
        SendBuf[10] = OneWireSendFrameHeader[index][3];//命令码
    }
    //控制拓展电机控制命令发送
    function ExpandMotorCtl(index: number): void
    {
        let len = index;// 3+SPISendFrameHeaderForExpandMotor[index-1][2];
        for (let i: number = 0; i < len; i++) {
            pins.spiWrite(SendBuf[i]);
        }
    }
    //SPI初始化
    function SPIBUS_Init(): void
    {
        if (!spi_initialized)
        {
            SPI_Init();
            spi_initialized = true;
        }
    } 
    //计算校验码
    function CalcCheckValue(len: number):number
    {
        let index = 10;
        let sum = 0;
        for (let i: number = 0; i < len; i++) {
            sum += SendBuf[index+i];
        }
        sum = ~sum;
        return sum;
    }


   
       


    //SPI初始化
    function SPI_Init(): void
    {
        pins.digitalWritePin(DigitalPin.P16, 1);
        pins.spiPins(DigitalPin.P15, DigitalPin.P14, DigitalPin.P13);
        pins.spiFormat(8, 3);
        pins.spiFrequency(1000000);
    }    
    //SPI读写数据
    function rgbvalue(): number
    {
        pins.digitalWritePin(DigitalPin.P16, 0);
        control.waitMicros(5000);
        let command = pins.spiWrite(143);
        return command;
    }

    //帧头设置
    //% block
    function _SpiFrameHead(dir:number,port:number,devicetype:number,index:number) : void
    {
        SendBuf[0] = FRAMEHEADER_BYTE;//帧头
        SendBuf[1] = dir;//写操作
        SendBuf[3] = port;//数据量
        SendBuf[4] = devicetype;//设备类型
        SendBuf[5] = index;//设备型号 
    }


  //--------------------------------------------------------------------------------------------------------------------------
  // 普通电机
    //% blockId=AISTEAM_SetDualMotorStatus block="电机 M1速度 $speed1 M2速度 $speed2 "
    //% group="普通电机"
    //% weight=69
    //% blockGap=10
    //% color="#de7372"
    //% speed1.min=-100 speed1.max=100
    //% Speed1.shadow=turnRatioPicker
    //% speed2.min=-100 speed2.max=100
    //% Speed2.shadow=turnRatioPicker
    export function SetDualMotorStatus(speed1: number, speed2: number): void
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE,MOTOR_TYPE,DEVICEYTPE_DIGITALOUT,DEVIDE_OPENMOTOR);
        SendBuf[i++] = 1;
        SendBuf[i++] = speed1;
        SendBuf[i++] = speed2;
        SendBuf[2] = i - 3;
        SPIBUSSendByte(i);
    }



    //% blockId=ns_SetMotorStatus block="电机|%index|速度|%speed"
    //% weight=68
    //% blockGap=10
    //% color="#de7372"
    //% speed.min=-100 speed.max=100
    //% Speed.shadow=turnRatioPicker
    //% group="普通电机"
    export function SetMotorStatus(index: MotorPort, speed: number): void
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE,MOTOR_TYPE,DEVICEYTPE_DIGITALOUT,DEVIDE_OPENMOTOR);
        SendBuf[i++] = 2;
        SendBuf[i++] = index;
        SendBuf[i++] = speed;
        SendBuf[2] = i - 3;
        SPIBUSSendByte(i);
    }
    
    //% blockId=ns_SetMotorruntime block="电机|%index|速度|%speed|时间(ms) $time "
    //% weight=67
    //% blockGap=10
    //% color="#de7372"
    //% time.min=0 time.max = 600000
    //% speed.min=-100 speed.max=100
    //% Speed.shadow=turnRatioPicker
    //% group="普通电机"
    export function SetMotorruntime(index: MotorPort, speed: number,time:number): void
    {
        let i = 6;
        let f = 1.0;
        f = time;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE,MOTOR_TYPE,DEVICEYTPE_DIGITALOUT,DEVIDE_OPENMOTOR);
        SendBuf[i++] = 3;
        SendBuf[i++] = index;
        SendBuf[i++] = speed;
        SendBuf[i++] = f;
        SendBuf[i++] = f>>8;
        SendBuf[i++] = f>>16;
        SendBuf[i++] = f>>24;
        SendBuf[2] = i - 3;
        SPIBUSSendByte(i);
        pause(f * 10);
    }

    /*
    // m1coef.min=-1.00 m1coef.max=1.00 m1coef.defl=0.5
    // m2coef.min=-1.00 m2coef.max=1.00 m2coef.defl=0.5    
    */
    //% blockId=ns_SetMotorcoef block="设置电机功率(-1.0 ~ 1.0)|M1 $m1coef M2 $m2coef"
    //% weight=66
    //% blockGap=10
    //% color="#de7372"
    //% group="普通电机"
    //% m1coef.defl=1.00
    //% m2coef.defl=1.00 
   

    export function SetMotorcoef(m1coef: number, m2coef: number): void
    {
        let i = 6;
        let f = 1.0;
        f = m1coef * 100.0; 
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE,MOTOR_TYPE,DEVICEYTPE_DIGITALOUT,DEVIDE_OPENMOTOR);
        SendBuf[i++] = 4;
        SendBuf[i++] = f;
       SendBuf[i++] = f>>8;
        SendBuf[i++] = f>>16;
        SendBuf[i++] = f >> 24; 
        f = m2coef * 100.0;
        SendBuf[i++] = f; 
        SendBuf[i++] = f>>8;
        SendBuf[i++] = f>>16;
        SendBuf[i++] = f >> 24;
        SendBuf[2] = i - 3;
        SPIBUSSendByte(i);
    }

    //% blockId=ns_SetOpenMotorStop block="$index|电机停止运行"
    //% weight=65
    //% blockGap=10
    //%m1coef.defl="全部"
    //% color="#de7372"
    //% group="普通电机"
    export function SetOpenMotorStop(index: MotorStopPort): void
    {
        let i = 6;
        let f = 1.0;
      
       
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE,MOTOR_TYPE,DEVICEYTPE_DIGITALOUT,DEVIDE_OPENMOTOR);
        SendBuf[i++] = 5;
        SendBuf[i++] = index;
        SendBuf[2] = i - 3;
        SPIBUSSendByte(i);
    }

    //----------------------------------------------------------------------------------------------------------------------
    //编码电机
    //% blockId=ns_SetCarSpeedAngleTime block="ID $id 麦克纳姆轮 速度|%Speed|角度|%Angle|时间(ms)|%Time"
    //% weight=60
    //% blockGap=10
    //% color="#de7372"
    //% Speed.min=-100 Speed.max=100
    //% Angle.min=-180 Angle.max=180,Angle.def1=90
    //% Time.min=0 Time.max=60000
    //% group="麦克纳姆轮"
    //% inlineInputMode=inline
    export function SetCarSpeedAngleTime(id: MOTOR_EXT,Speed: number, Angle: number,Time: number): void
    {
        let i = 11;
        let f = Time;
        SPIBUS_Init();                                        //总线初始化
        SPIHeaderForExpandMotor(CMD_SET_CAR_SPEED_ANGLE_TIME);//拓展电机模块控制SPI前端部分字节处理
        FrameHeaderData(id,CMD_SET_CAR_SPEED_ANGLE_TIME);        //帧前端部分数据进行赋值操作
        SendBuf[i++] = Speed;                                  //速度
        SendBuf[i++] = Angle;                                  //角度
        SendBuf[i++] = Angle>>8;                               //角度
        SendBuf[i++] = f;                                   //时间
        SendBuf[i++] = f >> 8;                              //时间
        SendBuf[i++] = f >> 16;                             //时间
        SendBuf[i++] = f >>24;                               //时间
        SendBuf[i++] = CalcCheckValue(SendBuf[9]);             //校验
        ExpandMotorCtl(i);         //发送
        pause(f );
    }

    //% blockId=ns_SetCarSpeedAngle block="ID $id 麦克纳姆轮 速度|%Speed|角度|%Angle"
    //% weight=59
    //% blockGap=10
    //% color="#de7372"
    //% Speed.min=-100 Speed.max=100
    //% Angle.min=-180 Angle.max=180
    //% group="麦克纳姆轮"
    export function SetCarSpeedAngle(id: MOTOR_EXT,Speed: number, Angle: number): void
    {
        let i = 11;
        SPIBUS_Init();                                    //总线初始化
        SPIHeaderForExpandMotor(CMD_SET_CAR_SPEED_ANGLE );//拓展电机模块控制SPI前端部分字节处理
        FrameHeaderData(id,CMD_SET_CAR_SPEED_ANGLE );        //帧前端部分数据进行赋值操作
        SendBuf[i++] = Speed;                              //速度
        SendBuf[i++] = Angle;                              //角度
        SendBuf[i++] = Angle>>8;                           //角度
        SendBuf[i++] = CalcCheckValue(SendBuf[9]);         //校验
        ExpandMotorCtl(i );         //发送
    }
    
    //% blockId=ns_SetCarRotatSpeedTime block="ID $id 麦克纳姆轮旋转 速度|%Speed|时间(ms)|%Time"
    //% weight=58
    //% blockGap=10
    //% color="#de7372"
    //% Speed.min=-100 Speed.max=100
    //% Time.min=-0 Time.max=60000
    //% group="麦克纳姆轮"
    export function SetCarRotatSpeedTime(id: MOTOR_EXT,Speed: number, Time: number): void
    {
        let i = 11;
        let f = 1.0;
        f = Time ;
        SPIBUS_Init();                                      //总线初始化
        SPIHeaderForExpandMotor(CMD_SET_ROTAT_SPEED_TIME  );//拓展电机模块控制SPI前端部分字节处理
        FrameHeaderData(id,CMD_SET_ROTAT_SPEED_TIME  );        //帧前端部分数据进行赋值操作
        SendBuf[i++] = Speed;                                //速度
        SendBuf[i++] = f;                                 //时间
        SendBuf[i++] = f >> 8;                            //时间
        SendBuf[i++] = f >> 16;                           //时间
        SendBuf[i++] = f >> 24;                           //时间
        SendBuf[i++] = CalcCheckValue(SendBuf[9]);           //校验
        ExpandMotorCtl(i);         //发送
        pause(f );
    }

    //% blockId=ns_SetCarRotatSpeed block="ID $id 麦克纳姆轮旋转 速度|%Speed"
    //% weight=57
    //% blockGap=10
    //% color="#de7372"
    //% Speed.min=-100 Speed.max=100
    //% group="麦克纳姆轮"
    export function SetCarRotatSpeed(id: MOTOR_EXT,Speed: number): void
    {
        let i = 11;
        SPIBUS_Init();                               //总线初始化
        SPIHeaderForExpandMotor(CMD_SET_ROTAT_SPEED);//拓展电机模块控制SPI前端部分字节处理
        FrameHeaderData(id,CMD_SET_ROTAT_SPEED);        //帧前端部分数据进行赋值操作
        SendBuf[i++] = Speed;                         //速度
        SendBuf[i++] = CalcCheckValue(SendBuf[9]);    //校验
        ExpandMotorCtl(i);         //发送
    }

    //% blockId=ns_SetExpandMotorSpeed block="拓展电机模块 ID $id 电机 $MotorId1 速度 $Speed1|电机|%MotorId2|速度|%Speed2|电机|%MotorId3|速度|%Speed3|电机|%MotorId4|速度|%Speed4"
    //% weight=55
    //% blockGap=10
    //% color="#de7372"
    //% Speed1.min=-100 Speed1.max=100  Speed1.defl=0
    //% Speed2.min=-100 Speed2.max=100  Speed2.defl=0
    //% Speed3.min=-100 Speed3.max=100  Speed3.defl=0
    //% Speed4.min=-100 Speed4.max=100  Speed4.defl=0
    //% MotorId1.defl = MOTOR_EXT1
    //% MotorId2.def1 = "M4"
    //% MotorId3.defl = "ExpandMotor_M5"
    //% inlineInputMode=inline
    //% group="编码电机"
    //% expandableArgumentMode="enabled"
    export function SetExpandMotorSpeed(id: MOTOR_EXT,
        MotorId1: ExpandMotorPort, Speed1: number,
        MotorId2: ExpandMotorPort, Speed2: number,
        MotorId3: ExpandMotorPort, Speed3: number,
        MotorId4: ExpandMotorPort, Speed4: number): void
    {
        let i = 11;
        SPIBUS_Init();                                      //总线初始化
        SPIHeaderForExpandMotor(CMD_SET_EXPANDMOTOE_SPEED );//拓展电机模块控制SPI前端部分字节处理
        FrameHeaderData(id,CMD_SET_EXPANDMOTOE_SPEED );        //帧前端部分数据进行赋值操作
        SendBuf[i++] = MotorId1;                             //电机序号
        SendBuf[i++] = Speed1;                               //速度
        SendBuf[i++] = MotorId2;                             //电机序号
        SendBuf[i++] = Speed2;                               //速度
        SendBuf[i++] = MotorId3;                             //电机序号
        SendBuf[i++] = Speed3;                               //速度
        SendBuf[i++] = MotorId4;                             //电机序号
        SendBuf[i++] = Speed4;                               //速度
        SendBuf[i++] = CalcCheckValue(SendBuf[9]);           //校验
        ExpandMotorCtl(i );         //发送
     }  





    //% blockId=ns_SetSingleMotorSpeed block="ID $id 电机|%MotorId|速度|%Speed"
    //% weight=54
    //% blockGap=10
    //% color="#de7372"
    //% Speed.min=-100 Speed.max=100
    //% group="编码电机"
    //% inlineInputMode=inline
    export function SetSingleMotorSpeed(id: MOTOR_EXT,MotorId: ExpandMotorPort,Speed: number): void
    {
        let i = 11;
        SPIBUS_Init();                                     //总线初始化
        SPIHeaderForExpandMotor(CMD_SET_SINGLEMOTOR_SPEED);//拓展电机模块控制SPI前端部分字节处理
        FrameHeaderData(id,CMD_SET_SINGLEMOTOR_SPEED);        //帧前端部分数据进行赋值操作
        SendBuf[i++] = MotorId;                             //电机序号
        SendBuf[i++] = Speed;                               //电机速度
        SendBuf[i++] = CalcCheckValue(SendBuf[9]);          //校验
        ExpandMotorCtl(i);         //发送
    }

    //% blockId=ns_SetSingleMotorSpeedTime block="ID $id 电机|%MotorId|速度|%Speed|时间(ms)|%Time"
    //% weight=53
    //% blockGap=10
    //% color="#de7372"
    //% Speed.min=-100 Speed.max=100
    //% Time.min=0 Time.max=60000
    //% group="编码电机"
    //% inlineInputMode=inline
    export function SetSingleMotorSpeedTime(id: MOTOR_EXT,MotorId: ExpandMotorPort,Speed: number,Time: number): void
    {
        let i = 11;
        let f = 1.0;
        f = Time ;
        SPIBUS_Init();                                          //总线初始化
        SPIHeaderForExpandMotor(CMD_SET_SINGLEMOTOR_SPEED_TIME);//拓展电机模块控制SPI前端部分字节处理
        FrameHeaderData(id,CMD_SET_SINGLEMOTOR_SPEED_TIME);        //帧前端部分数据进行赋值操作
        SendBuf[i++] = MotorId;                                  //电机序号
        SendBuf[i++] = Speed;                                    //速度
        SendBuf[i++] = f;                                     //时间
        SendBuf[i++] = f>>8;                                  //时间
        SendBuf[i++] = f>>16;                                 //时间
        SendBuf[i++] = f>>24;                                 //时间
        SendBuf[i++] = CalcCheckValue(SendBuf[9]);               //校验
        ExpandMotorCtl(i);         //发送
        pause(f );
       
    }
    //% blockId=ns_SetFourMotorCoef block="ID $id 设置编码电机功率(-1.0 ~ 1.0) M3 $Motor1coef|M4 $Motor2coef|M5 $Motor3coef|M6 $Motor4coef"
    //% weight=52
    //% blockGap=10
    //% color="#de7372"
    //% Motor1coef.def1=1.0
    //% Motor2coef.def1=1.0
    //% Motor3coef.def1=1.0
    //% Motor4coef.def1=1.0
    //% group="编码电机"
    //% inlineInputMode=inline
    export function SetFourMotorCoef(id: MOTOR_EXT,Motor1coef: number,Motor2coef: number,Motor3coef: number,Motor4coef: number): void
    {
        let i = 11;
        let f = 1.0;
        SPIBUS_Init();                                           //总线初始化
        SPIHeaderForExpandMotor(CMD_SET_FOUR_MOTOR_COEF);//拓展电机模块控制SPI前端部分字节处理
        FrameHeaderData(id, CMD_SET_FOUR_MOTOR_COEF);        //帧前端部分数据进行赋值操作
        f = Motor1coef * 100;
        SendBuf[i++] = f;                                     //时间
        SendBuf[i++] = f>>8;                                  //时间
        SendBuf[i++] = f>>16;                                 //时间
        SendBuf[i++] = f >> 24;                                 //时间
        f = Motor2coef * 100;
        SendBuf[i++] = f;                                     //时间
        SendBuf[i++] = f>>8;                                  //时间
        SendBuf[i++] = f>>16;                                 //时间
        SendBuf[i++] = f >> 24;                                 //时间
        f = Motor3coef * 100;
        SendBuf[i++] = f;                                     //时间
        SendBuf[i++] = f>>8;                                  //时间
        SendBuf[i++] = f>>16;                                 //时间
        SendBuf[i++] = f >> 24;                                 //时间
        f = Motor4coef * 100;
        SendBuf[i++] = f;                                     //时间
        SendBuf[i++] = f>>8;                                  //时间
        SendBuf[i++] = f>>16;                                 //时间
        SendBuf[i++] = f>>24;                                 //时间
        SendBuf[i++] = CalcCheckValue(SendBuf[9]);                //校验
        ExpandMotorCtl(i);         //发送
        pause(1);
    }
    
    //% blockId=ns_StopSingleMotor block="ID $id|%MotorId 编码电机停止运行"
    //% weight=51
    //% blockGap=10
    //% color="#de7372"
    //% group="编码电机"
    //% inlineInputMode=inline
    export function StopSingleMotor(id: MOTOR_EXT,MotorId: ExpandStopMotorPort): void
    {
        let i = 11;
        SPIBUS_Init();                                //总线初始化
        SPIHeaderForExpandMotor(CMD_STOP_SINGLEMOTOR);//拓展电机模块控制SPI前端部分字节处理
        FrameHeaderData(id,CMD_STOP_SINGLEMOTOR);        //帧前端部分数据进行赋值操作
        SendBuf[i++] = MotorId;                        //电机序号
        SendBuf[i++] = CalcCheckValue(SendBuf[9]);     //校验
        ExpandMotorCtl(i);         //发送
    }

    
    //% blockId=ns_ClearSingleMotorEncoder block="ID $id|%MotorId 编码器电机编码值清零 "
    //% weight=50
    //% blockGap=10
    //% color="#de7372"
    //% group="编码电机"
    export function ClearSingleMotorEncoder(id: MOTOR_EXT,MotorId: ExpandStopMotorPort): void
    {
        let i = 11;
        SPIBUS_Init();                                //总线初始化
        SPIHeaderForExpandMotor(CMD_CLEAR_SINGLEMOTOR_ENCODER);//拓展电机模块控制SPI前端部分字节处理
        FrameHeaderData(id,CMD_CLEAR_SINGLEMOTOR_ENCODER);        //帧前端部分数据进行赋值操作
        SendBuf[i++] = MotorId;                        //电机序号
        SendBuf[i++] = CalcCheckValue(SendBuf[9]);     //校验
        ExpandMotorCtl(i);         //发送
    }

    //% blockId=ns_getMotorEncoder block="ID $id|%MotorId 编码器电机编码值 "
    //% weight=49
    //% blockGap=10
    //% color="#de7372"
    //% group="编码电机"
    export function getMotorEncoder(id: MOTOR_EXT,MotorId: ExpandMotorPort): number
    {
        let i = 11;
        SPIBUS_Init();                                //总线初始化
        SPIHeaderForExpandMotor(CMD_GET_SINGLEMOTOR_ENCODER);//拓展电机模块控制SPI前端部分字节处理
        FrameHeaderData(id,CMD_GET_SINGLEMOTOR_ENCODER);        //帧前端部分数据进行赋值操作
        SendBuf[i++] = MotorId;                        //电机序号
        SendBuf[i++] = CalcCheckValue(SendBuf[9]);     //校验
        ExpandMotorCtl(i);         //发送
        let res = pins.spiWrite(SPI_READDUMMY);
        res |= (pins.spiWrite(SPI_READDUMMY)<<8);
        res |= (pins.spiWrite(SPI_READDUMMY) << 16);
        res |= (pins.spiWrite(SPI_READDUMMY)<<24);
        return res;
    }


    //-------------------------------------------------------------------------------------------------------------------------------------
    //传感器
    //% blockId=ns_OneIrTrack_Sensor block="单路巡迹|端口 %index"
    //% weight=99
    //% blockGap=10
    //% color="#de7372"
    //% group="感知"
    export function OneIrTrack_Sensor(index: DigitalInputPort): boolean
    {
       
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_READ,index,DEVICEYTPE_DIGITALIN,SENSOR_TRACK);
        SendBuf[2] = i - 3;
        SPIBUSSendByte(i);
        pause(1);
        //动作代号
        SendBuf[6] = SPI_READDUMMY; 
        let res = pins.spiWrite(SendBuf[6]);
        if (1 == res)//1--成立
            return true;
        else//2--不成立
            return false;
    }

    //% blockId=ns_Knock_Sensor block="碰撞开关|端口 %index"
    //% weight=98
    //% blockGap=10
    //% color="#de7372"
    //% group="感知"
    export function Knock_Sensor(index: DigitalInputPort): boolean
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_READ,index,DEVICEYTPE_DIGITALIN,SENSOR_CRASH);
        SendBuf[2] = i - 3;
        SPIBUSSendByte(i);
        
        //动作代号
        SendBuf[6] = SPI_READDUMMY; 
        let res = pins.spiWrite(SendBuf[6]);
        if (1 == res)//1--成立
            return true;
        else//2--不成立
            return false;
    }
   
    //% blockId=ns_Flame_Sensor block="火焰传感器|端口 %index 有火焰"
    //% weight=97
    //% blockGap=10
    //% color="#de7372"
     //% group="感知"
    export function Flame_Sensor(index: DigitalInputPort): boolean
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_READ,index,DEVICEYTPE_DIGITALIN,SENSOR_FLAM);
        SendBuf[2] = i - 3;
        SPIBUSSendByte(i);
        //动作代号
        SendBuf[6] = SPI_READDUMMY; 
        let res = pins.spiWrite(SendBuf[6]);
        if (1 == res)//1--成立
            return true;
        else//2--不成立
            return false;

    }
 

    //% blockId=ns_ReadVoiceSensor block="声强检测 端口|%index"
    //% weight=96
    //% blockGap=10
    //% color="#de7372"
    //% group="感知"
    export function ReadVoiceSensor(index: AnalogInputPort):number
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_READ,index,DEVICEYTPE_ANALOGIN,SENSOR_VOICE);
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
        //动作代号
        SendBuf[6] = SPI_READDUMMY; 
        let res = pins.spiWrite(SendBuf[6]);
        return res;
    }

   //% blockId=ns_ReadPhotoSensitiveSensor block="光敏检测 端口|%index"
    //% weight=95
    //% blockGap=10
    //% color="#de7372"
    //% group="感知"
    export function ReadPhotoSensitiveSensor(index: AnalogInputPort):number
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_READ,index,DEVICEYTPE_ANALOGIN,SENSOR_PHOTOSENSITIVE);
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
        //动作代号
        SendBuf[6] = SPI_READDUMMY; 
        let res = pins.spiWrite(SendBuf[6]);
        return res;
    }
 
    //% blockId=ns_TempHumiSensor_Humi block="湿度检测 端口|%index"
    //% weight=94
    //% blockGap=10
    //% group="感知"
    export function TempHumiSensor_Humi(index: AnalogInputPort): number
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_READ, index, DEVICEYTPE_BUS, SENSOR_TH);
        SendBuf[i++] = 2;  //读取湿度
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
        
 
        //动作代号
        SendBuf[7] = SPI_READDUMMY; 
        let cnt=10;
        let res = pins.spiWrite(SendBuf[7]);
        while (cnt) {
            if (res != 0) break;
            pause(1);
            res = pins.spiWrite(SendBuf[7]);
            cnt--;
        }
        return res;
      
    }
    //% blockId=ns_TempHumiSensor_Temp block="温度检测 端口|%index"
    //% weight=93
    //% blockGap=10
    //% group="感知"
    export function TempHumiSensor_Temp(index: AnalogInputPort): number
    {
        let i = 6;

        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_READ, index, DEVICEYTPE_BUS, SENSOR_TH);
        SendBuf[i++] = 1;  //读取温度
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);

        //动作代号
        SendBuf[7] = SPI_READDUMMY;
        let cnt=10;
        let res = pins.spiWrite(SendBuf[7]);
        while (cnt) {
            if (res != 0) break;
            pause(1);
            res = pins.spiWrite(SendBuf[7]);
            cnt--;
        }
        return res;
    }

    //% blockId=ns_Ir_Distance block="红外避障 端口|%index"
    //% weight=92
    //% blockGap=10
    //% color="#de7372"
    //% group="感知"
    export function Ir_Distance(index: AnalogInputPort):number
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_READ,index,DEVICEYTPE_ANALOGIN,SENSOR_IRDISTANCE);
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
        //动作代号
        SendBuf[6] = SPI_READDUMMY; 
        let res = pins.spiWrite(SendBuf[6]);
        return res;
    }

    //% blockId=ns_IrRemote block="$key 红外遥控 端口|%index"
    //% weight=91
    //% blockGap=10
    //% color="#de7372"
    //% group="感知"
    export function Ir_Remote(key:ns_IrValue,index: AnalogInputPort):boolean
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_READ,index,DEVICEYTPE_BUS,SENSOR_IR);
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
        //动作代号
        SendBuf[6] = SPI_READDUMMY; 
        let res = pins.spiWrite(SendBuf[6]);
        if (key == res)//1--成立
            return true;
        else//2--不成立
            return false;
    }


    //% blockId=ns_FiveGray block="$index 集成灰度检测 端口A5"
    //% weight=90
    //% blockGap=10
    //% color="#de7372"
    //% group="感知"
    export function FiveGray(index: GRAY_ID):number
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_READ, 5, DEVICEYTPE_BUS, SENSOR_FIVEGRAY);
        SendBuf[i++] = index;  //灰度id
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
        pause(1);
        //动作代号
        SendBuf[7] = 0xF1; 
        let res = pins.spiWrite(0xF1);
        res |= (pins.spiWrite(0xF1)<<8);  
        return res;
    }


    
    //% blockId=ns_DS_ShowNum block="数码管 端口|%index|显示|%num"
    //% weight=89
    //% blockGap=10
    //% color="#de7372"
    //% num.min=-99 num.max=999
    //% group="感知"
    export function DS_ShowNum(index: DigitalOutputPort, num: number): void
    {   
        
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_READ, index, DEVICEYTPE_DIGITALOUT, DEVIDE_DIGITALTUBE);
        //动作代号
        SendBuf[i++] = num; 
        SendBuf[i++] = num >> 8;
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);  
    } 
    
  
    //% blockId=ns_SetFanOn block="开启风扇|%index"
    //% weight=89
    //% blockGap=10
    //% color="#de7372"
   //% group="风扇"
    export function SetFanOn(index: DigitalOutputPort): void
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE, index, DEVICEYTPE_DIGITALOUT, DEVIDE_FAN);
        SendBuf[i++] = 1;  //开启
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
        //动作代号  
    }
    //% blockId=ns_SetFanOff block="关闭风扇|%index"
    //% weight=88
    //% blockGap=10
    //% color="#de7372"
   //% group="风扇"
   export function SetFanOff(index: DigitalOutputPort): void
   {
       let i = 6;
       SPIBUS_Init();//总线初始化
       _SpiFrameHead(OPERATION_WRITE, index, DEVICEYTPE_DIGITALOUT, DEVIDE_FAN);
       SendBuf[i++] = 2;  //开启
       SendBuf[2] = i-3;
       SPIBUSSendByte(i);
       //动作代号  
   }
    //% blockId=ns_SetLedOn block="开启单色灯|%index"
    //% weight=87
    //% blockGap=10
    //% color="#de7372"
    //% group="单色灯"
    export function SetLedOn(index: DigitalOutputPort): void
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE, index, DEVICEYTPE_DIGITALOUT, DEVIDE_LED);
        SendBuf[i++] = 1;  //开启
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
        //动作代号  
    }
    //% blockId=ns_SetLedOff block="关闭单色灯|%index"
    //% weight=87
    //% blockGap=10
    //% color="#de7372"
    //% group="单色灯"
    export function SetLedOff(index: DigitalOutputPort): void
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE, index, DEVICEYTPE_DIGITALOUT, DEVIDE_LED);
        SendBuf[i++] = 2;  //关闭
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
        //动作代号  
    }

    //% blockId=ns_SetFullColorLEDStatus block="开启全彩灯 端口|%index|红|%red|绿|%green|蓝|%blue"
    //% weight=85
    //% blockGap=10
    //% red.min=0 red.max=255
    //% green.min=0 green.max=255
    //% blue.min=0 blue.max=255
    //% color="#de7372"
    //% inlineInputMode=inline
    //% group="全彩灯"
    export function SetFullColorLEDStatus(index: DigitalOutputPort, red: number,green: number,blue: number): void
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE, index, DEVICEYTPE_DIGITALOUT, DEVIDE_FULLCOLORLED);
        SendBuf[i++] = red;
        SendBuf[i++] = green;
        SendBuf[i++] = blue; 
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
    }
    //% blockId=ns_FullColorLEDOff block="关闭全彩灯 端口|%index"
    //% weight=84
    //% blockGap=10
    //% color="#de7372"
    //% group="全彩灯"
    export function FullColorLEDOff(index: DigitalOutputPort): void
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE, index, DEVICEYTPE_DIGITALOUT, DEVIDE_FULLCOLORLED);
        SendBuf[i++] = 0;
        SendBuf[i++] = 0;
        SendBuf[i++] = 0; 
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
    }


    //% blockId=ns_SetBeep block="开启蜂鸣器 端口|%index|音色 $nsnote|持续 %ms" 
    //% weight=90
    //% blockGap=10
    //% color="#de7372"
    //% nsnote.fieldEditor="gridpicker" nsnote.fieldOptions.columns=7
    //% group="蜂鸣器"
    export function SetBeep(index: DigitalOutputPort, nsnote: ToneNote, ms: BeatFraction): void
    {
        let i = 6;
        let beat = Math.idiv(60000, 120);
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE, index, DEVICEYTPE_DIGITALOUT, DEVIDE_BEEP);
        SendBuf[i++] = 1; 
        SendBuf[i++] = nsnote;
        SendBuf[i++] = nsnote >> 8;
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
        switch (ms) {
            case 1: beat = beat >> 1; break;
            case BeatFraction.Quarter: beat =  beat >> 2;break;
            case BeatFraction.Eighth: beat =  beat >> 3;break;
            case BeatFraction.Sixteenth: beat =  beat >> 4;break;
            case BeatFraction.Double: beat =  beat << 1;break;
            case BeatFraction.Breve: beat =  beat << 2;break;
            default: beat =  beat;break;
        }
        pause(beat);
        i = 6;
        _SpiFrameHead(OPERATION_WRITE, index, DEVICEYTPE_DIGITALOUT, DEVIDE_BEEP);
        SendBuf[i++] = 0; 
        SendBuf[i++] = nsnote;
        SendBuf[i++] = nsnote>>8;
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
    }



    //% blockId=ns_SetAnalogServo block="模拟舵机 角度|%angle"
    //% weight=69
    //% blockGap=10
    //% color="#de7372"
    //% angle.min=0 angle.max=180
    //% group="5V舵机"
    export function SetAnalogServo(angle: number): void
    {
        let i = 6;
        SPIBUS_Init();//总线初始化
        _SpiFrameHead(OPERATION_WRITE, PORT_ANALOGSERVO, DEVICEYTPE_DIGITALOUT, DEVIDE_ANALOGSERVO);
        SendBuf[i++] = angle;
        SendBuf[2] = i-3;
        SPIBUSSendByte(i);
        
    }
    
    
 


      
}
