#include <CAN.h>

#define ADDRESS (0xf1)

#define CAN_Address (0xF1)


/***************************J1939 地址配置*****************************/
//设备默认的地址（地址命名是有规定的，参考J1939的附录B 地址和标识符的分配）
#define J1939_STARTING_ADDRESS_1 CAN_Address
#define J1939_STARTING_ADDRESS_2 244
#define J1939_STARTING_ADDRESS_3 247
#define J1939_STARTING_ADDRESS_4 0


/******************************J1939功能配置***************************/
#define J1939_RX_QUEUE_SIZE 3
//当mcu来不及处理消息，接收消息列队是否允许被新的消息覆盖
#define J1939_OVERWRITE_RX_QUEUE J1939_FALSE
#define J1939_TX_QUEUE_SIZE 3
//当mcu来不及处理消息，发送消息列队是否允许被新的消息覆盖
#define J1939_OVERWRITE_TX_QUEUE J1939_FALSE
//是否使用轮询模式（否则使用中断模式）
#define J1939_POLL_ECAN J1939_TRUE
//是否启用软件滤波器
#define J1939SoftwareFilterEn J1939_TRUE
/******************************J1939移植配置函数************************/

#define Port_CAN_Transmit(MsgPtr) J1939_CAN_Transmit(MsgPtr)
#define Port_CAN_Receive(MsgPtr) J1939_CAN_Receive(MsgPtr)
#define Port_SetAddressFilter(Address) J1939_SetAddressFilter(Address)


#define FALSE 0
#define TRUE 1

typedef uint32_t   j1939_uint32_t;  /** < 32位无符号整形*/
typedef int32_t          j1939_int32_t;   /** < 32位整形*/
typedef uint16_t j1939_uint16_t;  /** < 16位无符号整形*/
typedef uint8_t  j1939_uint8_t;   /** < 8位无符号整形*/
typedef int8_t           j1939_int8_t;    /** < 8位无符号整形*/
#define J1939_NULL     0

//函数返回代码
#define RC_SUCCESS          0  /**< 成功*/
#define RC_QUEUEEMPTY     1  /**< 列队为空*/
#define RC_QUEUEFULL      1  /**< 列队满*/
#define RC_CANNOTRECEIVE    2  /**< 不能接收*/
#define RC_CANNOTTRANSMIT   2  /**< 不能传输*/
#define RC_PARAMERROR     3  /**< 参数错误*/
 
//内部常量
#define J1939_FALSE       0  /**< 代表函数错误返回*/
#define J1939_TRUE        1  /**< 代表函数正确返回*/

// J1939 默认的优先级（参考J1939文档）
#define J1939_CONTROL_PRIORITY      0x03 /**< J1939文档默认的优先级*/
#define J1939_INFO_PRIORITY         0x06 /**< J1939文档默认的优先级*/ 
#define J1939_PROPRIETARY_PRIORITY    0x06 /**< J1939文档默认的优先级*/ 
#define J1939_REQUEST_PRIORITY      0x06 /**< J1939文档默认的优先级*/ 
#define J1939_ACK_PRIORITY          0x06 /**< J1939文档默认的优先级*/ 
#define J1939_TP_CM_PRIORITY      0x07 /**< J1939文档默认的优先级*/ 
#define J1939_TP_DT_PRIORITY      0x07 /**< J1939文档默认的优先级*/ 
 
// J1939 定义的地址
#define J1939_GLOBAL_ADDRESS      255 /**< 全局地址*/
#define J1939_NULL_ADDRESS          254 /**< 空地址*/
 
//J1939协议栈的PNG请求响应，相关的定义
#define J1939_PF_REQUEST2       201  /**< J1939协议栈的请求 PF */
#define J1939_PF_TRANSFER       202  /**< J1939协议栈的转移 PF */

#define J1939_PF_REQUEST        234  /**< 请求 或 用于握手机制*/
#define J1939_PF_ACKNOWLEDGMENT     232  /**< 确认请求 或 用于握手机制*/

#define J1939_ACK_CONTROL_BYTE      0    /**< 用于TP(长帧数据)，代表确认*/
#define J1939_NACK_CONTROL_BYTE     1    /**< 用于TP(长帧数据)，PNG不被支持。否定消息*/
#define J1939_ACCESS_DENIED_CONTROL_BYTE  2/**< 拒绝访问，但是信息是被支持，暂时不能响应（需要再次发送请求）*/
#define J1939_CANNOT_RESPOND_CONTROL_BYTE 3/**< 不能做出反应，有空但是接受的缓存不够，或则发送资源被占领，暂时不能响应（需要再次发送请求）*/

//TP协议的一些宏定义 
#define J1939_PF_DT               235     /**< 协议传输---数据传输 PF*/
#define J1939_PF_TP_CM            236   /**< 协议传输---链接管理 PF*/

//TP的超时时间，单位（ms）
#define J1939_TP_Tr           200 /**< 宏定义TP的超时时间*/
#define J1939_TP_Th           500 /**< 宏定义TP的超时时间*/
#define J1939_TP_T1           750 /**< 宏定义TP的超时时间*/
#define J1939_TP_T2           1250 /**< 宏定义TP的超时时间*/
#define J1939_TP_T3           1250 /**< 宏定义TP的超时时间*/
#define J1939_TP_T4           1050 /**< 宏定义TP的超时时间*/
#define J1939_TP_TIMEOUT_NORMAL     0    /**< 未超时正常*/
#define J1939_TP_TIMEOUT_ABNORMAL   1    /**< 超时*/
#define J1939_RTS_CONTROL_BYTE      16   /**< TP.CM_RTS*/
#define J1939_CTS_CONTROL_BYTE      17   /**< TP.CM_CTS*/
#define J1939_EOMACK_CONTROL_BYTE   19   /**< 消息应答结束*/
#define J1939_BAM_CONTROL_BYTE      32   /**< 广播公告消息*/
#define J1939_CONNABORT_CONTROL_BYTE  255  /**< 连接中断控制字节（放弃连接）*/
#define J1939_RESERVED_BYTE             0xFF /**< 变量的保留位的值*/
 
//与J1939网络层有关的定义 
#define J1939_PGN2_REQ_ADDRESS_CLAIM  0x00 
#define J1939_PGN1_REQ_ADDRESS_CLAIM  0xEA   
#define J1939_PGN0_REQ_ADDRESS_CLAIM  0x00 
 
#define J1939_PGN2_COMMANDED_ADDRESS  0x00 
#define J1939_PGN1_COMMANDED_ADDRESS  0xFE  /**< 命令地址消息*/
#define J1939_PGN0_COMMANDED_ADDRESS  0xD8    /**< 参考J1939-81 地址命令配置*/
 
#define J1939_PF_ADDRESS_CLAIMED    238
#define J1939_PF_CANNOT_CLAIM_ADDRESS 238
#define J1939_PF_PROPRIETARY_A      239     /**< 专用A*/ 
#define J1939_PF_PROPRIETARY_B      255     /**< 专用B*/ 

/**< 是否对TP协议的支持（是否支持长帧（大于8字节的数据）的发送与接受）*/
#define J1939_TP_RX_TX J1939_TRUE
/**< TP协议的支持的最大接受发送消息长度(最大可配置为1785)*/
#define J1939_TP_MAX_MESSAGE_LENGTH 512

/**CAN节点的选择枚举
*
* 默认支持最大4路CAN硬件\n
*/
typedef enum
{
  Select_CAN_NODE_Null, /**< 不选择任何CAN硬件*/ 
  Select_CAN_NODE_1,    /**< 选择CAN硬件 1*/ 
  Select_CAN_NODE_2,    /**< 选择CAN硬件 2*/
  Select_CAN_NODE_3,    /**< 选择CAN硬件 3*/
  Select_CAN_NODE_4,    /**< 选择CAN硬件 4*/
}CAN_NODE;

#if J1939_TP_RX_TX
/**TP的状态描述枚举
*
*/
typedef enum
{
  J1939_TP_NULL,  /**< 长数据传输处于空闲，只有TP系统处于空闲，才能用处理下一个发送，和接受请求*/ 
  J1939_TP_RX,  /**< 长数据传输处于接收*/
  J1939_TP_TX,  /**< 长数据传输处于发送*/
  J1939_TP_OSBUSY,/**< 长数据传输处于繁忙，比如刚接受一整段长数据，但是CPU没来得处理，又一个长数据请求到来，为了数据不被覆盖，将状态设为本值*/
}J1939_TP_State;
/**TP的标志位结构体
*
* 本结构体记录了TP的状态，使用TP发送和接受的CAN硬件编号
*/
typedef struct
{
  J1939_TP_State state;          /**< TP的连接状态*/
  CAN_NODE       TP_RX_CAN_NODE; /**< TP接受请求产生的 CAN硬件编号*/
  CAN_NODE       TP_TX_CAN_NODE; /**< TP接受发送产生的 CAN硬件编号*/
}J1939_TP_Flags;
/**J1939消息对象的结构体
*
* 本结构体实现了 J1939的消息对象
*/
typedef struct
{
  j1939_uint32_t PGN ;  /**< J1939的消息对象的 PGN*/
  j1939_uint8_t  data[J1939_TP_MAX_MESSAGE_LENGTH] ;/**< J1939的消息对象的 数据*/
  j1939_uint16_t byte_count;/**< J1939的消息对象的 数据大小*/
  j1939_uint8_t  SA;   /**< J1939的消息对象的 目标地址（发送目的地  或  接受来源地）*/

} J1939_MESSAGE_T ;
/**J1939消息对象的结构体
*
* 本结构体实现了 J1939的多帧消息对象
*/
typedef struct
{
  j1939_uint32_t PGN ;        /**< J1939的消息对象的 PGN*/
    j1939_uint8_t *data;        /**< 缓存区指针*/
    j1939_uint16_t data_num;    /**< 缓存区大小*/
  j1939_uint16_t byte_count;  /**< J1939的消息对象的 数据大小*/
    j1939_uint8_t SA;           /**< J1939的消息对象的 数据 源地址*/
}TP_RX_MESSAGE;
/**J1939_TP_Tx_Step枚举
*
* 实现了记录长帧（多帧）传输的TX 的步骤
*/
typedef enum
{
  J1939_TP_TX_WAIT,
  J1939_TP_TX_CM_START,
  J1939_TP_TX_CM_WAIT,
  J1939_TP_TX_DT,
  J1939_TP_WAIT_ACK,
  J1939_TP_TX_ERROR,
  J1939_TX_DONE,
}J1939_TP_Tx_Step;//协议的发送步骤
/**J1939_TRANSPORT_TX_INFO 结构体
*
* 实现了长帧传输中产生的临时数据，和一些传输交换数据
*/
typedef struct
{
  J1939_MESSAGE_T       tp_tx_msg;           /**< J1939的消息对象*/
  j1939_uint16_t        time;                /**< 时间*/
  j1939_uint8_t         packet_offset_p;     /**< 数据包偏移指针*/
  j1939_uint8_t         packets_total;       /**< 总共有多少个数据包*/
  j1939_uint8_t         packets_request_num; /**< 请求发送的数据包数（接受方准备接受的数据包数）*/
  J1939_TP_Tx_Step      state ;              /**< 协议的发送步骤*/
} J1939_TRANSPORT_TX_INFO;
/**J1939_TP_Rx_Step枚举
*
* 实现了记录长帧（多帧）传输的RX 的步骤
*/
typedef enum
{
  J1939_TP_RX_WAIT,
  J1939_TP_RX_READ_DATA,
  J1939_TP_RX_DATA_WAIT,
  J1939_TP_RX_ERROR,
  J1939_RX_DONE,
}J1939_TP_Rx_Step;//协议的接收步骤

/**J1939_TRANSPORT_RX_INFO 结构体
*
* 实现了长帧传输中产生的临时数据，和一些传输交换数据
*/
typedef struct
{
  J1939_MESSAGE_T      tp_rx_msg; /**< J1939的消息对象*/
  j1939_uint8_t    osbusy;    /**< 此位置1，代表系统繁忙，cpu需要处理其他的事物，直接拒绝一切的链接请求\n 如果正在接受中，此位置1，则会发出链接保持消息帧。*/
  j1939_uint16_t       time;    /**< 时间*/
  j1939_uint8_t        packets_total; /**< 总共有多少个数据包*/
  j1939_uint8_t        packets_ok_num;/**< 已经接受的数据包数*/
  J1939_TP_Rx_Step     state ;     /**< 协议的接受步骤*/
} J1939_TRANSPORT_RX_INFO;

#endif //J1939_TP_RX_TX

/**
* @note 实现Request_PGN 的响应
*/
struct Request_List{
  j1939_uint8_t  *data;
  j1939_uint16_t lenght;
  j1939_uint32_t PGN;
  CAN_NODE       Can_Node;
  void (*update)();  /**< 在函数里需要对data更新，如果不用更新data赋值为J1939_NULL*/
  struct Request_List *next;   /**< 链表末尾，需要一直保持J1939_NULL*/
};

// J1939 Data Structures 
// J1939_MESSAGE_STRUCT旨在J1939消息块映射到设备的地址映射。 只有字段PDU格式不映射到设备寄存器。
// 结构应该简单地使用PDUFormat和忽视PDUFormat_Top。调整将立即接收和传输之前。
// 注:编译器创建结构从低一点的位置高一些位置，所以可能出现不匹配的设备寄存器。
#define J1939_MSG_LENGTH  9  //消息长度
#define J1939_DATA_LENGTH 8  //数据长度

/** J1939_MESSAGE_UNION 结构体
* 实现了J1939消息对象
*
*
*/
union J1939_MESSAGE_UNION 
{ 
/** j1939 的 ID 组成结构体
*
*/
  struct   j1939_PID
  { 
    j1939_uint8_t DataPage      : 1;  /**< 数据页*/
    j1939_uint8_t Res         : 1;  /**< Res位*/
    j1939_uint8_t Priority      : 3;  /**< 优先级*/
    j1939_uint8_t Reserve       : 3;  /**< 空闲*/
    j1939_uint8_t PDUFormat;          /**< PF*/
    j1939_uint8_t PDUSpecific;        /**< PS*/
    j1939_uint8_t SourceAddress;            /**< SA*/
    j1939_uint8_t DataLength      : 4;  /**< 数据长度*/
    j1939_uint8_t RTR         : 4;  /**< RTR位*/
    j1939_uint8_t Data[J1939_DATA_LENGTH];  /**< 数据*/
    j1939_uint32_t  PGN         :24;  /**< 参数群编号*/
    j1939_uint32_t  ReservePGN      : 8;  /**< 空闲*/
  };
  struct j1939_PID Mxe;  /**< j1939 的 ID 组成结构体*/
  j1939_uint8_t   Array[J1939_MSG_LENGTH + J1939_DATA_LENGTH]; /**< 联合体数组，方便快速处理结构体赋值*/
};

#define GroupExtension    PDUSpecific 
#define DestinationAddress  PDUSpecific 
/** 一个宏定义，具体变量名称作用命名
*
*/
typedef union J1939_MESSAGE_UNION J1939_MESSAGE; 

union J1939_FLAGS_UNION
{ 
  struct 
  { 
    j1939_uint8_t TransmitMessagesdCover        : 1;  //发送数据时，J1939协议接受缓存有数据覆盖
    j1939_uint8_t ReceivedMessagesdCoverOrDroppedNode : 3;
    j1939_uint8_t ReceivedMessagesdCover        : 1;  //接受数据时，J1939协议接受缓存有数据覆盖
    j1939_uint8_t ReceivedMessagesDropped       : 1;  //接受数据时，J1939协议接受缓存有数据溢出
   }; 
    j1939_uint8_t   FlagVal; 
}; 

typedef union J1939_FLAGS_UNION J1939_FLAG; 

#define J1939_TRUE         1  /**< 代表函数正确返回*/
#define J1939_FALSE        0  /**< 代表函数错误返回*/
#define ADDRESS_CLAIM_TX   1  /**< 进入地址竞争发送处理模式*/
#define ADDRESS_CLAIM_RX   2  /**< 进入地址竞争接受处理模式*/

CAN_NODE                        Can_Node;

//初始化函数
void       J1939_Initialization(void);
//CAN驱动收发中断入口
void         J1939_ISR( void );
//心跳函数,定时被调用
void       J1939_Poll( void );
//读取单帧消息
j1939_uint8_t  J1939_Read_Message( J1939_MESSAGE *MsgPtr, CAN_NODE  _Can_Node);
//发送单帧消息
j1939_uint8_t    J1939_Send_Message( J1939_MESSAGE *MsgPtr, CAN_NODE  _Can_Node);
//多帧（多组）消息发送函数  (RTS/CTS传输协议)
j1939_int8_t   J1939_TP_TX_Message(j1939_uint32_t PGN, j1939_uint8_t DA, j1939_uint8_t *data, j1939_uint16_t data_num, CAN_NODE  _Can_Node);
//多帧（多组）消息接受函数  (RTS/CTS传输协议)
j1939_int8_t   J1939_TP_RX_Message(TP_RX_MESSAGE *msg, CAN_NODE _Can_Node);
//请求获去一个PGN
void             J1939_Request_PGN(j1939_uint32_t pgn ,j1939_uint8_t DA, CAN_NODE  _Can_Node);
//创建一个PGN响应
void             J1939_Create_Response(j1939_uint8_t data[], j1939_uint16_t dataLenght, j1939_uint32_t PGN, void (*dataUPFun)(), CAN_NODE  _Can_Node);


/*
*输入：
*输出： 
*说明：基于SAE J1939协议，我们需要CAN控制器提供至少3个滤波器给J1939协议代码。三个滤波器分别配置如下： 
    1. 设置滤波器0，只接受广播信息（PF = 240 -255）。 
    2. 设置设置滤波器1，2只接受全局地址（J1939_GLOBAL_ADDRESS） 
        3. 随着程序的运行，将改变滤波器2，来适应程序逻辑。
  J1939_SetAddressFilter() 是用来设置滤波器2的， 函数主要设置PS位（目标地址），其目的是，让控制器
  只接受发送给本设备的消息。
*警告： 滤波器0，1是在CAN驱动里配置，如果对硬件滤波配置不是很熟练，可以使能软件滤波器，#define J1939SoftwareFilterEn 
*则可跳过本函数的移植和CAN硬件滤波器的配置，为了J1939协议栈性能最优化，建议只是用硬件滤波。
*/
void J1939_SetAddressFilter(unsigned char Ps_Address)
{
  switch (Can_Node)
  {
    case  Select_CAN_NODE_1:
        {
            //todo: 这里需要加入硬件过滤器
            Serial.println(__LINE__);
      break;
    }
    case  Select_CAN_NODE_2:
    {
      break;
    }
    case  Select_CAN_NODE_3:
    {
      break;
    }
    case  Select_CAN_NODE_4:
    {
      break;
    }
    default  :
    {
      break;
    }
  }
}

/*
*输入：  *MsgPtr ，协议要发送的消息，
*输出： 
*说明：      将数据 从MsgPtr结构体赋值到CAN驱动自带的结构体中
    先将传入函数的MsgPtr中的数据写到CAN的结构体，再调用CAN驱动的发送函数
    默认支持4路CAN硬件的收发。如少于4路，只需配置相应的Can_Node开关代码区，
    其他（Select_CAN_NODE）保持不变。就直接返回（break）。
*/
void J1939_CAN_Transmit(J1939_MESSAGE *MsgPtr)
{
  uint32_t i;
  switch (Can_Node)
  {
    case  Select_CAN_NODE_1:
    {
      /*加载第一路CAN硬件的29位ID*/
        uint32_t id = 0;
        MsgPtr->Mxe.Reserve = 0;
        for(i=0; i < 4; i++)
        {
            id = (id << 8) | MsgPtr->Array[i];
        }

      CAN.beginExtendedPacket(id, MsgPtr->Mxe.DataLength, false);
      CAN.write(MsgPtr->Mxe.Data, 8);
  

      //CAN硬件开始发送数据
      if(CAN.endPacket() == 0)
      {
        Serial.print(MsgPtr->Mxe.Reserve, HEX);
        Serial.println(" send fail");
      }
      break;
    }
    case  Select_CAN_NODE_2:
    {
  
      /*加载第二路CAN硬件的29位ID*/

      /*CAN硬件加载数据长度*/
  
      /*CAN硬件加载数据*/

      /*CAN硬件加载RTR*/

      //CAN硬件开始发送数据
      break;
    }
    case  Select_CAN_NODE_3:
    {
      
      /*加载第三路CAN硬件的29位ID*/

      /*CAN硬件加载数据长度*/
  
      /*CAN硬件加载数据*/

      /*CAN硬件加载RTR*/

      //CAN硬件开始发送数据
      break;
    }
    case  Select_CAN_NODE_4:
    {
      /*加载第四路CAN硬件的29位ID*/

      /*CAN硬件加载数据长度*/
  
      /*CAN硬件加载数据*/

      /*CAN硬件加载RTR*/

      //CAN硬件开始发送数据
      break;
    }
    default  :
    {
      break;
    }
  }
}
/*
*输入：     *MsgPtr 数据要存入的内存的指针
*输出：      1 | 0
*说明：      读取CAN驱动的数据，如果没有数据，返回0
    将CAN中的数据取出，存入J1939_MESSAGE结构体中
    默认支持4路CAN硬件的收发。如少于4路，只需配置相应的Can_Node开关代码区，
    其他（Select_CAN_NODE）保持不变。就直接返回（return 0）
*/

int J1939_CAN_Receive(J1939_MESSAGE *MsgPtr)
{
  uint32_t i;
  int ret = 0;
  switch (Can_Node)
  {
    case  Select_CAN_NODE_1:
    {
      if(CAN.parsePacket())//判断CAN硬件1是否有数据到来
      {
        //你的代码，从CAN硬件1 中将数据读取后，存入 MsgPtr
          MsgPtr->Mxe.DataLength = CAN.packetDlc();
          MsgPtr->Mxe.RTR = CAN.packetRtr();
          uint32_t j = CAN.packetId();
          for(i=0; i < 4; i++)
          {
              MsgPtr->Array[3-i] = j;
              j = j >> 8;
          }
        i = 0;
        while(CAN.available())
        {
          MsgPtr->Mxe.Data[i] = CAN.read();
          i++;
        }
        ret = 1;
      }
      else
      {
        ret = 0;
      }
      break;
    }
    case  Select_CAN_NODE_2:
    {
      ret = 0;
      break;

    }
    case  Select_CAN_NODE_3:
    {
      ret = 0;
      break;

    }
    case  Select_CAN_NODE_4:
    {
      ret = 0;
      break;
    }
    default  :
    {
      ret = 0;
    }
  }
  return ret;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    
  // start the CAN bus at 250 kbps
  if (!CAN.begin(250E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  J1939_Initialization();
}

TP_RX_MESSAGE _tp_msg;
byte tp_data[512];
void loop()
{
    J1939_MESSAGE _msg;
 //   Serial.print(micros());
 //   Serial.print(",");
    _tp_msg.data = tp_data;
    _tp_msg.data_num = 512;
    delayMicroseconds(1000);
    J1939_Poll();
 //   Serial.println(micros());


    if(RC_SUCCESS == J1939_TP_RX_Message(&_tp_msg, Select_CAN_NODE_1))
    {
        if(_tp_msg.byte_count < 512) _tp_msg.data[_tp_msg.byte_count] = 0;
        String output((char*)_tp_msg.data);
        Serial.print(output);
    }
}


//全局变量。
/** 设备的标称符
 *  
 *  我们需要在"J1939_config.H"中配置
 *  @note 在初始化中赋值，赋值参考参考1939-81文档
 */
j1939_uint8_t                   CA_Name[J1939_DATA_LENGTH];
j1939_uint8_t                   CommandedAddress; 
 
j1939_uint8_t                   J1939_Address;   
J1939_FLAG                      J1939_Flags;   
J1939_MESSAGE                   OneMessage;   
//节点地址
j1939_uint8_t         NodeAddress_1;
j1939_uint8_t         NodeAddress_2;
j1939_uint8_t         NodeAddress_3;
j1939_uint8_t         NodeAddress_4;
//接受列队全局变量(CAN_NODE_1) 
j1939_uint8_t                   RXHead_1;
j1939_uint8_t                   RXTail_1;
j1939_uint8_t                   RXQueueCount_1;
J1939_MESSAGE                   RXQueue_1[J1939_RX_QUEUE_SIZE];
//发送列队全局变量 (CAN_NODE_1)
j1939_uint8_t                   TXHead_1;
j1939_uint8_t                   TXTail_1;
j1939_uint8_t                   TXQueueCount_1;
J1939_MESSAGE                   TXQueue_1[J1939_TX_QUEUE_SIZE];
//接受列队全局变量(CAN_NODE_2)
j1939_uint8_t                   RXHead_2;
j1939_uint8_t                   RXTail_2;
j1939_uint8_t                   RXQueueCount_2;
J1939_MESSAGE                   RXQueue_2[J1939_RX_QUEUE_SIZE];
//发送列队全局变量 (CAN_NODE_2)
j1939_uint8_t                   TXHead_2;
j1939_uint8_t                   TXTail_2;
j1939_uint8_t                   TXQueueCount_2;
J1939_MESSAGE                   TXQueue_2[J1939_TX_QUEUE_SIZE];
//接受列队全局变量(CAN_NODE_3)
j1939_uint8_t                   RXHead_3;
j1939_uint8_t                   RXTail_3;
j1939_uint8_t                   RXQueueCount_3;
J1939_MESSAGE                   RXQueue_3[J1939_RX_QUEUE_SIZE];
//发送列队全局变量 (CAN_NODE_3)
j1939_uint8_t                   TXHead_3;
j1939_uint8_t                   TXTail_3;
j1939_uint8_t                   TXQueueCount_3;
J1939_MESSAGE                   TXQueue_3[J1939_TX_QUEUE_SIZE];
//接受列队全局变量(CAN_NODE_4)
j1939_uint8_t                   RXHead_4;
j1939_uint8_t                   RXTail_4;
j1939_uint8_t                   RXQueueCount_4;
J1939_MESSAGE                   RXQueue_4[J1939_RX_QUEUE_SIZE];
//发送列队全局变量 (CAN_NODE_4)
j1939_uint8_t                   TXHead_4;
j1939_uint8_t                   TXTail_4;
j1939_uint8_t                   TXQueueCount_4;
J1939_MESSAGE                   TXQueue_4[J1939_TX_QUEUE_SIZE];

struct Request_List REQUEST_LIST;

#if J1939_TP_RX_TX
//TP协议全局变量  
J1939_TP_Flags                  J1939_TP_Flags_t;   
J1939_TRANSPORT_RX_INFO         TP_RX_MSG;    
J1939_TRANSPORT_TX_INFO         TP_TX_MSG;
#endif //J1939_TP_RX_TX


static void         J1939_ReceiveMessages( void );
static j1939_uint8_t  J1939_TransmitMessages( void );

/**
* @note  硬件滤波器2 或 软件滤波器  滤波配置（设置PS段）\n
*/
void SetAddressFilter( j1939_uint8_t Address )   
{   
  /*软件滤波*/
#if J1939SoftwareFilterEn == J1939_TRUE
  switch (Can_Node)
  {
    case  Select_CAN_NODE_1:
    {
      NodeAddress_1 = Address;
      break;
    }
    case  Select_CAN_NODE_2:
    {
      NodeAddress_2 = Address;
      break;
    }
    case  Select_CAN_NODE_3:
    {
      NodeAddress_3 = Address;
      break;
    }
    case  Select_CAN_NODE_4:
    {
      NodeAddress_4 = Address;
      break;
    }
    default  :
    {
      break;
    }
  }
#endif//J1939SoftwareFilterEn
  /*硬件滤波*/
  Port_SetAddressFilter(Address);
}   

/**
* @param[in]  J1939_MESSAGE *
* @note 发送*MsgPtr的信息，所有的数据字段（比如数据长度、优先级、和源地址）必须已经设置。\n
*/
void SendOneMessage( J1939_MESSAGE *MsgPtr )   
{    
    //设置消息的最后部分,确保DataLength规范。（参考CAN B2.0）
  MsgPtr->Mxe.Res = 0;//参考J1939的数据链路层（SAE J1939-21）
  MsgPtr->Mxe.RTR = 0;
    if (MsgPtr->Mxe.DataLength > 8)
      MsgPtr->Mxe.DataLength = 8;
    //发送一帧消息，将 J1939_MESSAGE 中的所有消息加载道can模块自有的结构中     
     Port_CAN_Transmit(MsgPtr);
}   

/**
* @param[in]  MsgPtr             用户要出队的消息
* @param[in]  _Can_Node          要出队的CAN硬件编号
* @return    RC_SUCCESS          消息出队成功
* @return    RC_QUEUEEMPTY       没有消息返回
* @note      从接受队列中读取一个信息到*MsgPtr。如果我们用的是中断，需要将中断失能，在获取接受队列数据时
*/
j1939_uint8_t J1939_DequeueMessage( J1939_MESSAGE *MsgPtr, CAN_NODE  _Can_Node)
{   
    j1939_uint8_t   _rc = RC_SUCCESS;

 //***************************关接受中断********************************  
 #if J1939_POLL_ECAN == J1939_FALSE
    Port_RXinterruptDisable();
 #endif  
    switch (_Can_Node)
  {
    case  Select_CAN_NODE_1:
    {
        if (RXQueueCount_1 == 0)
        {
                _rc = RC_QUEUEEMPTY;
        }
        else
        {
            *MsgPtr = RXQueue_1[RXHead_1];
            RXHead_1 ++;
            if (RXHead_1 >= J1939_RX_QUEUE_SIZE)
              RXHead_1 = 0;
            RXQueueCount_1 --;
        }
      break;
    }
    case  Select_CAN_NODE_2:
    {
        if (RXQueueCount_2 == 0)
        {
                _rc = RC_QUEUEEMPTY;
        }
        else
        {
            *MsgPtr = RXQueue_2[RXHead_2];
            RXHead_2 ++;
            if (RXHead_2 >= J1939_RX_QUEUE_SIZE)
              RXHead_2 = 0;
            RXQueueCount_2 --;
        }
      break;
    }
    case  Select_CAN_NODE_3:
    {
        if (RXQueueCount_3 == 0)
        {
                _rc = RC_QUEUEEMPTY;
        }
        else
        {
            *MsgPtr = RXQueue_3[RXHead_3];
            RXHead_3 ++;
            if (RXHead_3 >= J1939_RX_QUEUE_SIZE)
              RXHead_3 = 0;
            RXQueueCount_3 --;
        }
      break;
    }
    case  Select_CAN_NODE_4:
    {
      if (RXQueueCount_4 == 0)
      {
          _rc = RC_QUEUEEMPTY;
      }
      else
      {
        *MsgPtr = RXQueue_4[RXHead_4];
        RXHead_4 ++;
        if (RXHead_4 >= J1939_RX_QUEUE_SIZE)
          RXHead_4 = 0;
        RXQueueCount_4 --;
      }
      break;
    }
    default  :
    {
      _rc = RC_CANNOTRECEIVE;
      break;
    }
  }
  //***************************开接受中断********************************   
#if J1939_POLL_ECAN == J1939_FALSE
   Port_RXinterruptEnable();
#endif

   return _rc;
}
/**
* @param[in] MsgPtr              存储读取消息的缓存
* @param[in] _Can_Node           读取消息的CAN硬件编号（从哪一路CAN读取数据）
* @return    RC_SUCCESS          读取消息成功，
* @return    RC_QUEUEEMPTY       读取消息不成功，没有消息。
* @note      从接受队列中读取一个信息到*MsgPtr。
*/
j1939_uint8_t J1939_Read_Message( J1939_MESSAGE *MsgPtr, CAN_NODE  _Can_Node)
{
  return  J1939_DequeueMessage(MsgPtr,_Can_Node);
}
/**
* @param[in]  MsgPtr     用户要入队的消息
* @param[in]  _Can_Node  要入队的CAN硬件编号（要选择的使用的CAN硬件编号）
* @return     RC_SUCCESS          消息入队成功 
* @return     RC_QUEUEFULL        发送列队满，消息入队失败 
* @return     RC_CANNOTTRANSMIT   系统目前不能发送消息  
* @note     这段程序，将*MsgPtr放入发送消息列队中\n
      如果信息不能入队或者发送，将有一个相应的返回提示，\n
      如果发送中断被设置（可用），当消息列队后，发送中断被使能
*/
j1939_uint8_t J1939_EnqueueMessage( J1939_MESSAGE *MsgPtr, CAN_NODE  _Can_Node)
{   
    j1939_uint8_t   _rc = RC_SUCCESS;

#if J1939_POLL_ECAN == J1939_FALSE  
    Port_TXinterruptDisable();
#endif 
   
    if (0)   
        _rc = RC_CANNOTTRANSMIT;
    else   
    {   
      switch (_Can_Node)
    {
      case  Select_CAN_NODE_1:
      {
        if ((J1939_OVERWRITE_TX_QUEUE == J1939_TRUE) ||
           (TXQueueCount_1 < J1939_TX_QUEUE_SIZE))
        {
          if (TXQueueCount_1 < J1939_TX_QUEUE_SIZE)
          {
            TXQueueCount_1 ++;
            TXTail_1 ++;
            if (TXTail_1 >= J1939_TX_QUEUE_SIZE)
              TXTail_1 = 0;
          }else{
            J1939_Flags.TransmitMessagesdCover = 1;//发送数据被覆盖，上一帧数据被覆盖
          }
          TXQueue_1[TXTail_1] = *MsgPtr;
        }
        else
          _rc = RC_QUEUEFULL;
        break;
      }
      case  Select_CAN_NODE_2:
      {
        if ((J1939_OVERWRITE_TX_QUEUE == J1939_TRUE) ||
           (TXQueueCount_2 < J1939_TX_QUEUE_SIZE))
        {
          if (TXQueueCount_2 < J1939_TX_QUEUE_SIZE)
          {
            TXQueueCount_2 ++;
            TXTail_2 ++;
            if (TXTail_2 >= J1939_TX_QUEUE_SIZE)
              TXTail_2 = 0;
          }else{
            J1939_Flags.TransmitMessagesdCover = 1;//发送数据被覆盖，上一帧数据被覆盖
          }
          TXQueue_2[TXTail_2] = *MsgPtr;
        }
        else
          _rc = RC_QUEUEFULL;
        break;
      }
      case  Select_CAN_NODE_3:
      {
        if ((J1939_OVERWRITE_TX_QUEUE == J1939_TRUE) ||
           (TXQueueCount_3 < J1939_TX_QUEUE_SIZE))
        {
          if (TXQueueCount_3 < J1939_TX_QUEUE_SIZE)
          {
            TXQueueCount_3 ++;
            TXTail_3 ++;
            if (TXTail_3 >= J1939_TX_QUEUE_SIZE)
              TXTail_3 = 0;
          }else{
            J1939_Flags.TransmitMessagesdCover = 1;//发送数据被覆盖，上一帧数据被覆盖
          }
          TXQueue_3[TXTail_3] = *MsgPtr;
        }
        else
          _rc = RC_QUEUEFULL;
        break;
      }
      case  Select_CAN_NODE_4:
      {
        if ((J1939_OVERWRITE_TX_QUEUE == J1939_TRUE) ||
           (TXQueueCount_4 < J1939_TX_QUEUE_SIZE))
        {
          if (TXQueueCount_4 < J1939_TX_QUEUE_SIZE)
          {
            TXQueueCount_4 ++;
            TXTail_4 ++;
            if (TXTail_4 >= J1939_TX_QUEUE_SIZE)
              TXTail_4 = 0;
          }else{
            J1939_Flags.TransmitMessagesdCover = 1;//发送数据被覆盖，上一帧数据被覆盖
          }
          TXQueue_4[TXTail_4] = *MsgPtr;
        }
        else
          _rc = RC_QUEUEFULL;
        break;
      }
      default  :
      {
        break;
      }
    }
    }   

#if J1939_POLL_ECAN == J1939_FALSE   
    Port_TXinterruptEnable();
    //触发发送中断
    Port_TXinterruptOk();
#endif   
    return _rc;
}   
/**
* @param[in]  MsgPtr              存储发送消息的缓存
* @param[in]  _Can_Node           发送消息的CAN硬件编号（从哪一路CAN发送数据）
* @return     RC_SUCCESS          发送消息成功 
* @return     RC_QUEUEFULL        发送消息不成功，发送列队满，消息入队失败 
* @return     RC_CANNOTTRANSMIT   发送消息不成功，系统目前不能发送消息  
* @note       如果信息不能入队或者发送，将有一个相应的返回提示，\n
*/
j1939_uint8_t J1939_Send_Message( J1939_MESSAGE *MsgPtr, CAN_NODE  _Can_Node)
{
  return  J1939_EnqueueMessage(MsgPtr,_Can_Node);
}
/**
* 
* @note 这段代码在系统初始化中被调用,（放在CAN设备初始化之后）\n
    初始化J1939全局变量\n
*/
void J1939_Initialization()
{
    /*初始化全局变量*/   
    J1939_Flags.FlagVal = 0; //没有声明地址，其他的标识位将被设置为0（复位）

    /*初始化接受和发送列队*/
    TXHead_1 = 0;
    TXHead_2 = 0;
    TXHead_3 = 0;
    TXHead_4 = 0;
    TXTail_1 = 0xFF;
    TXTail_2 = 0xFF;
    TXTail_3 = 0xFF;
    TXTail_4 = 0xFF;
    RXHead_1 = 0;
    RXHead_2 = 0;
    RXHead_3 = 0;
    RXHead_4 = 0;
    RXTail_1 = 0xFF;
    RXTail_2 = 0xFF;
    RXTail_3 = 0xFF;
    RXTail_4 = 0xFF;
  TXQueueCount_1 = 0;
  TXQueueCount_2 = 0;
  TXQueueCount_3 = 0;
  TXQueueCount_4 = 0;
    RXQueueCount_1 = 0;
    RXQueueCount_2 = 0;
    RXQueueCount_3 = 0;
    RXQueueCount_4 = 0;
  /*初始化节点地址*/
  NodeAddress_1 = J1939_STARTING_ADDRESS_1;
  NodeAddress_2 = J1939_STARTING_ADDRESS_2;
  NodeAddress_3 = J1939_STARTING_ADDRESS_3;
  NodeAddress_4 = J1939_STARTING_ADDRESS_4;
    /*初始化CAN节点的选择*/
    Can_Node = Select_CAN_NODE_1;
    /*初始化请求链表*/
    REQUEST_LIST.PGN = 0;
    REQUEST_LIST.data = J1939_NULL;
  REQUEST_LIST.update = J1939_NULL;
  REQUEST_LIST.lenght = 0;
  REQUEST_LIST.Can_Node = Select_CAN_NODE_Null;
  REQUEST_LIST.next = J1939_NULL;
    /*将TP协议置为空闲*/
#if J1939_TP_RX_TX
    J1939_TP_Flags_t.state = J1939_TP_NULL;
    J1939_TP_Flags_t.TP_RX_CAN_NODE = Select_CAN_NODE_Null;
    J1939_TP_Flags_t.TP_TX_CAN_NODE = Select_CAN_NODE_Null;

    TP_TX_MSG.packets_request_num = 0;
    TP_TX_MSG.packets_total = 0;
  TP_TX_MSG.packet_offset_p = 0;
  TP_TX_MSG.time = 0;
  TP_TX_MSG.state = J1939_TP_TX_WAIT;

    TP_RX_MSG.packets_ok_num = 0;
  TP_RX_MSG.packets_total = 0;
  TP_RX_MSG.time = 0;
  TP_RX_MSG.state = J1939_TP_RX_WAIT;
#endif
}   

/**
* @note 这个函数被调用，当设备产生CAN中断（可能是接受中断，也可能是发送中断）\n
        首先我们要清除中断标识位\n
        然后调用接受或者发送函数。
*/
#if J1939_POLL_ECAN == J1939_FALSE   
void J1939_ISR( void )   
{   
    //判断相关标识位,是接受还是发送
    //清除标识位
    Port_CAN_identifier_clc();
    //调用相关的处理函数    
    J1939_ReceiveMessages();   
    J1939_TransmitMessages();   
    #if J1939_TP_RX_TX
        J1939_TP_Poll();
    #endif //J1939_TP_RX_TX
    //可能存在因为错误产生中断，直接清除相关的标识位 
}   
#endif   
    
/**
* @param[in]  ElapsedTime   一个大概的毫秒数，通常设置 5 或 3
* @note 如果我们采用轮询的方式获取信息，这个函数每几个毫秒将被调用一次。\n
        不断的接受消息和发送消息从消息队列中\n
        此外，如果我们正在等待一个地址竞争反应。\n 
        如果超时，我们只接收特定的消息（目标地址 = J1939_Address）\n
  
        如果设备使用中断，此函数被调用，在调用J1939_Initialization（）函数后，因为\n
        J1939_Initialization（）可能初始化WaitingForAddressClaimContention标识位为1.\n

        如果接受到命令地址消息，这个函数也必须被调用，以防万一总线要求我们改变地址\n

        如果使用中断模式，本程序将不会处理接受和发送消息，只处理地址竞争超时。\n
*/
//声明TP轮询函数
void     J1939_TP_Poll(void);
void J1939_Poll( )   
{
    //我们必须调用J1939_ReceiveMessages接受函数，在时间被重置为0之前。
#if J1939_POLL_ECAN == J1939_TRUE
      Can_Node = Select_CAN_NODE_1;
      J1939_Address = NodeAddress_1;
        J1939_ReceiveMessages();
        J1939_TransmitMessages();
      Can_Node = Select_CAN_NODE_2;
      J1939_Address = NodeAddress_2;
        J1939_ReceiveMessages();
        J1939_TransmitMessages();
      Can_Node = Select_CAN_NODE_3;
      J1939_Address = NodeAddress_3;
        J1939_ReceiveMessages();
        J1939_TransmitMessages();
      Can_Node = Select_CAN_NODE_4;
        J1939_Address = NodeAddress_4;
        J1939_ReceiveMessages();
        J1939_TransmitMessages();
#if J1939_TP_RX_TX
        J1939_TP_Poll();
#endif //J1939_TP_RX_TX
#endif //J1939_POLL_ECAN == J1939_TRUE 
}   
void J1939_Response(const j1939_uint32_t PGN);

#if J1939SoftwareFilterEn == J1939_TRUE

/**
* @return    RC_SUCCESS         消息是可以接受
* @return    RC_CANNOTTRANSMIT  消息是不可以接受
* @note 软件滤波器\n
* @note 基于SAE J1939协议，我们需要CAN控制器提供至少3个滤波器给J1939协议代码。三个滤波器分别配置如下：
        1. 设置滤波器0，只接受广播信息（PF = 240 -255）。
        2. 设置设置滤波器1，2只接受全局地址（J1939_GLOBAL_ADDRESS）
        3. 随着程序的运行，将改变滤波器2，来适应程序逻辑。
*/
j1939_uint8_t J1939_Messages_Filter(J1939_MESSAGE *MsgPtr)
{
    /*滤波器0*/
    if((MsgPtr->Mxe.PDUFormat) >= 240)
    {
        return RC_SUCCESS;
    }
    /*滤波器1*/
    if(((MsgPtr->Mxe.PDUFormat) < 240) && (MsgPtr->Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS))
    {
        return RC_SUCCESS;
    }
  /*滤波器2*/
  switch (Can_Node)
  {
    case  Select_CAN_NODE_1:
    {
      if(((MsgPtr->Mxe.PDUFormat) < 240) && (MsgPtr->Mxe.PDUSpecific == NodeAddress_1))
      {
        return RC_SUCCESS;
      }
      break;
    }
    case  Select_CAN_NODE_2:
    {
      if(((MsgPtr->Mxe.PDUFormat) < 240) && (MsgPtr->Mxe.PDUSpecific == NodeAddress_2))
      {
        return RC_SUCCESS;
      }
      break;
    }
    case  Select_CAN_NODE_3:
    {
      if(((MsgPtr->Mxe.PDUFormat) < 240) && (MsgPtr->Mxe.PDUSpecific == NodeAddress_3))
      {
        return RC_SUCCESS;
      }
      break;
    }
    case  Select_CAN_NODE_4:
    {
      if(((MsgPtr->Mxe.PDUFormat) < 240) && (MsgPtr->Mxe.PDUSpecific == NodeAddress_4))
      {
        return RC_SUCCESS;
      }
      break;
    }
    default  :
    {
      break;
    }
  }
    return RC_CANNOTTRANSMIT;
}

#endif //J1939SoftwareFilterEn

/**
* @note 这段程序被调用，当CAN收发器接受数据（中断 或者 轮询）。\n
        如果一个信息被接受, 它将被调用\n
        如果信息是一个网络管理信息或长帧传输（TP），接受的信息将被加工处理，在这个函数中。\n
        否则, 信息将放置在用户的接收队列。\n
        注意：在这段程序运行期间中断是失能的。\n
*/
void J1939_ReceiveMessages( void )   
{
#if J1939_TP_RX_TX
  j1939_uint32_t _pgn = 0;
#endif //J1939_TP_RX_TX
    /*从接收缓存中读取信息到OneMessage中，OneMessage是一个全局变量*/
    /*Port_CAN_Receive函数读取到数据返回1，没有数据则返回0*/
    if(Port_CAN_Receive(&OneMessage))
    {
#if J1939SoftwareFilterEn == J1939_TRUE
    if(J1939_Messages_Filter(&OneMessage) != RC_SUCCESS)
    {
        return ;
    }
#endif //J1939SoftwareFilterEn
#if 0
    String out;
    out += "PF:";
    out += OneMessage.Mxe.PDUFormat;
    out += "PS:";
    out += OneMessage.Mxe.PDUSpecific;
    out += "SA:";
    out +=OneMessage.Mxe.SourceAddress;
    out += "\n";
    out += "Data:";
    out += OneMessage.Mxe.Data[0];
    out += " ";
    out += OneMessage.Mxe.Data[1];
    out += " ";
    out += OneMessage.Mxe.Data[2];
    out += " ";
    out += OneMessage.Mxe.Data[3];
    out += " ";
    out += OneMessage.Mxe.Data[4];
    out += " ";
    out += OneMessage.Mxe.Data[5];
    out += " ";
    out += OneMessage.Mxe.Data[6];
    out += " ";
    out += OneMessage.Mxe.Data[7];
    Serial.println(out);
#endif
        switch( OneMessage.Mxe.PDUFormat)
        { 
#if J1939_TP_RX_TX
      case J1939_PF_TP_CM:       //参考J1939-21 TP多帧传输协议
                _pgn = (j1939_uint32_t)(((j1939_uint32_t)OneMessage.Mxe.Data[7]<<16)&0xFF0000)
                            +(j1939_uint32_t)(((j1939_uint32_t)OneMessage.Mxe.Data[6]<<8)&0xFF00)
                            +(j1939_uint32_t)(((j1939_uint32_t)OneMessage.Mxe.Data[5])&0xFF);
                if((J1939_TP_Flags_t.state == J1939_TP_NULL) && (TP_RX_MSG.state == J1939_TP_RX_WAIT))
        {
                    if(OneMessage.Mxe.Data[0] == 16)
                    {
                      J1939_TP_Flags_t.state = J1939_TP_RX;
                      J1939_TP_Flags_t.TP_RX_CAN_NODE = Can_Node;

                      TP_RX_MSG.tp_rx_msg.SA = OneMessage.Mxe.SourceAddress;
                      TP_RX_MSG.tp_rx_msg.PGN = (j1939_uint32_t)(((j1939_uint32_t)OneMessage.Mxe.Data[7]<<16)&0xFF0000)
                                        +(j1939_uint32_t)(((j1939_uint32_t)OneMessage.Mxe.Data[6]<<8)&0xFF00)
                                        +(j1939_uint32_t)(((j1939_uint32_t)OneMessage.Mxe.Data[5])&0xFF);
                      /*如果系统繁忙*/
                      if(TP_RX_MSG.osbusy)
                      {
                        TP_RX_MSG.state = J1939_TP_RX_ERROR;
                        break;
                      }
                      /*判断是否有足够的内存接收数据，如果没有直接，断开连接*/
                      if(((j1939_uint32_t)(((j1939_uint32_t)OneMessage.Mxe.Data[2]<<8)&0xFF00)
                          +(j1939_uint32_t)((OneMessage.Mxe.Data[1])&0xFF)) > J1939_TP_MAX_MESSAGE_LENGTH)
                      {
                        TP_RX_MSG.state = J1939_TP_RX_ERROR;
                        break;
                      }
                      TP_RX_MSG.tp_rx_msg.byte_count = ((j1939_uint32_t)((OneMessage.Mxe.Data[2]<<8)&0xFF00)
                                       +(j1939_uint32_t)((OneMessage.Mxe.Data[1])&0xFF));
                      TP_RX_MSG.packets_total = OneMessage.Mxe.Data[3];
                      TP_RX_MSG.time = J1939_TP_T2;
                      TP_RX_MSG.state = J1939_TP_RX_READ_DATA;
                      break;
                    }
                    goto PutInReceiveQueue;
                    break;
        }
                if(J1939_TP_Flags_t.state == J1939_TP_TX)
        {
          /*校验PGN*/
          if (_pgn == TP_TX_MSG.tp_tx_msg.PGN)
          {
                switch(OneMessage.Mxe.Data[0])
                {
                  case J1939_RTS_CONTROL_BYTE:
                      /* 程序运行到这里，说明已经与网络中设备1建立虚拟链接（作为发送端），但是收到设备2的链接请求，并且同一个PGN消息请求*/
                      /* 根据J1939-21数据链路层的规定，我们要保持原有的链接，不做任何事，设备2会应为超时自动放弃链接*/
                      break;
                  case J1939_CTS_CONTROL_BYTE:
                if((J1939_TP_TX_CM_WAIT == TP_TX_MSG.state) || (J1939_TP_WAIT_ACK == TP_TX_MSG.state))
                {
                  /* 发送等待保持 */
                  if(0x00u == OneMessage.Mxe.Data[1])
                  {
                    /* 刷新等待计数器 */
                    TP_TX_MSG.time = J1939_TP_T4;
                  }
                  else
                  {
                    if((OneMessage.Mxe.Data[2]+OneMessage.Mxe.Data[1]) > (TP_TX_MSG.packets_total+1))
                    {
                      /*请求超出数据包范围*/
                      TP_TX_MSG.state = J1939_TP_TX_ERROR;
                    }
                    else
                    { /* response parameter OK */
                      TP_TX_MSG.packets_request_num = OneMessage.Mxe.Data[1];
                      TP_TX_MSG.packet_offset_p = (j1939_uint8_t)(OneMessage.Mxe.Data[2] - 1);
                      TP_TX_MSG.state = J1939_TP_TX_DT;
                    }

                  }
                }
                      break;
                  case J1939_EOMACK_CONTROL_BYTE:
                    if(J1939_TP_WAIT_ACK == TP_TX_MSG.state)
                {
                      TP_TX_MSG.state = J1939_TX_DONE;
                }
                    //这里可以增加一个对数据的校验
                      break;
                  case J1939_CONNABORT_CONTROL_BYTE:
                    //收到一个放弃连接，什么都不做，协议会在一段延时时间后主动放弃链接
                      break;
                  default:
                      break;
                }
          }
        }
        goto PutInReceiveQueue;
        break;
#endif//J1939_TP_RX_TX

#if J1939_TP_RX_TX
            case J1939_PF_DT:   
                if((TP_RX_MSG.state == J1939_TP_RX_DATA_WAIT)&&(TP_RX_MSG.tp_rx_msg.SA == OneMessage.Mxe.SourceAddress))
                {
                  TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0]-1)*7u]=OneMessage.Mxe.Data[1];
                  TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0]-1)*7u+1]=OneMessage.Mxe.Data[2];
                  TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0]-1)*7u+2]=OneMessage.Mxe.Data[3];
                  TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0]-1)*7u+3]=OneMessage.Mxe.Data[4];
                  TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0]-1)*7u+4]=OneMessage.Mxe.Data[5];
                  TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0]-1)*7u+5]=OneMessage.Mxe.Data[6];
                  TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0]-1)*7u+6]=OneMessage.Mxe.Data[7];
          /*特殊处理重新接受已接受过的数据包*/
                  if((OneMessage.Mxe.Data[0]) > TP_RX_MSG.packets_ok_num)
          {
                    TP_RX_MSG.packets_ok_num++;
          }
          TP_RX_MSG.time = J1939_TP_T1;
          /*判断是否收到偶数个数据包或者读取到最后一个数据包*/
        //  if((TP_RX_MSG.packets_ok_num%2 == 0) ||(TP_RX_MSG.packets_ok_num == TP_RX_MSG.packets_total))
          if((true) ||(TP_RX_MSG.packets_ok_num == TP_RX_MSG.packets_total))
          {
            TP_RX_MSG.state = J1939_TP_RX_READ_DATA;
            break ;
          }
          break ;
                }
                //程序不可能运行到这，但是我们不能放弃接受的数据包
                goto PutInReceiveQueue;
#endif//J1939_TP_RX_TX
            case J1939_PF_REQUEST:   
        /*用OneMessage.Mxe.PGN 来存下被请求的PGN*/
        if(OneMessage.Mxe.Data[1] < 240)
        {
          OneMessage.Mxe.PGN = (j1939_uint32_t)(((j1939_uint32_t)OneMessage.Mxe.Data[2]<<16)&0x030000)
                    +(j1939_uint32_t)(((j1939_uint32_t)OneMessage.Mxe.Data[1]<<8)&0xFF00)
                    +0x00;
        }else{
          OneMessage.Mxe.PGN = (j1939_uint32_t)(((j1939_uint32_t)OneMessage.Mxe.Data[2]<<16)&0x030000)
                    +(j1939_uint32_t)(((j1939_uint32_t)OneMessage.Mxe.Data[1]<<8)&0xFF00)
                    +(j1939_uint32_t)((OneMessage.Mxe.Data[0])&0xFF);
        }
        J1939_Response(OneMessage.Mxe.PGN);
                break;    
            default:   
PutInReceiveQueue:   
      {
/*
if(OneMessage.Mxe.PDUFormat < 240){
  OneMessage.Mxe.PGN = (j1939_uint32_t)((OneMessage.Array[0]<<16)&0x030000)
            +(j1939_uint32_t)((OneMessage.Array[1]<<8)&0xFF00)
            +0x00;
}else{
  OneMessage.Mxe.PGN = (j1939_uint32_t)((OneMessage.Array[0]<<16)&0x030000)
            +(j1939_uint32_t)((OneMessage.Array[1]<<8)&0xFF00)
            +(j1939_uint32_t)((OneMessage.Array[2])&0xFF);
}
*/
              if(OneMessage.Mxe.PDUFormat < 240){
                OneMessage.Mxe.PGN = ((j1939_uint32_t)OneMessage.Mxe.Res << 17)
                          +((j1939_uint32_t)OneMessage.Mxe.DataPage << 16)
                    +((j1939_uint32_t)OneMessage.Mxe.PDUFormat << 8);
              }else{
                OneMessage.Mxe.PGN = ((j1939_uint32_t)OneMessage.Mxe.Res << 17)
                          +((j1939_uint32_t)OneMessage.Mxe.DataPage << 16)
                    +((j1939_uint32_t)OneMessage.Mxe.PDUFormat << 8)
                    + OneMessage.Mxe.PDUSpecific;
              }

              switch (Can_Node)
        {
          case  Select_CAN_NODE_1:
          {
            if ( (J1939_OVERWRITE_RX_QUEUE == J1939_TRUE) ||
              (RXQueueCount_1 < J1939_RX_QUEUE_SIZE))
            {
              if (RXQueueCount_1 < J1939_RX_QUEUE_SIZE)
              {
                RXQueueCount_1 ++;
                RXTail_1 ++;
                if (RXTail_1 >= J1939_RX_QUEUE_SIZE)
                  RXTail_1 = 0;
              }else{
                J1939_Flags.ReceivedMessagesdCover = 1; //产生数据覆盖
                J1939_Flags.ReceivedMessagesdCoverOrDroppedNode = Select_CAN_NODE_1;
              }
              RXQueue_1[RXTail_1] = OneMessage;
            }
            else
              J1939_Flags.ReceivedMessagesDropped = 1; //产生数据溢出
            break;
          }
          case  Select_CAN_NODE_2:
          {
            if ( (J1939_OVERWRITE_RX_QUEUE == J1939_TRUE) ||
              (RXQueueCount_2 < J1939_RX_QUEUE_SIZE))
            {
              if (RXQueueCount_2 < J1939_RX_QUEUE_SIZE)
              {
                RXQueueCount_2 ++;
                RXTail_2 ++;
                if (RXTail_2 >= J1939_RX_QUEUE_SIZE)
                  RXTail_2 = 0;
              }else{
                J1939_Flags.ReceivedMessagesdCover = 1; //产生数据覆盖
                J1939_Flags.ReceivedMessagesdCoverOrDroppedNode = Select_CAN_NODE_2;
              }
              RXQueue_2[RXTail_2] = OneMessage;
            }
            else
              J1939_Flags.ReceivedMessagesDropped = 1;
            break;
          }
          case  Select_CAN_NODE_3:
          {
            if ( (J1939_OVERWRITE_RX_QUEUE == J1939_TRUE) ||
              (RXQueueCount_3 < J1939_RX_QUEUE_SIZE))
            {
              if (RXQueueCount_3 < J1939_RX_QUEUE_SIZE)
              {
                RXQueueCount_3 ++;
                RXTail_3 ++;
                if (RXTail_3 >= J1939_RX_QUEUE_SIZE)
                  RXTail_3 = 0;
              }else{
                J1939_Flags.ReceivedMessagesdCover = 1; //产生数据覆盖
                J1939_Flags.ReceivedMessagesdCoverOrDroppedNode = Select_CAN_NODE_3;
              }
              RXQueue_3[RXTail_3] = OneMessage;
            }
            else
              J1939_Flags.ReceivedMessagesDropped = 1;
            break;
          }
          case  Select_CAN_NODE_4:
          {
            if ( (J1939_OVERWRITE_RX_QUEUE == J1939_TRUE) ||
              (RXQueueCount_4 < J1939_RX_QUEUE_SIZE))
            {
              if (RXQueueCount_4 < J1939_RX_QUEUE_SIZE)
              {
                RXQueueCount_4 ++;
                RXTail_4 ++;
                if (RXTail_4 >= J1939_RX_QUEUE_SIZE)
                  RXTail_4 = 0;
              }else{
                J1939_Flags.ReceivedMessagesdCover = 1; //产生数据覆盖
                J1939_Flags.ReceivedMessagesdCoverOrDroppedNode = Select_CAN_NODE_4;
              }
              RXQueue_4[RXTail_4] = OneMessage;
            }
            else
              J1939_Flags.ReceivedMessagesDropped = 1;
            break;
          }
          default  :
          {
            break;
          }
        }
      }

        }   
    }

}   


/**
* @return    RC_SUCCESS          信息发送成功
* @return    RC_CANNOTTRANSMIT   系统没有发送消息,没有要发送的消息,或错误的CAN设备
* @note      调用这个函数后，如果发送消息列队中有消息就位，则会发送消息 ，如果不能发送消息，相关的错误代码将返回。\n
             程序运行期间，中断是被失能的。
*/
static j1939_uint8_t J1939_TransmitMessages()
{   
  switch (Can_Node)
  {
    case  Select_CAN_NODE_1:
    {
        if (TXQueueCount_1 == 0)
        {
            //如果没有要发送的消息从发送消息列队中，恢复中断(清空发送标识位)
    #if J1939_POLL_ECAN == J1939_FALSE
            Port_TXinterruptEnable();
    #endif
            return RC_CANNOTTRANSMIT;
        }
        else
        {
            while(TXQueueCount_1 > 0)
            {
                /*确保上次数据发送成功*/
                /**************可增加一个判断函数**************************/

              TXQueue_1[TXHead_1].Mxe.SourceAddress = NodeAddress_1;

                SendOneMessage( (J1939_MESSAGE *) &(TXQueue_1[TXHead_1]) );
                TXHead_1 ++;
                if (TXHead_1 >= J1939_TX_QUEUE_SIZE)
                  TXHead_1 = 0;
                TXQueueCount_1 --;
            }

           /*配置了一些标识位，使能中断*/
    #if J1939_POLL_ECAN == J1939_FALSE
            Port_TXinterruptEnable();
    #endif
        }
      break;
    }
    case  Select_CAN_NODE_2:
    {
      if (TXQueueCount_2 == 0)
      {
        //如果没有要发送的消息从发送消息列队中，恢复中断(清空发送标识位)
    #if J1939_POLL_ECAN == J1939_FALSE
        Port_TXinterruptEnable();
    #endif
        return RC_CANNOTTRANSMIT;
      }
      else
      {

        while(TXQueueCount_2 > 0)
        {
          /*确保上次数据发送成功*/
          /**************可增加一个判断函数**************************/

          TXQueue_2[TXHead_2].Mxe.SourceAddress = NodeAddress_2;

          SendOneMessage( (J1939_MESSAGE *) &(TXQueue_2[TXHead_2]) );
          TXHead_2 ++;
          if (TXHead_2 >= J1939_TX_QUEUE_SIZE)
            TXHead_2 = 0;
          TXQueueCount_2 --;
        }

         /*配置了一些标识位，使能中断*/
    #if J1939_POLL_ECAN == J1939_FALSE
        Port_TXinterruptEnable();
    #endif
      }
      break;
    }
    case  Select_CAN_NODE_3:
    {
      if (TXQueueCount_3 == 0)
      {
        //如果没有要发送的消息从发送消息列队中，恢复中断(清空发送标识位)
    #if J1939_POLL_ECAN == J1939_FALSE
        Port_TXinterruptEnable();
    #endif
        return RC_CANNOTTRANSMIT;
      }
      else
      {
        while(TXQueueCount_3 > 0)
        {
          /*确保上次数据发送成功*/
          /**************可增加一个判断函数**************************/

          TXQueue_3[TXHead_3].Mxe.SourceAddress = NodeAddress_3;

          SendOneMessage( (J1939_MESSAGE *) &(TXQueue_3[TXHead_3]) );
          TXHead_3 ++;
          if (TXHead_3 >= J1939_TX_QUEUE_SIZE)
            TXHead_3 = 0;
          TXQueueCount_3 --;
        }

         /*配置了一些标识位，使能中断*/
    #if J1939_POLL_ECAN == J1939_FALSE
        Port_TXinterruptEnable();
    #endif
      }
      break;
    }
    case  Select_CAN_NODE_4:
    {
      if (TXQueueCount_4 == 0)
      {
        //如果没有要发送的消息从发送消息列队中，恢复中断(清空发送标识位)
    #if J1939_POLL_ECAN == J1939_FALSE
        Port_TXinterruptEnable();
    #endif
        return RC_CANNOTTRANSMIT;
      }
      else
      {

        while(TXQueueCount_4 > 0)
        {
          /*确保上次数据发送成功*/
          /**************可增加一个判断函数**************************/

          TXQueue_4[TXHead_4].Mxe.SourceAddress = NodeAddress_4;

          SendOneMessage( (J1939_MESSAGE *) &(TXQueue_4[TXHead_4]) );
          TXHead_4 ++;
          if (TXHead_4 >= J1939_TX_QUEUE_SIZE)
            TXHead_4 = 0;
          TXQueueCount_4 --;
        }

         /*配置了一些标识位，使能中断*/
    #if J1939_POLL_ECAN == J1939_FALSE
        Port_TXinterruptEnable();
    #endif
      }
      break;
    }
    default  :
    {
      return RC_CANNOTTRANSMIT;
      break;
    }
  }

    return RC_SUCCESS;   
}
#if J1939_TP_RX_TX
/**
* @note 发送TP.DT，参考J1939-21
*/
void J1939_TP_DT_Packet_send(void)
{
  J1939_MESSAGE _msg;
  j1939_uint16_t _packet_offset_p;
  j1939_int32_t _i=0;
  _msg.Mxe.Priority = J1939_TP_DT_PRIORITY;
  _msg.Mxe.DataPage =0;
  _msg.Mxe.PDUFormat = J1939_PF_DT;
  _msg.Mxe.DestinationAddress = TP_TX_MSG.tp_tx_msg.SA;
  _msg.Mxe.DataLength = 8;


    /*获取请求发送的数据包数量*/
    if(TP_TX_MSG.packets_request_num > 0)
    {
      TP_TX_MSG.packets_request_num--;
      /*获取数据偏移指针*/
      _packet_offset_p = (j1939_uint16_t)(TP_TX_MSG.packet_offset_p*7u);
      /*加载数据包编号*/
      _msg.Mxe.Data[0] = (j1939_uint8_t)(1u + TP_TX_MSG.packet_offset_p);

        for(_i = 0; _i<7; _i++)
        {
          _msg.Mxe.Data[_i+1] = TP_TX_MSG.tp_tx_msg.data[_packet_offset_p + _i];
        }
        /*是否是最后一包数据消息*/
        if(TP_TX_MSG.packet_offset_p == (TP_TX_MSG.packets_total - 1u))
        {
          /*参数群是否能被填满，是否需要填充，*/
            if ( _packet_offset_p > TP_TX_MSG.tp_tx_msg.byte_count - 7 )
            {
              /*计算需要填充的数据数*/
              _i = TP_TX_MSG.tp_tx_msg.byte_count- _packet_offset_p - 7 ;

                for (    ; _i < 0  ; _i++ )
                {
                  /*我们默认J1939的参数群大小为8*/
                  _msg.Mxe.Data[_i+8] = J1939_RESERVED_BYTE ;
                }
            }


            TP_TX_MSG.packets_request_num = 0;
            TP_TX_MSG.packet_offset_p = 0;
            TP_TX_MSG.time = J1939_TP_T3;
            /* 跳转步骤，等待结束确认或则重新发送数据请求*/
            TP_TX_MSG.state = J1939_TP_WAIT_ACK;
        }
        else
        {
          /*为下一个数据发送做准备*/
          TP_TX_MSG.packet_offset_p++;
          TP_TX_MSG.state = J1939_TP_TX_DT;
        }

        /*可能队列已满，发不出去，但是这里不能靠返回值进行无限的死等*/
        J1939_EnqueueMessage(&_msg, Can_Node);
    }
    else
    {

      TP_TX_MSG.packets_request_num = 0;
      TP_TX_MSG.packet_offset_p = 0;
      TP_TX_MSG.time = J1939_TP_T3;
      TP_TX_MSG.state = J1939_TP_WAIT_ACK;
    }

}
/**
* @note 发送TP。CM-RTS,16,23,4,255,PGN消息，参考J1939-21，
*/
void J1939_CM_Start(void)
{
  j1939_uint32_t pgn_num;
  J1939_MESSAGE _msg;

    pgn_num = TP_TX_MSG.tp_tx_msg.PGN;

    _msg.Mxe.Priority = J1939_TP_CM_PRIORITY;
    _msg.Mxe.DataPage =0;
    _msg.Mxe.PDUFormat = J1939_PF_TP_CM;
    _msg.Mxe.DestinationAddress = TP_TX_MSG.tp_tx_msg.SA;
    _msg.Mxe.DataLength = 8;
    _msg.Mxe.Data[0] = J1939_RTS_CONTROL_BYTE;
    _msg.Mxe.Data[1] = (j1939_uint8_t) TP_TX_MSG.tp_tx_msg.byte_count ;
    _msg.Mxe.Data[2] = (j1939_uint8_t) ((TP_TX_MSG.tp_tx_msg.byte_count)>>8);
    _msg.Mxe.Data[3] = TP_TX_MSG.packets_total;
    _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num>>16) & 0xff);
    _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num>>8 & 0xff);
    _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);

    /*可能队列已满，发不出去，但是这里不能靠返回值进行无限的死等*/
    J1939_EnqueueMessage(&_msg, Can_Node);

  /*刷新等待时间，触发下一个步骤（）*/
    TP_TX_MSG.time = J1939_TP_T3;
    TP_TX_MSG.state = J1939_TP_TX_CM_WAIT;

}
/**
* @note 中断TP链接
*/
void J1939_TP_TX_Abort(void)
{
  J1939_MESSAGE _msg;
  j1939_uint32_t pgn_num;

  pgn_num = TP_TX_MSG.tp_tx_msg.PGN;

  _msg.Mxe.Priority = J1939_TP_CM_PRIORITY;
  _msg.Mxe.DataPage =0;
  _msg.Mxe.PDUFormat = J1939_PF_TP_CM;
  _msg.Mxe.DestinationAddress = TP_TX_MSG.tp_tx_msg.SA;
  _msg.Mxe.DataLength = 8;
  _msg.Mxe.Data[0] = J1939_CONNABORT_CONTROL_BYTE;
  _msg.Mxe.Data[1] = J1939_RESERVED_BYTE;
  _msg.Mxe.Data[2] = J1939_RESERVED_BYTE;
  _msg.Mxe.Data[3] = J1939_RESERVED_BYTE;
  _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
  _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num>>16) & 0xff);
  _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num>>8 & 0xff);
  _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);

    /*可能队列已满，发不出去，但是这里不能靠返回值进行无限的死等*/
    J1939_EnqueueMessage(&_msg, Can_Node);
  /*结束发送*/
    TP_TX_MSG.state = J1939_TX_DONE;

}
/**
* @note 中断TP链接
*/
void J1939_TP_RX_Abort(void)
{
  J1939_MESSAGE _msg;
  j1939_uint32_t pgn_num;

  pgn_num = TP_RX_MSG.tp_rx_msg.PGN;

  _msg.Mxe.Priority = J1939_TP_CM_PRIORITY;
  _msg.Mxe.DataPage =0;
  _msg.Mxe.PDUFormat = J1939_PF_TP_CM;
  _msg.Mxe.DestinationAddress = TP_RX_MSG.tp_rx_msg.SA;
  _msg.Mxe.DataLength = 8;
  _msg.Mxe.Data[0] = J1939_CONNABORT_CONTROL_BYTE;
  _msg.Mxe.Data[1] = J1939_RESERVED_BYTE;
  _msg.Mxe.Data[2] = J1939_RESERVED_BYTE;
  _msg.Mxe.Data[3] = J1939_RESERVED_BYTE;
  _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
  _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num>>16) & 0xff);
  _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num>>8 & 0xff);
  _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);

    /*可能队列已满，发不出去，但是这里不能靠返回值进行无限的死等*/
    J1939_EnqueueMessage(&_msg, Can_Node);
  /*结束发送*/
    TP_RX_MSG.state = J1939_RX_DONE;

}
/**
* @note TP的计时器
*/
j1939_uint8_t J1939_TP_TX_RefreshCMTimer(j1939_uint16_t dt_ms)
{
  if((J1939_TP_TX_CM_WAIT == TP_TX_MSG.state)||(J1939_TP_WAIT_ACK == TP_TX_MSG.state))
  {
    if(TP_TX_MSG.time > dt_ms)
    {
      TP_TX_MSG.time = TP_TX_MSG.time - dt_ms;
      return J1939_TP_TIMEOUT_NORMAL;
    }
    else
    {
      /*超时 */
      TP_TX_MSG.time = 0u;
      return  J1939_TP_TIMEOUT_ABNORMAL;
    }

  }
  else
  {
    return  J1939_TP_TIMEOUT_NORMAL;
  }
}
/**
* @note TP的计时器
*/
j1939_uint8_t J1939_TP_RX_RefreshCMTimer(j1939_uint16_t dt_ms)
{
  if((J1939_TP_RX_DATA_WAIT == TP_RX_MSG.state))
  {
    if(TP_RX_MSG.time > dt_ms)
    {
      TP_RX_MSG.time = TP_RX_MSG.time - dt_ms;
      return J1939_TP_TIMEOUT_NORMAL;
    }
    else
    {
      /*超时 */
      TP_RX_MSG.time = 0u;
      return  J1939_TP_TIMEOUT_ABNORMAL;
    }

  }
  else
  {
    return  J1939_TP_TIMEOUT_NORMAL;
  }
}
/**
* @note 发送读取数据 TP.CM_CTS 和 EndofMsgAck消息。
*/
void J1939_read_DT_Packet()
{
  J1939_MESSAGE _msg;
  j1939_uint32_t pgn_num;
  pgn_num = TP_RX_MSG.tp_rx_msg.PGN;

  _msg.Mxe.Priority = J1939_TP_CM_PRIORITY;
  _msg.Mxe.DataPage =0;
  _msg.Mxe.PDUFormat = J1939_PF_TP_CM;
  _msg.Mxe.DestinationAddress = TP_RX_MSG.tp_rx_msg.SA;
  _msg.Mxe.DataLength = 8;

  /*如果系统繁忙,保持链接但是不传送消息*/
  if(TP_RX_MSG.osbusy)
  {
    _msg.Mxe.Data[0] = J1939_CTS_CONTROL_BYTE;
    _msg.Mxe.Data[1] = 0;
    _msg.Mxe.Data[2] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[3] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num>>16) & 0xff);
    _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num>>8 & 0xff);
    _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);
        /*可能队列已满，发不出去，但是这里不能靠返回值进行无限的死等*/
        J1939_EnqueueMessage(&_msg, Can_Node);
    return ;
  }
  if(TP_RX_MSG.packets_total > TP_RX_MSG.packets_ok_num)
  {
    /*最后一次响应，如果不足2包数据*/
    if((TP_RX_MSG.packets_total - TP_RX_MSG.packets_ok_num)==1)
    {
      _msg.Mxe.Data[0] = J1939_CTS_CONTROL_BYTE;
      _msg.Mxe.Data[1] = 1;
      _msg.Mxe.Data[2] = TP_RX_MSG.packets_total;
      _msg.Mxe.Data[3] = J1939_RESERVED_BYTE;
      _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
      _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num>>16) & 0xff);
      _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num>>8 & 0xff);
      _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);
            /*可能队列已满，发不出去，但是这里不能靠返回值进行无限的死等*/
            J1939_EnqueueMessage(&_msg, Can_Node);
      TP_RX_MSG.state = J1939_TP_RX_DATA_WAIT;
      return ;
    }
    _msg.Mxe.Data[0] = J1939_CTS_CONTROL_BYTE;
    //_msg.Mxe.Data[1] = TP_RX_MSG.packets_total - TP_RX_MSG.packets_ok_num;
    _msg.Mxe.Data[1] = 1;
    _msg.Mxe.Data[2] = (TP_RX_MSG.packets_ok_num + 1);
    _msg.Mxe.Data[3] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num>>16) & 0xff);
    _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num>>8 & 0xff);
    _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);

    /*可能队列已满，发不出去，但是这里不能靠返回值进行无限的死等*/
    J1939_EnqueueMessage(&_msg, Can_Node);
    TP_RX_MSG.state = J1939_TP_RX_DATA_WAIT;
    return ;
  }else
  {
    /*发送传输正常结束消息，EndofMsgAck*/
    _msg.Mxe.Data[0] = J1939_EOMACK_CONTROL_BYTE;
    _msg.Mxe.Data[1] = (TP_RX_MSG.tp_rx_msg.byte_count & 0x00ff);
    _msg.Mxe.Data[2] = ((TP_RX_MSG.tp_rx_msg.byte_count >> 8) & 0x00ff);
    _msg.Mxe.Data[3] = TP_RX_MSG.packets_total;
    _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num>>16) & 0xff);
    _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num>>8 & 0xff);
    _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);
        /*可能队列已满，发不出去，但是这里不能靠返回值进行无限的死等*/
        J1939_EnqueueMessage(&_msg, Can_Node);
    TP_RX_MSG.state = J1939_RX_DONE;
    return ;
  }
}
/**
* @note TP协议的心跳，为了满足在总线的计时准确，10ms轮询一次   J1939_TP_TX_RefreshCMTimer(10)\n
    如果想要更高的分辨率，1ms轮询一次，但是要改下面计时函数  J1939_TP_TX_RefreshCMTimer(1)
*/
void J1939_TP_Poll()
{
  if(J1939_TP_Flags_t.state == J1939_TP_NULL || J1939_TP_Flags_t.state == J1939_TP_OSBUSY)
  {
    return ;
  }
  if(J1939_TP_Flags_t.state == J1939_TP_RX)
  {
    Can_Node = J1939_TP_Flags_t.TP_RX_CAN_NODE;
    switch(TP_RX_MSG.state)
    {
    case J1939_TP_RX_WAIT:
      ;
        break;
    case J1939_TP_RX_READ_DATA:
      /*发送读取数据 TP.CM_CTS 和 EndofMsgAck消息*/
      J1939_read_DT_Packet();
      break;
    case J1939_TP_RX_DATA_WAIT:
      /*等待TP.DT帧传输的消息*/
      if(J1939_TP_TIMEOUT_ABNORMAL == J1939_TP_RX_RefreshCMTimer(1))
      {
        /* 等待超时，发生连接异常，跳转到异常步骤 */
        Serial.println("timeout");
        TP_RX_MSG.state = J1939_TP_RX_ERROR;
      }
      break;
    case J1939_TP_RX_ERROR:
      J1939_TP_RX_Abort();
      J1939_TP_Flags_t.TP_RX_CAN_NODE = Select_CAN_NODE_Null;
      break;
    case J1939_RX_DONE:
      TP_RX_MSG.packets_ok_num = 0;
      TP_RX_MSG.packets_total = 0;
      TP_RX_MSG.time = J1939_TP_T3;
      TP_RX_MSG.state = J1939_TP_RX_WAIT;
      J1939_TP_Flags_t.state = J1939_TP_NULL;
      break;
        default:
            break;
    }
    return ;
  }
  if(J1939_TP_Flags_t.state == J1939_TP_TX)
  {
    Can_Node = J1939_TP_Flags_t.TP_TX_CAN_NODE;
    switch (TP_TX_MSG.state)
    {
      case J1939_TP_TX_WAIT:
        /*没有要发送的数据*/
          break;
      case J1939_TP_TX_CM_START:
        /*发送TP.CM_RTS帧传输的消息(参考j1939-21)*/
        J1939_CM_Start();
        break;
      case J1939_TP_TX_CM_WAIT:
          /*等待TP.CM_CTS帧传输的消息*/
        if(J1939_TP_TIMEOUT_ABNORMAL == J1939_TP_TX_RefreshCMTimer(1))
        {
          /* 等待超时，发生连接异常，跳转到异常步骤 */
          TP_TX_MSG.state = J1939_TP_TX_ERROR;
        }
      break;
      case J1939_TP_TX_DT:
        J1939_TP_DT_Packet_send();
          break;
          case J1939_TP_WAIT_ACK:
            /*等待TP.EndofMsgACK帧传输的消息*/
        if(J1939_TP_TIMEOUT_ABNORMAL == J1939_TP_TX_RefreshCMTimer(1))
        {
          /* 等待超时，发生连接异常，跳转到异常步骤 */
          TP_TX_MSG.state = J1939_TP_TX_ERROR;
        }
              break;
      case J1939_TP_TX_ERROR:
        J1939_TP_TX_Abort();

          break;
      case J1939_TX_DONE:
        TP_TX_MSG.packets_request_num = 0;
        TP_TX_MSG.packet_offset_p = 0;
        TP_TX_MSG.time = J1939_TP_T3;
        TP_TX_MSG.state = J1939_TP_TX_WAIT;
        J1939_TP_Flags_t.state = J1939_TP_NULL;
          break;
          default:
            //程序不会运行到这里来，可以增加一个调试输出
              break;
    }
    return ;
  }
}

/**这是一个非阻塞io接口
*
* @param[in] PGN  TP会话的参数群编号
* @param[in] SA   TP会话的目标地址
* @param[in] *data  TP会话的数据缓存地址
* @param[in] data_num TP会话的数据大小
* @param[in]  _Can_Node  要入队的CAN硬件编号（要选择的使用的CAN硬件编号）
* @return    RC_SUCCESS        成功打开TP链接，开始进入发送流程
* @return    RC_CANNOTTRANSMIT 不能发送，因为TP协议已经建立虚拟链接，并且未断开
* @note      TP协议的发送函数
*/
j1939_int8_t J1939_TP_TX_Message(j1939_uint32_t PGN,j1939_uint8_t DA,j1939_uint8_t *data,j1939_uint16_t data_num, CAN_NODE  _Can_Node)
{
  j1939_uint16_t _byte_count =0;
  /*取得发送权限*/
  if(J1939_TP_Flags_t.state == J1939_TP_NULL)
  {
    J1939_TP_Flags_t.state = J1939_TP_TX;
    J1939_TP_Flags_t.TP_TX_CAN_NODE = _Can_Node;
  }else
  {
    return RC_CANNOTTRANSMIT;//不能发送，因为TP协议已经建立虚拟链接，并且未断开
  }

  TP_TX_MSG.tp_tx_msg.PGN = PGN;
  TP_TX_MSG.tp_tx_msg.SA = DA;
  TP_TX_MSG.tp_tx_msg.byte_count = data_num;
  for(_byte_count = 0;_byte_count < data_num;_byte_count++)
  {
    TP_TX_MSG.tp_tx_msg.data[_byte_count] = data[_byte_count];
  }
  TP_TX_MSG.packet_offset_p = 0;
  TP_TX_MSG.packets_request_num = 0;
  TP_TX_MSG.packets_total = data_num/7;
  if((data_num%7) != 0)
  {
    TP_TX_MSG.packets_total ++;
  }
  TP_TX_MSG.time = J1939_TP_T3;
  //触发开始CM_START
  TP_TX_MSG.state = J1939_TP_TX_CM_START;


  return RC_SUCCESS;
}

/**
* @param[in]  msg.data       读取数据的缓存
* @param[in]  msg.data_num   读取数据的缓存大小
* @param[in]  _Can_Node      要入队的CAN硬件编号（要选择的使用的CAN硬件编号）
* @param[out] msg.SA         数据源地址
* @param[out] msg.byte_count 数据大小
* @param[out] msg.PGN        数据参数群编号
* @return  RC_CANNOTRECEIVE 不能接受，TP协议正在接受数据中
* @return  RC_SUCCESS   读取数据成功
* @note TP的接受函数 , 接受缓存的大小必须大于接受数据的大小，建议初始化缓存大小用  J1939_TP_MAX_MESSAGE_LENGTH\n
    请正确带入 缓存区的大小，参数错误程序运行有风险
*/
j1939_int8_t J1939_TP_RX_Message(TP_RX_MESSAGE *msg, CAN_NODE _Can_Node)
{
  j1939_uint16_t _a = 0;
  /*判断是否能读取数据*/
  if(J1939_TP_Flags_t.state == J1939_TP_NULL && TP_RX_MSG.tp_rx_msg.PGN != 0)
  {
    J1939_TP_Flags_t.state = J1939_TP_OSBUSY;
  }else
  {
    return RC_CANNOTRECEIVE;//不能接受，TP协议正在接受数据中,或没有数据
  }
  //判断是不是要读取那一路CAN数据
  if(_Can_Node != J1939_TP_Flags_t.TP_RX_CAN_NODE)
  {
        /*释放TP接管权限*/
        if(J1939_TP_Flags_t.state == J1939_TP_OSBUSY)
        {
            J1939_TP_Flags_t.state = J1939_TP_NULL;
        }
    return RC_CANNOTRECEIVE;
  }
    //判断数据缓存够不够
    if((msg->data_num) < TP_RX_MSG.tp_rx_msg.byte_count)
  {
    return RC_CANNOTRECEIVE;//不能接受，缓存区太小
  }

    /*获取数据*/
    for(_a = 0;_a < msg->data_num;_a++)
  {
        msg->data[_a] = TP_RX_MSG.tp_rx_msg.data[_a];
  }
    /*获取数据 源地址*/
    msg->SA  =  TP_RX_MSG.tp_rx_msg.SA;
    /*获取数据的大小*/
    msg->byte_count  =  TP_RX_MSG.tp_rx_msg.byte_count;
    /*获取数据PGN*/
    msg->PGN  =  TP_RX_MSG.tp_rx_msg.PGN;

    /*丢弃读取过的数据*/
  TP_RX_MSG.tp_rx_msg.byte_count= 0u;
  TP_RX_MSG.tp_rx_msg.PGN = 0;

  /*释放TP接管权限*/
  if(J1939_TP_Flags_t.state == J1939_TP_OSBUSY)
  {
    J1939_TP_Flags_t.state = J1939_TP_NULL;
  }

  return RC_SUCCESS;
}
/**
* @param[in] pgn  被请求的参数群
* @param[in] DA   目标地址（DestinationAddress） 当DA = 0xff表示是全局请求
* @param[in] _Can_Node  要入队的CAN硬件编号（要选择的使用的CAN硬件编号）
* @note      请求（从全局范围或则特定目的地的）参数群，请求规则J1939-21的16-17页，有明确的说明
*/
void J1939_Request_PGN(j1939_uint32_t pgn ,j1939_uint8_t DA, CAN_NODE  _Can_Node)
{
  J1939_MESSAGE _msg;

  _msg.Mxe.DataPage                = 0;
  _msg.Mxe.Priority                = J1939_REQUEST_PRIORITY;
  _msg.Mxe.DestinationAddress      = DA;
  _msg.Mxe.DataLength              = 3;
  _msg.Mxe.PDUFormat               = J1939_PF_REQUEST;
  _msg.Mxe.Data[0]             = (j1939_uint8_t)(pgn & 0x000000FF);
  _msg.Mxe.Data[1]             = (j1939_uint8_t)((pgn & 0x0000FF00) >> 8);
  _msg.Mxe.Data[2]         = (j1939_uint8_t)((pgn & 0x00FF0000) >> 16);

  while (J1939_EnqueueMessage( &_msg, _Can_Node) != RC_SUCCESS);
}
/**
* @param[in]  data        需要发送数据的缓存
* @param[in]  dataLenght  发送数据的缓存大小
* @param[in]  PGN         需要发送数据的PGN(参数群编号)
* @param[in]  void (*dataUPFun)()  用于更新缓存data 的函数地址指针
* @param[in]  _Can_Node      要入队的CAN硬件编号（要选择的使用的CAN硬件编号）
* @note 创建一个PGN 的 请求 对应的 响应\n 如果收到改请求则先运行 REQUEST_LIST.dataUPFun(),在将数据REQUEST_LIST.data发送出去
* @warning  本函数只能被串行调用，（多线程）并行调用请在函数外加互斥操作
*/
void J1939_Create_Response(j1939_uint8_t data[],j1939_uint16_t dataLenght,j1939_uint32_t PGN,void (*dataUPFun)(),CAN_NODE  _Can_Node)
{
  /*查找可用的链表项*/
  struct Request_List * _requestList = &REQUEST_LIST;
  while(J1939_NULL != _requestList->next)
  {
    _requestList = _requestList->next;
  }
  _requestList->next = (struct Request_List *)malloc(sizeof(struct Request_List));
  _requestList = _requestList->next;

  /*对新的链表项赋值*/
  _requestList->data = data;
  _requestList->lenght = dataLenght;
  _requestList->PGN = PGN;
  _requestList->update = dataUPFun;
    _requestList->Can_Node = _Can_Node;
  _requestList->next = J1939_NULL;

}
/**
* @note 当收到一个PGN请求后，如果有REQUEST_LIST中有相应的PGN，则会自动发送REQUEST_LIST中的PGN。\n
  如果没有则会发送一个NACK; 本函数的响应逻辑，参考J1939-21 17页表4
*/
void J1939_Response(const j1939_uint32_t PGN)
{
  J1939_MESSAGE _msg;

  /*查找可用的链表项*/
  struct Request_List * _requestList = &REQUEST_LIST;
  while((PGN != _requestList->PGN) || (Can_Node != _requestList->Can_Node))
  {
    if(_requestList->next == J1939_NULL)
    {
      /*原文档规定 全局请求不被支持时不能响应 NACK*/
      if(OneMessage.Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS)
      {
        return;
      }
      if((PGN & 0xFF00) >= 0xF000)
      {
        return;
      }

      /*没有相应的PGN响应被创建，向总线发送一个NACK*/
      _msg.Mxe.Priority            = J1939_ACK_PRIORITY;
      _msg.Mxe.DataPage            = 0;
      _msg.Mxe.PDUFormat           = J1939_PF_ACKNOWLEDGMENT;
      _msg.Mxe.DestinationAddress  = OneMessage.Mxe.SourceAddress;
      _msg.Mxe.DataLength          = 8;
      _msg.Mxe.SourceAddress     = J1939_Address;
      _msg.Mxe.Data[0]         = J1939_NACK_CONTROL_BYTE;
      _msg.Mxe.Data[1]         = 0xFF;
      _msg.Mxe.Data[2]         = 0xFF;
      _msg.Mxe.Data[3]         = 0xFF;
      _msg.Mxe.Data[4]         = 0xFF;
      _msg.Mxe.Data[5]         = (PGN & 0x0000FF);
      _msg.Mxe.Data[6]         = ((PGN >> 8) & 0x0000FF);
      _msg.Mxe.Data[7]         = ((PGN >> 16) & 0x0000FF);

      SendOneMessage( (J1939_MESSAGE *) &_msg);
      return ;
    }else
    {
      _requestList = _requestList->next;
    }
  }

  /*调用dataUPFun（）函数，主要用于参数群数据更新*/
  if(J1939_NULL != _requestList->update)
  {
    _requestList->update();
  }

  /*响应请求*/
  if(_requestList->lenght > 8)
  {
    /*回一个确认响应多帧(非广播多帧)*/
    if(RC_SUCCESS != J1939_TP_TX_Message(_requestList->PGN,OneMessage.Mxe.SourceAddress,_requestList->data,_requestList->lenght,Can_Node))
    {
      /*原文档规定 全局请求不被支持时不能响应 NACK*/
      if(OneMessage.Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS)
      {
        return;
      }

      /*如果长帧发送不成功*/
      _msg.Mxe.Priority            = J1939_ACK_PRIORITY;
      _msg.Mxe.DataPage            = 0;
      _msg.Mxe.PDUFormat           = J1939_PF_ACKNOWLEDGMENT;
      _msg.Mxe.DestinationAddress  = OneMessage.Mxe.SourceAddress;
      _msg.Mxe.DataLength          = 8;
      _msg.Mxe.SourceAddress     = J1939_Address;
      _msg.Mxe.Data[0]         = J1939_ACCESS_DENIED_CONTROL_BYTE;
      _msg.Mxe.Data[1]         = 0xFF;
      _msg.Mxe.Data[2]         = 0xFF;
      _msg.Mxe.Data[3]         = 0xFF;
      _msg.Mxe.Data[4]         = 0xFF;
      _msg.Mxe.Data[5]         = (PGN & 0x0000FF);
      _msg.Mxe.Data[6]         = ((PGN >> 8) & 0x0000FF);
      _msg.Mxe.Data[7]         = ((PGN >> 16) & 0x0000FF);

      SendOneMessage( (J1939_MESSAGE *) &_msg);
      return ;
    }

    /*回一个确认响应*/
    _msg.Mxe.Priority            = J1939_ACK_PRIORITY;
    _msg.Mxe.DataPage            = 0;
    _msg.Mxe.PDUFormat           = J1939_PF_ACKNOWLEDGMENT;
    /*原文档规定 全局请求响应到全局*/
    if(OneMessage.Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS)
    {
      _msg.Mxe.DestinationAddress  = 0XFF;
    }else{
      _msg.Mxe.DestinationAddress  = OneMessage.Mxe.SourceAddress;
    }
    _msg.Mxe.DataLength          = 8;
    _msg.Mxe.SourceAddress     = J1939_Address;
    _msg.Mxe.Data[0]         = J1939_ACK_CONTROL_BYTE;
    _msg.Mxe.Data[1]         = 0xFF;
    _msg.Mxe.Data[2]         = 0xFF;
    _msg.Mxe.Data[3]         = 0xFF;
    _msg.Mxe.Data[4]         = 0xFF;
    _msg.Mxe.Data[5]         = (PGN & 0x0000FF);
    _msg.Mxe.Data[6]         = ((PGN >> 8) & 0x0000FF);
    _msg.Mxe.Data[7]         = ((PGN >> 16) & 0x0000FF);
    SendOneMessage( (J1939_MESSAGE *) &_msg);
  }else{

    /*回一个确认响应*/
    _msg.Mxe.Priority            = J1939_ACK_PRIORITY;
    _msg.Mxe.DataPage            = 0;
    _msg.Mxe.PDUFormat           = J1939_PF_ACKNOWLEDGMENT;
    _msg.Mxe.SourceAddress     = J1939_Address;
    /*原文档规定 全局请求响应到全局*/
    if((OneMessage.Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS) || ((PGN & 0xFF00) >= 0xF000))
    {
      _msg.Mxe.DestinationAddress  = 0XFF;
    }else{
      _msg.Mxe.DestinationAddress  = OneMessage.Mxe.SourceAddress;
    }
    _msg.Mxe.DataLength          = 8;
    _msg.Mxe.SourceAddress     = J1939_Address;
    _msg.Mxe.Data[0]         = J1939_ACK_CONTROL_BYTE;
    _msg.Mxe.Data[1]         = 0xFF;
    _msg.Mxe.Data[2]         = 0xFF;
    _msg.Mxe.Data[3]         = 0xFF;
    _msg.Mxe.Data[4]         = 0xFF;
    _msg.Mxe.Data[5]         = (PGN & 0x0000FF);
    _msg.Mxe.Data[6]         = ((PGN >> 8) & 0x0000FF);
    _msg.Mxe.Data[7]         = ((PGN >> 16) & 0x0000FF);
    SendOneMessage( (J1939_MESSAGE *) &_msg);

    /*回一个确认响应单帧*/
    _msg.Mxe.Priority            = J1939_ACK_PRIORITY;
    _msg.Mxe.DataPage            = (((_requestList->PGN)>>16) & 0x1);
    _msg.Mxe.PDUFormat           = ((_requestList->PGN)>>8) & 0xFF;
    _msg.Mxe.SourceAddress     = J1939_Address;
    /*原文档规定 全局请求响应到全局*/
    if(OneMessage.Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS)
    {
      _msg.Mxe.DestinationAddress  = 0XFF;
    }else{
      _msg.Mxe.DestinationAddress  = OneMessage.Mxe.SourceAddress;
    }
    _msg.Mxe.DataLength          = _requestList->lenght;
    {
      j1939_uint8_t _i = 0;
      for(_i = 0;_i < (_requestList->lenght);_i++)
      {
        _msg.Mxe.Data[_i] = _requestList->data[_i];
      }
      for(;_i<8;_i++)
      {
        _msg.Mxe.Data[_i] = 0xFF;
      }
    }
    SendOneMessage( (J1939_MESSAGE *) &_msg);
  }
}
#endif
