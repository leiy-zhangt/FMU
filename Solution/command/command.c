#include "command.h"

uint8_t Remote_state = 0;//遥控器状态
uint8_t Command_State = 0;//指令状态
uint8_t DataNumber = 255;//读取数据量
uint32_t Storage_Number = 0;
uint32_t Storage_Addr = 0x10000;//变量存储地址
uint32_t Fuse_State = 1;//开伞状态
uint16_t RemoteChannel[5];//遥控器输入通道
uint16_t ServeOutput[4];//舵机通道输出

void Command_Receive(uint8_t *buffer)
{
  if(strcmp(buffer,"BMI_TEST") == 0) {Command_State = BMI_TEST;Sample_Start();}
  else if(strcmp(buffer,"ADXL_TEST") == 0) {Command_State = ADXL_TEST;Sample_Start();}
  else if(strcmp(buffer,"BMM_TEST") == 0) {Command_State = BMM_TEST;Sample_Start();}
  else if(strcmp(buffer,"MagnetismOffset_INIT") == 0) {Command_State = MagnetismOffset_INIT;}
  else if(strcmp(buffer,"MagnetismOffset_STOP") == 0) {Command_State = MagnetismOffset_STOP;}
  else if(strcmp(buffer,"Sample_STOP") == 0) Sample_Stop();
  else if(strcmp(buffer,"W25Q_DataConsult") == 0) W25Q_DataConsult();
  else if(strcmp(buffer,"W25Q_DataClear") == 0) W25Q_DataClear();
  else if(strcmp(buffer,"W25Q_ChipErase") == 0) {LED_EN;W25Q_ChipErase();LED_DIS;}
  else if(strcmp(buffer,"Q_INIT") == 0) {Q_Init();}
  else if(strcmp(buffer,"AttitudeSolution_TEST") == 0) {Command_State = AttitudeSolution_TEST;AttitudeSolution_Ttst();}
  else if(strcmp(buffer,"AttitudeCompensation_TEST") == 0) {Command_State = AttitudeCompensation_TEST;AttitudeSolution_Ttst();}
  else if(strcmp(buffer,"AccelerationSolution_TEST") == 0) {Command_State = AccelerationSolution_TEST;AccelerationSolution_Test();}
  else if(strcmp(buffer,"VelocitySolution_TEST") == 0) {Command_State = VelocitySolution_TEST;VelocitySolution_Test();}
  else if(strcmp(buffer,"PositionSolution_TEST") == 0) {Command_State = PositionSolution_TEST;PositionSolution_Test();}
  else if(strcmp(buffer,"IMUOffset_INIT") == 0) {Command_State = IMUUpOffset;}
  else if(strcmp(buffer,"IMUBackOffset") == 0) {Command_State = IMUBackOffset;}
  else if(strcmp(buffer,"FMUOffset_DEINIT") == 0) FMUOffset_DeInit();
  else if(strcmp(buffer,"Data_STORAGE") == 0) {Command_State = Data_STORAGE;DataStorage_Init();}
  else if(strcmp(buffer,"Data_READ") == 0) {Command_State = Data_READ;DataRead(0x10000);}
  else if(strcmp(buffer,"Height_TEST") == 0) {Command_State = Height_TEST;Height_Test();}
  else if(strcmp(buffer,"Position_INIT") == 0) Position_Init();
  else if(strcmp(buffer,"Position_DEINIT") == 0) Position_DeInit();
  else if(strcmp(buffer,"ParafoilControl_START") == 0) {Command_State = ParafoilControl_START;ParafoilControl_Start();}
  else if(strcmp(buffer,"ParafoilControl_STOP") == 0) ParafoilControl_Stop();
  else if(strcmp(buffer,"MotorCal_START") == 0) MotorCal_Start();
  else if(strcmp(buffer,"MotorCal_STOP") == 0) MotorCal_Stop();
  else if(strcmp(buffer,"FixdWingControl_TEST") == 0) {Command_State = FixdWingControl_TEST;FixdWingControl_Start();}
  else Command_State = 0;
}

void Q_Init(void)
{
  double acc_x = 0,acc_y = 0,acc_z = 0,Xh,Yh;
  double pitch,yaw,roll;
  for(uint8_t i = 0;i<10;i++)
  {
    BMI088_Measure(&BMI088_Data);
    ADXL357_Measure(&ADXL357_Data);
    acc_x += ADXL357_Data.acc_x;
    acc_y += ADXL357_Data.acc_y;
    acc_z += ADXL357_Data.acc_z;
    delay_ms(20);
  }
  BMM150_Measure(&BMM150_Data);
  acc_x = acc_x/10;
  acc_y = acc_y/10;
  acc_z = acc_z/10;
  acc_y = acc_y>g?g:acc_y;//加速度补偿，防止超过定义域
  acc_y = acc_y<-g?-g:acc_y;
  pitch = asin(acc_y/g);
  roll = atan2(-acc_x,acc_z);
  Xh= BMM150_Data.data_y*cos(roll)-BMM150_Data.data_z*sin(roll);
  Yh= BMM150_Data.data_y*sin(roll)*sin(pitch)+BMM150_Data.data_x*cos(pitch)+BMM150_Data.data_z*sin(pitch)*cos(roll);
  yaw = atan2(Xh,Yh);
  MotionData.pitch = pitch;
  MotionData.roll = roll;
  MotionData.yaw = yaw;
  T_11 = cos(roll)*cos(yaw)-sin(roll)*sin(pitch)*sin(yaw);
  T_21 = cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw);
  T_31 = -sin(roll)*cos(pitch);
  T_12 = -cos(pitch)*sin(yaw);
  T_22 = cos(pitch)*cos(yaw);
  T_32 = sin(pitch);
  T_13 = sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw);
  T_23 = sin(roll)*sin(yaw)-cos(roll)*sin(pitch)*cos(yaw);
  T_33 = cos(roll)*cos(pitch);
  q[0] = 0.5*sqrt(1+T_11+T_22+T_33);
  q[1] = 0.5*sqrt(1+T_11-T_22-T_33);
  q[2] = 0.5*sqrt(1-T_11+T_22-T_33);
  q[3] = 0.5*sqrt(1-T_11-T_22+T_33);
  if((T_32 - T_23)<0) q[1] = -q[1];
  if((T_13 - T_31)<0) q[2] = -q[2];
  if((T_21 - T_12)<0) q[3] = -q[3];
  USART_printf("Q Init is finished!pitch:%0.4f  yaw:%0.4f  roll:%0.4f\r\n",pitch*180/PI,yaw*180/PI,roll*180/PI);
}


void AttitudeSolution_Ttst(void)
{
  Q_Init();
  Sample_Start();
}

void AccelerationSolution_Test(void)
{
  Q_Init();
  Sample_Start();
}

void VelocitySolution_Test(void)
{
  Q_Init();
  MotionData.velocity_x = 0;
  MotionData.velocity_y = 0;
  MotionData.velocity_z = 0;
  Sample_Start();
}

void PositionSolution_Test(void)
{
  Q_Init();
  MotionData.velocity_x = 0;
  MotionData.velocity_y = 0;
  MotionData.velocity_z = 0;
  MotionData.position_x = 0;
  MotionData.position_y = 0;
  MotionData.position_z = 0;
  Sample_Start();
}

void Sample_Start(void)
{
  sample_time = 0;
  sample_number = 0;
  TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
  TIM_SetCounter(TIM2,0);
  sample_state = 1;
  Storage_Number = 0;
  TIM_Cmd(TIM2,ENABLE);
}

void Sample_Stop(void)
{
  TIM_Cmd(TIM2,DISABLE);
  sample_state = 1;
  USART_printf("FMU stop working!\r\n");
  Command_State = Sample_STOP;
}


void W25Q_DataConsult(void)//查询已经保存的数据量及地址
{
  uint32_t *addr;
  W25Q_DataReceive(0x1000,W25Q_buffer,256);
  addr = W25Q_buffer;
  for(uint8_t i = 0;i<64;i++)
  {
    if(*addr != 0xFFFFFFFF) W25Q_DataAddress[i] = *addr;
    else 
    {
      W25Q_DataNumber = i;
      USART_printf("%d datas have been stored!\r\n",i);
      break;
    }
    addr++;
  }
}

void W25Q_DataClear(void)//清除已经保存的数据及地址表
{
  uint32_t i,*res;
//  W25Q_DataConsult();
  USART_printf("Data is clearing!\r\n");
  LED_EN;
  for(i = 1;i<16;i++) W25Q_SectorErase(i);
  for(i=16;i<16384;i++)
  {
    W25Q_DataReceive(i*0x1000,W25Q_buffer,16);
    res = W25Q_buffer;
    if((*res == 0xFFFFFFFF)||(*(res+1) == 0xFFFFFFFF)||(*(res+2) == 0xFFFFFFFF)||(*(res+3) == 0xFFFFFFFF)) break;
    else W25Q_SectorErase(i);
  }
  W25Q_DataNumber = 0;
  LED_DIS;
  USART_printf("Data has been cleared!\r\n");
}

ErrorStatus NumberChoose(uint8_t *buffer)
{
  if(((*buffer)>='0') && ((*buffer)<='9')) DataNumber = ((*buffer++)-0x30)*10 + (*buffer - 0x30);
  else return ERROR;
  return SUCCESS;
}



void DataRead(uint32_t addr)//数据读取函数
{
  double *tran,data[16];
  uint32_t *number,i;
  USART_printf("Data is sending!\r\n");
  USART_printf("number:time lat lon hei v_e v_n a_x a_y a_z g_x g_y g_z roll pitch yaw pre hei r1~5 c_r c_p c_y o1_4\r\n");
  while(Command_State == Data_READ)
  {
    W25Q_DataReceive(addr,W25Q_buffer,256);
    number = W25Q_buffer;
    if(*number == 0xFFFFFFFF) break;
    else
    {
      USART_printf("%u:",i);
      tran = W25Q_buffer;
      for(uint8_t n=0;n<32;n++)
      {
        if(n>0&&n<3) USART_printf("  %+0.8f",*tran);
        else if((n>16&&n<22)||(n>27)) USART_printf("  %d",(int)(*tran));
        else USART_printf("  %+0.4f",*tran);
        tran++;
      }
      USART_printf("\r\n");
    }
    addr += 256;
    i++;
  }
  USART_printf("Data has been sended!\r\n");
}


void DataStorage(void)
{
  static uint16_t i=0;
  double data[32];
  uint8_t *tran = data;
  data[0] = sample_time;
//  data[1] = GPS_Data.lat;
//  data[2] = GPS_Data.lon;
//  data[3] = GPS_Data.height;
//  data[4] = GPS_Data.velocity_e;
//  data[5] = GPS_Data.velocity_n;
	data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = MotionData.acc_x;
  data[7] = MotionData.acc_y;
  data[8] = MotionData.acc_z;
  data[9] = MotionData.gyr_x;
  data[10] = MotionData.gyr_y;
  data[11] = MotionData.gyr_z;
  data[12] = MotionData.roll;
  data[13] = MotionData.pitch;
  data[14] = MotionData.yaw;
  data[15] = MotionData.pressure;
  data[16] = MotionData.height;
//  data[17] = RemoteChannel[0];
//  data[18] = RemoteChannel[1];
//  data[19] = RemoteChannel[2];
//  data[20] = RemoteChannel[3];
//  data[21] = RemoteChannel[4];
	data[17] = 0;
  data[18] = 0;
  data[19] = 0;
  data[20] = 0;
  data[21] = 0;
  data[22] = roll_e;
  data[23] = pitch_e;
  data[24] = yaw_init;
  data[25] = control_roll;
  data[26] = control_pitch;
  data[27] = control_yaw;
  data[28] = ServeOutput[0];
  data[29] = ServeOutput[1];
  data[30] = ServeOutput[2];
  data[31] = ServeOutput[3];
  tran = data;
  for(i=0;i<256;i++) W25Q_buffer[i] = *tran++;
  W25Q_DataStorage(Storage_Addr,W25Q_buffer,256);
  Storage_Addr = Storage_Addr + 256;
  Storage_Number++;
}

void DataStorage_Init(void)
{
  Q_Init();
  Storage_Number=0;
  Storage_Addr = 0x10000;
  USART_printf("Data is storaging!\r\n");
  Sample_Start();
}

void IMUOffset_Init(void)
{
  double data[10];
  uint8_t *tran;
  double acc_x_offset = 0;
  double acc_y_offset = 0;
  double acc_z_offset = 0;
  double gyr_x_offset = 0;
  double gyr_y_offset = 0;
  double gyr_z_offset = 0;
  double adxl_x_offset = 0;
  double adxl_y_offset = 0;
  double adxl_z_offset = 0;
  double g_max,g_min;
  USART_printf("FMU UP offset is begining!\r\n");
  W25Q_DataReceive(0x00,W25Q_buffer,128);
  W25Q_SectorErase(0);
  FMUOffset_DeInit();
  for(uint8_t i = 0;i<100;i++)
  {
    BMI088_Measure(&BMI088_Data);
    ADXL357_Measure(&ADXL357_Data);
    acc_x_offset += BMI088_Data.acc_x;
    acc_y_offset += BMI088_Data.acc_y;
    acc_z_offset += BMI088_Data.acc_z;
    gyr_x_offset += BMI088_Data.gyr_x;
    gyr_y_offset += BMI088_Data.gyr_y;
    gyr_z_offset += BMI088_Data.gyr_z;
    adxl_x_offset += ADXL357_Data.acc_x;
    adxl_y_offset += ADXL357_Data.acc_y;
    adxl_z_offset += ADXL357_Data.acc_z;
    delay_ms(20);
  }
  g_max = adxl_z_offset/100.0;
  adxl_z_offset = 0;
  USART_printf("First offset stop!\r\n");
  while(Command_State==IMUUpOffset) ;
  USART_printf("Second offset start!\r\n");
  for(uint8_t i = 0;i<100;i++)
  {
    BMI088_Measure(&BMI088_Data);
    ADXL357_Measure(&ADXL357_Data);
    acc_x_offset += BMI088_Data.acc_x;
    acc_y_offset += BMI088_Data.acc_y;
    acc_z_offset += BMI088_Data.acc_z;
    gyr_x_offset += BMI088_Data.gyr_x;
    gyr_y_offset += BMI088_Data.gyr_y;
    gyr_z_offset += BMI088_Data.gyr_z;
    adxl_x_offset += ADXL357_Data.acc_x;
    adxl_y_offset += ADXL357_Data.acc_y;
    adxl_z_offset += ADXL357_Data.acc_z;
    delay_ms(20);
  }
  MotionOffset.acc_x_offset = acc_x_offset/200;
  MotionOffset.acc_y_offset = acc_y_offset/200;
  MotionOffset.acc_z_offset = acc_z_offset/100;
  MotionOffset.gyr_x_offset = gyr_x_offset/200;
  MotionOffset.gyr_y_offset = gyr_y_offset/200;
  MotionOffset.gyr_z_offset = gyr_z_offset/200;
  MotionOffset.adxl_x_offset = adxl_x_offset/200;
  MotionOffset.adxl_y_offset = adxl_y_offset/200;
  g_min = adxl_z_offset/100;
  MotionOffset.adxl_z_offset = (g_max + g_min)/2;
  MotionOffset.g_position = g_max - MotionOffset.adxl_z_offset;
  data[0] = MotionOffset.acc_x_offset;
  data[1] = MotionOffset.acc_y_offset;
  data[2] = MotionOffset.acc_z_offset;
  data[3] = MotionOffset.gyr_x_offset;
  data[4] = MotionOffset.gyr_y_offset;
  data[5] = MotionOffset.gyr_z_offset;
  data[6] = MotionOffset.adxl_x_offset;
  data[7] = MotionOffset.adxl_y_offset;
  data[8] = MotionOffset.adxl_z_offset;
  data[9] = MotionOffset.g_position;
  tran = data;
  for(uint8_t i = 0;i<80;i++)
  {
    W25Q_buffer[i] =  *tran++;
  }
  W25Q_DataStorage(0x00,W25Q_buffer,128);
  USART_printf("FMU offset has finished!\r\n");
}

void MagnetismOffset_Init(void)
{
  double x_min=999,x_max=-999;
  double y_min=999,y_max=-999;
  double z_min=999,z_max=-999;
  double data[6];
  uint8_t *tran;
  BMM150_CalData.offset_x = 0;
  BMM150_CalData.offset_y = 0;
  BMM150_CalData.offset_z = 0;
  BMM150_CalData.scale_x = 1;
  BMM150_CalData.scale_y = 1;
  BMM150_CalData.scale_z = 1;
  W25Q_DataReceive(0x00,W25Q_buffer,128);
  W25Q_SectorErase(0);
  while(Command_State==MagnetismOffset_INIT)
  {
    BMM150_Measure(&BMM150_Data);
    if(x_min>BMM150_Data.data_x) x_min = BMM150_Data.data_x;
    if(x_max<BMM150_Data.data_x) x_max = BMM150_Data.data_x;
    if(y_min>BMM150_Data.data_y) y_min = BMM150_Data.data_y;
    if(y_max<BMM150_Data.data_y) y_max = BMM150_Data.data_y;
    if(z_min>BMM150_Data.data_z) z_min = BMM150_Data.data_z;
    if(z_max<BMM150_Data.data_z) z_max = BMM150_Data.data_z;
    delay_ms(20);
    USART_printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f \r\n",x_min,x_max,y_min,y_max,z_min,z_max);
  }
  BMM150_CalData.offset_x = (x_min+x_max)/2;
  BMM150_CalData.offset_y = (y_min+y_max)/2;
  BMM150_CalData.offset_z = (z_min+z_max)/2;
  BMM150_CalData.scale_x = 2/(x_max - x_min);
  BMM150_CalData.scale_y = 2/(y_max - y_min);
  BMM150_CalData.scale_z = 2/(z_max - z_min);
  data[0] = BMM150_CalData.offset_x;
  data[1] = BMM150_CalData.offset_y;
  data[2] = BMM150_CalData.offset_z;
  data[3] = BMM150_CalData.scale_x;
  data[4] = BMM150_CalData.scale_y;
  data[5] = BMM150_CalData.scale_z;
  tran = data;
  for(uint8_t i=80;i<128;i++) W25Q_buffer[i] =  *tran++;
  W25Q_DataStorage(0,W25Q_buffer,128);
  USART_printf("%0.4f %0.4f %0.4f\r\n",BMM150_CalData.offset_x,BMM150_CalData.offset_y,BMM150_CalData.offset_z);
}

void FMUOffset_DeInit(void)
{
  MotionOffset.acc_x_offset = 0;
  MotionOffset.acc_y_offset = 0;
  MotionOffset.acc_z_offset = 0;
  MotionOffset.gyr_x_offset = 0;
  MotionOffset.gyr_y_offset = 0;
  MotionOffset.gyr_z_offset = 0;
  MotionOffset.adxl_x_offset = 0;
  MotionOffset.adxl_y_offset = 0;
  MotionOffset.adxl_z_offset = 0;
  MotionOffset.g_position = g;
  BMM150_CalData.offset_x = 0;
  BMM150_CalData.offset_y = 0;
  BMM150_CalData.offset_z = 0;
  BMM150_CalData.scale_x = 1;
  BMM150_CalData.scale_y = 1;
  BMM150_CalData.scale_z = 1;
  USART_printf("FMU offset has cleared!\r\n");
}

void FMUOffset_Get(void)
{
  double *tran;
  tran = W25Q_buffer;
	//从Flash芯片获得偏差
//  W25Q_DataReceive(0x00,W25Q_buffer,128);
//  MotionOffset.acc_x_offset = *tran++;
//  MotionOffset.acc_y_offset = *tran++;
//  MotionOffset.acc_z_offset = *tran++;
//  MotionOffset.gyr_x_offset = *tran++;
//  MotionOffset.gyr_y_offset = *tran++;
//  MotionOffset.gyr_z_offset = *tran++;
//  MotionOffset.adxl_x_offset = *tran++;
//  MotionOffset.adxl_y_offset = *tran++;
//  MotionOffset.adxl_z_offset = *tran++;
//  MotionOffset.g_position = *tran++;
//  BMM150_CalData.offset_x = *tran++;
//  BMM150_CalData.offset_y = *tran++;
//  BMM150_CalData.offset_z = *tran++;
//  BMM150_CalData.scale_x = *tran++;
//  BMM150_CalData.scale_y = *tran++;
//  BMM150_CalData.scale_z = *tran;
  MotionOffset.acc_x_offset = 0;
  MotionOffset.acc_y_offset = 0;
  MotionOffset.acc_z_offset = 0;
  MotionOffset.gyr_x_offset = 0;
  MotionOffset.gyr_y_offset = 0;
  MotionOffset.gyr_z_offset = 0;
  MotionOffset.adxl_x_offset = 0;
  MotionOffset.adxl_y_offset = 0;
  MotionOffset.adxl_z_offset = 0;
  MotionOffset.g_position = 0;
  BMM150_CalData.offset_x = 0;
  BMM150_CalData.offset_y = 0;
  BMM150_CalData.offset_z = 0;
  BMM150_CalData.scale_x = 0;
  BMM150_CalData.scale_y = 0;
  BMM150_CalData.scale_z = 0;
}

void Height_Test(void)
{
  Sample_Start();
}

void Position_DeInit(void)
{
  height_init = 0;
  GPS_Data.lat_init=0;
  GPS_Data.lon_init=0;
}

void Position_Init(void)
{
  uint8_t i;
  double height_avg[100];
  LED_EN;
  Position_DeInit();
  MotionData.height = BMP388_HeightGet();
  for(i=0;i<100;i++) 
  {
    MotionData.height = BMP388_HeightCalibration();
    height_avg[i] =  MotionData.height;
    delay_ms(10);
  }
  for(i=0;i<100;i++) height_init += height_avg[i];
  height_init = height_init/100;
  MotionData.height = 0;
  GPS_Data.lat_init = GPS_Data.lat;
  GPS_Data.lon_init = GPS_Data.lon;
  USART_printf("PositionInit has finished!\r\n");
  LED_DIS;
}


void MotorCal_Start(void)
{
  TIM_SetCompare1(TIM3,2000);
  TIM_SetCompare2(TIM3,2000);
  TIM_SetCompare3(TIM3,2000);
  TIM_SetCompare4(TIM3,2000);
}

void MotorCal_Stop(void)
{
  TIM_SetCompare1(TIM3,1000);
  TIM_SetCompare2(TIM3,1000);
  TIM_SetCompare3(TIM3,1000);
  TIM_SetCompare4(TIM3,1000);
}

void ParafoilControl_Start(void)
{
  Q_Init();
  Storage_Number=0;
  Storage_Addr = 0x10000;
  USART_printf("ParafoilControl is start!\r\n");
  Sample_Start();
}

void ParafoilControl_Stop(void)
{
  Sample_Stop();
  USART_printf("ParafoilControl is start!\r\n");
}

void FixdWingControl_Start(void)
{
  Q_Init();
  Storage_Number=0;
  Storage_Addr = 0x10000;
  USART_printf("FixdWing test is start!\r\n");
  USART_printf("Data is storaging!\r\n");
  Sample_Start();
}

