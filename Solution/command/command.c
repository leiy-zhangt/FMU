#include "command.h"

uint8_t Command_State = 0;//指令状态
uint8_t DataNumber = 255;//读取数据量
uint32_t Storage_Number = 0;
uint32_t Storage_Addr = 0x10000;//变量存储地址
uint32_t Fuse_State = 1;//开伞状态

void Command_Receive(uint8_t *buffer)
{
  if(strcmp(buffer,"BMI_START") == 0) {Command_State = BMI_START;Sample_Start();}
  else if(strcmp(buffer,"Sample_STOP") == 0) Sample_Stop();
  else if(strcmp(buffer,"W25Q_DataConsult") == 0) W25Q_DataConsult();
  else if(strcmp(buffer,"W25Q_DataClear") == 0) W25Q_DataClear();
  else if(strcmp(buffer,"W25Q_ChipErase") == 0) {LED_EN;W25Q_ChipErase();LED_DIS;}
  else if(strcmp(buffer,"AttitudeSolution_TEST") == 0) {Command_State = AttitudeSolution_TEST;AttitudeSolution_Ttst();}
  else if(strcmp(buffer,"AccelerationSolution_TEST") == 0) {Command_State = AccelerationSolution_TEST;AccelerationSolution_Test();}
  else if(strcmp(buffer,"VelocitySolution_TEST") == 0) {Command_State = VelocitySolution_TEST;VelocitySolution_Test();}
  else if(strcmp(buffer,"PositionSolution_TEST") == 0) {Command_State = PositionSolution_TEST;PositionSolution_Test();}
  else if(strcmp(buffer,"MotionOffset_INIT") == 0) MotionOffset_Init();
  else if(strcmp(buffer,"MotionOffset_DEINIT") == 0) MotionOffset_DeInit();
  else if(strcmp(buffer,"Data_STORAGE") == 0) {Command_State = Data_STORAGE;DataStorage_Init();}
  else if(strcmp(buffer,"Data_READ") == 0) {Command_State = Data_READ;DataRead(0x10000);}
  else if(strcmp(buffer,"Height_TEST") == 0) {Command_State = Height_TEST;Height_Test();}
  else if(strcmp(buffer,"Position_INIT") == 0) Position_Init();
  else if(strcmp(buffer,"Position_DEINIT") == 0) Position_DeInit();
  else if(strcmp(buffer,"Control_START") == 0) {Command_State = Control_START;Control_Start();}
  else if(strcmp(buffer,"Control_EMERGENCY") == 0) Control_Emergency();
  else if(strcmp(buffer,"MotorCal_START") == 0) MotorCal_Start();
  else if(strcmp(buffer,"MotorCal_STOP") == 0) MotorCal_Stop();
  else Command_State = 0;
}

void Q_Init(void)
{
  double acc_x = 0,acc_y = 0,acc_z = 0;
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
  acc_x = acc_x/10;
  acc_y = acc_y/10;
  acc_z = acc_z/10;
  pitch = asin(acc_y/g);
  roll = atan2(-acc_x,acc_z);
  yaw = 0;
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
  printf("Q Init is finished!pitch:%0.4f  yaw:%0.4f  roll:%0.4f\r\n",pitch*180/PI,yaw*180/PI,roll*180/PI);
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
  LED_DIS;
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
//  LED_DIS;
  FUSE1 = 0;
  printf("FMU stop working!\r\n");
  USART3_printf("FMU stop working!\r\n");
  if(Command_State == Data_STORAGE) printf("%u points have been storaged!\r\n",Storage_Number);
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
      printf("%d datas have been stored!\r\n",i);
      break;
    }
    addr++;
  }
}

void W25Q_DataClear(void)//清除已经保存的数据及地址表
{
  uint32_t i,*res;
//  W25Q_DataConsult();
  printf("Data is clearing!\r\n");
  USART3_printf("Data is clearing!\r\n");
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
  printf("Data has been cleared!\r\n");
  USART3_printf("Data has been cleared!\r\n");
}

ErrorStatus NumberChoose(uint8_t *buffer)
{
  if(((*buffer)>='0') && ((*buffer)<='9')) DataNumber = ((*buffer++)-0x30)*10 + (*buffer - 0x30);
  else return ERROR;
  return SUCCESS;
}

//void DataRead(uint32_t addr)//数据读取函数
//{
//  double *tran,data[10];
//  uint32_t *number,r;
//  printf("Data is sending!\r\n");
//  printf("number  acc_x  acc_y  acc_z  gyr_x  gyr_y  gyr_z  yaw  pitch  roll  height\r\n");
//  while(Command_State == Data_READ)
//  {
//    W25Q_DataReceive(addr,W25Q_buffer,256);
//    number = W25Q_buffer;
//    if(*number == 0xFFFFFFFF) break;
//    else
//    {
//      for(uint8_t n=0;n<3;n++)
//      {
//        tran = W25Q_buffer + 8 +84*n;
//        number = W25Q_buffer + 4 +84*n;
//        for(uint8_t i=0;i<10;i++)
//        {
//          data[i] = *tran++;
//          if(i>5 && i<9) data[i] = data[i]*180/PI;
//        }
//        r = *number;
//        printf("%u  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f\r\n",r,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9]);
//      }
//    }
//    addr += 256;
//  }
//  printf("Data has been sended!\r\n");
//}

void DataRead(uint32_t addr)//数据读取函数
{
  double *tran,data[16];
  uint32_t *number,i;
  printf("Data is sending!\r\n");
  printf("number:p_x p_y p_z v_e v_n a_x a_y a_z g_x g_y g_z pitch roll yaw s_1 s_2\r\n");
  while(Command_State == Data_READ)
  {
    W25Q_DataReceive(addr,W25Q_buffer,128);
    number = W25Q_buffer;
    if(*number == 0xFFFFFFFF) break;
    else
    {
      printf("%u:",i);
      tran = W25Q_buffer;
      for(uint8_t n=0;n<16;n++)
      {
        printf("  %+0.4f",*tran);
        tran++;
      }
      printf("\r\n");
    }
    addr += 128;
    i++;
  }
  printf("Data has been sended!\r\n");
}

//void DataStorage(void)
//{
//  uint8_t *tran,i,offset;
//  double data[10];
//  for(i=0;i<10;i++)
//  {
//    switch(i)
//    {
//      case 0:
//        data[i] = MotionData.acc_x;
//        break;
//      case 1:
//        data[i] = MotionData.acc_y;
//        break;
//      case 2:
//        data[i] = MotionData.acc_z;
//        break;
//      case 3:
//        data[i] = MotionData.gyr_x;
//        break;
//      case 4:
//        data[i] = MotionData.gyr_y;
//        break;
//      case 5:
//        data[i] = MotionData.gyr_z;
//        break;
//      case 6:
//        data[i] = MotionData.yaw;
//        break;
//      case 7:
//        data[i] = MotionData.pitch;
//        break;
//      case 8:
//        data[i] = MotionData.roll;
//        break;
//      case 9:
//        data[i] = MotionData.height;
//        break;
//    }
//  }
//  offset = (Storage_Number%3)*84 + 4;
//  for(i = 0;i<84;i++)
//  {
//    if(i==0) tran = &Storage_Number;
//    else if(i==4) tran = data;
//    *(&W25Q_buffer[0]+i+offset) = *tran++;
//  }
////  if(Storage_Number == 0) for(uint8_t i=0;i<4;i++) W25Q_buffer[i] = 0x00;
////  else for(uint8_t i=0;i<4;i++) W25Q_buffer[i] = 0x00;
//  
//  if((Storage_Number%3 == 2)) 
//  {
//    if(Storage_Number == 0) 
//    {
//      for(i=0;i<4;i++) W25Q_buffer[i] = 0x00;
//    }
//    else 
//    { 
//      for(i=0;i<4;i++) W25Q_buffer[i] = 0x00;
//      W25Q_DataStorage(Storage_Addr,W25Q_buffer,256);
//      Storage_Addr += 256;
//    }
//  }
//  Storage_Number++;
//  if(sample_time >= 7500.0) Sample_Stop();
//}

 void DataStorage(void)
{
  static uint16_t i=0;
  double data[16];
  uint8_t *tran = data;
  data[0] = MotionData.position_x;
  data[1] = MotionData.position_y;
  data[2] = MotionData.height;
  data[3] = GPS_Data.velocity_e;
  data[4] = GPS_Data.velocity_n;
  data[5] = MotionData.acc_x;
  data[6] = MotionData.acc_y;
  data[7] = MotionData.acc_z;
  data[8] = MotionData.gyr_x;
  data[9] = MotionData.gyr_y;
  data[10] = MotionData.gyr_z;
  data[11] = MotionData.pitch;
  data[12] = MotionData.roll;
  data[13] = MotionData.yaw;
  data[14] = MotionData.serve[0];
  data[15] = MotionData.serve[1];
  tran = data;
  for(i=(Storage_Number%2)*128;i<(Storage_Number%2+1)*128;i++) W25Q_buffer[i] = *tran++;
  if((Storage_Number%2 == 0)&&(Storage_Number!=0))
  {
    W25Q_DataStorage(Storage_Addr,W25Q_buffer,256);
    Storage_Addr = Storage_Addr + 256;
  }
  Storage_Number++;
}

void DataStorage_Init(void)
{
  Q_Init();
  Storage_Number=0;
  Storage_Addr = 0x10000;
  Fuse_State = 0;
  printf("Data is storaging!\r\n");
  USART3_printf("Data is storaging!\r\n");
  Sample_Start();
}

void MotionOffset_Init(void)
{
  double data[9];
  uint8_t *tran;
  tran = data;
  printf("FMU offset is begining!\r\n");
  USART3_printf("FMU offset is begining!\r\n");
  double acc_x_offset = 0;
  double acc_y_offset = 0;
  double acc_z_offset = 0;
  double gyr_x_offset = 0;
  double gyr_y_offset = 0;
  double gyr_z_offset = 0;
  double adxl_x_offset = 0;
  double adxl_y_offset = 0;
  double adxl_z_offset = 0;
  MotionOffset.acc_x_offset = 0;
  MotionOffset.acc_y_offset = 0;
  MotionOffset.acc_z_offset = 0;
  MotionOffset.gyr_x_offset = 0;
  MotionOffset.gyr_y_offset = 0;
  MotionOffset.gyr_z_offset = 0;
  MotionOffset.adxl_x_offset = 0;
  MotionOffset.adxl_y_offset = 0;
  MotionOffset.adxl_z_offset = 0;
  W25Q_SectorErase(0);
  for(uint16_t i = 0;i<500;i++)
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
    delay_ms(10);
  }
  MotionOffset.acc_x_offset = acc_x_offset/500;
  MotionOffset.acc_y_offset = acc_y_offset/500;
  MotionOffset.acc_z_offset = acc_z_offset/500 - g;
  MotionOffset.gyr_x_offset = gyr_x_offset/500;
  MotionOffset.gyr_y_offset = gyr_y_offset/500;
  MotionOffset.gyr_z_offset = gyr_z_offset/500;
  MotionOffset.adxl_x_offset = adxl_x_offset/500;
  MotionOffset.adxl_y_offset = adxl_y_offset/500;
  MotionOffset.adxl_z_offset = adxl_z_offset/500 - g;
  data[0] = MotionOffset.acc_x_offset;
  data[1] = MotionOffset.acc_y_offset;
  data[2] = MotionOffset.acc_z_offset;
  data[3] = MotionOffset.gyr_x_offset;
  data[4] = MotionOffset.gyr_y_offset;
  data[5] = MotionOffset.gyr_z_offset;
  data[6] = MotionOffset.adxl_x_offset;
  data[7] = MotionOffset.adxl_y_offset;
  data[8] = MotionOffset.adxl_z_offset;
  for(uint8_t i = 0;i<72;i++)
  {
    W25Q_buffer[i] =  *tran++;
  }
  W25Q_DataStorage(0x00,W25Q_buffer,72);
  printf("FMU offset has finished!\r\n");
  USART3_printf("FMU offset has finished!\r\n");
}

void MotionOffset_DeInit(void)
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
  printf("FMU offset has cleared!\r\n");
  USART3_printf("FMU offset has cleared!\r\n");
}

void MotionOffset_Get(void)
{
  double *tran;
  tran = W25Q_buffer;
  W25Q_DataReceive(0x00,W25Q_buffer,72);
  MotionOffset.acc_x_offset = *tran++;
  MotionOffset.acc_y_offset = *tran++;
  MotionOffset.acc_z_offset = *tran++;
  MotionOffset.gyr_x_offset = *tran++;
  MotionOffset.gyr_y_offset = *tran++;
  MotionOffset.gyr_z_offset = *tran++;
  MotionOffset.adxl_x_offset = *tran++;
  MotionOffset.adxl_y_offset = *tran++;
  MotionOffset.adxl_z_offset = *tran;
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
  printf("PositionInit has finished!\r\n");
  USART3_printf("PositionInit has finished!\r\n");
  LED_DIS;
}

void Control_Start(void)
{
  ze_p = 0;
  pe_p = 0;
  re_p = 0;
  Q_Init();
  Sample_Start();
}

void Control_Emergency(void)
{
  Sample_Stop();
  TIM_SetCompare1(TIM3,1000);
  TIM_SetCompare2(TIM3,1000);
  TIM_SetCompare3(TIM3,1000);
  TIM_SetCompare4(TIM3,1000);
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
