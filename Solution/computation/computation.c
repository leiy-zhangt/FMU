#include "computation.h"

const double PI = 3.141592654;
uint8_t sample_state;
float dt;//采样时间间隔
MotionDataStruct MotionData;
MotionOffsetStruct MotionOffset;
double sample_time = 0;
uint32_t sample_number;
double q[4];
double T_11,T_12,T_13,T_21,T_22,T_23,T_31,T_32,T_33;//坐标准换矩阵

void SampleFrequency_Configuration(SampleFrequency frequency)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure; 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); 
  
  TIM_TimeBaseStructure.TIM_Prescaler=83;  
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
  TIM_TimeBaseStructure.TIM_Period=frequency;   
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 	
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//配置优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM2,DISABLE);
  
  if(frequency == Frequency_1Hz) dt = 1;
  else if(frequency == Frequency_10Hz) dt = 0.1;
  else if(frequency == Frequency_50Hz) dt = 0.02;
  else if(frequency == Frequency_100Hz) dt = 0.01;
  else if(frequency == Frequency_200Hz) dt = 0.005;
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)  
	{ 
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    BMI088_Measure(&BMI088_Data);
    ADXL357_Measure(&ADXL357_Data);
    BMM150_Measure(&BMM150_Data);
    MotionData.acc_x = ADXL357_Data.acc_x;
    MotionData.acc_y = ADXL357_Data.acc_y;
    MotionData.acc_z = ADXL357_Data.acc_z;
    MotionData.gyr_x = BMI088_Data.gyr_x;
    MotionData.gyr_y = BMI088_Data.gyr_y;
    MotionData.gyr_z = BMI088_Data.gyr_z;
    MotionData.height = BMP388_HeightCalibration();
    MotionData.pressure = BMP388_Data.pre;
    sample_state = 0;
    sample_time += dt;
    sample_number++;
	}
}

void AttitudeSolution(double gyr_x,double gyr_y,double gyr_z)  //对角速度进行处理，得到弧度值形式的姿态角
{
  double w[3],dq[4],q_norm;
  w[0] = gyr_x * PI /180;
  w[1] = gyr_y * PI /180;
  w[2] = gyr_z * PI /180;
  dq[0] = 0.5*(-w[0]*q[1]-w[1]*q[2]-w[2]*q[3]);
  dq[1] = 0.5*(w[0]*q[0]+w[2]*q[2]-w[1]*q[3]);
  dq[2] = 0.5*(w[1]*q[0]-w[2]*q[1]+w[0]*q[3]);
  dq[3] = 0.5*(w[2]*q[0]+w[1]*q[1]-w[0]*q[2]);
  q[0] = q[0] + dq[0]*dt;
  q[1] = q[1] + dq[1]*dt;
  q[2] = q[2] + dq[2]*dt;
  q[3] = q[3] + dq[3]*dt;
  q_norm = q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
  q_norm = sqrt(q_norm);
  q[0] = q[0]/q_norm;
  q[1] = q[1]/q_norm;
  q[2] = q[2]/q_norm;
  q[3] = q[3]/q_norm;
  MotionData.pitch = asin(2*(q[0]*q[1]+q[2]*q[3]));//初始矩阵俯仰角
  MotionData.yaw = atan2(-2*(q[1]*q[2]-q[0]*q[3]),(pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2)));//初始矩阵偏航角
  MotionData.roll = atan2(-2*(q[1]*q[3]-q[0]*q[2]),pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2));//初始矩阵滚转角
  T_11 = cos(MotionData.roll)*cos(MotionData.yaw)-sin(MotionData.roll)*sin(MotionData.pitch)*sin(MotionData.yaw);
  T_21 = cos(MotionData.roll)*sin(MotionData.yaw)+sin(MotionData.roll)*sin(MotionData.pitch)*cos(MotionData.yaw);
  T_31 = -sin(MotionData.roll)*cos(MotionData.pitch);
  T_12 = -cos(MotionData.pitch)*sin(MotionData.yaw);
  T_22 = cos(MotionData.pitch)*cos(MotionData.yaw);
  T_32 = sin(MotionData.pitch);
  T_13 = sin(MotionData.roll)*cos(MotionData.yaw)+cos(MotionData.roll)*sin(MotionData.pitch)*sin(MotionData.yaw);
  T_23 = sin(MotionData.roll)*sin(MotionData.yaw)-cos(MotionData.roll)*sin(MotionData.pitch)*cos(MotionData.yaw);
  T_33 = cos(MotionData.roll)*cos(MotionData.pitch);
}

double AttitudeCompensation(void)
{
  const float pitch_coefficient = 0.05,roll_coefficient = 0.05,yaw_coefficient = 0.1;
  double mag_x,mag_y;
  double acc_norm;
  acc_norm = sqrt(pow(MotionData.acc_x,2)+pow(MotionData.acc_y,2)+pow(MotionData.acc_z,2));
  if((acc_norm>9.6)&&(acc_norm<10))
  {
    MotionData.acc_y = MotionData.acc_y>g?g:MotionData.acc_y;//加速度补偿，防止超过定义域
    MotionData.acc_y = MotionData.acc_y<-g?-g:MotionData.acc_y;
    MotionData.pitch = asin(MotionData.acc_y/acc_norm)*pitch_coefficient + MotionData.pitch*(1-pitch_coefficient);
    MotionData.roll = atan2(-MotionData.acc_x,MotionData.acc_z)*roll_coefficient + MotionData.roll*(1-roll_coefficient);
  }
  mag_x = BMM150_Data.data_y*cos(MotionData.roll)-BMM150_Data.data_z*sin(MotionData.roll);
  mag_y = BMM150_Data.data_y*sin(MotionData.roll)*sin(MotionData.pitch)+BMM150_Data.data_x*cos(MotionData.pitch)+BMM150_Data.data_z*sin(MotionData.pitch)*cos(MotionData.roll);
  MotionData.yaw = atan2(mag_x,mag_y)*yaw_coefficient + MotionData.yaw*(1-yaw_coefficient);
//  MotionData.yaw = atan2(mag_x,mag_y);
//  printf("%+0.4f\r\n",MotionData.yaw*57.3);
  T_11 = cos(MotionData.roll)*cos(MotionData.yaw)-sin(MotionData.roll)*sin(MotionData.pitch)*sin(MotionData.yaw);
  T_21 = cos(MotionData.roll)*sin(MotionData.yaw)+sin(MotionData.roll)*sin(MotionData.pitch)*cos(MotionData.yaw);
  T_31 = -sin(MotionData.roll)*cos(MotionData.pitch);
  T_12 = -cos(MotionData.pitch)*sin(MotionData.yaw);
  T_22 = cos(MotionData.pitch)*cos(MotionData.yaw);
  T_32 = sin(MotionData.pitch);
  T_13 = sin(MotionData.roll)*cos(MotionData.yaw)+cos(MotionData.roll)*sin(MotionData.pitch)*sin(MotionData.yaw);
  T_23 = sin(MotionData.roll)*sin(MotionData.yaw)-cos(MotionData.roll)*sin(MotionData.pitch)*cos(MotionData.yaw);
  T_33 = cos(MotionData.roll)*cos(MotionData.pitch);
  q[0] = 0.5*sqrt(1+T_11+T_22+T_33);
  q[1] = 0.5*sqrt(1+T_11-T_22-T_33);
  q[2] = 0.5*sqrt(1-T_11+T_22-T_33);
  q[3] = 0.5*sqrt(1-T_11-T_22+T_33);
  if((T_32 - T_23)<0) q[1] = -q[1];
  if((T_13 - T_31)<0) q[2] = -q[2];
  if((T_21 - T_12)<0) q[3] = -q[3];
  return acc_norm;
}

void AccelerationSolution(double acc_x,double acc_y,double acc_z)
{
  MotionData.acc_x = acc_x*T_11+acc_y*T_12+acc_z*T_13;
  MotionData.acc_y = acc_x*T_21+acc_y*T_22+acc_z*T_23;
  MotionData.acc_z = acc_x*T_31+acc_y*T_32+acc_z*T_33 - g;
}

void VelocitySolution(void)
{
  MotionData.velocity_x += MotionData.acc_x*dt;
  MotionData.velocity_y += MotionData.acc_y*dt;
  MotionData.velocity_z += MotionData.acc_z*dt;
}

void PositionSolution(void)
{
  MotionData.position_x += MotionData.velocity_x*dt;
  MotionData.position_y += MotionData.velocity_y*dt;
  MotionData.position_z += MotionData.velocity_z*dt;
}

void GPS_Solution(uint8_t *buffer)
{
  volatile uint8_t i,j = 0,k;
  volatile double lat,lon,height;//经纬度
  volatile double velocity,degree;
  volatile double point;
  if(buffer[0] == 0x24)
  {
    if((buffer[3] == 'R')&&(buffer[4] == 'M')&&(buffer[5] == 'C'))
    {
//      LED=!LED;
      for(i = 1;i<200;i++)
      {
        if(buffer[i] == ',') 
        {
          j++;
          if(j>14) break;
          if(j == 2) 
          {
            if(buffer[i+1]=='A') 
            {
              GPS_status = 1;
            }
            else if(buffer[i+1]=='V')
            {
              GPS_status = 2;
              break;
            }
          }
          else if(j == 3)//get lat（维度）
          {
            k=i+1;
            if(buffer[k]==',') continue;
            while(1)
            {
              lat = (buffer[k++]-0x30)*10;
              lat = lat + (buffer[k++]-0x30);
              lat = lat + (buffer[k++]-0x30)/6.0;
              lat = lat + (buffer[k++]-0x30)/60.0;
              k++;
              point = 0.1f;
              while(1)
              {
                lat=lat + (buffer[k++]-0x30)*point/60;
                point = 0.1f*point;
                if(buffer[k] == ',') break;
              }
              if(buffer[k+1] == 'S') lat = -lat;
              break;
            }
          }
          else if(j == 5)//get lon（经度）
          {
            k=i+1;
            if(buffer[k]==',') continue;
            while(1)
            {
              lon = (buffer[k++]-0x30)*100;
              lon = lon + (buffer[k++]-0x30)*10;
              lon = lon + (buffer[k++]-0x30);
              lon = lon + (buffer[k++]-0x30)/6.0;
              lon = lon + (buffer[k++]-0x30)/60.0;
              k++;
              point = 0.1f;
              while(1)
              {
                lon = lon + (buffer[k++]-0x30)*point/60;
                point = 0.1f*point;
                if(buffer[k] == ',') break;
              }
              if(buffer[k+1] == 'W') lon = -lon;
              break;
            }
          }
          else if(j == 7)//get velocity
          {
            if(buffer[i+1]==',') continue;
            for(k=i+1;k<200;k++) 
            {
              if(buffer[k] == '.') 
              {
                velocity = 0;
                for(i++;i<k;i++) velocity += (buffer[i]-0x30)*pow(10,k-i-1);
                point = 0.1f;
                i++;
                while(1)
                {
                  velocity += (buffer[i++]-0x30)*point;
                  if(buffer[i]==',') break;
                  point *= 0.1f;
                }
                break;
              }
            }
            if(buffer[i+1]==',') continue;
            for(k=i+1;k<200;k++) 
            {
              if(buffer[k] == '.') 
              {
                degree = 0;
                for(i++;i<k;i++) degree += (buffer[i]-0x30)*pow(10,k-i-1);
                point = 0.1f;
                i++;
                while(1)
                {
                  degree += (buffer[i++]-0x30)*point;
                  if(buffer[i]==',') break;
                  point *= 0.1f;
                }
                break;
              }
            }
            GPS_Data.velocity_course = degree;
            GPS_Data.velocity_n = velocity*0.514*cos(degree/57.3);
            GPS_Data.velocity_e = velocity*0.514*sin(degree/57.3);
            MotionData.velocity_x = GPS_Data.velocity_e;
            MotionData.velocity_y = GPS_Data.velocity_n;
          }
        }
      }
      if((GPS_init == 0)&&(GPS_status == 1)) 
      {
        GPS_init = 1;
        GPS_Data.lat_init = lat;
        GPS_Data.lon_init = lon;
      }
      else if(GPS_init&&(GPS_status == 1))
      {
        GPS_Data.lat = lat;
        GPS_Data.lon = lon;
        Coordinate2Position();
        GPS_init = 1;
      }
    }
    else if((buffer[3] == 'G')&&(buffer[4] == 'G')&&(buffer[5] == 'A')&&(GPS_status == 1))
    {
      for(i = 1;i<200;i++)
      {
        if(buffer[i] == ',') 
        {
          j++;
          if(j == 9)//get height
          {
            if(buffer[i+1]==',') continue;
            for(k=i+1;k<200;k++) 
            {
              if(buffer[k] == '.') 
              {
                height = 0;
                for(i++;i<k;i++) height += (buffer[i]-0x30)*pow(10,k-i-1);
                point = 0.1f;
                k++;
                while(1)
                {
                  height += (buffer[k++]-0x30)*point;
                  if(buffer[k]==',') break;
                  point *= 0.1f;
                }
                GPS_Data.height = height;
                MotionData.position_z = height;
                break;
              }
            }
          }
        }
      }
    }
  }
}


