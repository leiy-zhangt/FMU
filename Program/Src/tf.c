#include "tf.h"
#include "stdio.h"
#include "gnss.h"
#include "teleport.h"

uint8_t FileName[50];
FRESULT SDRet;

void FileCreate(void)
{
	sprintf((char *)FileName,"%u-%u-%u %d-%d-%d.txt",GNSS_msg.utc.year,GNSS_msg.utc.month,GNSS_msg.utc.date,GNSS_msg.utc.hour,GNSS_msg.utc.min,GNSS_msg.utc.sec);
	SDRet = f_open(&SDFile,(char *)FileName,FA_WRITE|FA_CREATE_ALWAYS);
	if(SDRet == FR_OK) DebugPrint(DebugChannel,"SD open successfully!\r\n");
	else DebugPrint(DebugChannel,"SD open failed!\r\n");
}
