#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

extern BaseType_t LEDTwink_Ret;
extern UBaseType_t LEDTwink_Prio;
extern TaskHandle_t *LEDTwink_TCB;

void LEDTwink(void);



extern BaseType_t SDWrite_Ret;
extern UBaseType_t SDWrite_Prio;
extern TaskHandle_t *SDWrite_TCB;

void SDwrite(void);

