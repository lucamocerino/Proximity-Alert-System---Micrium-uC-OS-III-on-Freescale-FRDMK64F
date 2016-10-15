/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "fsl_interrupt_manager.h"
#include "fsl_clock_manager.h"
#include "fsl_gpio_common.h"
#include "fsl_ftm_common.h"
#include "fsl_ftm_driver.h"
#include "fsl_ftm_hal.h"
#include "fsl_ftm_features.h"
#include "stdint.h"    
#include "fsl_device_registers.h"
#include  <math.h>
#include  <lib_math.h>
#include  <cpu_core.h>
#include  <app_cfg.h>
#include  <os.h>
#include  <fsl_os_abstraction.h>
#include  <system_MK64F12.h>
#include  <board.h>
#include  <bsp_ser.h>

#define BUF_SIZE 10

#define FREQUENCY                 (0x00)
#define FTM0_COUNTER              (0x00)
#define FTM0_MODE_SET             (0xFFFF)


  
/*
*********************************************************************************************************
*********************************************************************************************************
*/

static  OS_TCB       AppTaskStartTCB;
static  CPU_STK      AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB       toggleTaskTCB;
static  CPU_STK      toggleTaskStk[APP_CFG_TASK_START_STK_SIZE];



static  OS_TCB       controlTaskTCB;
static  CPU_STK      controlTaskStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB       ledTaskTCB;
static  CPU_STK      ledTaskStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_SEM       print_sem;


static int ready=0;
static float distance=0;
static unsigned freqG=0;                //green led frequency
static unsigned freqB=0;                //blue led frequency
static unsigned freqR=0;                //red led frequency
static unsigned green=DEF_OFF;
static unsigned blue=DEF_OFF;
static unsigned red=DEF_OFF;
static float buffer[BUF_SIZE];           //buffer to achieve measures
static int head=0;                        //buffer index
static float average=0;
/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart (void  *p_arg);
static  void  toggleTask (void  *p_arg);

static void controlTask (void *p_arg);
static void ledTask (void *p_arg);

/*
*********************************************************************************************************
*                                                main()
*********************************************************************************************************
*/
void FTM0_IRQHandler(void)              //FTM handler
{
  CPU_CRITICAL_ENTER();
  OSIntEnter();
  OS_ERR        err;  

     uint16_t before = 0;
     uint16_t after = 0;
     
    if ( FTM_HAL_GetChnEventStatus(g_ftmBaseAddr[HW_FTM0], HW_CHAN1) ) { //if an event was detected
    
         if(FTM_HAL_HasChnEventOccurred(g_ftmBaseAddr[HW_FTM0], HW_CHAN0)) { //if an event was detected on channel0 (rising edge)
           
           before=FTM_HAL_GetChnCountVal(g_ftmBaseAddr[HW_FTM0], HW_CHAN0, before);
           
         }
         
         if(FTM_HAL_HasChnEventOccurred(g_ftmBaseAddr[HW_FTM0], HW_CHAN1)) {  //if an event was detected on channel1 (falling edge)
           
           after=FTM_HAL_GetChnCountVal(g_ftmBaseAddr[HW_FTM0], HW_CHAN1, after);
            ready=1;  
            OSSemPost(&print_sem, OS_OPT_POST_1+OS_OPT_POST_NO_SCHED, &err );  //unlock the control task when measured value is ready
         } else ready=0;
    
           distance=(float)(1.0*(((after-before)*64)/60)/58); //use prescaler to divide the clock (divided for 64),
                                                              // 60 MHz is BUS Clock, cause ftm use this kind of clock
                                                              // a time interval is (t_after-T_before)/frequency
                                                              // in this case bus frequency (60 MH) but i use the prescaler to divide the clock
                                                              // in our case divide to 64, so (60/64) => frequency*64/60 and then 58 to have the distance in cm
                          
     //After each measurement it need to clear and re-set some register in order to re start the measurement.
         
     FTM_HAL_SetCounter(g_ftmBaseAddr[HW_FTM0], 0x00);

     FTM_HAL_ClearChnEventStatus(g_ftmBaseAddr[HW_FTM0], HW_CHAN0);  //Channel0 Flag , clearing to enable another cycle
     FTM_HAL_ClearChnEventStatus(g_ftmBaseAddr[HW_FTM0], HW_CHAN1);  //Channel1 Flag
     FTM_HAL_SetDualChnDecapCmd(g_ftmBaseAddr[HW_FTM0], HW_CHAN0, true);  //Reset dual capture mode to re-enable a new cycle
    
  }
     CPU_CRITICAL_EXIT();

  OSIntExit();

}

void DualEdgeCaptureMode(void){
  
 ftm_user_config_t ftm0Info = 
 {  .tofFrequency= FREQUENCY,   //initialize fmt frequency (0)
    .isWriteProtection = false, //disable the writing protection of FTM's register
 };
 
 FTM_DRV_Init(HW_FTM0, &ftm0Info);   //initialize all register
 FTM_HAL_SetDualEdgeCaptureCmd(g_ftmBaseAddr[HW_FTM0], HW_CHAN0, true);  //Enable dual capture mode, write bit DECAPEN0 
 FTM_HAL_SetChnEdgeLevel(g_ftmBaseAddr[HW_FTM0], HW_CHAN0,1);           //Set the first channel in rising edge capture mode (ELS0B:ELS0A)
 FTM_HAL_SetChnMSnBAMode(g_ftmBaseAddr[HW_FTM0], HW_CHAN0,1);           // One-shot mode (MS0B:MS0A)

 FTM_HAL_SetChnEdgeLevel(g_ftmBaseAddr[HW_FTM0], HW_CHAN1,2);           //Set the second channel in rising edge capture mode (ELS0B:ELS0A)
 FTM_HAL_EnableChnInt(g_ftmBaseAddr[HW_FTM0], HW_CHAN1);                //Channel1 enable (CH1IE)

 
 FTM_HAL_SetMod(g_ftmBaseAddr[HW_FTM0], FTM0_MODE_SET);                     //Set bits in MODE register
 FTM_HAL_SetCounterInitVal(g_ftmBaseAddr[HW_FTM0], FTM0_COUNTER);           //Set the initial value of the counter
 FTM_HAL_SetCounter(g_ftmBaseAddr[HW_FTM0], 0);
 FTM_HAL_SetClockSource(g_ftmBaseAddr[HW_FTM0], kClock_source_FTM_SystemClk); //select the clock source

    
 FTM_HAL_SetDualChnDecapCmd(g_ftmBaseAddr[HW_FTM0], HW_CHAN0, true);            //Enable

}
/********************************************************************************
Sort the buffer's values and take out the middle element of the buffer

*********************************************************************************/

float bubble_sort(){
  
  int j,i;
  float tmp=0;
  
for (j = 0; j <BUF_SIZE; j++ ) 
{

for (i=BUF_SIZE-1; i>=j; i--) 
  { 
  if (buffer[i]>buffer[i+1]) 
    { 
    tmp = buffer[i]; 
    buffer[i] = buffer[i+1]; 
    buffer[i+1] = tmp; 
    } 
  }
}
 return buffer[BUF_SIZE/2];
 
}


//*********************************************************************

int  main (void)
{
    OS_ERR   err;

#if (CPU_CFG_NAME_EN == DEF_ENABLED)
    CPU_ERR  cpu_err;
#endif

    hardware_init();
    
   
    GPIO_DRV_Init(switchPins, ledPins);
   

#if (CPU_CFG_NAME_EN == DEF_ENABLED)
    CPU_NameSet((CPU_CHAR *)"MK64FN1M0VMD12",
                (CPU_ERR  *)&cpu_err);
#endif

    OSA_Init();                                                 /* Init uC/OS-III.                                      */
    
    //FlexTimer configuration
    configure_ftm_pins(HW_FTM0);
    DualEdgeCaptureMode();
    INT_SYS_InstallHandler(FTM0_IRQn, FTM0_IRQHandler);
    //************************************************

    
    
        
         OSSemCreate( &print_sem, "Print Semaphore", 0, &err );  //Unlock the controll task when average is ready
   
    
    OSTaskCreate(&AppTaskStartTCB,                              /* Create the start task                                */
                 "App Task Start",
                  AppTaskStart,
                  0u,
                  APP_CFG_TASK_START_PRIO,
                 &AppTaskStartStk[0u],
                 (APP_CFG_TASK_START_STK_SIZE / 10u),
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &err);

    OSA_Start();                                                

    while (DEF_ON) {                                           
        ;
    }
}


/*
*********************************************************************************************************
*********************************************************************************************************
*/

static  void  AppTaskStart (void *p_arg)
{
    OS_ERR      err;
    
    (void)p_arg;                                                /* See Note #1                                          */


    CPU_Init();                                                 /* Initialize the uC/CPU Services.                      */
    Mem_Init();                                                 /* Initialize the Memory Management Module              */
    Math_Init();                                                /* Initialize the Mathematical Module                   */


    BSP_Ser_Init(115200u);

    APP_TRACE_DBG(("Blinking RGB LED...\n\r"));
    
    OSTaskCreate(&toggleTaskTCB,                              /* Create the start task                                */
                 "Toggle Task",
                  toggleTask,
                  0u,
                  APP_CFG_TASK_START_PRIO,      
                 &toggleTaskStk[0u],
                 (APP_CFG_TASK_START_STK_SIZE / 10u),
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &err);
    
    OSTaskCreate(&controlTaskTCB,                             
                 "control Task Start",
                  controlTask,
                  0u,
                  APP_CFG_TASK_START_PRIO,
                 &controlTaskStk[0u],
                 (APP_CFG_TASK_START_STK_SIZE / 10u),
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &err);
    
    OSTaskCreate(&ledTaskTCB,                             
                 "led task",
                  ledTask,
                  0u,
                  APP_CFG_TASK_START_PRIO,
                 &ledTaskStk[0u],
                 (APP_CFG_TASK_START_STK_SIZE / 10u),
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                 &err);
  
    
    while (DEF_ON) {                                            
        ;
    }
}

static  void  toggleTask (void *p_arg)
{
    
   
    OS_ERR      err;
   
    (void)p_arg;
     

     
    while (DEF_ON) { 
	
        OSTimeDlyHMSM(0u, 0u, 0u, 60u, OS_OPT_TIME_HMSM_STRICT, &err);  //Between two measure it needs almos 60 ms
        
       
     //Toggle the pin in order to generate a square wave to start the measurement (almost 1us)
         GPIO_DRV_SetPinOutput(outPTB23);
         OSTimeDlyHMSM(0u, 0u, 0u, 1u, OS_OPT_TIME_HMSM_STRICT, &err);
         GPIO_DRV_ClearPinOutput(outPTB23);
      }
}


static void controlTask (void *p_arg)
{
  

  OS_ERR      err;
    CPU_TS      os_ts;
   
  (void)p_arg;
  CPU_Init();
  Mem_Init();
  Math_Init();
 


 
  while (DEF_ON) {
  
                  
        OSSemPend(&print_sem, 0,OS_OPT_PEND_BLOCKING,&os_ts, &err);//Lock this task untill the value measured is ready 
                  
       //If the value measured was ready
       if(ready){
         
         if(distance<500.0 && distance>0){ //if value are in sensor's range [2cm, 4 m], i put them into buffer
           
                
                buffer[head] = distance;
                head++;
                }
         }
                if (head == BUF_SIZE) {
               
			head=0;
		        average=bubble_sort();
                        
                      }
                  
      //According to range set the right led and frequecy
        
        if(average>=200.0){             //GREEN
         
             green=DEF_ON;
             blue=DEF_OFF;
             red=DEF_OFF;
             freqG=2000;
        }
             
             
        if (average>100.0 && average <200.0){   //BLUE
             green=DEF_OFF;
             blue=DEF_ON;
             red=DEF_OFF;
             
             if(average>=100.0 && average <120.0){
               freqB=200;
             } 
              if(average>=120.0 && average <140.0){
                 freqB=400;
             }
              if(average>=140.0 && average <160.0){
                 freqB=600;
             }
              if(average>=160.0 && average <180.0){
                 freqB=800;
             }
               if(average>=180.0 && average <200.0){
                 freqB=1000;
             }
      
       
        }
        
             
        
         if (average<100.0){            //RED
             
              green=DEF_OFF;
              blue=DEF_OFF;
              red=DEF_ON;
             
             
            
              if( average <10.0){
               freqR=0;
            
             
              }
             
              if(average>=10.0 && average <25.0){
                 freqR=400;
                
             }
              else if(average>=25.0 && average <50.0){
                 freqR=600;
               
             }
              else if(average>=50.0 && average <75.0){
                 freqR=800;
                 
             }
               else if(average>=75.0 && average <100.0){
                 freqR=1000;
                 
               }
             
             
          }
        
         
     
  }
}

static void ledTask (void *p_arg)
{
  OS_ERR err;

 
  (void)p_arg;
  CPU_Init();
  Mem_Init();
  Math_Init();



 
  while (DEF_ON) {
  
      
        
        if(blue){
              
           
                GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_RED);              //Turn off the other leds, to be sure 
                GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_GREEN);            //Turn off the other leds, to be sure 
      
               GPIO_DRV_ClearPinOutput(BOARD_GPIO_LED_BLUE);
               OSTimeDlyHMSM(0u, 0u, 0u, freqB/2, OS_OPT_TIME_HMSM_STRICT, &err);
               GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_BLUE);
               OSTimeDlyHMSM(0u, 0u, 0u, freqB/2, OS_OPT_TIME_HMSM_STRICT, &err);
               
            
        
        }
        
          
        
        
       
               
                
           if(red){
             
                GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_BLUE);             //Turn off the other leds, to be sure 
                GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_GREEN);            //Turn off the other leds, to be sure 
                
             if(!freqR){
            
            
            blue=DEF_OFF;
            green=DEF_OFF;
            
               
                
            GPIO_DRV_ClearPinOutput(BOARD_GPIO_LED_RED);
         
              
            
              
              }
              
             else{
               
                 
               GPIO_DRV_ClearPinOutput(BOARD_GPIO_LED_RED);
               OSTimeDlyHMSM(0u, 0u, 0u, freqR/2, OS_OPT_TIME_HMSM_STRICT, &err);
               GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_RED);
               OSTimeDlyHMSM(0u, 0u, 0u, freqR/2, OS_OPT_TIME_HMSM_STRICT, &err);
           
              
             }
            }
    
            
        
        if(green){
          
          
          
             
         
                GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_BLUE);              //Turn off the other leds, to be sure 
                GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_RED);              //Turn off the other leds, to be sure 
                
               GPIO_DRV_ClearPinOutput(BOARD_GPIO_LED_GREEN);
               OSTimeDlyHMSM(0u, 0u, 0u, freqG/2, OS_OPT_TIME_HMSM_STRICT, &err);
               GPIO_DRV_SetPinOutput(BOARD_GPIO_LED_GREEN);
               OSTimeDlyHMSM(0u, 0u, 0u, freqG/2, OS_OPT_TIME_HMSM_STRICT, &err);
               
              
            }
    
  }
}
