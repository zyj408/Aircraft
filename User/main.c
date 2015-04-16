#include  <includes.h>


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
           
static  OS_TCB   AppCommTaskTCB;
static  CPU_STK  AppCommTaskStk[APP_COMM_TASK_STK_SIZE];

static  OS_TCB   AppTaskStartTCB;
static  CPU_STK  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB   AppSampleTaskTCB;
static  CPU_STK  AppSampleTaskStk[APP_SAMPLE_TASK_STK_SIZE];

static  OS_TCB   AppOutputTaskTCB;
static  CPU_STK  AppOutputTaskStk[APP_OUTPUT_TASK_STK_SIZE];
/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static void AppTaskCreate(void);
static void AppTaskStart(void *p_arg);


extern void AppSampleTask(void *p_arg);
extern void AppOutPutTask(void *p_arg);
extern void AppCommTask(void *p_arg);
/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: ��׼c������ڡ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int main(void)
{
    OS_ERR  err;
    OSInit(&err);                                               /* Init uC/OS-III.                                      */


	OSTaskCreate((OS_TCB       *)&AppTaskStartTCB,              /* Create the start task                                */
                 (CPU_CHAR     *)"App Task Start",
                 (OS_TASK_PTR   )AppTaskStart, 
                 (void         *)0,
                 (OS_PRIO       )APP_CFG_TASK_START_PRIO,
                 (CPU_STK      *)&AppTaskStartStk[0],
                 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
                 (OS_MSG_QTY    )0,
                 (OS_TICK       )0,
                 (void         *)0,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
    
    (void)&err;
    
    return (0);
}

/*
*********************************************************************************************************
*	�� �� ��: AppTaskStart
*	����˵��: ����һ�����������ڶ�����ϵͳ�����󣬱����ʼ���δ������(��BSP_Init��ʵ��)
*	��    �Σ�p_arg ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
	�� �� ����2
*********************************************************************************************************
*/
static  void  AppTaskStart (void *p_arg)
{
	 OS_ERR err;
   (void)p_arg;

 	bsp_Init();
	CPU_Init();
	BSP_Tick_Init();                      

#if OS_CFG_STAT_TASK_EN > 0u
     OSStatTaskCPUUsageInit(&err);   
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif
                                        
    AppTaskCreate();                                           
  while (1)
	{   
   
		bsp_LedToggle(1);
		
		BSP_OS_TimeDlyMs(500);     
  }
}

/*
*********************************************************************************************************
*	�� �� ��: AppTaskCreate
*	����˵��: ����Ӧ������
*	��    �Σ�p_arg ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static  void  AppTaskCreate (void)
{
	OS_ERR      err;
	
	/***********************************/
	OSTaskCreate((OS_TCB       *)&AppSampleTaskTCB,             
                 (CPU_CHAR     *)"App Task Sample",
                 (OS_TASK_PTR   )AppSampleTask, 
                 (void         *)0,
                 (OS_PRIO       )APP_CFG_TASK_SAMPLE_PRIO,
                 (CPU_STK      *)&AppSampleTaskStk[0],
                 (CPU_STK_SIZE  )APP_SAMPLE_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE  )APP_SAMPLE_TASK_STK_SIZE,
                 (OS_MSG_QTY    )1,
                 (OS_TICK       )0,
                 (void         *)0,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);
								 
	/***********************************/
	OSTaskCreate((OS_TCB       *)&AppOutputTaskTCB,             
                 (CPU_CHAR     *)"App Task Output",
                 (OS_TASK_PTR   )AppOutPutTask, 
                 (void         *)0,
                 (OS_PRIO       )APP_CFG_TASK_OUTPUT_PRIO,
                 (CPU_STK      *)&AppOutputTaskStk[0],
                 (CPU_STK_SIZE  )APP_OUTPUT_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE  )APP_OUTPUT_TASK_STK_SIZE,
                 (OS_MSG_QTY    )1,
                 (OS_TICK       )0,
                 (void         *)0,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);
/***********************************/
	OSTaskCreate((OS_TCB       *)&AppCommTaskTCB,             
                 (CPU_CHAR     *)"App Task Comm",
                 (OS_TASK_PTR   )AppCommTask, 
                 (void         *)0,
                 (OS_PRIO       )APP_CFG_TASK_COMM_PRIO,
                 (CPU_STK      *)&AppCommTaskStk[0],
                 (CPU_STK_SIZE  )APP_COMM_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE  )APP_COMM_TASK_STK_SIZE,
                 (OS_MSG_QTY    )1,
                 (OS_TICK       )0,
                 (void         *)0,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);
}
