#include "debug.h"
#include "UAVCAN4PX4.h"
#include "ch32v20x_tim.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_rcc.h"
#include "ch32v20x_misc.h"
#include "ch32v20x_dbgmcu.h"

/* Global typedef */
typedef struct {
    uint32_t last_heartbeat;
    uint32_t last_esc_update;
    uint32_t status_led_toggle;
} UAVCAN_Controller_t;

/* Global define */
#define HEARTBEAT_INTERVAL_MS    1000
#define ESC_UPDATE_INTERVAL_MS   20
#define STATUS_LED_INTERVAL_MS   500
#define LED_GPIO_PORT           GPIOA
#define LED_PIN                 GPIO_Pin_4

/* Global Variable */
static UAVCAN_Controller_t uavcan_ctrl;
static UAVCAN_NodeInfo_t node_info = {
    .node_id = 42,
    .mode = UAVCAN_NODE_MODE_OPERATIONAL,
    .health = UAVCAN_NODE_HEALTH_OK,
    .uptime_sec = 0,
    .name = "CH32V20x UAVCAN Node",
    .software_version = {1, 0, 0, 0},
    .hardware_version = {1, 0, 0, 0},
    .unique_id = 0x123456789ABCDEF0
};

/* Function prototypes */
void LED_Init(void);
void Timer_Init(void);
uint32_t Get_Microseconds(void);

/*********************************************************************
 * @fn      LED_Init
 *
 * @brief   Initialize LED on PA4
 *********************************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(LED_GPIO_PORT, LED_PIN);
}

/*********************************************************************
 * @fn      Timer_Init
 *
 * @brief   Initialize timer for microsecond timestamp
 *********************************************************************/
void Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM2, ENABLE);
}

/*********************************************************************
 * @fn      Get_Microseconds
 *
 * @brief   Get current timestamp in microseconds
 *********************************************************************/
uint32_t Get_Microseconds(void)
{
    return TIM_GetCounter(TIM2);
}

/*********************************************************************
 * @fn      UAVCAN_MessageHandler
 *
 * @brief   Handler for incoming UAVCAN messages
 *********************************************************************/
void UAVCAN_MessageHandler(const UAVCAN_Transfer_t* transfer)
{
    switch(transfer->data_type_id)
    {
        case UAVCAN_MSG_TYPE_GET_NODE_INFO:
            printf("[UAVCAN] Node info request received\r\n");
            UAVCAN_SendNodeInfo(&node_info);
            break;

        case UAVCAN_MSG_TYPE_RESTART_NODE:
            printf("[UAVCAN] Restart command received\r\n");
            NVIC_SystemReset();
            break;

        default:
            printf("[UAVCAN] Received message type: %d\r\n", transfer->data_type_id);
            break;
    }
}

/*********************************************************************
 * @fn      UAVCAN_ErrorHandler
 *
 * @brief   Handler for UAVCAN errors
 *********************************************************************/
void UAVCAN_ErrorHandler(uint32_t error_code)
{
    printf("[UAVCAN] Error: %lu\r\n", error_code);
    for(int i = 0; i < 5; i++) {
        GPIO_SetBits(LED_GPIO_PORT, LED_PIN);
        Delay_Ms(100);
        GPIO_ResetBits(LED_GPIO_PORT, LED_PIN);
        Delay_Ms(100);
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program entry
 *********************************************************************/
int main(void)
{
    // System initialization
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);

    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    Timer_Init();
    LED_Init();

    printf("UAVCAN Controller Initializing...\r\n");

    // UAVCAN initialization
    UAVCAN_Init(node_info.node_id, 1000000);
    UAVCAN_RegisterMessageHandler(UAVCAN_MessageHandler);
    UAVCAN_RegisterErrorHandler(UAVCAN_ErrorHandler);

    printf("UAVCAN Node ID: %d\r\n", node_info.node_id);
    printf("Controller Ready! LED on PA4 will blink at %d ms interval\r\n", STATUS_LED_INTERVAL_MS);

    // Initialize timing variables
    uavcan_ctrl.last_heartbeat = Get_Microseconds() / 1000;
    uavcan_ctrl.last_esc_update = Get_Microseconds() / 1000;
    uavcan_ctrl.status_led_toggle = Get_Microseconds() / 1000;

    // Main loop
    while(1)
    {
        uint32_t current_time = Get_Microseconds() / 1000;

        // Process incoming UAVCAN messages
        UAVCAN_Process();

        // Send periodic heartbeat
        if((current_time - uavcan_ctrl.last_heartbeat) >= HEARTBEAT_INTERVAL_MS)
        {
            UAVCAN_SendHeartbeat(node_info.mode, node_info.health);
            uavcan_ctrl.last_heartbeat = current_time;
            node_info.uptime_sec = current_time / 1000;
        }

        // Update ESC commands
        if((current_time - uavcan_ctrl.last_esc_update) >= ESC_UPDATE_INTERVAL_MS)
        {
            UAVCAN_ESC_Command_t esc_cmd = {
                .command = {1000, 1000, 1000, 1000, 0, 0, 0, 0},
                .count = 4
            };
            UAVCAN_SendESCCommand(&esc_cmd);
            uavcan_ctrl.last_esc_update = current_time;
        }

        // Status LED blink on PA4
        if((current_time - uavcan_ctrl.status_led_toggle) >= STATUS_LED_INTERVAL_MS)
        {
            if(GPIO_ReadOutputDataBit(LED_GPIO_PORT, LED_PIN) == Bit_SET) {
                GPIO_ResetBits(LED_GPIO_PORT, LED_PIN);
            } else {
                GPIO_SetBits(LED_GPIO_PORT, LED_PIN);
            }
            uavcan_ctrl.status_led_toggle = current_time;
        }

        Delay_Ms(1);
    }
}
