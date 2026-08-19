/* USER CODE BEGIN Header */
/**
  ****************************************************************************
  * @file           : freertos.c
  * @brief          : Taches FreeRTOS + application micro-ROS ping-pong
  *
  * Toute la logique micro-ROS est ici. main.c reste purement CubeMX.
  *
  * REGENERATION .ioc : ce fichier est "regeneration-safe" tant que tout code
  * ajoute vit exclusivement entre des paires "USER CODE BEGIN xxx" /
  * "USER CODE END xxx". ATTENTION : la version du template FreeRTOS utilisee
  * ici (verifiee apres regeneration le 19/08/2026) NE POSSEDE PAS de tag
  * "ThreadAttributes" ni de tag "1". Les tags reellement disponibles et
  * utilises sont : Includes, PTD, PD, PM, Variables, FunctionPrototypes,
  * 4, 5, RTOS_MUTEX, RTOS_SEMAPHORES, RTOS_TIMERS, RTOS_QUEUES,
  * RTOS_THREADS, RTOS_EVENTS, Header_StartDefaultTask, StartDefaultTask,
  * Application. En consequence :
  *   - la definition statique de la tache micro-ROS (handle/buffer/
  *     control block/attributes) vit dans "Variables" (pas "ThreadAttributes")
  *   - fatal_blink(), RCCHECK/RCSOFTCHECK et ping_callback() vivent en tete
  *     de "Application" (pas de tag "1" disponible)
  * Si une future regeneration fait REapparaitre ces tags (changement de
  * version CubeMX/CubeIDE), il est possible (mais pas obligatoire) de les
  * y deplacer ; ce n'est pas necessaire pour que le code fonctionne.
  *
  * Dependances deja presentes dans Core/Src (extra_sources), NE PAS
  * redefinir les symboles suivants :
  *   dma_transport.c        -> cubemx_transport_open/close/write/read
  *   microros_allocators.c  -> microros_allocate/deallocate/reallocate/
  *                             zero_allocate
  *   microros_time.c        -> clock_gettime
  *
  * Prerequis .ioc (a re-verifier apres toute regeneration) :
  *   - Timebase Source = TIM1 (pas SysTick)
  *   - USART2 DMA RX = DMA1 Stream5, mode CIRCULAR
  *   - USART2 DMA TX = DMA1 Stream6, mode NORMAL
  *   - USART2 global interrupt = Enabled
  *   - FREERTOS : TOTAL_HEAP_SIZE = 25000, CMSIS_V2
  *   - Project Manager -> Code Generator -> "Generate peripheral
  *     initialization as a pair of '.c/.h' files" = COCHE (sinon
  *     MX_FREERTOS_Init() disparait et tout est reinjecte dans main.c)
  ****************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/allocator.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include <uxr/client/transport.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Codes de clignotement de LD2 (PA5) : diagnostic sans debugger ---- */
#define BLINK_RCL_ERROR       100u   /* rapide      : appel rcl/rclc en echec */
#define BLINK_MALLOC_FAILED   500u   /* lent        : heap FreeRTOS epuise    */
#define BLINK_STACK_OVERFLOW   50u   /* tres rapide : stack overflow          */

/* ---- Identite du noeud et des topics ping/pong ---- */
#define PINGPONG_NODE_NAME     "stm32_pingpong_node"
#define TOPIC_PING              "ping"
#define TOPIC_PONG              "pong"

/* ---- Cadence de la boucle micro-ROS ---- */
#define MICROROS_SPIN_TIMEOUT_MS   100u
#define MICROROS_LOOP_DELAY_MS      10u
#define MICROROS_AGENT_RETRY_MS    200u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern UART_HandleTypeDef huart2;

/* ---- Transport serie fourni par dma_transport.c ---- */
extern bool   cubemx_transport_open(struct uxrCustomTransport * transport);
extern bool   cubemx_transport_close(struct uxrCustomTransport * transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport * transport,
                                     const uint8_t * buf, size_t len, uint8_t * err);
extern size_t cubemx_transport_read(struct uxrCustomTransport * transport,
                                    uint8_t * buf, size_t len, int timeout, uint8_t * err);

/* ---- Allocateurs fournis par microros_allocators.c ---- */
extern void * microros_allocate(size_t size, void * state);
extern void   microros_deallocate(void * pointer, void * state);
extern void * microros_reallocate(void * pointer, size_t size, void * state);
extern void * microros_zero_allocate(size_t number_of_elements,
                                     size_t size_of_element, void * state);

/* ---- Entites micro-ROS : topic ping/pong ---- */
static rcl_publisher_t      pong_publisher;
static rcl_subscription_t   ping_subscriber;
static std_msgs__msg__Int32 pong_msg;
static std_msgs__msg__Int32 ping_msg;

/* ---- Tache micro-ROS : pas de tag "ThreadAttributes" dans ce template,
 * donc definie ici, dans "Variables". Stack STATIQUE de 16 Ko (4096 mots),
 * en .bss et non depuis le heap FreeRTOS, pour laisser tout le heap aux
 * allocations internes de micro-ROS.                                        */
osThreadId_t microrosTaskHandle;
uint32_t microrosTaskBuffer[ 4096 ];
osStaticThreadDef_t microrosTaskControlBlock;
const osThreadAttr_t microrosTask_attributes = {
  .name = "microrosTask",
  .cb_mem = &microrosTaskControlBlock,
  .cb_size = sizeof(microrosTaskControlBlock),
  .stack_mem = &microrosTaskBuffer[0],
  .stack_size = sizeof(microrosTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void MicroROSTask(void *argument);

static void fatal_blink(uint32_t period_ms);
static void ping_callback(const void * msgin);
static void microros_wait_for_agent(void);
static void microros_entities_init(rclc_support_t *support,
                                    rcl_node_t *node,
                                    rclc_executor_t *executor);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationMallocFailedHook(void)
{
    fatal_blink(BLINK_MALLOC_FAILED);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    fatal_blink(BLINK_STACK_OVERFLOW);
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  microrosTaskHandle = osThreadNew(MicroROSTask, NULL, &microrosTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ==========================================================================
 *  Signalisation d'erreur + macros de controle rcl/rclc + callback ping.
 *  Places ici (pas de tag "1" dans ce template) mais definis AVANT leur
 *  utilisation par microros_entities_init()/MicroROSTask() plus bas dans
 *  ce meme bloc "Application".
 * ========================================================================== */

/* LD2 clignote a une frequence identifiant la panne. Interruptions coupees,
 * donc pas de HAL_Delay() (uwTick serait fige) : boucle d'attente calibree
 * pour 84 MHz.                                                              */
static void fatal_blink(uint32_t period_ms)
{
    __disable_irq();
    for (;;)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        for (volatile uint32_t i = 0; i < period_ms * 8400UL; i++)
        {
            __NOP();
        }
    }
}

#define RCCHECK(fn)     { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) { fatal_blink(BLINK_RCL_ERROR); } }
#define RCSOFTCHECK(fn) { rcl_ret_t _rc = (fn); (void)_rc; }

/* ---- Callback de souscription : ping -> pong (valeur + 1) ---- */
static void ping_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * incoming = (const std_msgs__msg__Int32 *)msgin;
    pong_msg.data = incoming->data + 1;
    RCSOFTCHECK(rcl_publish(&pong_publisher, &pong_msg, NULL));
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);   /* preuve visuelle de trafic */
}

/* ==========================================================================
 *  Helpers d'initialisation micro-ROS (appeles une seule fois par
 *  MicroROSTask, dans l'ordre). Separes pour lisibilite uniquement : le
 *  comportement est strictement identique a l'implementation d'origine.
 * ========================================================================== */

/* Bloque (osDelay) jusqu'a ce que l'agent micro-ROS reponde sur le lien
 * serie. Necessaire avant tout appel rcl/rclc.                              */
static void microros_wait_for_agent(void)
{
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
    {
        osDelay(MICROROS_AGENT_RETRY_MS);
    }
}

/* Cree le support/contexte, le noeud, le publisher /pong, le subscriber
 * /ping, puis l'executor qui les lie. Bloque via fatal_blink() si une
 * etape rcl/rclc echoue (voir RCCHECK).                                     */
static void microros_entities_init(rclc_support_t *support,
                                    rcl_node_t *node,
                                    rclc_executor_t *executor)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(support, 0, NULL, &allocator));

    RCCHECK(rclc_node_init_default(node, PINGPONG_NODE_NAME, "", support));

    RCCHECK(rclc_publisher_init_default(
        &pong_publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        TOPIC_PONG));

    RCCHECK(rclc_subscription_init_default(
        &ping_subscriber, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        TOPIC_PING));

    RCCHECK(rclc_executor_init(executor, &support->context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        executor, &ping_subscriber, &ping_msg, &ping_callback, ON_NEW_DATA));
}

/* ==========================================================================
 *  Tache micro-ROS : node stm32_pingpong_node
 *    subscribe  /ping  (std_msgs/Int32)
 *    publish    /pong  (std_msgs/Int32)  = ping + 1
 * ========================================================================== */
void MicroROSTask(void *argument)
{
    (void)argument;

    rclc_support_t  support;
    rcl_node_t      node;
    rclc_executor_t executor;

    /* 1. Transport serie DMA. Le 2e argument DOIT etre &huart2 :
     *    dma_transport.c lit transport->args pour retrouver l'UART.        */
    rmw_uros_set_custom_transport(
        true,                     /* framing = true en serie */
        (void *) &huart2,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    /* 2. Allocateurs FreeRTOS, AVANT tout appel rcl/rclc.
     *    Sans ca micro-ROS utilise malloc() -> heap newlib (512 octets) et
     *    la creation du subscriber echoue silencieusement.                 */
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate      = microros_allocate;
    freeRTOS_allocator.deallocate    = microros_deallocate;
    freeRTOS_allocator.reallocate    = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator))
    {
        fatal_blink(BLINK_RCL_ERROR);
    }

    /* 3. Attendre l'agent avant d'initialiser quoi que ce soit */
    microros_wait_for_agent();

    /* 4-7. Support / noeud / publisher "pong" / subscriber "ping" / executor */
    microros_entities_init(&support, &node, &executor);

    pong_msg.data = 0;

    /* 8-9. Boucle d'execution : spin non bloquant + cadence de tache */
    for (;;)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(MICROROS_SPIN_TIMEOUT_MS));
        osDelay(MICROROS_LOOP_DELAY_MS);
    }
}

/* USER CODE END Application */

