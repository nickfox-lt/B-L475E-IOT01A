#include "stm32l4xx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
void DefaultTask(void *argument)
{
  for (;;)
  {
    vTaskDelay(1000);
  }
}

int main(void)
{
  board_init();

  xTaskCreate(DefaultTask,              /* Function that implements the task. */
              "NAME",                   /* Text name for the task. */
              configMINIMAL_STACK_SIZE, /* Stack size in words, not bytes. */
              (void *)1,                /* Parameter passed into the task. */
              tskIDLE_PRIORITY,         /* Priority at which the task is created. */
              NULL);                    /* Used to pass out the created task's handle. */

  vTaskStartScheduler();

  while (1)
  {
  }
}
