#include "bridge.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define SPI_RX_BUFFER_LENGTH (256U)
#define SPI_RX_BUFFER_HALF_LENGTH (SPI_RX_BUFFER_LENGTH / 2U)
#define USART_RX_BUFFER_LENGTH (256U)
#define USART_RX_BUFFER_HALF_LENGTH (USART_RX_BUFFER_LENGTH / 2U)

#define TX_BYTE_BUFFER_NUMBER (16U)
#define TX_BYTE_BUFFER_LENGTH (16U)

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

TaskHandle_t SpiTxRxTaskHandle;
TaskHandle_t UsartTxRxTaskHandle;

// Очереди, в которых приходят уведомления о том, какая часть буффера приёмника заполнена.
QueueHandle_t spi_buffer_notification;
QueueHandle_t usart_buffer_notification;

// Очереди, в которых хранятся данные, которые надо передать дальше.
// Длина очереди TX_BYTE_BUFFER_NUMBER x TX_BYTE_BUFFER_LENGTH.
QueueHandle_t spi_to_usart_queue;
QueueHandle_t usart_to_spi_queue;

// Флаг, сигнализирующий о возможности передачи.
__IO uint8_t spi_tx_ready = 0U;
__IO uint8_t usart_tx_ready = 0U;

// Буфферы для приёма сырых байт.
uint8_t spi_rx_buffer[SPI_RX_BUFFER_LENGTH] = { 0U, };
uint8_t usart_rx_buffer[USART_RX_BUFFER_LENGTH] = { 0U, };

void SpiTxRxTask(void *argument);
void UsartTxRxTask(void *argument);

static void rxdBufferProcessing(const uint8_t *input, size_t buffer_length, QueueHandle_t *output);

void bridge_enable(void) {
	if (NULL == (spi_buffer_notification = xQueueCreate(1U, sizeof(uint8_t)))) {
		Error_Handler();
	}

	if (NULL == (usart_buffer_notification = xQueueCreate(1U, sizeof(uint8_t)))) {
		Error_Handler();
	}

	if (NULL == (spi_to_usart_queue = xQueueCreate(TX_BYTE_BUFFER_NUMBER, TX_BYTE_BUFFER_LENGTH))) {
		Error_Handler();
	}

	if (NULL == (usart_to_spi_queue = xQueueCreate(TX_BYTE_BUFFER_NUMBER, TX_BYTE_BUFFER_LENGTH))) {
		Error_Handler();
	}

	if (pdPASS != xTaskCreate(SpiTxRxTask, "SPI Tx and Rx task", 256UL,
							  NULL, 23, &SpiTxRxTaskHandle)) {
		Error_Handler();
	}

	if (pdPASS != xTaskCreate(UsartTxRxTask, "USART Tx and Rx task", 256UL,
							  NULL, 23, &UsartTxRxTaskHandle)) {
		Error_Handler();
	}
}

void SpiTxRxTask(void *argument) {
	(void)argument;
	uint8_t ready_buffer = 0U;
	uint8_t txbuffer[TX_BYTE_BUFFER_LENGTH];

	// Набор очередей, чтобы можно было разблокировать задачу по нескольким источникам.
	QueueSetHandle_t notification;
	QueueSetMemberHandle_t activated;

	if (NULL == (notification = xQueueCreateSet(TX_BYTE_BUFFER_NUMBER + 1U))) {
		Error_Handler();
	}

	// Добавление в набор очереди уведомлений о состоянии принимающего буффера.
	xQueueAddToSet(spi_buffer_notification, notification);
	// Добавление в набор очереди с данными полученными от USART.
	xQueueAddToSet(usart_to_spi_queue, notification);
	// DMA настроен как Circular, поэтому его можно запустить один раз и всё.
	if (HAL_OK != HAL_SPI_Receive_DMA(&hspi1, spi_rx_buffer, SPI_RX_BUFFER_LENGTH)) {
		Error_Handler();
	}

	for (;;) {
		// Ждем пока что-нибудь не произойдет.
		activated = xQueueSelectFromSet(notification, osWaitForever);
		// Если прилетело уведомление о состоянии буффера приёма, то надо обработать данные и отправить их в USART.
		if (activated == spi_buffer_notification) {
			xQueueReceive(spi_buffer_notification, (void *)&ready_buffer, 10UL);

			if (ready_buffer == 0U) {
				rxdBufferProcessing(spi_rx_buffer, SPI_RX_BUFFER_HALF_LENGTH, &spi_to_usart_queue);
			} else {
				rxdBufferProcessing((spi_rx_buffer + SPI_RX_BUFFER_HALF_LENGTH), SPI_RX_BUFFER_HALF_LENGTH, &spi_to_usart_queue);
			}
		// Если USART что-то прислал, то надо это  отправить.
		} else if (activated == usart_to_spi_queue) {
			if (0U == spi_tx_ready) {
				xQueueReceive(usart_to_spi_queue, txbuffer, 10UL);
				spi_tx_ready = 1U;
				HAL_SPI_Transmit_DMA(&hspi1, txbuffer, TX_BYTE_BUFFER_LENGTH);
			}
		}
	}
}

// Здесь всё также как и в SpiTxRxTask, но в другую сторону.
void UsartTxRxTask(void *argument) {
	(void)argument;

	uint8_t ready_buffer = 0U;
	uint8_t txbuffer[TX_BYTE_BUFFER_LENGTH];

	QueueSetHandle_t notification;
	QueueSetMemberHandle_t activated;

	if (NULL == (notification = xQueueCreateSet(TX_BYTE_BUFFER_NUMBER + 1U))) {
		Error_Handler();
	}

	xQueueAddToSet(usart_buffer_notification, notification);
	xQueueAddToSet(spi_to_usart_queue, notification);

	if (HAL_OK != HAL_UART_Receive_DMA(&huart1, usart_rx_buffer, USART_RX_BUFFER_LENGTH)) {
		Error_Handler();
	}

	for (;;) {
		activated = xQueueSelectFromSet(notification, osWaitForever);

		if (activated == spi_buffer_notification) {
			xQueueReceive(usart_buffer_notification, (void *)&ready_buffer, 10UL);

			if (ready_buffer == 0U) {
				rxdBufferProcessing(usart_rx_buffer, USART_RX_BUFFER_HALF_LENGTH, &usart_to_spi_queue);
			} else {
				rxdBufferProcessing((usart_rx_buffer + USART_RX_BUFFER_HALF_LENGTH), USART_RX_BUFFER_HALF_LENGTH, &usart_to_spi_queue);
			}
		} else if (activated == spi_to_usart_queue) {
			if (0U == usart_tx_ready) {
				xQueueReceive(spi_to_usart_queue, txbuffer, 10UL);
				usart_tx_ready = 1U;
				HAL_UART_Transmit_DMA(&huart1, txbuffer, TX_BYTE_BUFFER_LENGTH);
			}
		}
	}
}

// Коллбеки, вызываемые при приёме или передачи данных.

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (SPI1 == hspi->Instance) {
		spi_tx_ready = 0U;
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t notify = 1U;

	if (SPI1 == hspi->Instance) {
		xQueueSendFromISR(spi_buffer_notification, (void *)&notify, &xHigherPriorityTaskWoken);
	}

	if (xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t notify = 0U;

	if (SPI1 == hspi->Instance) {
		xQueueSendFromISR(spi_buffer_notification, (void *)&notify, &xHigherPriorityTaskWoken);
	}

	if (xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (USART1 == huart->Instance) {
		usart_tx_ready = 0U;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t notify = 1U;

	if (USART1 == huart->Instance) {
		xQueueSendFromISR(usart_buffer_notification, (void *)&notify, &xHigherPriorityTaskWoken);
	}

	if (xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t notify = 0U;

	if (USART1 == huart->Instance) {
		xQueueSendFromISR(usart_buffer_notification, (void *)&notify, &xHigherPriorityTaskWoken);
	}

	if (xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

// Обработка принятых данных.
static void rxdBufferProcessing(const uint8_t *input, size_t buffer_length, QueueHandle_t *output) {
	uint8_t temptx[TX_BYTE_BUFFER_LENGTH];
	size_t byte_counter = 0UL;

	for (size_t index = 0U; index < buffer_length; index++) {
		// Если текущий символ не равен 0, то передаем дальше.
		if ('\0' != *(char *)(input + index)) {
			temptx[byte_counter] = *(char *)(input + index);
			byte_counter++;
		// Если текущий байт равен 0, но предыдущий нет, то тоже добавляем, т.к. это последний байт в строке.
		} else if (('\0' == *(char *)(input + index)) && ('\0' != *(char *)(input + index - 1U)) && (index != 0U)) {
			temptx[byte_counter] = 0;
			byte_counter++;
		} else {
			// Ничего не добавляем.
		}

		// Добавление в очередь
		if (((byte_counter == TX_BYTE_BUFFER_LENGTH) || ((index == (buffer_length - 1U) && (byte_counter != 0)))) && (uxQueueSpacesAvailable(*output) > 0)) {
			xQueueSend(*output, (void *)temptx, 1UL);
		}
	}
}

