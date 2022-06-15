#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

typedef struct{
  String msg;
  uint16_t num;
  uint16_t delay;
}Data_t;


/*Global variables ------------------------------------------*/
SemaphoreHandle_t xMutex;
QueueHandle_t xQueue;
/*Task -------------------------------------------------------*/
void task1(void *params){
  Data_t xData;
  BaseType_t i = 0;

  while (1)
  {
    /* code */
    //geracion de datos
    xData.msg = "Mendaje de tarea 1";
    xData.num = i;
    xData.delay = 100 * random(10);
    i++;
    //envio de datos
    if(xQueueSend(xQueue, (void*)&xData, pdMS_TO_TICKS(2000)) == pdPASS){
      if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdPASS){
        //todo
        Serial.println("Mensaje enviado");
        //liberar
        xSemaphoreGive(xMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
}

void task2(void *params){
  Data_t xData;
  while (1)
  {
    /* code */
    if(xQueueReceive(xQueue, (void*)&xData, pdMS_TO_TICKS(1000)) == pdPASS){
      if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdPASS){
        //todo
        Serial.print("mensaje recibido: ");
        Serial.println(xData.msg);
        Serial.printf("delay: %d\r\n", xData.delay);
        Serial.printf("numero de mensaje: %d\r\n",xData.num);

        //liberar
        xSemaphoreGive(xMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(xData.delay));
  }
  
}
void setup() {
  BaseType_t status;
  Serial.begin(115200);
  Serial.println("configurando....");
  // put your setup code here, to run once:
  /*status = xTaskCreate(task1, "tarea 1", 5000, NULL, 2, NULL);

  configASSERT(status == pdPASS);

  status = xTaskCreate(task2, "tarea 2", 5000, NULL, 2, NULL);

  configASSERT(status == pdPASS);*/
  /*Creacion del mutex*/
  /*MUTEX*/
  xMutex = xSemaphoreCreateMutex();
  if(xMutex == NULL){
    Serial.println("el mutex no se pudo crear");
  }
  /*QUEUE*/
  xQueue = xQueueCreate(5, sizeof(Data_t));
  if(xQueue == NULL){
    Serial.println("el queue no se pudo crear");
  }
  /*creacion de tareas*/
  status = xTaskCreatePinnedToCore(task1, "tarea 1", 5000, NULL, 1, NULL, 1);
  configASSERT(status == pdPASS);
  status = xTaskCreatePinnedToCore(task2, "tarea 1", 5000, NULL, 1, NULL, 0);
  configASSERT(status == pdPASS);

}

void loop() {
  // put your main code here, to run repeatedly:

}