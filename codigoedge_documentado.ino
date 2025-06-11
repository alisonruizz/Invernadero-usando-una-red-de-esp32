/**
 * @file codigoedge_documentado.ino
 * @brief Sistema de monitoreo ambiental con ESP32 usando ESP-NOW, Telegram y SD
 * @details Este código implementa un nodo sensor que recopila datos de temperatura, humedad y luz,
 *          los envía via ESP-NOW a un nodo RTC y gestiona ciclos de sueño para ahorro de energía.
 * @author Alison Daniela Ruiz, Johana Puerres Pizarro, Juan David Gutierrez
 * @date 11/06/24
 */

/** @defgroup includes Includes */
/** @{ */
#include <esp_now.h>                ///< Para comunicación ESP-NOW
#include <WiFi.h>                  ///< Conexión WiFi
#include <HTTPClient.h>             ///< Cliente HTTP
#include <WiFiClientSecure.h>       ///< Cliente WiFi seguro
#include <UniversalTelegramBot.h>   ///< API Telegram Bot
#include "esp_wifi.h"               ///< Funciones WiFi ESP32
#include <Wire.h>                   ///< Comunicación I2C
#include "RTClib.h"                ///< Manejo de RTC
#include "freertos/FreeRTOS.h"      ///< Sistema operativo FreeRTOS
#include "freertos/task.h"          ///< Funcionalidad de tareas
#include "freertos/queue.h"         ///< Colas RTOS
#include "freertos/semphr.h"        ///< Semaforos RTOS
#include "freertos/timers.h"        ///< Timers RTOS
#include "freertos/event_groups.h"  ///< Grupos de eventos RTOS
#include "FS.h"                     ///< Sistema de archivos
#include "SD.h"                     ///< Manejo de tarjeta SD
#include "SPI.h"                   ///< Comunicación SPI
/** @} */

/** @defgroup defines Definiciones */
/** @{ */
#define BOTtoken "7982045470:AAECMR_d-6v_EpKlqk137a4XbWbZ-yT1SV0"              ///< Token del bot de Telegram
#define CHAT_ID "1856694079"               ///< ID del chat de Telegram

#define SD_CS_PIN 5                 ///< Pin ChipSelect SD
#define SD_MOSI_PIN 23              ///< Pin MOSI SD
#define SD_MISO_PIN 19              ///< Pin MISO SD
#define SD_SCK_PIN 18               ///< Pin SCK SD

// Umbrales para alertas
#define UMBRAL_LUZ 50.0             ///< Umbral de luz para alertas
#define UMBRAL_TEMP_DHT 23.0        ///< Umbral temperatura DHT
#define UMBRAL_HUM_DHT 90.0         ///< Umbral humedad DHT
#define UMBRAL_DS18B20 30.0         ///< Umbral temperatura DS18B20

// Prioridades de tareas
#define PRIORITY_ESP_NOW_TASK 3     ///< Prioridad tarea ESP-NOW
#define PRIORITY_RTC_TASK 2         ///< Prioridad tarea RTC
#define PRIORITY_TELEGRAM_TASK 1    ///< Prioridad tarea Telegram
#define PRIORITY_SLEEP_MANAGER_TASK 4 ///< Prioridad tarea Sleep Manager
#define PRIORITY_SD_TASK 2          ///< Prioridad tarea SD

// Definiciones de ticks
#define TIEMPO_DESPIERTO_TICKS pdMS_TO_TICKS(60000)  ///< Ticks que estara despierto
#define TIEMPO_DORMIDO_TICKS pdMS_TO_TICKS(60000)    ///< Ticks que estara dormido
#define ENVIO_HORA_INTERVAL_TICKS pdMS_TO_TICKS(10000) ///< Ticks que estara dormido

// Tamaños de stack
#define STACK_SIZE_ESP_NOW 6144     ///< Stack tarea ESP-NOW
#define STACK_SIZE_RTC 4096         ///< Stack tarea RTC
#define STACK_SIZE_TELEGRAM 10240   ///< Stack tarea Telegram
#define STACK_SIZE_SLEEP_MANAGER 3072 ///< Stack tarea Sleep Manager
#define STACK_SIZE_SD 4096          ///< Stack tarea SD

// Bits de eventos
#define BIT_DATOS_RECIBIDOS (1 << 0) ///< Bit datos recibidos
#define BIT_ENVIAR_TELEGRAM (1 << 1) ///< Bit enviar Telegram
#define BIT_DORMIR_SISTEMA (1 << 2)  ///< Bit dormir sistema
#define BIT_GUARDAR_SD (1 << 3)      ///< Bit guardar en SD
/** @} */

/** @defgroup estructuras Estructuras de datos */
/** @{ */
/**
 * @brief Estructura para datos de sensores
 */
typedef struct {
  float temp_dht;     ///< Temperatura DHT
  float hum_dht;      ///< Humedad DHT
  float temp_ds18b20; ///< Temperatura DS18B20
  float luz;          ///< Nivel de luz
} struct_message;

/**
 * @brief Estructura para datos RTC
 */
typedef struct {
  char hora[9];       ///< Hora en formato HH:MM:SS
} struct_rtc;

/**
 * @brief Estructura para sincronización
 */
typedef struct {
  uint8_t comando;    ///< Comando de sincronización
  uint32_t timestamp; ///< Marca de tiempo
} struct_sync;

/**
 * @brief Estructura para actuadores
 */
typedef struct {
  char comando[20];   ///< Comando para actuador
  float temperatura;  ///< Temperatura objetivo
} struct_actuador;

/**
 * @brief Elemento de cola para datos de sensores
 */
typedef struct {
  struct_message datos;  ///< Datos de sensores
  TickType_t timestamp;  ///< Marca de tiempo RTOS
} sensor_data_queue_item_t;

/**
 * @brief Elemento de cola para Telegram
 */
typedef struct {
  String mensaje;       ///< Mensaje a enviar
} telegram_queue_item_t;

/**
 * @brief Elemento de cola para SD
 */
typedef struct {
  struct_message datos;      ///< Datos de sensores
  DateTime timestamp_rtc;   ///< Marca de tiempo RTC
} sd_data_queue_item_t;
/** @} */

/** @defgroup variables Variables globales */
/** @{ */

const char* ssid = "Alison";          ///< SSID de la red para la conexion a internet
const char* password = "Alison12";    ///< Contraseña de la red
int canal_espnow = 1;                 ///< Canal de ESP-NOW
int canal_wifi = 6;                   ///< Canal para comunicacion por Telegram

TaskHandle_t xTaskESPNOW = NULL;      ///< Handle tarea ESP-NOW
TaskHandle_t xTaskRTC = NULL;         ///< Handle tarea RTC
TaskHandle_t xTaskTelegram = NULL;     ///< Handle tarea Telegram
TaskHandle_t xTaskSleepManager = NULL; ///< Handle tarea Sleep Manager
TaskHandle_t xTaskSD = NULL;          ///< Handle tarea SD

QueueHandle_t xQueueSensorData;       ///< Cola datos sensores
QueueHandle_t xQueueTelegram;         ///< Cola mensajes Telegram
QueueHandle_t xQueueSD;               ///< Cola datos SD

SemaphoreHandle_t xMutexESPNOW;       ///< Mutex ESP-NOW
SemaphoreHandle_t xMutexWiFi;         ///< Mutex WiFi
SemaphoreHandle_t xMutexSD;           ///< Mutex SD

EventGroupHandle_t xEventGroup;       ///< Grupo de eventos
TimerHandle_t xTimerRTCEnvio;         ///< Timer envío RTC
TimerHandle_t xTimerSistemaActivo;    ///< Timer sistema activo

struct_message datos_global;          ///< Datos globales sensores
volatile bool sistema_activo = true;  ///< Estado del sistema
String fecha_actual = "";             ///< Fecha actual

uint8_t mac1[] = {0xC4, 0xDD, 0x57, 0xB7, 0x16, 0x24}; ///< MAC1 del esp con los sensores
uint8_t mac2[] = {0xF0, 0xF5, 0xBD, 0x0B, 0xE3, 0x4C}; ///< MAC2 del esp con el actuador
/** @} */

WiFiClientSecure secured_client;      ///< Objeto para la conexion WiFi
UniversalTelegramBot bot(BOTtoken, secured_client);  ///< Objeto para la conexion a Telegram
RTC_DS3231 rtc;                      ///< Objeto para el RTC

/**
 * @brief Inicializa el módulo SD
 * @return true si la inicialización fue exitosa, false en caso contrario
 * 
 * Configura los pines SPI e inicializa la comunicación con la tarjeta SD.
 * Muestra información sobre el tipo y tamaño de la tarjeta detectada.
 */
bool inicializarSD() {
  Serial.println("Inicializando módulo SD...");

  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Error al inicializar módulo SD");
    return false;
  }
  
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No se detectó tarjeta SD");
    return false;
  }
  
  Serial.print("Módulo SD inicializado. Tipo de tarjeta: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Tamaño de tarjeta SD: %lluMB\n", cardSize);
  
  return true;
}

/**
 * @brief Obtiene la fecha actual desde el RTC
 * @return String con la fecha en formato YYYY-MM-DD
 * 
 * Consulta el RTC y formatea la fecha actual como cadena.
 */
String obtenerFechaActual() {
  DateTime now = rtc.now();
  char fecha[11];
  snprintf(fecha, sizeof(fecha), "%04d-%02d-%02d", 
           now.year(), now.month(), now.day());
  return String(fecha);
}
/** @defgroup funciones_sd Funciones de manejo de SD */
/** @{ */

/**
 * @brief Crea una carpeta diaria en la tarjeta SD
 * @return true si la carpeta existe o se creó correctamente, false en caso de error
 * 
 * Verifica si existe una carpeta con la fecha actual y la crea si no existe.
 * La ruta sigue el formato "/YYYY-MM-DD".
 */
bool crearCarpetaDiaria() {
  String fecha = obtenerFechaActual();
  String ruta_carpeta = "/" + fecha;
  
  if (!SD.exists(ruta_carpeta)) {
    if (SD.mkdir(ruta_carpeta)) {
      Serial.printf("Carpeta creada: %s\n", ruta_carpeta.c_str());
      return true;
    } else {
      Serial.printf("Error al crear carpeta: %s\n", ruta_carpeta.c_str());
      return false;
    }
  }
  return true; 
}

/**
 * @brief Guarda datos de sensores en la tarjeta SD
 * @param datos Estructura con los valores de los sensores
 * @param timestamp Marca de tiempo de los datos
 * @return true si los datos se guardaron correctamente, false en caso de error
 * 
 * Almacena los datos en un archivo CSV dentro de la carpeta correspondiente
 * a la fecha actual. Crea un archivo por hora con formato "datos_sensores_HH.csv".
 * Si el archivo no existe, añade encabezados. Usa un mutex para acceso seguro.
 */
bool guardarDatosSD(struct_message datos, DateTime timestamp) {
  if (xSemaphoreTake(xMutexSD, pdMS_TO_TICKS(5000)) {
    String fecha = obtenerFechaActual(); 

    if (fecha != fecha_actual) {
      fecha_actual = fecha;
      if (!crearCarpetaDiaria()) {
        xSemaphoreGive(xMutexSD);
        return false;
      }
    }

    char nombre_archivo[64];
    snprintf(nombre_archivo, sizeof(nombre_archivo),
             "/%s/datos_sensores_%02d.csv",
             fecha.c_str(), timestamp.hour());
    bool archivo_nuevo = !SD.exists(nombre_archivo);

    File archivo = SD.open(nombre_archivo, FILE_APPEND);
    if (!archivo) {
      Serial.printf("Error al abrir archivo: %s\n", nombre_archivo);
      xSemaphoreGive(xMutexSD);
      return false;
    }

    if (archivo_nuevo) {
      archivo.println("Fecha,Hora,Temp_DHT(°C),Humedad_DHT(%),Temp_DS18B20(°C),Luz(%)");
      Serial.printf("Archivo nuevo creado con encabezados: %s\n", nombre_archivo);
    }

    char linea[150];
    snprintf(linea, sizeof(linea), 
             "%04d-%02d-%02d,%02d:%02d:%02d,%.2f,%.2f,%.2f,%.2f",
             timestamp.year(), timestamp.month(), timestamp.day(),
             timestamp.hour(), timestamp.minute(), timestamp.second(),
             datos.temp_dht, datos.hum_dht, datos.temp_ds18b20, datos.luz);

    archivo.println(linea);
    archivo.close();

    Serial.printf("Datos guardados en SD: %s\n", linea);

    xSemaphoreGive(xMutexSD);
    return true;
  }

  Serial.println("No se pudo obtener acceso exclusivo al SD");
  return false;
}

/** @} */

/** @defgroup funciones_wifi Funciones de manejo WiFi */
/** @{ */

/**
 * @brief Cambia el canal WiFi del ESP32
 * @param canal Número de canal a configurar (1-14)
 * 
 * Configura el canal WiFi para comunicación, necesario para ESP-NOW.
 * Primero activa el modo promiscuo para cambiar el canal y luego lo desactiva.
 */
void cambiarCanal(int canal) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(canal, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

/** @} */
/** @defgroup funciones_espnow Funciones ESP-NOW */
/** @{ */

/**
 * @brief Callback de envío de datos ESP-NOW
 * @param mac_addr Dirección MAC del destinatario
 * @param status Estado del envío (éxito/fallo)
 * 
 * Se ejecuta automáticamente cuando se completa un envío ESP-NOW.
 * Muestra por serial el resultado del envío y la MAC del dispositivo destino.
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Estado de envío a %02X:%02X:%02X:%02X:%02X:%02X: %s\n",
                mac_addr[0], mac_addr[1], mac_addr[2], 
                mac_addr[3], mac_addr[4], mac_addr[5],
                status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Falló");
}

/**
 * @brief Callback de recepción de datos ESP-NOW
 * @param info Información del remitente (incluye MAC)
 * @param data Datos recibidos
 * @param len Longitud de los datos
 * 
 * Se ejecuta cuando llegan datos via ESP-NOW. Valida el tamaño de los datos,
 * los encola para procesamiento y activa el bit BIT_DATOS_RECIBIDOS.
 * Diseñado para ejecutarse desde una ISR (Interrupt Service Routine).
 */
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  Serial.printf("Datos recibidos de: %02X:%02X:%02X:%02X:%02X:%02X, Tamaño: %d bytes\n",
                info->src_addr[0], info->src_addr[1], info->src_addr[2],
                info->src_addr[3], info->src_addr[4], info->src_addr[5], len);
  
  if (len == sizeof(struct_message)) {
    sensor_data_queue_item_t item;
    memcpy(&item.datos, data, sizeof(struct_message));
    item.timestamp = xTaskGetTickCountFromISR();

    if (xQueueSendFromISR(xQueueSensorData, &item, &xHigherPriorityTaskWoken) == pdTRUE) {
      Serial.println("Datos de sensores encolados correctamente");
      xEventGroupSetBitsFromISR(xEventGroup, BIT_DATOS_RECIBIDOS, &xHigherPriorityTaskWoken);
    } else {
      Serial.println("Error al encolar datos de sensores");
    }
  } else {
    Serial.printf("Tamaño de datos inesperado: %d (esperado: %d)\n", len, sizeof(struct_message));
  }
  
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/** @} */

/** @defgroup funciones_timer Callbacks de Timers */
/** @{ */

/**
 * @brief Callback del timer RTC
 * @param xTimer Handle del timer que activó el callback
 * 
 * Notifica a la tarea RTC cuando expira el timer de sincronización horaria.
 */
void vTimerRTCCallback(TimerHandle_t xTimer) {
  if (xTaskRTC != NULL) {
    xTaskNotifyGive(xTaskRTC);
  }
}

/**
 * @brief Callback del timer de sistema activo
 * @param xTimer Handle del timer que activó el callback
 * 
 * Cambia el estado del sistema a inactivo y activa el bit BIT_DORMIR_SISTEMA
 * cuando expira el timer de actividad del sistema.
 */
void vTimerSistemaActivoCallback(TimerHandle_t xTimer) {
  sistema_activo = false;
  xEventGroupSetBits(xEventGroup, BIT_DORMIR_SISTEMA);
}

/** @} */

/** @defgroup funciones_espnow Funciones ESP-NOW */
/** @{ */

/**
 * @brief Inicializa el protocolo ESP-NOW
 * 
 * Configura el canal WiFi, inicializa ESP-NOW, registra callbacks y añade
 * dispositivos peers (mac1 y mac2). Usa un mutex para acceso seguro.
 */
void inicializarESPNOW() {
  if (xSemaphoreTake(xMutexESPNOW, portMAX_DELAY) == pdTRUE) {
    cambiarCanal(canal_espnow);
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error al inicializar ESP-NOW");
      xSemaphoreGive(xMutexESPNOW);
      return;
    }
    
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    
    // Agregar peers
    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = canal_espnow;
    peerInfo.encrypt = false;
    
    memcpy(peerInfo.peer_addr, mac1, 6);
    esp_now_add_peer(&peerInfo);
    
    memcpy(peerInfo.peer_addr, mac2, 6);
    esp_now_add_peer(&peerInfo);
    
    Serial.println("ESP-NOW inicializado");
    xSemaphoreGive(xMutexESPNOW);
  }
}

/**
 * @brief Envía comando de sincronización a dispositivos peers
 * @param comando Comando a enviar (1=Despertar, 0=Dormir)
 * 
 * Envía un comando de sincronización con timestamp a ambos dispositivos
 * (mac1 y mac2) usando ESP-NOW. Protegido con mutex.
 */
void enviarComandoSincronizacion(uint8_t comando) {
  if (xSemaphoreTake(xMutexESPNOW, pdMS_TO_TICKS(1000)) {
    struct_sync sync_cmd;
    sync_cmd.comando = comando;
    sync_cmd.timestamp = rtc.now().unixtime();
    
    esp_now_send(mac1, (uint8_t *)&sync_cmd, sizeof(sync_cmd));
    esp_now_send(mac2, (uint8_t *)&sync_cmd, sizeof(sync_cmd));
    
    Serial.printf("Comando de sincronización enviado: %s\n", 
                  comando == 1 ? "DESPERTAR" : "DORMIR");
    
    xSemaphoreGive(xMutexESPNOW);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/** @} */
/** @defgroup funciones_envio Funciones de Envío */
/** @{ */

/**
 * @brief Envía la hora actual del RTC a los dispositivos peers
 * 
 * Obtiene la hora actual del RTC, la formatea y la envía via ESP-NOW
 * a ambos dispositivos (mac1 y mac2). Protegido con mutex para acceso seguro.
 * El formato de hora es HH:MM:SS.
 */
void enviarDatosRTC() {
  if (xSemaphoreTake(xMutexESPNOW, pdMS_TO_TICKS(1000)) {
    struct_rtc datos_rtc;
    DateTime now = rtc.now();
    
    snprintf(datos_rtc.hora, sizeof(datos_rtc.hora), "%02d:%02d:%02d", 
             now.hour(), now.minute(), now.second());

    esp_now_send(mac1, (uint8_t *)&datos_rtc, sizeof(datos_rtc));
    esp_now_send(mac2, (uint8_t *)&datos_rtc, sizeof(datos_rtc));
    
    Serial.printf("Enviando hora RTC: %s\n", datos_rtc.hora);
    
    xSemaphoreGive(xMutexESPNOW);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/**
 * @brief Envía un comando de control a un actuador
 * @param activar true para activar modo alerta, false para modo normal
 * @param temperatura Temperatura objetivo para el actuador
 * 
 * Envía un comando al dispositivo actuador (mac2) via ESP-NOW indicando
 * si debe activarse por umbral o volver a modo normal, junto con la
 * temperatura de referencia. Protegido con mutex.
 */
void enviarComandoActuador(bool activar, float temperatura) {
  if (xSemaphoreTake(xMutexESPNOW, pdMS_TO_TICKS(1000)) {
    struct_actuador cmd_actuador;
    
    if (activar) {
      strcpy(cmd_actuador.comando, "ALERTA_UMBRAL");
    } else {
      strcpy(cmd_actuador.comando, "NORMAL");
    }
    cmd_actuador.temperatura = temperatura;
    
    esp_err_t result = esp_now_send(mac2, (uint8_t *)&cmd_actuador, sizeof(cmd_actuador));
    if (result == ESP_OK) {
      Serial.printf("Comando enviado al actuador: %s (Temp: %.2f°C)\n", 
                    cmd_actuador.comando, cmd_actuador.temperatura);
    } else {
      Serial.printf("Error al enviar comando al actuador: %d\n", result);
    }
    
    xSemaphoreGive(xMutexESPNOW);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/** @} */

/** @defgroup tareas Tareas FreeRTOS */
/** @{ */

/**
 * @brief Manejador de la tarea de almacenamiento en SD
 * @param pvParameters Parámetros de la tarea (no utilizados)
 * 
 * Tarea que espera por el bit BIT_GUARDAR_SD para procesar los datos
 * encolados en xQueueSD y guardarlos en la tarjeta SD. Maneja los errores
 * de escritura e imprime el estado por serial.
 */
void vTaskSDHandler(void *pvParameters) {
  sd_data_queue_item_t sd_item;
  
  while (1) {
    EventBits_t bits = xEventGroupWaitBits(xEventGroup, BIT_GUARDAR_SD, 
                                           pdTRUE, pdFALSE, portMAX_DELAY);
    
    if (bits & BIT_GUARDAR_SD) {
      while (xQueueReceive(xQueueSD, &sd_item, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (!guardarDatosSD(sd_item.datos, sd_item.timestamp_rtc)) {
          Serial.println("Error al guardar datos en SD");
        }
      }
    }
  }
}

/** @} */
/** @defgroup tareas Tareas FreeRTOS */
/** @{ */

/**
 * @brief Manejador de la tarea ESP-NOW
 * @param pvParameters Parámetros de la tarea (no utilizados)
 * 
 * Procesa los datos recibidos via ESP-NOW desde la cola xQueueSensorData:
 * 1. Almacena datos en estructura global
 * 2. Prepara datos para almacenar en SD
 * 3. Verifica umbrales y genera alertas
 * 4. Envía comandos a actuador cuando se superan umbrales
 * 5. Prepara mensajes de alerta para Telegram
 */
void vTaskESPNOWHandler(void *pvParameters) {
  sensor_data_queue_item_t receivedData;
  
  while (1) {
    if (xQueueReceive(xQueueSensorData, &receivedData, portMAX_DELAY) == pdTRUE) {
      memcpy(&datos_global, &receivedData.datos, sizeof(struct_message));
      
      Serial.printf("Datos procesados - DHT: %.2f°C/%.2f%%, DS18B20: %.2f°C, Luz: %.2f\n",
                    datos_global.temp_dht, datos_global.hum_dht, 
                    datos_global.temp_ds18b20, datos_global.luz);

      // Preparar datos para SD
      sd_data_queue_item_t sd_item;
      memcpy(&sd_item.datos, &datos_global, sizeof(struct_message));
      sd_item.timestamp_rtc = rtc.now();
   
      if (xQueueSend(xQueueSD, &sd_item, pdMS_TO_TICKS(100)) == pdTRUE) {
        xEventGroupSetBits(xEventGroup, BIT_GUARDAR_SD);
      }

      // Verificación de umbrales
      String mensaje = "🚨 ALERTA DE SENSORES 🚨\n\n";
      bool enviar_telegram = false;
      bool activar_actuador = false;

      if (datos_global.temp_dht > UMBRAL_TEMP_DHT) {
        mensaje += "🌡 Temp DHT alta: " + String(datos_global.temp_dht) + "°C\n";
        enviar_telegram = true;
        activar_actuador = true;
      }
      
      if (datos_global.hum_dht > UMBRAL_HUM_DHT) {
        mensaje += "💧 Humedad DHT alta: " + String(datos_global.hum_dht) + "%\n";
        enviar_telegram = true;
      }
      
      if (datos_global.temp_ds18b20 > UMBRAL_DS18B20) {
        mensaje += "🌡 Temp DS18B20 alta: " + String(datos_global.temp_ds18b20) + "°C\n";
        enviar_telegram = true;
        activar_actuador = true;
      }
      
      if (datos_global.luz > UMBRAL_LUZ) {
        mensaje += "💡 Luz alta: " + String(datos_global.luz) + "%\n";
        enviar_telegram = true;
      }

      // Control de actuador
      enviarComandoActuador(activar_actuador, max(datos_global.temp_dht, datos_global.temp_ds18b20));

      // Notificación por Telegram
      if (enviar_telegram) {
        DateTime ahora = rtc.now();
        mensaje += "\n⏰ Hora: " + String(ahora.hour()) + ":" + String(ahora.minute()) + ":" + String(ahora.second());
        
        telegram_queue_item_t telegram_item;
        telegram_item.mensaje = mensaje;
        
        if (xQueueSend(xQueueTelegram, &telegram_item, pdMS_TO_TICKS(100)) == pdTRUE) {
          xEventGroupSetBits(xEventGroup, BIT_ENVIAR_TELEGRAM);
        }
      }
    }
  }
}

/**
 * @brief Manejador de la tarea RTC
 * @param pvParameters Parámetros de la tarea (no utilizados)
 * 
 * Tarea que espera notificaciones para enviar la hora actual via ESP-NOW.
 * Solo actúa cuando el sistema está en modo activo (sistema_activo = true).
 * Se activa periódicamente por el timer xTimerRTCEnvio.
 */
void vTaskRTCHandler(void *pvParameters) {
  while (1) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      if (sistema_activo) {
        enviarDatosRTC();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * @brief Manejador de la tarea de Telegram
 * @param pvParameters Parámetros de la tarea (no utilizados)
 * 
 * Tarea que procesa mensajes de la cola xQueueTelegram y los envía
 * al chat configurado via bot de Telegram. Se activa cuando se establece
 * el bit BIT_ENVIAR_TELEGRAM en el grupo de eventos.
 */
/** @defgroup Tareas FreeRTOS */
/** @{ */

/**
 * @brief Manejador de la tarea de Telegram
 * @param pvParameters Parámetros de la tarea (no utilizados)
 * 
 * Gestiona el envío de mensajes de alerta a Telegram:
 * 1. Espera el bit BIT_ENVIAR_TELEGRAM
 * 2. Recoge mensajes de la cola xQueueTelegram
 * 3. Configura conexión WiFi segura
 * 4. Realiza hasta 3 intentos de envío
 * 5. Maneja errores de conexión y envío
 * 
 * @note Libera recursos ESP-NOW durante la conexión WiFi
 */
void vTaskTelegramHandler(void *pvParameters) {
  telegram_queue_item_t mensaje_item;
  
  while (1) {
    EventBits_t bits = xEventGroupWaitBits(xEventGroup, BIT_ENVIAR_TELEGRAM, 
                                           pdTRUE, pdFALSE, portMAX_DELAY);
    
    if (bits & BIT_ENVIAR_TELEGRAM) {
      while (xQueueReceive(xQueueTelegram, &mensaje_item, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (xSemaphoreTake(xMutexWiFi, pdMS_TO_TICKS(30000)) == pdTRUE) {
          Serial.println("Conectando a WiFi para Telegram...");
 
          if (xSemaphoreTake(xMutexESPNOW, pdMS_TO_TICKS(5000)) == pdTRUE) {
            esp_now_deinit();
            xSemaphoreGive(xMutexESPNOW);
          }
          
          WiFi.disconnect(true);
          vTaskDelay(pdMS_TO_TICKS(500));
          cambiarCanal(canal_wifi);
          WiFi.mode(WIFI_STA);
          WiFi.begin(ssid, password);

          TickType_t startTime = xTaskGetTickCount();
          while (WiFi.status() != WL_CONNECTED && 
                 (xTaskGetTickCount() - startTime) < pdMS_TO_TICKS(15000)) {
            vTaskDelay(pdMS_TO_TICKS(500));
            Serial.print(".");
          }

          if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi conectado para Telegram.");
            secured_client.setInsecure();
            
            // Intentar enviar mensaje con reintentos
            bool mensaje_enviado = false;
            for (int intento = 0; intento < 3 && !mensaje_enviado; intento++) {
              if(bot.sendMessage(CHAT_ID, mensaje_item.mensaje, "")) {
                Serial.println("Mensaje enviado a Telegram.");
                mensaje_enviado = true;
              } else {
                Serial.printf("Intento %d fallido. Reintentando...\n", intento + 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
              }
            }
            
            if (!mensaje_enviado) {
              Serial.println("No se pudo enviar mensaje a Telegram después de 3 intentos.");
            }
          } else {
            Serial.println("\n No se pudo conectar a WiFi para Telegram.");
          }

          WiFi.disconnect(true);
          vTaskDelay(pdMS_TO_TICKS(1000));
          inicializarESPNOW();
          
          xSemaphoreGive(xMutexWiFi);
        }
      }
    }
  }
}

/**
 * @brief Manejador de la tarea de gestión del sueño
 * @param pvParameters Parámetros de la tarea (no utilizados)
 * 
 * Controla la transición a modo deep sleep:
 * 1. Espera el bit BIT_DORMIR_SISTEMA
 * 2. Detiene timers activos
 * 3. Envía comando de sincronización a peers
 * 4. Configura wakeup por timer
 * 5. Inicia deep sleep
 * 
 * @note El sistema se reiniciará automáticamente al despertar
 */
void vTaskSleepManager(void *pvParameters) {
  while (1) {
    EventBits_t bits = xEventGroupWaitBits(xEventGroup, BIT_DORMIR_SISTEMA, 
                                           pdTRUE, pdFALSE, portMAX_DELAY);
    
    if (bits & BIT_DORMIR_SISTEMA) {
      Serial.println("Preparando sistema para dormir...");
      xTimerStop(xTimerRTCEnvio, pdMS_TO_TICKS(1000));
      xTimerStop(xTimerSistemaActivo, pdMS_TO_TICKS(1000));
      enviarComandoSincronizacion(2);
      vTaskDelay(pdMS_TO_TICKS(2000));
 
      Serial.printf("Entrando en deep sleep por %d segundos...\n", 
                    (int)(TIEMPO_DORMIDO_TICKS * portTICK_PERIOD_MS / 1000));
      
      esp_sleep_enable_timer_wakeup((uint64_t)(TIEMPO_DORMIDO_TICKS * portTICK_PERIOD_MS) * 1000ULL);
      esp_deep_sleep_start();
    }
  }
}

/** @} */
/** @defgroup main_func Funciones Principales */
/** @{ */

/**
 * @brief Función de inicialización del sistema
 * 
 * Configura todo el hardware e inicializa el sistema FreeRTOS:
 * 1. Inicia comunicación serial y RTC
 * 2. Configura módulo SD y estructura de archivos
 * 3. Crea colas, semáforos, grupos de eventos y timers
 * 4. Inicializa comunicación ESP-NOW
 * 5. Crea todas las tareas FreeRTOS
 * 6. Inicia secuencia de sincronización
 * 
 * @note Si cualquier componente crítico falla, el sistema entra en bucle infinito
 */
void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando dispositivo RTC MASTER con FreeRTOS y SD...");

  if (!rtc.begin()) {
    Serial.println("Error al detectar el RTC DS3231");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  if (rtc.lostPower()) {
    Serial.println("RTC perdió energía, configurando fecha/hora...");
    rtc.adjust(DateTime(F(_DATE), F(TIME_)));
  }

  DateTime now = rtc.now();
  Serial.print("Hora actual RTC: ");
  Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));

  if (!inicializarSD()) {
    Serial.println("Error crítico: No se pudo inicializar el módulo SD");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  fecha_actual = obtenerFechaActual();
  if (!crearCarpetaDiaria()) {
    Serial.println("Error al crear carpeta inicial");
  }

  xQueueSensorData = xQueueCreate(10, sizeof(sensor_data_queue_item_t));
  xQueueTelegram = xQueueCreate(5, sizeof(telegram_queue_item_t));
  xQueueSD = xQueueCreate(15, sizeof(sd_data_queue_item_t));  
  xMutexESPNOW = xSemaphoreCreateMutex();
  xMutexWiFi = xSemaphoreCreateMutex();
  xMutexSD = xSemaphoreCreateMutex();  
  xEventGroup = xEventGroupCreate();
  
  xTimerRTCEnvio = xTimerCreate("RTCTimer", ENVIO_HORA_INTERVAL_TICKS, pdTRUE, 
                                (void*)0, vTimerRTCCallback);
  xTimerSistemaActivo = xTimerCreate("SistemaTimer", TIEMPO_DESPIERTO_TICKS, pdFALSE, 
                                    (void*)0, vTimerSistemaActivoCallback);

  if (xQueueSensorData == NULL || xQueueTelegram == NULL || xQueueSD == NULL ||
      xMutexESPNOW == NULL || xMutexWiFi == NULL || xMutexSD == NULL || 
      xEventGroup == NULL || xTimerRTCEnvio == NULL || xTimerSistemaActivo == NULL) {
    Serial.println("Error al crear objetos FreeRTOS");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  inicializarESPNOW();

  xTaskCreatePinnedToCore(vTaskESPNOWHandler, "ESP_NOW_Task", STACK_SIZE_ESP_NOW, 
                          NULL, PRIORITY_ESP_NOW_TASK, &xTaskESPNOW, 1);
  
  xTaskCreatePinnedToCore(vTaskRTCHandler, "RTC_Task", STACK_SIZE_RTC, 
                          NULL, PRIORITY_RTC_TASK, &xTaskRTC, 1);
  
  xTaskCreatePinnedToCore(vTaskTelegramHandler, "Telegram_Task", STACK_SIZE_TELEGRAM, 
                          NULL, PRIORITY_TELEGRAM_TASK, &xTaskTelegram, 0);
  
  xTaskCreatePinnedToCore(vTaskSleepManager, "Sleep_Manager", STACK_SIZE_SLEEP_MANAGER, 
                          NULL, PRIORITY_SLEEP_MANAGER_TASK, &xTaskSleepManager, 0);
  
  xTaskCreatePinnedToCore(vTaskSDHandler, "SD_Task", STACK_SIZE_SD, 
                          NULL, PRIORITY_SD_TASK, &xTaskSD, 0);

  if (xTaskESPNOW == NULL || xTaskRTC == NULL || 
      xTaskTelegram == NULL || xTaskSleepManager == NULL || xTaskSD == NULL) {
    Serial.println("Error al crear tareas");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  Serial.println("Todas las tareas FreeRTOS creadas exitosamente");
  Serial.printf("Memoria libre en heap: %d bytes\n", xPortGetFreeHeapSize());
  Serial.printf("Memoria libre mínima: %d bytes\n", xPortGetMinimumEverFreeHeapSize());
  vTaskDelay(pdMS_TO_TICKS(500));
  enviarComandoSincronizacion(1);
  vTaskDelay(pdMS_TO_TICKS(1000));

  enviarDatosRTC();
  vTaskDelay(pdMS_TO_TICKS(2000));

  xTimerStart(xTimerRTCEnvio, 0);
  xTimerStart(xTimerSistemaActivo, 0);

  Serial.println("Sistema FreeRTOS con SD iniciado - Ciclo activo de 1 minuto...");
  Serial.printf("Guardando datos en carpeta: /%s/\n", fecha_actual.c_str());
}

void loop() {
  // El loop permanece vacío ya que todo se maneja mediante tareas FreeRTOS
}
/** @} */