/**
 * @file codigosensores_documentado.ino
 * @brief Nodo sensor con ESP-NOW y FreeRTOS para sistema distribuido
 * @details Este código implementa un nodo sensor que recopila datos de temperatura, humedad y luz,
 *          los envía via ESP-NOW a un nodo RTC y gestiona ciclos de sueño para ahorro de energía.
 * @author Alison Daniela Ruiz, Johana Puerres Pizarro, Juan David Gutierrez
 * @date 11/06/24
 */

#include <WiFi.h>
#include <esp_now.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pines y sensores
#define DHTPIN 4           ///< Pin GPIO conectado al sensor DHT
#define DHTTYPE DHT11      ///< Tipo de sensor DHT (DHT11)
#define LDR_PIN 34         ///< Pin GPIO conectado al sensor de luz (LDR)
#define ONE_WIRE_BUS 5     ///< Pin GPIO para el bus OneWire (DS18B20)

// Tiempos de sincronización (en segundos)
#define TIEMPO_DESPIERTO 60  ///< Tiempo que permanece activo el nodo
#define TIEMPO_DORMIDO 60    ///< Tiempo en deep sleep

DHT dht(DHTPIN, DHTTYPE);  ///< Objeto para sensor DHT
OneWire oneWire(ONE_WIRE_BUS); ///< Objeto para bus OneWire
DallasTemperature ds18b20(&oneWire); ///< Objeto para sensor DS18B20

/**
 * @struct struct_sensores
 * @brief Estructura para datos de sensores
 */
typedef struct {
  float temp_dht;        ///< Temperatura del DHT (ºC)
  float hum_dht;         ///< Humedad del DHT (%)
  float temp_ds18b20;    ///< Temperatura del DS18B20 (ºC)
  float luz;             ///< Nivel de luz (porcentaje)
} struct_sensores;

/**
 * @struct struct_rtc
 * @brief Estructura para sincronización de hora
 */
typedef struct {
  char hora[9];          ///< Cadena de hora en formato "HH:MM:SS"
} struct_rtc;

/**
 * @struct struct_sync
 * @brief Estructura para comandos de sincronización
 */
typedef struct {
  uint8_t comando;       ///< 1 = despertar, 2 = dormir
  uint32_t timestamp;    ///< Marca de tiempo para sincronización
} struct_sync;

struct_sensores datos_sensores; ///< Almacena los últimos datos de sensores leídos
char hora_actual[9] = "00:00:00"; ///< Hora actual sincronizada con RTC
bool comando_dormir_recibido = false; ///< Flag para comando de dormir
unsigned long inicio_ciclo_activo = 0; ///< Marca de tiempo de inicio de ciclo activo

uint8_t rtcAddress[] = {0xB8, 0xD6, 0x1A, 0x42, 0x47, 0xB4}; ///< Dirección MAC del nodo RTC

// Prototipos de funciones
void leerSensoresTask(void *pvParameters);
void enviarDatosTask(void *pvParameters);
void verificarSleepTask(void *pvParameters);

/**
 * @brief Callback de recepción de datos ESP-NOW
 * @param info Información del remitente
 * @param incomingData Puntero a los datos recibidos
 * @param len Longitud de los datos recibidos
 */
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(struct_rtc)) {
    struct_rtc hora_temp;
    memcpy(&hora_temp, incomingData, sizeof(struct_rtc));
    strncpy(hora_actual, hora_temp.hora, sizeof(hora_actual));
    hora_actual[8] = '\0';
    Serial.printf(" Hora actualizada desde RTC: %s\n", hora_actual);
    
  } else if (len == sizeof(struct_sensores)) {
    struct_sensores sensores_temp;
    memcpy(&sensores_temp, incomingData, sizeof(struct_sensores));
    Serial.printf("[%s]  Datos externos - Temp DHT: %.2f°C, Hum: %.2f%%, Temp DS18B20: %.2f°C, Luz: %.2f%%\n",
                  hora_actual, sensores_temp.temp_dht, sensores_temp.hum_dht, 
                  sensores_temp.temp_ds18b20, sensores_temp.luz);
                  
  } else if (len == sizeof(struct_sync)) {
    struct_sync sync_cmd;
    memcpy(&sync_cmd, incomingData, sizeof(struct_sync));
    
    if (sync_cmd.comando == 1) {
      Serial.println(" Comando DESPERTAR recibido del RTC");
      inicio_ciclo_activo = millis();
      comando_dormir_recibido = false;
    } else if (sync_cmd.comando == 2) {
      Serial.println(" Comando DORMIR recibido del RTC");
      comando_dormir_recibido = true;
    }
  }
}

/**
 * @brief Callback de envío de datos ESP-NOW
 * @param mac_addr Dirección MAC del destinatario
 * @param status Estado del envío
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf(" Estado de envío: %s\n", status == ESP_NOW_SEND_SUCCESS ? " Éxito" : " Falló");
}

/**
 * @brief Muestra la dirección MAC del dispositivo
 */
void mostrarMAC() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf(" MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/**
 * @brief Lee todos los sensores conectados
 * @return struct_sensores con los datos leídos
 */
struct_sensores leerTodosSensores() {
  struct_sensores datos;
  float temp_dht = dht.readTemperature();
  float hum_dht = dht.readHumidity();
  if (isnan(temp_dht) || isnan(hum_dht)) {
    temp_dht = 0.0;
    hum_dht = 0.0;
  }

  ds18b20.requestTemperatures();
  float temp_ds = ds18b20.getTempCByIndex(0);
  if (temp_ds == DEVICE_DISCONNECTED_C) {
    temp_ds = 0.0;
  }

  int ldrValue = analogRead(LDR_PIN);
  float luz = (ldrValue / 4095.0) * 100.0;

  datos.temp_dht = temp_dht;
  datos.hum_dht = hum_dht;
  datos.temp_ds18b20 = temp_ds;
  datos.luz = luz;
  return datos;
}

/**
 * @brief Inicializa la comunicación ESP-NOW
 */
void inicializarESPNOW() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println(" Error al iniciar ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rtcAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(" Error al agregar peer RTC");
  } else {
    Serial.println(" Peer RTC agregado");
  }
}

/**
 * @brief Tarea FreeRTOS para lectura de sensores
 * @param pvParameters Parámetros de la tarea (no utilizado)
 */
void leerSensoresTask(void *pvParameters) {
  while (true) {
    if (!comando_dormir_recibido) {
      datos_sensores = leerTodosSensores();
      Serial.printf("[%s] Sensores locales - Temp DHT: %.2f°C, Hum: %.2f%%, Temp DS18B20: %.2f°C, Luz: %.2f%%\n",
                    hora_actual, datos_sensores.temp_dht, datos_sensores.hum_dht,
                    datos_sensores.temp_ds18b20, datos_sensores.luz);
    }
    vTaskDelay(pdMS_TO_TICKS(5000));  // cada 5 segundos
  }
}

/**
 * @brief Tarea FreeRTOS para envío de datos al RTC
 * @param pvParameters Parámetros de la tarea (no utilizado)
 */
void enviarDatosTask(void *pvParameters) {
  while (true) {
    if (!comando_dormir_recibido) {
      esp_err_t result = esp_now_send(rtcAddress, (uint8_t *)&datos_sensores, sizeof(datos_sensores));
      Serial.printf("[%s] Envío de datos al RTC: %s\n", hora_actual,
                    result == ESP_OK ? " Éxito" : " Fallo");
    }
    vTaskDelay(pdMS_TO_TICKS(10000));  // cada 10 segundos
  }
}

/**
 * @brief Tarea FreeRTOS para verificar condiciones de deep sleep
 * @param pvParameters Parámetros de la tarea (no utilizado)
 */
void verificarSleepTask(void *pvParameters) {
  while (true) {
    if (comando_dormir_recibido || (millis() - inicio_ciclo_activo >= TIEMPO_DESPIERTO * 1000)) {
      Serial.println("Entrando en deep sleep...");
      esp_sleep_enable_timer_wakeup((uint64_t)TIEMPO_DORMIDO * 1000000ULL);
      esp_deep_sleep_start();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * @brief Función de configuración inicial
 */
void setup() {
  Serial.begin(115200);
  mostrarMAC();
  dht.begin();
  ds18b20.begin();
  pinMode(LDR_PIN, INPUT);
  inicializarESPNOW();

  Serial.println("Esperando comando de despertar del RTC...");
  unsigned long timeout = millis();
  while (!inicio_ciclo_activo && (millis() - timeout < 10000)) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (!inicio_ciclo_activo) {
    Serial.println("No se recibió comando de despertar, iniciando ciclo por defecto");
    inicio_ciclo_activo = millis();
  }

  Serial.println("Iniciando ciclo activo sincronizado...");

  xTaskCreate(leerSensoresTask, "LecturaSensores", 4096, NULL, 1, NULL);
  xTaskCreate(enviarDatosTask, "EnvioDatos", 4096, NULL, 1, NULL);
  xTaskCreate(verificarSleepTask, "VerificarDormir", 2048, NULL, 1, NULL);
}

/**
 * @brief Función principal de loop (no utilizada)
 * @details No se utiliza porque todo el control se maneja mediante FreeRTOS
 */
void loop() {
}