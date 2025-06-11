/**
 * @file codigoactuadores_documentado.ino
 * @brief Código para ESP32 actuador con FreeRTOS que recibe comandos via ESP-NOW
 * @details Este código implementa un actuador controlado por comandos recibidos via ESP-NOW
 *          desde otros dispositivos ESP32. Incluye manejo de relay con timeout de seguridad
 *          y recepción de diferentes estructuras de datos.
 * @author Alison Daniela Ruiz, Johana Puerres Pizarro, Juan David Gutierrez
 * @date 11/06/24
 */

#include <WiFi.h>
#include <esp_now.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define RELAY_PIN 5 ///< Pin GPIO conectado al relay

/**
 * @struct struct_actuador
 * @brief Estructura para comandos de actuador con temperatura
 */
typedef struct {
  char comando[20];    ///< Comando de texto (ej. "ALERTA_UMBRAL", "NORMAL")
  float temperatura;   ///< Valor de temperatura asociado al comando
} struct_actuador;

/**
 * @struct struct_sync
 * @brief Estructura para comandos de sincronización del sistema
 */
typedef struct {
  uint8_t comando;     ///< Código de comando (1=DESPERTAR, 2=DORMIR)
  uint32_t timestamp;  ///< Marca de tiempo para sincronización
} struct_sync;

/**
 * @struct struct_rtc
 * @brief Estructura para sincronización de hora RTC
 */
typedef struct {
  char hora[9];        ///< Cadena de hora en formato HH:MM:SS
} struct_rtc;

struct_actuador datos_recibidos; ///< Almacena los últimos datos recibidos del tipo actuador
bool relay_estado = false;       ///< Estado actual del relay (true=activado, false=desactivado)
unsigned long ultimo_comando = 0; ///< Marca de tiempo del último comando recibido
const unsigned long TIMEOUT_RELAY = 30000; ///< Tiempo máximo de activación del relay en ms (30 segundos)

// Declaración de la tarea
TaskHandle_t taskTimeoutHandle = NULL; ///< Manejador de la tarea de verificación de timeout

/**
 * @brief Imprime la dirección MAC del dispositivo en el puerto serie
 */
void imprimirMAC() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("MAC Address del actuador: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/**
 * @brief Controla el estado del relay y muestra información por serial
 * @param activar true para activar el relay, false para desactivarlo
 * @param temp Temperatura asociada (opcional, solo para información)
 */
void controlarRelay(bool activar, float temp = 0.0) {
  if (activar != relay_estado) {
    digitalWrite(RELAY_PIN, activar ? HIGH : LOW);
    relay_estado = activar;
    ultimo_comando = millis();
    
    Serial.printf("Relay %s", activar ? "ACTIVADO" : "DESACTIVADO");
    if (activar && temp > 0) {
      Serial.printf(" - Temperatura: %.2f°C", temp);
    }
    Serial.println();
  }
}

/**
 * @brief Callback que se ejecuta cuando se reciben datos via ESP-NOW
 * @param recv_info Información del remitente
 * @param incomingData Puntero a los datos recibidos
 * @param len Longitud de los datos recibidos
 */
void onDataReceive(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  Serial.printf("Datos recibidos de: %02X:%02X:%02X:%02X:%02X:%02X, Tamaño: %d bytes\n",
                recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5], len);

  if (len == sizeof(struct_actuador)) {
    memcpy(&datos_recibidos, incomingData, sizeof(struct_actuador));
    
    Serial.printf("Comando recibido: %s, Temperatura: %.2f°C\n", 
                  datos_recibidos.comando, datos_recibidos.temperatura);
    
    if (strcmp(datos_recibidos.comando, "ALERTA_UMBRAL") == 0) {
      controlarRelay(true, datos_recibidos.temperatura);
      Serial.println("ALERTA: Umbral de temperatura superado!");
    } else if (strcmp(datos_recibidos.comando, "NORMAL") == 0) {
      controlarRelay(false);
      Serial.println("Estado normal: Relay desactivado");
    } else {
      Serial.printf("Comando desconocido: %s\n", datos_recibidos.comando);
    }
  } else if (len == sizeof(struct_sync)) {
    struct_sync sync_data;
    memcpy(&sync_data, incomingData, sizeof(struct_sync));
    
    Serial.printf("Comando de sincronización: %s (Timestamp: %u)\n", 
                  sync_data.comando == 1 ? "DESPERTAR" : "DORMIR", sync_data.timestamp);
    
    if (sync_data.comando == 1) {
      Serial.println("Sistema despertando - Actuador listo");
    } else if (sync_data.comando == 2) {
      Serial.println("Sistema entrando en modo dormido");
      controlarRelay(false);
    }
  } else if (len == sizeof(struct_rtc)) {
    struct_rtc hora_data;
    memcpy(&hora_data, incomingData, sizeof(struct_rtc));
    Serial.printf("Hora sincronizada: %s\n", hora_data.hora);
  } else {
    Serial.printf("Tamaño de datos inesperado: %d bytes\n", len);
    Serial.print("Datos hex: ");
    for (int i = 0; i < len; i++) {
      Serial.printf("%02X ", incomingData[i]);
    }
    Serial.println();
  }
}

/**
 * @brief Tarea FreeRTOS que verifica timeout del relay
 * @param parameter Parámetros de la tarea (no utilizado)
 */
void tareaVerificarTimeout(void *parameter) {
  const TickType_t delayTicks = pdMS_TO_TICKS(5000); // 5 segundos
  while (true) {
    if (relay_estado && (millis() - ultimo_comando > TIMEOUT_RELAY)) {
      Serial.println("Timeout del relay - Desactivando por seguridad");
      controlarRelay(false);
    }
    vTaskDelay(delayTicks); // Espera sin bloquear el sistema
  }
}

/**
 * @brief Función de configuración inicial del dispositivo
 */
void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando ESP32 Actuador...");

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  relay_estado = false;
  
  imprimirMAC();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("WiFi configurado en modo STA");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al inicializar ESP-NOW");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  esp_now_register_recv_cb(onDataReceive);

  Serial.println("ESP-NOW inicializado correctamente");
  Serial.println("Callback de recepción registrado");
  Serial.println("Sistema actuador listo para recibir comandos");
  Serial.println("==========================================");

  // Crear tarea para verificación de timeout
  xTaskCreatePinnedToCore(
    tareaVerificarTimeout,    // Función
    "Tarea Timeout Relay",    // Nombre
    2048,                     // Stack size
    NULL,                     // Parámetro
    1,                        // Prioridad
    &taskTimeoutHandle,       // Handle
    1                         // Núcleo (core 1)
  );
}

/**
 * @brief Función principal de loop (no utilizada)
 * @details No se utiliza porque todo el control se maneja mediante FreeRTOS y callbacks
 */
void loop() {
  // No se utiliza porque todo lo maneja FreeRTOS
}