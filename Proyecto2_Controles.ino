#include <PS4Controller.h>

// Dirección MAC del controlador PS4
const char* ps4ControllerAddress = "24:dc:c3:46:1e:6e";

// Pines para UART Serial1 (hacia el ESP32 Maestro)
#define RXD1 16
#define TXD1 17

// Sensibilidad mínima para detectar movimiento del joystick
const int threshold = 90;  // Umbral más alto para menor sensibilidad

// Variables para guardar el estado del joystick y botones
bool xButtonPressed = false;  // Estado del botón 'X'
bool downCommandSent = false;  // Verificación de comando 'D' enviado

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);  // Comunicación con ESP32 maestro

  if (PS4.begin(ps4ControllerAddress)) {
    Serial.println("Bluetooth iniciado. Control PS4 conectado.");
  } else {
    Serial.println("Error al iniciar el Bluetooth.");
  }
}

void loop() {
  if (PS4.isConnected()) {
    // Leer la posición del joystick izquierdo
    int lx = PS4.LStickX();  // Eje X del joystick (-128 a 127)
    int ly = PS4.LStickY();  // Eje Y del joystick (-128 a 127)

    // Manejar el movimiento continuo hacia la derecha e izquierda
    if (lx > threshold) {
      Serial1.write('Z');  // Enviar 'R' continuamente
      Serial.println("Joystick movido a la derecha");
    } else if (lx < -threshold) {
      Serial1.write('C');  // Enviar 'L' continuamente
      Serial.println("Joystick movido a la izquierda");
    }


    // Manejo del botón 'X' para presionar y soltar
    if (PS4.Cross()) {
      if (!xButtonPressed) {  // Botón 'X' recién presionado
        Serial1.write('P');
        Serial.println("Botón X presionado");
        xButtonPressed = true;
      }
    } else {
      if (xButtonPressed) {  // Botón 'X' recién liberado
        Serial1.write('p');
        Serial.println("Botón X liberado");
        xButtonPressed = false;
      }
    }
  } else {
    Serial.println("Esperando conexión del control PS4...");
  }

  delay(50);  // Pequeño retardo para evitar envíos demasiado rápidos
}
