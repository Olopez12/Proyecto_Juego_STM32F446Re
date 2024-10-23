
#include <PS4Controller.h>

// Dirección MAC del controlador PS4 (Maestro)
const char* ps4ControllerAddress = "c0:49:ef:32:76:be";

// Pines para UART Serial1 (hacia STM32)
#define RXD1 16
#define TXD1 17

// Pines para UART Serial2 (hacia ESP32 Esclavo)
#define RXD2 32
#define TXD2 33

// Pines GPIO para controlar el DFPlayer Mini
const int DFPLAYER_NEXT = 5;  // GPIO para avanzar a la siguiente canción
const int DFPLAYER_PLAY = 4;  // GPIO para iniciar la reproducción

// Sensibilidad mínima para detectar movimiento del joystick
const int threshold = 90;

// Variables de estado
bool xButtonPressed = false;
bool downCommandSent = false;
bool shareButtonPressed = false;
bool isPlaying = false;  // Indica si la reproducción está activa
int currentSong = 1;  // Canción actual
const int totalSongs = 3;  // Total de canciones en la lista

void setup() {
  // Inicialización de comunicación serial
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);  // Comunicación con STM32
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // Comunicación con ESP32 esclavo

  // Configurar los pines GPIO para el DFPlayer Mini
  pinMode(DFPLAYER_NEXT, OUTPUT);
  pinMode(DFPLAYER_PLAY, OUTPUT);
  digitalWrite(DFPLAYER_NEXT, HIGH);  // Estado inicial
  digitalWrite(DFPLAYER_PLAY, HIGH);  // Estado inicial

  // Conectar el control PS4
  if (PS4.begin(ps4ControllerAddress)) {
    Serial.println("Bluetooth iniciado. Control PS4 conectado.");
  } else {
    Serial.println("Error al iniciar el Bluetooth.");
  }
}

void loop() {
  if (PS4.isConnected()) {
    // Leer la posición del joystick izquierdo
    int lx = PS4.LStickX();
    int ly = PS4.LStickY();

    // Comandos al STM32 según el movimiento del joystick
    if (lx > threshold) {
      Serial1.write('R');
      Serial.println("Joystick movido a la derecha");
    } else if (lx < -threshold) {
      Serial1.write('L');
      Serial.println("Joystick movido a la izquierda");
      delay(500);
    }

    if (ly > threshold && !downCommandSent) {
      Serial1.write('D');
      Serial.println("Joystick movido hacia abajo");
      downCommandSent = true;
    } else if (ly < threshold) {
      downCommandSent = false;
    }

    // Manejo del botón X para enviar comandos al STM32
    if (PS4.Cross()) {
      if (!xButtonPressed) {
        Serial1.write('X');
        Serial.println("Botón X presionado");
        xButtonPressed = true;
        delay(500);
      }
    } else if (xButtonPressed) {
      Serial1.write('x');
      Serial.println("Botón X liberado");
      xButtonPressed = false;
    }

    // Iniciar la reproducción al presionar Share
    if (PS4.Share()) {
      if (!shareButtonPressed && !isPlaying) {
        Serial.println("Botón Share presionado. Iniciando reproducción.");
        startPlaying();  // Iniciar la reproducción
        shareButtonPressed = true;
      }
    } else if (shareButtonPressed) {
      Serial.println("Botón Share liberado.");
      shareButtonPressed = false;
    }
  }

  // Verificar si las canciones han terminado
  if (isPlaying && currentSong > totalSongs) {
    Serial.println("Todas las canciones se han reproducido.");
    isPlaying = false;  // Detener reproducción
  }

  // Recibir datos del ESP32 Esclavo y reenviarlos al STM32
  if (Serial2.available()) {
    char data = Serial2.read();
    Serial1.write(data);
    Serial.printf("Dato recibido del Esclavo: %c\n", data);
  }

  // Recibir datos del STM32 y reenviarlos al Esclavo
  if (Serial1.available()) {
    char data = Serial1.read();
    Serial2.write(data);
    Serial.printf("Dato recibido del STM32: %c\n", data);
  }

  delay(50);  // Pequeño retardo para evitar lecturas rápidas
}

// Función para iniciar la reproducción desde la primera canción
void startPlaying() {
  currentSong = 1;
  playSong(currentSong);  // Reproducir la primera canción
  isPlaying = true;  // Marcar la reproducción como activa
}

// Función para reproducir una canción usando GPIO
void playSong(int songNumber) {
  Serial.printf("Reproduciendo canción %d.\n", songNumber);

  // Enviar pulso para iniciar la reproducción
  digitalWrite(DFPLAYER_PLAY, LOW);
  delay(200);  // Pulso de 200ms
  digitalWrite(DFPLAYER_PLAY, INPUT);

  // Avanzar si no es la primera canción
  if (songNumber > 1) {
    for (int i = 1; i < songNumber; i++) {
      digitalWrite(DFPLAYER_NEXT, LOW);
      delay(200);  // Pulso de 200ms
      digitalWrite(DFPLAYER_NEXT, HIGH);
      delay(500);  // Espera entre pulsos
    }
  }
}


