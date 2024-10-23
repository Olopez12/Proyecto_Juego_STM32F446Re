/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "bitmaps.h"

#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	Appearance,
    LOADING_SCREEN,
    MAIN_MENU,
    SUBMENU,
    GAME
} GameState;

GameState game_state;

typedef enum {
    SKIN_BASE,
    SKIN_KING
} PlayerSkin;

PlayerSkin current_skin = SKIN_BASE; // Inicialmente, la skin base

typedef enum {
    ACTION_STANDING,
    ACTION_RUNNING,
    ACTION_JUMPING
} PlayerAction;

typedef struct {
    int x;
    int y;
    int length;
    int is_horizontal; // 1 para horizontal, 0 para vertical
} Line;

typedef struct {
    int x;
    int y;
    int width;
    int height;
} Rectangle;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

SPI_HandleTypeDef hpsi1;
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
UINT br; // Bytes leídos
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
char buffer[100];

#define PLAYER_WIDTH 32
#define PLAYER_HEIGHT 32
#define MAX_JUMP_CHARGE 20 // Máximo tiempo de carga del salto
#define PLAYER_WIDTH2 32
#define PLAYER_HEIGHT2 32
#define MAX_JUMP_CHARGE2 20 // Máximo tiempo de carga del salto
#define IMAGE_WIDTH  320
#define IMAGE_HEIGHT 240
#define fondo 0x9fdf
#define MAX_LINES 20
#define MAX_RECTANGLES 10
#define MAX_QUEENS 2
Line lines[MAX_LINES];
Rectangle rectangles[MAX_RECTANGLES];
Rectangle queens[MAX_QUEENS];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// Imágenes del personaje (debes asegurarte de que están correctamente definidas)
extern unsigned char jk[];   // Imagen del personaje parado
extern unsigned char jk2[];  // Imagen del personaje corriendo
extern unsigned char jk3[];  // Imagen del personaje saltando

extern unsigned char jkK[];   // Parado
extern unsigned char jkK2[];  // Corriendo
extern unsigned char jkK3[];  // Saltando


// Posición del personaje
int player_x = 160; // Posición inicial en X (centrado)
int player_y = 200; // Posición inicial en Y (cerca de la parte inferior)

// Posición del personaje 2
int player2_x = 160; // Posición inicial en X (centrado)
int player2_y = 200; // Posición inicial en Y (cerca de la parte inferior)

int queen_count = 0;



// Variables para el salto
int jump_height = 0;
int is_jumping = 0;
int jump_charge = 0;
// Variables para el salto
int jump_height2 = 0;
int is_jumping2 = 0;
int jump_charge2 = 0;
int button_pressed = 0; // 0 = no presionado, 1 = presionado
int button_pressed2 = 0; // 0 = no presionado, 1 = presionado
int line_count = 0;
int rectangle_count = 0;

char buffer[100];
char buffer2[100];
// Obtener el color de fondo de la imagen (primer píxel)

unsigned int transparent_color_jk;
unsigned int transparent_color_jk2;
unsigned int transparent_color_jk3;
unsigned int transparent_color_reina;
unsigned int transparent_color_jkK;
unsigned int transparent_color_jkK2;
unsigned int transparent_color_jkK3;

// Variables para el menú
#define MENU_OPTION_COUNT 3 // Número de opciones en el menú
#define SUBMENU_OPTION_COUNT 2 // 3 opciones de música + botón de "Back"
#define Appearence_COUNT 3 //


int menu_current_option = 0; // Opción actualmente seleccionada (0, 1, 2)
int in_menu = 1; // Indica si estamos en el menú
int submenu_current_option = 0; // Opción seleccionada en el submenú (0 a 3)
int appearence_current_option = 0; // Opción seleccionada en el submenú (0 a 3)


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
char Read_UART_Command(void);
void Clear_Player(int x, int y);
void Draw_Player(int x, int y, PlayerAction action);
void Move_Left(void);
void Move_Right(void);
void Start_Jump(void);
void Charge_Jump(void);
void Execute_Jump(void);
void Update_Jump(void);
void Clear_Player2(int x, int y);
void Draw_Player2(int x, int y, PlayerAction action);
void Move_Left2(void);
void Move_Right2(void);
void Start_Jump2(void);
void Charge_Jump2(void);
void Execute_Jump2(void);
void Update_Jump2(void);
void Initialize_Barras(void);
void transmit_uart(char *string);
void display_image_from_sd(const char *filename);
void display_image(const char *filename);
void Show_Loading_Screen(void);
void Show_Main_Menu(void);
void Handle_Jump_Command(char command);
void Draw_Menu_Selection(int option);
void Menu_Navigation(char command);
void Start_Game(void);
void Show_Options(void);
void Clear_Menu_Selection(int option);
void Exit_Game(void);
void Show_Submenu(void);
void Fill_Screen(unsigned int color);
void Draw_Submenu_Selection(int option);
void Clear_Submenu_Selection(int option);
void Submenu_Navigation(char command);
void Show_appearance(void);
void Draw_appearance_Selection(int option);
void Clear_appearance_Selection(int option);
void appearance_Navigation(char command);
void Nivel1(void);
void Play_Music1(void);
void Play_Music2(void);
void Play_Music3(void);
int Is_Hitting_Head(int x, int y);
int Is_Colliding_With_Rect_Side(int x, int y);
int Is_On_Platform(int x, int y);
int Is_Colliding_With_Vertical_Line(int x, int y);
int Check_Collision_Line(int x, int y, int w, int h, Line *line);
int Check_Collision_Rect(int x1, int y1, int w1, int h1,
                         int x2, int y2, int w2, int h2);
void Reset_Player_Position(void);
void Reset_Player_Position2(void);

int Is_On_Button(int x, int y, int width, int height);
void Open_Roof(void);
int Is_Touching_Queen(int x, int y, int width, int height);
void Check_Level_Completion(void);
void Load_Level2(void);
int Is_On_Button2(int x, int y, int width, int height);
void Open_Roof2(void);
void Check_Level_Completion2(void);
void Load_Level3(void);
void Nivel2(void);
void Nivel3(void);
void LCD_Bitmap_Transparent(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[], unsigned int transparent_color);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

    transparent_color_jk  = (jk[0] << 8) | jk[1];
    transparent_color_jk2 = (jk2[0] << 8) | jk2[1];
    transparent_color_jk3 = (jk3[0] << 8) | jk3[1];
    transparent_color_jkK  = (jkK[0] << 8) | jkK[1];
    transparent_color_jkK2 = (jkK2[0] << 8) | jkK2[1];
    transparent_color_jkK3 = (jkK3[0] << 8) | jkK3[1];
    transparent_color_reina  = (reina[0] << 8) | reina[1];


    HAL_Delay(500);
  // Inicializa la pantalla LCD
    LCD_Init();
    LCD_Clear(0xFFFF); // Limpiar pantalla en blanco
    game_state = LOADING_SCREEN;

        // Mostrar pantalla de carga
        Show_Loading_Screen();

        // Cambiar al estado del menú principal
        game_state = MAIN_MENU;

        // Mostrar menú principal
        Show_Main_Menu();

        char command;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		command = Read_UART_Command();

		    if (command != '\0') {
		        if (game_state == MAIN_MENU) {
		            Menu_Navigation(command);
		        } else if (game_state == SUBMENU) {
		            Submenu_Navigation(command);
		        } else if (game_state == Appearance){
		            appearance_Navigation(command);
		        } else if (game_state == GAME) {
		            // Manejar comandos del juego
		            switch (command) {
		                case 'L':
		                    Move_Left();
		                    break;
		                case 'R':
		                    Move_Right();
		                    break;
		                case 'C':
		                    Move_Left2();
		                    break;
		                case 'Z':
		                    Move_Right2();
		                    break;
		                case 'X':
		                    Start_Jump();
		                    break;
		                case 'x':
		                    Execute_Jump();
		                    break;
		                case 'P':
		                    Start_Jump2();
		                    break;
		                case 'p':
		                    Execute_Jump2();
		                    break;
		                default:
		                    break;
		            }
		        }
		    }

		    // Actualizar estado del juego
		    if (game_state == GAME) {
		    	// Verificar si algún jugador ha tocado la reina
		    	    if (Is_Touching_Queen(player_x, player_y, PLAYER_WIDTH, PLAYER_HEIGHT)) {
		    	        // Mostrar imagen de ganador 1
		    	        LCD_Clear(0xFFFF);
		    	        display_image_from_sd("Ganador1.txt");

		    	    }
		    	    if (Is_Touching_Queen(player2_x, player2_y, PLAYER_WIDTH2, PLAYER_HEIGHT2)) {
		    	        // Mostrar imagen de ganador 2
		    	        LCD_Clear(0xFFFF);
		    	        display_image_from_sd("Ganador2.txt");

		    	    }

		        if (is_jumping) {
		            Charge_Jump();
		        }
		        if (is_jumping2) { // Agregado para el jugador 2
		            Charge_Jump2();
		        }

		        Update_Jump();
		        Update_Jump2(); // Agregado para el jugador 2

		        // Verificar si el botón ha sido presionado y abrir el techo si es necesario
		        if (!button_pressed && Is_On_Button(player_x, player_y, PLAYER_WIDTH, PLAYER_HEIGHT)) {
		            transmit_uart("Jugador 1 presionó el botón.\n");
		            button_pressed = 1;
		            Open_Roof();
		        }

		        if (!button_pressed && Is_On_Button(player2_x, player2_y, PLAYER_WIDTH2, PLAYER_HEIGHT2)) {
		            transmit_uart("Jugador 2 presionó el botón.\n");
		            button_pressed = 1;
		            Open_Roof();
		        }

		        // Verificar si el jugador ha completado el nivel
		        if (button_pressed) {
		            Check_Level_Completion();
		        }
		        // Verificar si el botón ha sido presionado y abrir el techo si es necesario
		        if (!button_pressed2 && Is_On_Button2(player_x, player_y, PLAYER_WIDTH, PLAYER_HEIGHT)) {

		        	button_pressed2 = 1;
		        	Open_Roof2();
		        	}

		        if (!button_pressed2 && Is_On_Button2(player2_x, player2_y, PLAYER_WIDTH2, PLAYER_HEIGHT2)) {

		        	button_pressed2 = 1;
		        	Open_Roof2();
		        	}

		        		        // Verificar si el jugador ha completado el nivel
		        if (button_pressed2) {
		        	Check_Level_Completion2();
		        	}
		    }

		    HAL_Delay(60);
		    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin PB6 */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
char Read_UART_Command(void) {
    uint8_t received_char;
    if (HAL_UART_Receive(&huart3, &received_char, 1, 10) == HAL_OK) {
        return (char)received_char;
    } else {
        return '\0'; // No se recibió ningún dato
    }
}



void Clear_Player(int x, int y) {
    // Borra el área donde estaba el personaje
    FillRect(x, y, PLAYER_WIDTH, PLAYER_HEIGHT, fondo); // Color blanco de fondo
}
void Clear_Player2(int a, int b) {
    // Borra el área donde estaba el personaje
    FillRect(a, b, PLAYER_WIDTH2, PLAYER_HEIGHT2, fondo); // Color blanco de fondo
}

void Draw_Player(int x, int y, PlayerAction action) {
    unsigned char* image;
    unsigned int transparent_color;

    if (current_skin == SKIN_BASE) {
        if (action == ACTION_STANDING) {
            image = jk;
            transparent_color = transparent_color_jk;
        } else if (action == ACTION_RUNNING) {
            image = jk2;
            transparent_color = transparent_color_jk2;
        } else if (action == ACTION_JUMPING) {
            image = jk3;
            transparent_color = transparent_color_jk3;
        }
    } else if (current_skin == SKIN_KING) {
        if (action == ACTION_STANDING) {
            image = jkK;
            transparent_color = transparent_color_jkK;
        } else if (action == ACTION_RUNNING) {
            image = jkK2;
            transparent_color = transparent_color_jkK2;
        } else if (action == ACTION_JUMPING) {
            image = jkK3;
            transparent_color = transparent_color_jkK3;
        }
    }

    // Dibujar la imagen del jugador
    LCD_Bitmap_Transparent(x, y, PLAYER_WIDTH, PLAYER_HEIGHT, image, transparent_color);
}
void Draw_Player2(int a, int b, PlayerAction action) {
    unsigned char* image;
    unsigned int transparent_color;

    if (current_skin == SKIN_BASE) {
            if (action == ACTION_STANDING) {
                image = jk;
                transparent_color = transparent_color_jk;
            } else if (action == ACTION_RUNNING) {
                image = jk2;
                transparent_color = transparent_color_jk2;
            } else if (action == ACTION_JUMPING) {
                image = jk3;
                transparent_color = transparent_color_jk3;
            }
        } else if (current_skin == SKIN_KING) {
            if (action == ACTION_STANDING) {
                image = jkK;
                transparent_color = transparent_color_jkK;
            } else if (action == ACTION_RUNNING) {
                image = jkK2;
                transparent_color = transparent_color_jkK2;
            } else if (action == ACTION_JUMPING) {
                image = jkK3;
                transparent_color = transparent_color_jkK3;
            }
        }
    // Dibujar la imagen del jugador
    LCD_Bitmap_Transparent(a, b, PLAYER_WIDTH2, PLAYER_HEIGHT2, image, transparent_color);
}

void Move_Left(void) {
    int new_x = player_x - 5;
    if (new_x < 0) new_x = 0;

    if (!Is_Colliding_With_Vertical_Line(new_x, player_y) &&
        !Is_Colliding_With_Rect_Side(new_x, player_y)) {
        Clear_Player(player_x, player_y);
        player_x = new_x;
        Draw_Player(player_x, player_y, ACTION_RUNNING);
    } else {
        // No mover al jugador, pero podemos redibujarlo en su posición actual
        Draw_Player(player_x, player_y, ACTION_STANDING);
    }
    HAL_Delay(50);
}
void Move_Left2(void) {
    int new_a = player2_x - 5;
    if (new_a < 0) new_a = 0;

    if (!Is_Colliding_With_Vertical_Line(new_a, player2_y) &&
        !Is_Colliding_With_Rect_Side(new_a, player2_y)) {
        Clear_Player2(player2_x, player2_y);
        player2_x = new_a;
        Draw_Player2(player2_x, player2_y, ACTION_RUNNING);
    } else {
        // No mover al jugador, pero podemos redibujarlo en su posición actual
        Draw_Player2(player2_x, player2_y, ACTION_STANDING);
    }
    HAL_Delay(50);
}

void Move_Right(void) {
    int new_x = player_x + 5;//El jugador se ha movido 5 unidades a la derecha.
    if (new_x > (320 - PLAYER_WIDTH)) new_x = 320 - PLAYER_WIDTH;
    //Comprueba si la nueva posición new_x y la posición vertical actual player_y no causan colisiones.
    if (!Is_Colliding_With_Vertical_Line(new_x, player_y) &&
        !Is_Colliding_With_Rect_Side(new_x, player_y)) // Verifica si hay una colisión con una línea vertical en la nueva posición.
    {
        Clear_Player(player_x, player_y); // Borra el sprite del jugador de su posición actual
        player_x = new_x;
        Draw_Player(player_x, player_y, ACTION_RUNNING);
    } else {
        // No mover al jugador, pero podemos redibujarlo en su posición actual
        Draw_Player(player_x, player_y, ACTION_STANDING);
    }
    HAL_Delay(50);
}
void Move_Right2(void) {
    int new_a = player2_x + 5;
    if (new_a > (320 - PLAYER_WIDTH2)) new_a = 320 - PLAYER_WIDTH2;

    if (!Is_Colliding_With_Vertical_Line(new_a, player2_y) &&
        !Is_Colliding_With_Rect_Side(new_a, player2_y)) {
        Clear_Player2(player2_x, player2_y);
        player2_x = new_a;
        Draw_Player2(player2_x, player2_y, ACTION_RUNNING);
    } else {
        // No mover al jugador, pero podemos redibujarlo en su posición actual
        Draw_Player2(player2_x, player2_y, ACTION_STANDING);
    }
    HAL_Delay(50);
}

void Start_Jump(void) {
    if (!is_jumping) {
        jump_charge = 0;
        is_jumping = 1;
    }
}
void Start_Jump2(void) {
    if (!is_jumping2) {
        jump_charge2 = 0;
        is_jumping2 = 1;
    }
}

void Charge_Jump(void) {
    if (jump_charge < MAX_JUMP_CHARGE) {
        jump_charge++;
        // Opcionalmente, puedes mostrar una animación de carga

    }
}
void Charge_Jump2(void) {
    if (jump_charge2 < MAX_JUMP_CHARGE2) {
        jump_charge2++;
        // Opcionalmente, puedes mostrar una animación de carga

    }
}
void Execute_Jump(void) {
    if (is_jumping) {
        jump_height = jump_charge * 3; // Altura proporcional al tiempo de carga
        jump_charge = 0;
        is_jumping = 0;
    }
    Clear_Player(player_x, player_y);
    Draw_Player(player_x, player_y, ACTION_JUMPING);
}
void Execute_Jump2(void) {
    if (is_jumping2) {
        jump_height2 = jump_charge2 * 3; // Altura proporcional al tiempo de carga
        jump_charge2 = 0;
        is_jumping2 = 0;
    }
    Clear_Player2(player2_x, player2_y);
    Draw_Player2(player2_x, player2_y, ACTION_JUMPING);
}
void Update_Jump(void) {
    if (jump_height > 0) {
        // Movimiento hacia arriba
        int new_y = player_y - 4;

        // Verificar colisión hacia arriba
        if (Is_Hitting_Head(player_x, new_y)) {
            // Colisión detectada, detener el salto
            jump_height = 0;

            // Ajustar la posición para estar justo debajo de la plataforma
            int platform_y = player_y;
            const int margin = 5;

            // Encontrar la plataforma con la que chocó
            for (int i = 0; i < line_count; i++) {
                if (lines[i].is_horizontal) {
                    if ((new_y <= lines[i].y + margin) &&
                        (new_y >= lines[i].y - margin) &&
                        (player_x + PLAYER_WIDTH > lines[i].x) &&
                        (player_x < lines[i].x + lines[i].length)) {
                        platform_y = lines[i].y + margin;
                        break;
                    }
                }
            }
            for (int i = 0; i < rectangle_count; i++) {
                if ((new_y <= rectangles[i].y + rectangles[i].height + margin) &&
                    (new_y >= rectangles[i].y + rectangles[i].height - margin) &&
                    (player_x + PLAYER_WIDTH > rectangles[i].x) &&
                    (player_x < rectangles[i].x + rectangles[i].width)) {
                    platform_y = rectangles[i].y + rectangles[i].height + margin;
                    break;
                }
            }

            Clear_Player(player_x, player_y);
            player_y = platform_y;
            Draw_Player(player_x, player_y, ACTION_JUMPING);
        } else {
            // No hay colisión, continuar moviéndose hacia arriba
            Clear_Player(player_x, player_y);
            player_y = new_y;
            jump_height -= 4;
            Draw_Player(player_x, player_y, ACTION_JUMPING);
        }
    } else {
        // Verificar si el jugador está sobre una plataforma en su posición actual
        if (!Is_On_Platform(player_x, player_y)) {
            // El jugador no está sobre una plataforma, aplicar gravedad
            int new_y = player_y + 4;
            if (Is_On_Platform(player_x, new_y)) {
                // Ajustar la posición para que esté exactamente sobre la plataforma
                int platform_y = new_y;
                const int margin = 5;
                // Encontrar la plataforma en la que está apoyado
                for (int i = 0; i < line_count; i++) {
                    if (lines[i].is_horizontal) {
                        if ((new_y + PLAYER_HEIGHT >= lines[i].y - margin) &&
                            (new_y + PLAYER_HEIGHT <= lines[i].y + margin) &&
                            (player_x + PLAYER_WIDTH > lines[i].x) &&
                            (player_x < lines[i].x + lines[i].length)) {
                            platform_y = lines[i].y - PLAYER_HEIGHT;
                            break;
                        }
                    }
                }
                for (int i = 0; i < rectangle_count; i++) {
                    if ((new_y + PLAYER_HEIGHT >= rectangles[i].y - margin) &&
                        (new_y + PLAYER_HEIGHT <= rectangles[i].y + margin) &&
                        (player_x + PLAYER_WIDTH > rectangles[i].x) &&
                        (player_x < rectangles[i].x + rectangles[i].width)) {
                        platform_y = rectangles[i].y - PLAYER_HEIGHT;
                        break;
                    }
                }
                Clear_Player(player_x, player_y);
                player_y = platform_y;
                is_jumping = 0;
                jump_height = 0;
                Draw_Player(player_x, player_y, ACTION_STANDING);
            } else {
                Clear_Player(player_x, player_y);
                player_y = new_y;
                Draw_Player(player_x, player_y, ACTION_JUMPING);

                // Verificar si ha caído fuera de la pantalla
                if (player_y >= 240 - PLAYER_HEIGHT) {
                    Reset_Player_Position();
                }
            }
        } else {

            // El jugador está sobre una plataforma, no hacer nada

            Draw_Player(player_x, player_y, ACTION_STANDING);
        }
    }
}
void Update_Jump2(void) {
    if (jump_height2 > 0) {
        // Movimiento hacia arriba
        int new_b = player2_y - 4;

        // Verificar colisión hacia arriba
        if (Is_Hitting_Head(player2_x, new_b)) {
            // Colisión detectada, detener el salto
            jump_height2 = 0;

            // Ajustar la posición para estar justo debajo de la plataforma
            int platform2_y = player2_y;
            const int margin = 5;

            // Encontrar la plataforma con la que chocó
            for (int i = 0; i < line_count; i++) {
                if (lines[i].is_horizontal) {
                    if ((new_b <= lines[i].y + margin) &&
                        (new_b >= lines[i].y - margin) &&
                        (player2_x + PLAYER_WIDTH2 > lines[i].x) &&
                        (player2_x < lines[i].x + lines[i].length)) {
                        platform2_y = lines[i].y + margin;
                        break;
                    }
                }
            }
            for (int i = 0; i < rectangle_count; i++) {
                if ((new_b <= rectangles[i].y + rectangles[i].height + margin) &&
                    (new_b >= rectangles[i].y + rectangles[i].height - margin) &&
                    (player2_x + PLAYER_WIDTH2 > rectangles[i].x) &&
                    (player2_x < rectangles[i].x + rectangles[i].width)) {
                    platform2_y = rectangles[i].y + rectangles[i].height + margin;
                    break;
                }
            }

            Clear_Player2(player2_x, player2_y);
            player2_y = platform2_y;
            Draw_Player2(player2_x, player2_y, ACTION_JUMPING);
        } else {
            // No hay colisión, continuar moviéndose hacia arriba
            Clear_Player2(player2_x, player2_y);
            player2_y = new_b;
            jump_height2 -= 4;
            Draw_Player2(player2_x, player2_y, ACTION_JUMPING);
        }
    } else {
        // Verificar si el jugador está sobre una plataforma en su posición actual
        if (!Is_On_Platform(player2_x, player2_y)) {
            // El jugador no está sobre una plataforma, aplicar gravedad
            int new_b = player2_y + 4;
            if (Is_On_Platform(player2_x, new_b)) {
                // Ajustar la posición para que esté exactamente sobre la plataforma
                int platform2_y = new_b;
                const int margin = 5;
                // Encontrar la plataforma en la que está apoyado
                for (int i = 0; i < line_count; i++) {
                    if (lines[i].is_horizontal) {
                        if ((new_b + PLAYER_HEIGHT2 >= lines[i].y - margin) &&
                            (new_b + PLAYER_HEIGHT2 <= lines[i].y + margin) &&
                            (player2_x + PLAYER_WIDTH2 > lines[i].x) &&
                            (player2_x < lines[i].x + lines[i].length)) {
                            platform2_y = lines[i].y - PLAYER_HEIGHT2;
                            break;
                        }
                    }
                }
                for (int i = 0; i < rectangle_count; i++) {
                    if ((new_b + PLAYER_HEIGHT2 >= rectangles[i].y - margin) &&
                        (new_b + PLAYER_HEIGHT2 <= rectangles[i].y + margin) &&
                        (player2_x + PLAYER_WIDTH2 > rectangles[i].x) &&
                        (player2_x < rectangles[i].x + rectangles[i].width)) {
                        platform2_y = rectangles[i].y - PLAYER_HEIGHT2;
                        break;
                    }
                }
                Clear_Player2(player2_x, player2_y);
                player2_y = platform2_y;
                is_jumping2 = 0;
                jump_height2 = 0;
                Draw_Player2(player2_x, player2_y, ACTION_STANDING);
            } else {
                Clear_Player2(player2_x, player2_y);
                player2_y = new_b;
                Draw_Player2(player2_x, player2_y, ACTION_JUMPING);

                // Verificar si ha caído fuera de la pantalla
                if (player2_y >= 240 - PLAYER_HEIGHT2) {
                    Reset_Player_Position2();
                }
            }
        } else {

            // El jugador está sobre una plataforma, no hacer nada

            Draw_Player2(player2_x, player2_y, ACTION_STANDING);
        }
    }
}
void Handle_Jump_Command(char command) {
    if (command == 'X') {
        Start_Jump();
    } else if (command == 'x') {
        Execute_Jump();
    }
}
void Handle_Jump_Command2(char command) {
    if (command == 'X') {
        Start_Jump2();
    } else if (command == 'x') {
        Execute_Jump2();
    }
}



void display_image_from_sd(const char *filename) {
    char c;
    uint8_t byte_array[2];
    int byte_array_index = 0;
    uint8_t byte;

    int total_pixels = 0;

    // Montar el sistema de archivos (asegúrate de haber inicializado la SD previamente)
    fres = f_mount(&fs, "", 1);
    if (fres != FR_OK) {
        // Manejar error de montaje
        transmit_uart("Error mounting SD card.\n");
        return;
    }

    // Abrir el archivo en modo lectura
    fres = f_open(&fil, filename, FA_READ);
    if (fres != FR_OK) {
        // Manejar error de apertura
        transmit_uart("Error opening file.\n");
        f_mount(NULL, "", 1); // Desmontar el sistema de archivos
        return;
    }

    // Configurar la ventana de escritura en la LCD
    SetWindows(0, 0, IMAGE_WIDTH - 1, IMAGE_HEIGHT - 1);
    LCD_CMD(0x2C); // Comando para escribir en la memoria
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

    // Bucle para leer y procesar los datos del archivo
    while (f_read(&fil, &c, 1, &br) == FR_OK && br == 1) {
        // Saltar espacios en blanco y comas
        if (c == ' ' || c == ',' || c == '\n' || c == '\r') {
            continue;
        }

        // Verificar si el carácter es '0', indicando el inicio de un valor hexadecimal
        if (c == '0') {
            // Leer el siguiente carácter para verificar 'x' o 'X'
            char next_char;
            if (f_read(&fil, &next_char, 1, &br) != FR_OK || br != 1) break;
            if (next_char == 'x' || next_char == 'X') {
                // Leer los siguientes dos caracteres que representan el valor hexadecimal
                char hex_str[3];
                if (f_read(&fil, &hex_str[0], 1, &br) != FR_OK || br != 1) break;
                if (f_read(&fil, &hex_str[1], 1, &br) != FR_OK || br != 1) break;
                hex_str[2] = '\0';

                // Convertir la cadena hexadecimal a un valor numérico
                byte = (uint8_t)strtol(hex_str, NULL, 16);

                // Almacenar el byte en el arreglo
                byte_array[byte_array_index++] = byte;

                // Si tenemos dos bytes, los combinamos y los enviamos a la LCD
                if (byte_array_index == 2) {
                    // Enviar datos a la LCD
                    LCD_DATA(byte_array[0]);
                    LCD_DATA(byte_array[1]);

                    // Reiniciar el índice del arreglo
                    byte_array_index = 0;

                    // Incrementar el contador de píxeles procesados
                    total_pixels++;
                    if (total_pixels >= IMAGE_WIDTH * IMAGE_HEIGHT) {
                        break; // Hemos procesado todos los píxeles necesarios
                    }
                }
            } else {
                // Si no es 'x' o 'X', continuamos al siguiente carácter
                continue;
            }
        } else {
            // Si no es '0', continuamos al siguiente carácter
            continue;
        }
    }

    // Cerrar el archivo y desmontar el sistema de archivos
    f_close(&fil);
    f_mount(NULL, "", 1);

    // Finalizar la escritura en la LCD
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void LCD_Bitmap_Transparent(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[], unsigned int transparent_color) {
    LCD_CMD(0x2C); // Comando para escribir en la memoria
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

    unsigned int x2 = x + width - 1;
    unsigned int y2 = y + height - 1;
    SetWindows(x, y, x2, y2);

    unsigned int k = 0;

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            // Obtener el color del píxel actual en RGB565
            unsigned int color = (bitmap[k] << 8) | bitmap[k + 1];

            if (color != transparent_color) {
                // Dibujar el píxel si no es el color transparente
                LCD_DATA(bitmap[k]);
                LCD_DATA(bitmap[k + 1]);
            } else {
                // Dibujar el color de fondo (blanco) si es transparente
                LCD_DATA((fondo >> 8) & 0xFF); // Parte alta del color blanco
                LCD_DATA(fondo & 0xFF); // Parte baja del color blanco
            }

            k += 2; // Avanzar al siguiente píxel
        }
    }

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}
void transmit_uart(char *string) {
    HAL_UART_Transmit(&huart3, (uint8_t*)string, strlen(string), HAL_MAX_DELAY);
}

void display_image(const char *filename) {
	fres = f_mount(&fs, "/", 1);
	if (fres == FR_OK) {
	        transmit_uart("SD mounted!\n ");
	}else if(fres != FR_OK){
			transmit_uart("SD not mounted!\n ");
		}
	fres = f_open(&fil, filename, FA_READ);
	if (fres == FR_OK) {
	        transmit_uart("File opened!\n ");

	}else if(fres != FR_OK){
		transmit_uart("File not opened!\n ");
	}

	 while (f_gets(buffer, sizeof(buffer), &fil)){
		 transmit_uart(buffer);}

	//Cerrar archivo
	     fres = f_close(&fil);
	     if (fres == FR_OK){
	     	  transmit_uart("File closed!\n ");
	     }else if (fres != FR_OK){
	   	  transmit_uart("File closed!\n");
	     }

	     fres = f_mount(NULL,"",1);
	       	 if (fres == FR_OK){
	       	   transmit_uart("Micro SD card is unmounteD!\n ");
	       	 }else if (fres != FR_OK){
	       	  	transmit_uart("Micro SD card's unmount error!\n");
	       	 }


}

void Show_Loading_Screen(void) {
    // Mostrar imagen de carga desde "1.txt"
    display_image_from_sd("2.txt");
    HAL_Delay(2000); // Esperar 4 segundos
    FillRect(104, 210, 5, 8,0x07eb);
    FillRect(106, 207, 19, 17,0x07eb);
    HAL_Delay(1000); // Esperar 4 segundos
    FillRect(123, 207, 17, 17,0x07eb);

    HAL_Delay(1000); // Esperar 4 segundos
    FillRect(140, 207, 30, 17,0x07eb);
    HAL_Delay(500); // Esperar 4 segundos
    FillRect(157, 207, 63, 17,0x07eb);
    HAL_Delay(1500); // Esperar 4 segundos
    LCD_Clear(0xFFFF);
}

void Show_Main_Menu(void) {
    // Mostrar imagen del menú desde "Menu.txt"
    display_image_from_sd("Menu.txt");

    // Dibujar los botones del menú (si es necesario)
    // Rect(100, 85, 120, 40, 0xFFFF);
     //Rect(100, 130, 120, 35, 0xFFFF);
     //Rect(100, 172, 120, 35, 0xFFFF);
    // Inicializar la opción seleccionada
        menu_current_option = 0;
    // Dibujar la selección inicial
    Draw_Menu_Selection(menu_current_option);
}
void Draw_Menu_Selection(int option) {
    // Posiciones Y de las opciones del menú
    int menu_positions[MENU_OPTION_COUNT] = {85, 130, 172};

    // Dibujar un rectángulo transparente sobre la opción seleccionada
    // Por ejemplo, un rectángulo rojo semi-transparente
    int x = 100;
    int y = menu_positions[option];
    int width = 120;
    int height = (option == 0) ? 40 : 35; // Altura dependiendo de la opción

    // Dibujar el rectángulo de selección
    // Puedes usar una función que permita transparencia o un color que destaque
    Rect(x, y, width, height, 0xF800); // Rojo brillante
}

void Show_Options(void) {
   /* // Mostrar pantalla de opciones
	LCD_Clear(0xFFFF);
	display_image_from_sd("MenuMusica.txt");
	//Posicion del boton 1
	Rect(100, 72, 120, 37, 0xf8e0);
	//Posicion del boton 2
	Rect(100, 116, 120, 37, 0xf8e0);
	//Posicion del boton 3
	Rect(100, 161, 120, 36, 0xf8e0);
	//Posicion del boton bacK
	Rect(139, 210, 43, 23, 0xf8e0);*/
}
void Start_Game(void) {
    // Limpiar pantalla
    LCD_Clear(0xFFFF);
    Fill_Screen(fondo);
    // Inicializar posición del jugador
    player_x = 150;
    player_y = 174;
    player2_x = 210;
    player2_y = 174;
   //Obstaculos y decoracion nivel1

    Nivel1();
    // Dibujar personaje inicial con l skin seleccionada
    Draw_Player(player_x, player_y, ACTION_STANDING);
    Draw_Player2(player2_x, player2_y, ACTION_STANDING);
    // Inicializar otras variables si es necesario
}

void Menu_Navigation(char command) {
    if (command == 'U') {

        menu_current_option = (menu_current_option - 1 + MENU_OPTION_COUNT) % MENU_OPTION_COUNT;
        Draw_Menu_Selection(menu_current_option);
        Clear_Menu_Selection(menu_current_option);
    } else if (command == 'D') {

        menu_current_option = (menu_current_option + 1) % MENU_OPTION_COUNT;
        Draw_Menu_Selection(menu_current_option);
        Clear_Menu_Selection(menu_current_option);
    } else if (command == 'X' || command == 'x') {
        if (menu_current_option == 0) {
            game_state = GAME;
            Start_Game();
        } else if (menu_current_option == 1) {
            game_state = SUBMENU;
            Show_Submenu();
        } else if (menu_current_option == 2) {
        	game_state = Appearance;
        	Show_appearance();
        }
    }
}

void Clear_Menu_Selection(int option){
    if (option == 0){
        // Cuando la opción seleccionada es 0, borramos las opciones 1 y 2
        Rect(100, 130, 120, 35, 0xFFFF); // Borra opción 1
        Rect(100, 172, 120, 35, 0xFFFF); // Borra opción 2
    } else if(option == 1){
        // Cuando la opción seleccionada es 1, borramos las opciones 0 y 2
        Rect(100, 85, 120, 40, 0xFFFF);  // Borra opción 0
        Rect(100, 172, 120, 35, 0xFFFF); // Borra opción 2
    } else if (option == 2){
        // Cuando la opción seleccionada es 2, borramos las opciones 0 y 1
        Rect(100, 85, 120, 40, 0xFFFF);  // Borra opción 0
        Rect(100, 130, 120, 35, 0xFFFF); // Borra opción 1
    }
}



void Exit_Game(void) {
    // Puedes reiniciar el sistema o mostrar un mensaje
    transmit_uart("Exiting game...\n");
    HAL_Delay(1000);
    NVIC_SystemReset(); // Reiniciar el microcontrolador
}

/**
 * @brief Pinta completamente la pantalla LCD con el color especificado.
 *
 * @param color El color con el que se llenará la pantalla en formato RGB565.
 */
void Fill_Screen(unsigned int color) {
    // Configurar la ventana para cubrir toda la pantalla
    SetWindows(0, 0, 319, 239); // Coordenadas de la pantalla ILI9341 (320x240)

    // Enviar el comando de escritura en memoria
    LCD_CMD(0x2C); // Comando de memoria para escribir datos de píxeles

    // Seleccionar el registro de datos para enviar colores
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);

    // Activar la selección de chip
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

    // Calcular el número total de píxeles
    const int total_pixels = 320 * 240;

    // Enviar los datos de color para cada píxel
    for (int i = 0; i < total_pixels; i++) {
        LCD_DATA((color >> 8) & 0xFF); // Enviar el byte alto del color
        LCD_DATA(color & 0xFF);        // Enviar el byte bajo del color
    }

    // Desactivar la selección de chip
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void Show_Submenu(void) {
    // Mostrar pantalla de opciones
    LCD_Clear(0xFFFF);
    display_image_from_sd("MenuMusica.txt");

    // Inicializar la opción seleccionada
    submenu_current_option = 0;

    // Dibujar la selección inicial
    Draw_Submenu_Selection(submenu_current_option);
}

void Draw_Submenu_Selection(int option) {
    // Posiciones Y de las opciones del submenú
    int submenu_positions[SUBMENU_OPTION_COUNT] = {72, 116};

    int x, y, width, height;

    if (option == 0) {
        x = 100;
        y = submenu_positions[option];
        width = 120;
        height = 37;
    } else if (option == 1) {
        x = 100;
        y = submenu_positions[option];
        width = 120;
        height = 37;
    }

    // Dibujar el rectángulo de selección
    Rect(x, y, width, height, 0xF800); // Rojo brillante
}


void Clear_Submenu_Selection(int option) {
    int submenu_positions[SUBMENU_OPTION_COUNT] = {72, 116};

    int x, y, width, height;

    if (option == 0) {
        x = 100;
        y = submenu_positions[option];
        width = 120;
        height = 37;
    } else if (option == 1) {
        x = 100;
        y = submenu_positions[option];
        width = 120;
        height = 37;
    }

    // Borrar el rectángulo de selección
    Rect(x, y, width, height, 0xFFFF); // Asumiendo fondo blanco
}

void Submenu_Navigation(char command) {
    if (command == 'U') {
        Clear_Submenu_Selection(submenu_current_option);
        submenu_current_option = (submenu_current_option - 1 + SUBMENU_OPTION_COUNT) % SUBMENU_OPTION_COUNT;
        Draw_Submenu_Selection(submenu_current_option);
    } else if (command == 'D') {
        Clear_Submenu_Selection(submenu_current_option);
        submenu_current_option = (submenu_current_option + 1) % SUBMENU_OPTION_COUNT;
        Draw_Submenu_Selection(submenu_current_option);
    } else if (command == 'X' || command == 'x') {
        if (submenu_current_option == 0) {
            // Acción para la primera opción de música
            Play_Music1();
        } else if (submenu_current_option == 1) {
            // Acción para la segunda opción de música
        	// Botón de "Back" para regresar al menú principal
        	game_state = MAIN_MENU;
        	LCD_Clear(0xFFFF);
        	Show_Main_Menu();
        }
    }
}

void Show_appearance(void){// Mostrar pantalla de opciones
    LCD_Clear(0xFFFF);
    display_image_from_sd("MenuApp.txt");

    // Inicializar la opción seleccionada
    appearence_current_option = 0;

    // Dibujar la selección inicial
    Draw_appearance_Selection(appearence_current_option);
}

void Draw_appearance_Selection(int option){
	// Posiciones Y de las opciones del submenú
	    int app_positions[Appearence_COUNT] = {61, 61, 201};

	    int x, y, width, height;

	    if (option == 0) {
	        x = 93;
	        y = app_positions[option];
	        width = 55;
	        height = 136;
	    } else if (option == 1) {
	        x = 171;
	        y = app_positions[option];
	        width = 55;
	        height = 136;
	    } else if (option == 2) {
	        x = 152;
	        y = app_positions[option];
	        width = 19;
	        height = 30;
	    }

	    // Dibujar el rectángulo de selección
	    Draw_Rect_Thick(x, y, width, height, 0xF800,3); // Rojo brillante
}

void Clear_appearance_Selection(int option){

	int app_positions[Appearence_COUNT] = {61, 61, 201};

	    int x, y, width, height;

	    if (option == 0) {
	        x = 93;
	        y = app_positions[option];
	        width = 55;
	        height = 136;
	    } else if (option == 1) {
	        x = 171;
	        y = app_positions[option];
	        width = 55;
	        height = 136;
	    } else if (option == 2) {
	        x = 152;
	        y = app_positions[option];
	        width = 19;
	        height = 30;
	    }

	    // Borrar el rectángulo de selección
	    Draw_Rect_Thick(x, y, width, height, 0x948f,3); // Rojo brillante
}

void appearance_Navigation(char command){
    if (command == 'U') {
        Clear_appearance_Selection(appearence_current_option); // Borra la selección actual
        appearence_current_option = (appearence_current_option - 1 + Appearence_COUNT) % Appearence_COUNT;
        Draw_appearance_Selection(appearence_current_option);   // Dibuja la nueva selección
    } else if (command == 'D') {
        Clear_appearance_Selection(appearence_current_option); // Borra la selección actual
        appearence_current_option = (appearence_current_option + 1) % Appearence_COUNT;
        Draw_appearance_Selection(appearence_current_option);   // Dibuja la nueva selección
    } else if (command == 'X' || command == 'x') {
        if (appearence_current_option == 0) {
            // Seleccionar la skin base
            current_skin = SKIN_BASE;
            transmit_uart("Skin base seleccionada.\n");
        } else if (appearence_current_option == 1) {
            // Seleccionar la skin 'king'
            current_skin = SKIN_KING;
            transmit_uart("Skin king seleccionada.\n");
        } else if (appearence_current_option == 2) {
            // Regresar al menú principal
            game_state = MAIN_MENU;
            LCD_Clear(0xFFFF);
            Show_Main_Menu();
        }
    }
}



void Nivel1(void){


	 // Reiniciar contadores
	    line_count = 0;
	    rectangle_count = 0;

	    // Agregar líneas horizontales
	    lines[line_count++] = (Line){80, 207, 240, 1}; // Línea horizontal larga
	    lines[line_count++] = (Line){0, 138, 40, 1};   // Línea horizontal corta
	    lines[line_count++] = (Line){40, 162, 40, 1};  // Otra línea horizontal

	    // Agregar líneas verticales
	    lines[line_count++] = (Line){40, 138, 34, 0};  // Línea vertical izquierda
	    lines[line_count++] = (Line){80, 162, 43, 0};  // Línea vertical derecha
	    lines[line_count++] = (Line){280, 0, 206, 0};  // Línea vertical derecha

	    // Agregar rectángulos (plataformas)
	    rectangles[rectangle_count++] = (Rectangle){170, 65, 20, 22};
	    rectangles[rectangle_count++] = (Rectangle){110, 75, 20, 22};

	    rectangles[rectangle_count++] = (Rectangle){240, 150, 40, 22};
	    rectangles[rectangle_count++] = (Rectangle){260, 147, 6, 3};

	    rectangles[rectangle_count++] = (Rectangle){0, 0, 160, 22};
	    rectangles[rectangle_count++] = (Rectangle){55, 60, 20, 22};
	    rectangles[rectangle_count++] = (Rectangle){200, 50, 20, 22};

	    rectangles[rectangle_count++] = (Rectangle){260, 0, 60, 22};
	    rectangles[rectangle_count++] = (Rectangle){0, 80, 20, 22};


	LCD_Bitmap(0, 206, 40, 34, dec);
	LCD_Bitmap(40, 206, 40, 34, dec);
	LCD_Bitmap(80, 206, 40, 34, dec);
	LCD_Bitmap(120, 206, 40, 34, dec);
	LCD_Bitmap(160, 206, 40, 34, dec);
	LCD_Bitmap(200, 206, 40, 34, dec);
	LCD_Bitmap(240, 206, 40, 34, dec);
	LCD_Bitmap(280, 206, 40, 34, dec);

	LCD_Bitmap(0, 185, 40, 34, dec);
	LCD_Bitmap(40, 185, 40, 34, dec);

	LCD_Bitmap(0, 163, 40, 34, dec);
	LCD_Bitmap(40, 163, 40, 34, dec);

	LCD_Bitmap(280, 185, 40, 34, dec);
	LCD_Bitmap(280, 163, 40, 34, dec);

	LCD_Bitmap(280, 145, 40, 34, dec);
	LCD_Bitmap(280, 125, 40, 34, dec);

	LCD_Bitmap(280, 107, 40, 34, dec);
	LCD_Bitmap(280, 82, 40, 34, dec);

	LCD_Bitmap(280, 64, 40, 34, dec);
	LCD_Bitmap(280, 45, 40, 34, dec);
	LCD_Bitmap(280, 20, 40, 34, dec);
	LCD_Bitmap(280, 0, 40, 34, dec);

	LCD_Bitmap_Transparent(260, 147, 6, 3, button, fondo);
	LCD_Bitmap(260, 150, 20, 22, dec2);
	LCD_Bitmap(240, 150, 20, 22, dec2);

	LCD_Bitmap(280, 30, 22, 23, dec3);
	LCD_Bitmap(280, 100, 22, 23, dec3);

	LCD_Bitmap(0, 139, 40, 34, dec);

	LCD_Bitmap(170, 65, 20, 22, dec2);
	LCD_Bitmap(200, 50, 20, 22, dec2);
	LCD_Bitmap(110, 75, 20, 22, dec2);
	LCD_Bitmap(55, 60, 20, 22, dec2);
	LCD_Bitmap(0, 80, 20, 22, dec2);


	//TECHO CERRADO
	LCD_Bitmap(0, 0,20, 22, dec2);
	LCD_Bitmap(20, 0, 20, 22, dec2);
	LCD_Bitmap(40, 0, 20, 22, dec2);
	LCD_Bitmap(60, 0, 20, 22, dec2);
	LCD_Bitmap(80, 0, 20, 22, dec2);
	LCD_Bitmap(100, 0, 20, 22, dec2);
	LCD_Bitmap(120, 0, 20, 22, dec2);
	LCD_Bitmap(140, 0, 20, 22, dec2);
	LCD_Bitmap(160, 0, 20, 22, dec2);
	LCD_Bitmap(180, 0, 20, 22, dec2);
	LCD_Bitmap(200, 0, 20, 22, dec2);
	LCD_Bitmap(220, 0, 20, 22, dec2);
	LCD_Bitmap(240, 0, 20, 22, dec2);
	LCD_Bitmap(260, 0, 20, 22, dec2);
	LCD_Bitmap(280, 0, 20, 22, dec2);
	LCD_Bitmap(300, 0, 20, 22, dec2);
//FIN DE TECHO CERRADO



}

void Nivel2(void){


	 // Reiniciar contadores
	line_count = 0;
	rectangle_count = 0;

	lines[line_count++] = (Line){20, 217, 160, 1}; // Línea horizontal larga
	lines[line_count++] = (Line){260, 217, 20, 1}; // Línea horizontal larga

	lines[line_count++] = (Line){40, 0, 217, 0}; // Línea V larga
	lines[line_count++] = (Line){280, 0, 217, 0}; // Línea V larga

	LCD_Bitmap(0, 218, 20, 22, dec2);
	LCD_Bitmap(20, 218, 20, 22, dec2);
	LCD_Bitmap(40, 218, 20, 22, dec2);
	LCD_Bitmap(60, 218, 20, 22, dec2);
	LCD_Bitmap(80, 218, 20, 22, dec2);
	LCD_Bitmap(100, 218, 20, 22, dec2);
	LCD_Bitmap(120, 218, 20, 22, dec2);
	LCD_Bitmap(140, 218, 20, 22, dec2);
	LCD_Bitmap(160, 218, 20, 22, dec2);

	LCD_Bitmap(260, 218, 20, 22, dec2);
	LCD_Bitmap(280, 218, 20, 22, dec2);
	LCD_Bitmap(300, 218, 20, 22, dec2);

	LCD_Bitmap(40, 0, 20, 22, dec2);
	LCD_Bitmap(60, 0, 20, 22, dec2);
	LCD_Bitmap(80, 0, 20, 22, dec2);
	LCD_Bitmap(100, 0, 20, 22, dec2);
	LCD_Bitmap(120, 0, 20, 22, dec2);
	LCD_Bitmap(140, 0, 20, 22, dec2);
	LCD_Bitmap(160, 0, 20, 22, dec2);
	LCD_Bitmap(180, 0, 20, 22, dec2);
	LCD_Bitmap(200, 0, 20, 22, dec2);
	LCD_Bitmap(220, 0, 20, 22, dec2);
	LCD_Bitmap(240, 0, 20, 22, dec2);
	LCD_Bitmap(260, 0, 20, 22, dec2);

	//Primer torre

	LCD_Bitmap(0, 185, 40, 34, dec);
	LCD_Bitmap(0, 163, 40, 34, dec);

	LCD_Bitmap(0, 145, 40, 34, dec);
	LCD_Bitmap(0, 125, 40, 34, dec);

	LCD_Bitmap(0, 107, 40, 34, dec);
	LCD_Bitmap(0, 82, 40, 34, dec);

	LCD_Bitmap(0, 64, 40, 34, dec);
	LCD_Bitmap(0, 45, 40, 34, dec);
	LCD_Bitmap(0, 20, 40, 34, dec);
	LCD_Bitmap(0, 0, 40, 34, dec);
	LCD_Bitmap(17, 85, 22, 23, dec3);
	LCD_Bitmap(17, 170, 22, 23, dec3);

//Sgunda torre

	LCD_Bitmap(280, 185, 40, 34, dec);
	LCD_Bitmap(280, 163, 40, 34, dec);

	LCD_Bitmap(280, 145, 40, 34, dec);
	LCD_Bitmap(280, 125, 40, 34, dec);

	LCD_Bitmap(280, 107, 40, 34, dec);
	LCD_Bitmap(280, 82, 40, 34, dec);

	LCD_Bitmap(280, 64, 40, 34, dec);
	LCD_Bitmap(280, 45, 40, 34, dec);
	LCD_Bitmap(280, 20, 40, 34, dec);
	LCD_Bitmap(280, 0, 40, 34, dec);
	LCD_Bitmap(280, 30, 22, 23, dec3);
	LCD_Bitmap(280, 100, 22, 23, dec3);



	rectangles[rectangle_count++] = (Rectangle){78, 81, 20, 22};
	rectangles[rectangle_count++] = (Rectangle){40, 140, 20, 22};
	rectangles[rectangle_count++] = (Rectangle){260, 110, 20, 22};
	rectangles[rectangle_count++] = (Rectangle){260, 0, 60, 22};
	rectangles[rectangle_count++] = (Rectangle){190, 160, 20, 22};
	rectangles[rectangle_count++] = (Rectangle){140, 79, 20, 22};
	rectangles[rectangle_count++] = (Rectangle){74, 180, 20, 22};
	rectangles[rectangle_count++] = (Rectangle){220, 150, 20, 22};
	rectangles[rectangle_count++] = (Rectangle){160, 67, 20, 22};


	LCD_Bitmap(78, 81, 20, 22, dec2);
	LCD_Bitmap(40, 140, 20, 22, dec2);

	LCD_Bitmap_Transparent(260, 107, 6, 3, button, fondo);
	LCD_Bitmap(260, 110, 20, 22, dec2);
	LCD_Bitmap(190, 160, 20, 22, dec2);

	LCD_Bitmap(140, 79, 20, 22, dec2);

	LCD_Bitmap(74, 180, 20, 22, dec2);
	LCD_Bitmap(220, 150, 20, 22, dec2);
	LCD_Bitmap(160, 67, 20, 22, dec2);


	/*
	LCD_Bitmap(300, 218, 20, 22, dec2);
	LCD_Bitmap(300, 218, 20, 22, dec2);
	LCD_Bitmap(300, 218, 20, 22, dec2);

*/


}

void Nivel3(void){
	// Reiniciar contadores
		line_count = 0;
		rectangle_count = 0;
		queen_count = 0; // Reiniciar el contador de reinas

		lines[line_count++] = (Line){0, 218, 320, 1}; // Línea horizontal larga
		lines[line_count++] = (Line){140, 0, 320, 0}; // Línea horizontal larga
		lines[line_count++] = (Line){180, 0, 320, 0}; // Línea horizontal larga


	//Primer torre

		LCD_Bitmap(140, 185, 40, 34, dec);
		LCD_Bitmap(140, 163, 40, 34, dec);

		LCD_Bitmap(140, 145, 40, 34, dec);
		LCD_Bitmap(140, 125, 40, 34, dec);

		LCD_Bitmap(140, 107, 40, 34, dec);
		LCD_Bitmap(140, 82, 40, 34, dec);

		LCD_Bitmap(140, 64, 40, 34, dec);
		LCD_Bitmap(140, 45, 40, 34, dec);
		LCD_Bitmap(140, 20, 40, 34, dec);
		LCD_Bitmap(140, 0, 40, 34, dec);
		LCD_Bitmap(140, 85, 22, 23, dec3);
		LCD_Bitmap(140, 170, 22, 23, dec3);

		LCD_Bitmap_Transparent(50, 30, 28, 28, reina, transparent_color_reina);
		LCD_Bitmap_Transparent(240, 30, 28,28, reina, transparent_color_reina);


		rectangles[rectangle_count++] = (Rectangle){55, 58, 20, 22};
		queens[queen_count++] = (Rectangle){50, 30, 28, 28};
		queens[queen_count++] = (Rectangle){240, 30, 28, 28};
		LCD_Bitmap(55, 58, 20, 22, dec2);
		rectangles[rectangle_count++] = (Rectangle){245, 58, 20, 22};

		LCD_Bitmap(245, 58, 20, 22, dec2);


		LCD_Bitmap(0, 218, 20, 22, dec2);
		LCD_Bitmap(20, 218, 20, 22, dec2);
		LCD_Bitmap(40, 218, 20, 22, dec2);
		LCD_Bitmap(60, 218, 20, 22, dec2);
		LCD_Bitmap(80, 218, 20, 22, dec2);
		LCD_Bitmap(100, 218, 20, 22, dec2);
		LCD_Bitmap(120, 218, 20, 22, dec2);
		LCD_Bitmap(140, 218, 20, 22, dec2);
		LCD_Bitmap(160, 218, 20, 22, dec2);
		LCD_Bitmap(180, 218, 20, 22, dec2);
		LCD_Bitmap(200, 218, 20, 22, dec2);
		LCD_Bitmap(220, 218, 20, 22, dec2);
		LCD_Bitmap(240, 218, 20, 22, dec2);
		LCD_Bitmap(260, 218, 20, 22, dec2);
		LCD_Bitmap(280, 218, 20, 22, dec2);
		LCD_Bitmap(300, 218, 20, 22, dec2);

		//escaleras primer player

		LCD_Bitmap(0, 185, 40, 34, dec);
		rectangles[rectangle_count++] = (Rectangle){0, 185, 40, 34};
		LCD_Bitmap(75, 160, 20, 22, dec2);
		rectangles[rectangle_count++] = (Rectangle){75, 160, 20, 22};
		LCD_Bitmap(120, 120, 20, 22, dec2);
		rectangles[rectangle_count++] = (Rectangle){120, 120, 20, 22};
		LCD_Bitmap(65, 68, 20, 22, dec2);
		rectangles[rectangle_count++] = (Rectangle){65, 68, 20, 22};


		//Escaleras segundo player
		LCD_Bitmap(280, 185, 40, 34, dec);
		rectangles[rectangle_count++] = (Rectangle){280, 185, 40, 34};
		LCD_Bitmap(228, 160, 20, 22, dec2);
		rectangles[rectangle_count++] = (Rectangle){228, 160, 20, 22};
		LCD_Bitmap(180, 120, 20, 22, dec2);
		rectangles[rectangle_count++] = (Rectangle){180, 120, 20, 22};
		LCD_Bitmap(235, 68, 20, 22, dec2);
		rectangles[rectangle_count++] = (Rectangle){235, 68, 20, 22};


}






int Check_Collision_Rect(int x1, int y1, int w1, int h1,
                         int x2, int y2, int w2, int h2) {
    return (x1 < x2 + w2) && (x1 + w1 > x2) &&
           (y1 < y2 + h2) && (y1 + h1 > y2);
}

int Check_Collision_Line(int x, int y, int w, int h, Line *line) {
    if (line->is_horizontal) {
        return (y + h >= line->y) && (y <= line->y) &&
               (x + w > line->x) && (x < line->x + line->length);
    } else {
        return (x + w >= line->x) && (x <= line->x) &&
               (y + h > line->y) && (y < line->y + line->length);
    }
}

int Is_Colliding_With_Vertical_Line(int x, int y) {
    for (int i = 0; i < line_count; i++) {
        if (!lines[i].is_horizontal) {
            if (Check_Collision_Line(x, y, PLAYER_WIDTH, PLAYER_HEIGHT, &lines[i])) {
                return 1;
            }
        }
    }
    return 0;
}

int Is_Hitting_Head(int x, int y) {
    const int margin = 5;
    // Verificar colisión con la parte inferior de líneas horizontales
    for (int i = 0; i < line_count; i++) {
        if (lines[i].is_horizontal) {
            if ((y <= lines[i].y + margin) &&
                (y >= lines[i].y - margin) &&
                (x + PLAYER_WIDTH > lines[i].x) &&
                (x < lines[i].x + lines[i].length)) {
                return 1;
            }
        }
    }
    // Verificar colisión con la parte inferior de rectángulos
    for (int i = 0; i < rectangle_count; i++) {
        if ((y <= rectangles[i].y + rectangles[i].height + margin) &&
            (y >= rectangles[i].y + rectangles[i].height - margin) &&
            (x + PLAYER_WIDTH > rectangles[i].x) &&
            (x < rectangles[i].x + rectangles[i].width)) {
            return 1;
        }
    }
    return 0;
}

int Is_On_Platform(int x, int y) {
    const int margin = 5;
    for (int i = 0; i < line_count; i++) {
        if (lines[i].is_horizontal) {
            if ((y + PLAYER_HEIGHT >= lines[i].y - margin) &&
                (y + PLAYER_HEIGHT <= lines[i].y + margin) &&
                (x + PLAYER_WIDTH > lines[i].x) &&
                (x < lines[i].x + lines[i].length)) {
                return 1;
            }
        }
    }
    for (int i = 0; i < rectangle_count; i++) {
        if ((y + PLAYER_HEIGHT >= rectangles[i].y - margin) &&
            (y + PLAYER_HEIGHT <= rectangles[i].y + margin) &&
            (x + PLAYER_WIDTH > rectangles[i].x) &&
            (x < rectangles[i].x + rectangles[i].width)) {
            return 1;
        }
    }
    return 0;
}


int Is_Colliding_With_Rect_Side(int x, int y) {
    for (int i = 0; i < rectangle_count; i++) {
        if (Check_Collision_Rect(x, y, PLAYER_WIDTH, PLAYER_HEIGHT,
                                 rectangles[i].x, rectangles[i].y, rectangles[i].width, rectangles[i].height)) {
            if ((x + PLAYER_WIDTH <= rectangles[i].x + 5) ||
                (x >= rectangles[i].x + rectangles[i].width - 5)) {
                return 1;
            }
        }
    }
    return 0;
}

void Reset_Player_Position(void) {
    Clear_Player(player_x, player_y);
    player_x = 123;
    player_y = 174;
    is_jumping = 0;
    jump_height = 0;
    Draw_Player(player_x, player_y, ACTION_STANDING);
}
void Reset_Player_Position2(void) {
    Clear_Player2(player2_x, player2_y);
    player2_x = 93;
    player2_y = 174;
    is_jumping2 = 0;
    jump_height2 = 0;
    Draw_Player2(player2_x, player2_y, ACTION_STANDING);
}

int Is_On_Button(int x, int y, int width, int height) {
    // Coordenadas y dimensiones del botón
    int button_x = 260;
    int button_y = 140;
    int button_width = 6;
    int button_height = 3;

    // Verificar colisión entre el jugador y el botón
    if (Check_Collision_Rect(x, y, width, height,
                             button_x, button_y, button_width, button_height)) {
        return 1; // El jugador está sobre el botón
    }
    return 0; // El jugador no está sobre el botón
}

int Is_On_Button2(int x, int y, int width, int height) {
    // Coordenadas y dimensiones del botón
    int button_x = 260;
    int button_y = 100;
    int button_width = 6;
    int button_height = 3;

    // Verificar colisión entre el jugador y el botón
    if (Check_Collision_Rect(x, y, width, height,
                             button_x, button_y, button_width, button_height)) {
        return 1; // El jugador está sobre el botón
    }
    return 0; // El jugador no está sobre el botón
}

void Open_Roof(void) {
    transmit_uart("El techo se está abriendo.\n");
    // Borrar el techo cerrado dibujando el fondo sobre él
    for (int x = 160; x <= 260; x += 20) {
        FillRect(x, 0, 20, 22, fondo);
    }

}

void Open_Roof2(void) {
    transmit_uart("El techo se está abriendo.\n");
    // Borrar el techo cerrado dibujando el fondo sobre él
    for (int x = 120; x <= 200; x += 20) {
        FillRect(x, 0, 20, 22, fondo);
    }

}
void Check_Level_Completion(void) {
    if (player_y <= 0) {
        // El jugador ha llegado al tope de la pantalla, pasar al siguiente nivel
        Load_Level2();
    }
    if (player2_y <= 0) {
            // El jugador ha llegado al tope de la pantalla, pasar al siguiente nivel
            Load_Level2();
        }

}

void Check_Level_Completion2(void) {
    if (player2_y <= 0) {
        // El jugador ha llegado al tope de la pantalla, pasar al siguiente nivel
        Load_Level3();
    }
    if (player_y <= 0) {
            // El jugador ha llegado al tope de la pantalla, pasar al siguiente nivel
            Load_Level3();
        }

}

void Load_Level2(void) {
    // Limpiar pantalla
    LCD_Clear(0xFFFF);
    Fill_Screen(fondo);

    // Reiniciar posición de los jugadores
    player_x = 100; // Posición inicial en X en el Nivel 2 para el jugador 1
    player_y = 185; // Posición inicial en Y en el Nivel 2 para el jugador 1

    player2_x = 135; // Posición inicial en X en el Nivel 2 para el jugador 2
    player2_y = 185; // Posición inicial en Y en el Nivel 2 para el jugador 2

    // Reiniciar variables del jugador 1
    is_jumping = 0;
    jump_height = 0;

    // Reiniciar variables del jugador 2
    is_jumping2 = 0;
    jump_height2 = 0;

    // Reiniciar estado del botón
    button_pressed = 0;

    // Cargar el Nivel 2
    Nivel2();

    // Dibujar los jugadores
    Draw_Player(player_x, player_y, ACTION_STANDING);
    Draw_Player2(player2_x, player2_y, ACTION_STANDING);

}

void Load_Level3(void){

	LCD_Clear(0xFFFF);
	Fill_Screen(fondo);
	 // Reiniciar posición de los jugadores
	player_x = 41; // Posición inicial en X en el Nivel 2 para el jugador 1
	player_y = 	170; // Posición inicial en Y en el Nivel 2 para el jugador 1

	player2_x = 248; // Posición inicial en X en el Nivel 2 para el jugador 2
	player2_y = 170; // Posición inicial en Y en el Nivel 2 para el jugador 2
	    // Reiniciar variables del jugador 1
	 is_jumping = 0;
	 jump_height = 0;

	  // Reiniciar variables del jugador 2
	  is_jumping2 = 0;
	  jump_height2 = 0;

	// Reiniciar estado del botón
	   button_pressed = 0;
	   Nivel3();

}

void Play_Music1(void) {
    // Código para reproducir la música 1
}

void Play_Music2(void) {
    // Código para reproducir la música 2
}

void Play_Music3(void) {
    // Código para reproducir la música 3
}

int Is_Touching_Queen(int x, int y, int width, int height) {
    for (int i = 0; i < queen_count; i++) {
        if (Check_Collision_Rect(x, y, width, height,
                                 queens[i].x, queens[i].y, queens[i].width, queens[i].height)) {
            return 1; // El jugador está tocando la reina
        }
    }
    return 0; // El jugador no está tocando ninguna reina
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER COD4E END 6 */
}
#endif /* USE_FULL_ASSERT */
