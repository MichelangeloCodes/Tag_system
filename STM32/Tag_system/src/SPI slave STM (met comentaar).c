#include "main.h"

// SPI handler
SPI_HandleTypeDef hspi1;

// SPI buffer voor inkomende data
uint8_t spi_buffer = 0;

// Te verzenden data (voorbeeld)
uint8_t tx_data[] = { 0xAA };

// Functiedeclaraties
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

int main(void)
{
    HAL_Init();                     // HAL lib initialiseren
    SystemClock_Config();          // Systeemklok instellen
    MX_GPIO_Init();                // GPIO pinnen instellen
    MX_SPI1_Init();                // SPI1 instellen

    // Zorg dat CS (chip select) standaard hoog is
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

    // Verstuur initiële waarde naar RPI via SPI
    HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data), 1000);

    while (1)
    {
        // === ONTVANGEN VAN SPI-DATA VANUIT DE RPI ===
        if (HAL_SPI_Receive(&hspi1, &spi_buffer, 1, 1000) == HAL_OK)
        {
            // === HIER KUN JE SPI-GEGEVENS GEBRUIKEN VOOR REGELING ===
            // Bijvoorbeeld een PID-regelaar:
            /*
            float setpoint = spi_buffer; // Bijv. ingestelde waarde via RPI
            float measured_value = ...   // Meetwaarde van sensor
            float output = PID_Compute(setpoint, measured_value);
            */

            // Voorbeeld: controleer of waarde geldig is (bijv. max 60 sec)
            if (spi_buffer > 0 && spi_buffer <= 60)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // LED aan
                HAL_Delay(spi_buffer * 1000);                          // wacht N seconden
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // LED uit
            }

            // === TERUGSTUREN VAN SPI-DATA NAAR DE RPI (OPTIONEEL) ===
            // Bijvoorbeeld status terugsturen:
            uint8_t response = 0x55; // voorbeeldwaarde
            HAL_SPI_Transmit(&hspi1, &response, 1, 1000);

            // Reset buffer
            spi_buffer = 0;
        }
    }
}
// Configuratie van de systeemklok
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

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
// Init SPI1 in slave mode
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_SLAVE;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}
// Init GPIO voor LED en knoppen
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  
    // PB6 = LED-uitgang (of CS-lijn afhankelijk van je opzet)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// Foutafhandeling: oneindige lus
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
