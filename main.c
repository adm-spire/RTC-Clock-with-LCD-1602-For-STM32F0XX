#include "stm32f0xx.h"
#include <stdio.h>
#include <stdint.h>



#define LCD_ADDR  0x27
#define PWR_EN (1U << 28)
#define PWR_DBP (1U << 8)
#define LSION (1U << 0)
#define LSI_RDY (1U << 1)
#define BDCR_RTCSEL (3U << 8)
#define BDCR_LSI (2U << 8)
#define RTC_EN (1U << 15)
#define RTC_RSF (1U << 5)
#define RTC_INIT (1U << 7)
#define RTC_INITF (1U << 6)

void rtc_init(void);
void LCD_Init(void);
void LCD_SendString(char *str);
void LCD_I2C_WriteNibble(uint8_t nibble, uint8_t rs);
void LCD_SendCmd(uint8_t cmd);
void LCD_SendData(uint8_t data);
uint8_t bcd_to_dec(uint8_t bcd);
volatile uint32_t sysTickCounter = 0;


void SysTick_Handler(void) {
    sysTickCounter++;
}

void delay_ms(uint32_t ms) {
    uint32_t target = sysTickCounter + ms;
    while (sysTickCounter < target);
}




// I2C1 on PB8=SCL, PB9=SDA
void I2C1_Init(void) {
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER  |= (2 << (2 * 8)) | (2 << (2 * 9));  // PB8, PB9 = AF
    GPIOB->AFR[1] |= (1 << (0)) | (1 << (4));          // AF1 = I2C1

    I2C1->TIMINGR = 0x2000090E; // 100kHz I2C @ 8MHz
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_WriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
    I2C1->CR2 = (addr << 1) | (2 << 16) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = reg;
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = data;
    while (!(I2C1->ISR & I2C_ISR_TC));
    I2C1->CR2 |= I2C_CR2_STOP;
}

void I2C1_ReadBytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
    // Write register address
    I2C1->CR2 = (addr << 1) | (1 << 16) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = reg;
    while (!(I2C1->ISR & I2C_ISR_TC));

    // Read 'len' bytes
    I2C1->CR2 = (addr << 1) | (len << 16) | I2C_CR2_RD_WRN | I2C_CR2_START;
    for (uint8_t i = 0; i < len; i++) {
        while (!(I2C1->ISR & I2C_ISR_RXNE));
        data[i] = I2C1->RXDR;
    }
    while (!(I2C1->ISR & I2C_ISR_TC));
    I2C1->CR2 |= I2C_CR2_STOP;
}

void LCD_Init(void) {
    delay_ms(50); // Wait after power on
    LCD_I2C_WriteNibble(0x03, 0);
    delay_ms(5);
    LCD_I2C_WriteNibble(0x03, 0);
    delay_ms(5);
    LCD_I2C_WriteNibble(0x03, 0);
    delay_ms(5);
    LCD_I2C_WriteNibble(0x02, 0); // Set 4-bit mode

    LCD_SendCmd(0x28); // 4-bit, 2 lines, 5x8 font
    LCD_SendCmd(0x0C); // Display ON, Cursor OFF
    LCD_SendCmd(0x01); // Clear display
    delay_ms(2);
    LCD_SendCmd(0x06); // Entry mode set
}
void LCD_SendString(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

void LCD_I2C_WriteNibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = 0;

    data |= (nibble & 0x0F) << 4; // Correct shift to D4-D7
    if (rs) data |= (1 << 0);     // RS = P0
    data |= (1 << 2);             // EN = P2
    data |= (1 << 3);             // Backlight = P3

    I2C1_WriteByte(LCD_ADDR, 0x00, data);
    delay_ms(1);
    I2C1_WriteByte(LCD_ADDR, 0x00, data & ~(1 << 2)); // EN low
    delay_ms(1);
}

void LCD_I2C_SendByte(uint8_t byte, uint8_t rs) {
    LCD_I2C_WriteNibble(byte >> 4, rs);  // High nibble
    LCD_I2C_WriteNibble(byte & 0x0F, rs);  // Low nibble
}

void LCD_SendCmd(uint8_t cmd) {
    LCD_I2C_SendByte(cmd, 0);
}

void LCD_SendData(uint8_t data) {
    LCD_I2C_SendByte(data, 1);
}



void rtc_init(void){
    // 1. Enable Power Interface Clock
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // 2. Enable Backup Domain Access
    PWR->CR |= PWR_CR_DBP;
    while(!(PWR->CR & PWR_CR_DBP)); // Wait until DBP is enabled

    // 3. Reset Backup Domain (required for RTC reconfiguration)
    RCC->BDCR |= RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;

    // 4. Enable LSI Clock
    RCC->CSR |= RCC_CSR_LSION;
    while(!(RCC->CSR & RCC_CSR_LSIRDY)); // Wait for LSI ready

    // 5. Configure RTC Clock Source
    RCC->BDCR &= ~RCC_BDCR_RTCSEL;       // Clear previous selection
    RCC->BDCR |= RCC_BDCR_RTCSEL_LSI;    // Select LSI (0b10)

    // 6. Enable RTC Clock
    RCC->BDCR |= RCC_BDCR_RTCEN;

    // 7. Disable Write Protection
    RTC->WPR = 0xCA;   // First key
    RTC->WPR = 0x53;   // Second key

    // 8. Enter Initialization Mode
    RTC->ISR |= RTC_ISR_INIT;
    while(!(RTC->ISR & RTC_ISR_INITF)); // Wait for INITF flag

    // 9. Configure Prescaler for 1Hz (LSI = 40kHz)
    RTC->PRER = (127 << 16) | 249;  // (127+1)*(249+1) = 128*250 = 32,000
    // For precise 40kHz LSI: Use (124 << 16) | 319 (125*320=40,000)

    // 10. Set Time and Date (BCD format)
    RTC->TR = 0x161200;  // 16:12:00
    // 2025-05-13 Tuesday (BCD format)
    RTC->DR = (0x25 << 16) |  // Year (2025)
              (0x2 << 13)  |  // Weekday (Tuesday)
              (0x05 << 8)  |  // Month (May)
              (0x13);         // Day (13th)


    // 11. Exit Initialization Mode
    RTC->ISR &= ~RTC_ISR_INIT;

    // 12. Wait for Registers Synchronization
    RTC->ISR &= ~RTC_ISR_RSF;
    while(!(RTC->ISR & RTC_ISR_RSF));

    // 13. Re-enable Write Protection
    RTC->WPR = 0xFF;
}



uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}



int main(void)
{
	char time_str[30];  // HH:MM:SS
	char date_str[30]; // DD-MM-YYYY

    SysTick_Config(8000);

    I2C1_Init();

    LCD_Init();


    rtc_init();





while(1){

	// Read RTC->TR and RTC->DR
	uint32_t tr = RTC->TR;
	uint32_t dr = RTC->DR;

	// Extract time parts from BCD format
	uint8_t hours   = bcd_to_dec((tr >> 16) & 0x3F); // 6-bit hour
    uint8_t minutes = bcd_to_dec((tr >> 8)  & 0x7F); // 7-bit minute
    uint8_t seconds = bcd_to_dec(tr & 0x7F);         // 7-bit second

	// Extract date parts from BCD format
     uint8_t day   = bcd_to_dec((dr >> 0) & 0x3F);    // 6-bit day
	 uint8_t month = bcd_to_dec((dr >> 8) & 0x1F);    // 5-bit month
	 uint8_t year  = bcd_to_dec((dr >> 16) & 0xFF);   // 8-bit year (last 2 digits)

	// Format time and date



	LCD_SendCmd(0x01);  // Clear
	delay_ms(5);

	LCD_SendCmd(0x80);  // Line 1
	delay_ms(5);
	snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hours, minutes, seconds);

	LCD_SendString(time_str);

    LCD_SendCmd(0xC0);  // Line 2
    delay_ms(5);
    snprintf(date_str, sizeof(date_str), "%02d-%02d-20%02d", day, month, year);

      LCD_SendString(date_str);

    delay_ms(1000);



}
}
