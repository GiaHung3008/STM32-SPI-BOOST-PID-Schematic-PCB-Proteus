#include "stm32f10x.h"  // thu vien STM32F10x
#include <stdio.h>       // thu vien stdio cho sprintf

/* ==================== BI?N TOÀN C?C ==================== */
volatile uint32_t msTicks = 0;  // bien dem thoi gian tinh bang ms

/* ==================== SYSTICK ==================== */
void SysTick_Init(void){
    SysTick->LOAD = 720 - 1;  // dat gia tri nap cho SysTick (72MHz/1000 = 72000, nhung o day la 720)
    SysTick->VAL  = 0;        // xoa gia tri hien tai
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |  // chon nguon clock
                    SysTick_CTRL_TICKINT_Msk   |  // bat ngat
                    SysTick_CTRL_ENABLE_Msk;       // bat SysTick
}
void SysTick_Handler(void){ msTicks++; }  // ham xu ly ngat SysTick, tang bien dem

void delay_ms(uint32_t ms){
    uint32_t cur = msTicks;              // luu gia tri hien tai
    while(msTicks - cur < ms);           // cho den khi du thoi gian
}

/* ======================================================
   ========== THAY TH? TOÀN B? LCD C?A B?N ==============
   ================= LCD 4-BIT ===========================
   D4 = PB4
   D5 = PB5
   D6 = PB6
   D7 = PB7
   RS = PB8
   E  = PB9
   ====================================================== */
void lcd_GPIO_init(void){
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;  // bat clock cho GPIOB va AFIO
    
    // Disable JTAG, keep SWD
    AFIO->MAPR |= (0x2 << 24);  // tat JTAG, giu SWD

    // PB4-PB7 output push-pull (D4-D7)
    GPIOB->CRL &= ~(0xFFFF << (4*4));  // xoa cau hinh cu
    GPIOB->CRL |= (0x3 << (4*4)) |   // PB4 = D4, output push-pull 50MHz
                  (0x3 << (5*4)) |   // PB5 = D5, output push-pull 50MHz
                  (0x3 << (6*4)) |   // PB6 = D6, output push-pull 50MHz
                  (0x3 << (7*4));    // PB7 = D7, output push-pull 50MHz

    // PB8 = RS
    GPIOB->CRH &= ~(0xF << 0);  // xoa cau hinh cu
    GPIOB->CRH |=  (0x3 << 0);  // PB8 = RS, output push-pull 50MHz

    // PB9 = E
    GPIOB->CRH &= ~(0xF << 4);  // xoa cau hinh cu
    GPIOB->CRH |=  (0x3 << 4);  // PB9 = E, output push-pull 50MHz
}

static void lcd_send_nibble(uint8_t nibble){
    GPIOB->ODR &= ~((1<<4)|(1<<5)|(1<<6)|(1<<7));  // xoa cac bit data

    if(nibble & 0x01) GPIOB->BSRR = (1<<4);  // set bit 0 -> PB4 (D4)
    if(nibble & 0x02) GPIOB->BSRR = (1<<5);  // set bit 1 -> PB5 (D5)
    if(nibble & 0x04) GPIOB->BSRR = (1<<6);  // set bit 2 -> PB6 (D6)
    if(nibble & 0x08) GPIOB->BSRR = (1<<7);  // set bit 3 -> PB7 (D7)

    GPIOB->BSRR = (1<<9);  // E = 1, bat enable
    delay_ms(1);            // cho 1ms
    GPIOB->BRR  = (1<<9);  // E = 0, tat enable
    delay_ms(1);            // cho 1ms
}

void lcd_command(uint8_t cmd){
    GPIOB->BRR = (1<<8);      // RS = 0, che do command
    lcd_send_nibble(cmd >> 4);  // gui 4 bit cao
    lcd_send_nibble(cmd & 0x0F);  // gui 4 bit thap
    delay_ms(2);              // cho 2ms
}

void lcd_data(uint8_t data){
    GPIOB->BSRR = (1<<8);     // RS = 1, che do data
lcd_send_nibble(data >> 4);  // gui 4 bit cao
    lcd_send_nibble(data & 0x0F);  // gui 4 bit thap
    delay_ms(2);              // cho 2ms
}

void lcd_print(char *str){
    while(*str) lcd_data(*str++);  // in tung ky tu den khi gap NULL
}

void lcd_setCursor(uint8_t col, uint8_t row){
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;  // tinh dia chi: dong 0 = 0x80, dong 1 = 0xC0
    lcd_command(addr);  // gui lenh dat vi tri con tro
}

void lcd_begin(void){
    lcd_GPIO_init();  // khoi tao GPIO cho LCD
    delay_ms(50);     // cho LCD khoi dong

    lcd_send_nibble(0x03);  // khoi tao 8-bit mode (lan 1)
    delay_ms(5);            // cho 5ms
    lcd_send_nibble(0x03);  // khoi tao 8-bit mode (lan 2)
    delay_ms(1);            // cho 1ms
    lcd_send_nibble(0x03);  // khoi tao 8-bit mode (lan 3)
    delay_ms(1);            // cho 1ms
    lcd_send_nibble(0x02);  // chuyen sang 4-bit mode

    lcd_command(0x28);  // 4-bit mode, 2 dong, font 5x8
    lcd_command(0x0C);  // bat hien thi, tat con tro, tat nhap nhay
    lcd_command(0x06);  // tu dong tang con tro khi ghi
    lcd_command(0x01);  // xoa toan bo man hinh
    delay_ms(2);        // cho 2ms
}

/* ==================== SPI GPIO INIT ==================== */
void GPIO_SPI1_Master_Init(void){
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN |   // bat clock cho GPIOA va AFIO
                    RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPBEN;    // bat clock cho SPI1 va GPIOB

    GPIOA->CRL &= ~(0xFFFF << 16);  // xoa cau hinh cu cho PA4-PA7

    GPIOA->CRL |=  (0x3 << (4*4));  // PA4 NSS, output push-pull 50MHz
    GPIOA->CRL |=  (0xB << (5*4));  // PA5 SCK, alternate function push-pull 50MHz
    GPIOA->CRL |=  (0x4 << (6*4));  // PA6 MISO, input floating
    GPIOA->CRL |=  (0xB << (7*4));  // PA7 MOSI, alternate function push-pull 50MHz
}

/* ==================== SPI INIT ==================== */
void SPI1_Master_Init(void){
    SPI1->CR1 = 0;              // xoa thanh ghi CR1
    SPI1->CR1 |= (1<<2);        // bat SPI (bit SPE)
    SPI1->CR1 |= (0x2<<3);      // dat prescaler (bit 3-5), fPCLK/8
    SPI1->CR1 &= ~(1<<7);       // MSB first (bit LSBFIRST = 0)
    SPI1->CR1 &= ~(1<<1);       // CPOL = 0, clock idle low
    SPI1->CR1 &= ~(1<<0);       // CPHA = 0, sample on first edge
    SPI1->CR1 |= (1<<9)|(1<<8); // SSM = 1, SSI = 1 (software NSS)
    SPI1->CR1 |= (1<<11);       // che do master (bit MSTR = 1)
    SPI1->CR1 |= (1<<6);        // bat SPI (bit SPE = 1)
}
void ADC1_Init(void) {
    RCC->APB2ENR |= (1 << 2);  // bat clock cho GPIOA
    GPIOA->CRL &= ~(0xF << (0 * 4));  // PA0 che do analog

    RCC->APB2ENR |= (1 << 9);  // bat clock cho ADC1

    ADC1->SQR1 &= ~(0xF << 20);  // dat so luong chuyen doi = 1
    ADC1->SQR3 &= ~(0x1F << 0);  // xoa cau hinh cu
    ADC1->SQR3 |= (0 << 0);  // chon channel 0 (PA0)

    ADC1->SMPR2 &= ~(0x7 << (3 * 0));  // xoa cau hinh cu
    ADC1->SMPR2 |=  (7 << (3 * 0));  // thoi gian lay mau toi da (239.5 cycles)
ADC1->CR2 |= (1 << 0);  // bat ADC (bit ADON)
    ADC1->CR2 |= (1 << 2);  // bat calib (bit CAL)
    while (ADC1->CR2 & (1 << 2));  // cho calib hoan thanh
}

uint16_t read_adc(void) {
    ADC1->CR2 |= (1 << 0);  // bat ADC (bit ADON)
    ADC1->CR2 |= (1 << 22);  // bat chuyen doi bang phan mem (bit SWSTART)
    while (!(ADC1->SR & (1 << 1)));  // cho den khi chuyen doi xong (bit EOC)
    return ADC1->DR;  // tra ve gia tri ADC
}

uint16_t SPI1_SendRecv16(uint16_t data){
    while(!(SPI1->SR & SPI_SR_TXE));  // cho den khi transmit buffer trong
    SPI1->DR = data;  // gui du lieu
    while(SPI1->SR & SPI_SR_BSY);  // cho den khi SPI khong ban
    return SPI1->DR;  // tra ve du lieu nhan duoc
}

/* ==================== SPI READ ADC ==================== */
uint16_t SPI1_Read_ADC(void){
    uint16_t frame, adc_value;

    GPIOA->BRR = (1<<4);  // NSS = 0, chon slave
    frame = 0xAA01;  // khung lenh doc ADC
    SPI1_SendRecv16(frame);  // gui lenh
    while(SPI1->SR & SPI_SR_BSY);  // cho SPI xong
    GPIOA->BSRR = (1<<4);  // NSS = 1, bo chon slave

    delay_ms(5);  // cho 5ms

    GPIOA->BRR = (1<<4);  // NSS = 0, chon slave
    adc_value = SPI1_SendRecv16(0xFFFF);  // doc gia tri ADC
    while(SPI1->SR & SPI_SR_BSY);  // cho SPI xong
    GPIOA->BSRR = (1<<4);  // NSS = 1, bo chon slave

    return adc_value & 0x0FFF;  // tra ve 12 bit thap
}

/* ==================== MAIN ==================== */
int main(void){
    SysTick_Init();  // khoi tao SysTick
    GPIO_SPI1_Master_Init();  // khoi tao GPIO cho SPI1 master
    lcd_begin();  // khoi tao LCD
    SPI1_Master_Init();  // khoi tao SPI1 master
		ADC1_Init();  // khoi tao ADC1
    uint16_t adc_val;  // bien luu gia tri ADC (khong dung)
    char buffer[16];  // buffer de luu chuoi hien thi

    lcd_setCursor(0,0);  // dat con tro ve dong 0, cot 0
    lcd_print(" PWM:");  // in chuoi " PWM:"

   while(1){
    uint16_t adc_raw = read_adc();  // doc gia tri ADC tu PA0
    uint16_t duty = (adc_raw * 100) / 4095;  // tinh duty cycle (0-100%)

    // gui duty cho slave qua SPI
    uint16_t tx = duty & 0x00FF;  // lay 8 bit thap cua duty
    SPI1_SendRecv16(tx);  // gui duty qua SPI

    // Hien thi LCD
		lcd_setCursor(4,0);  // dat con tro ve dong 0, cot 4
		 sprintf(buffer, ":%3d", duty);  // chuyen duty thanh chuoi
		lcd_print(buffer);  // in ra LCD


    delay_ms(50);  // cho 50ms
}

}
