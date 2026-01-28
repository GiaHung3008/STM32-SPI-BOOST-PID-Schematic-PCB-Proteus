#include "stm32f10x.h"

volatile uint32_t msTicks = 0;   // Dem mili giay de su dung delay
volatile uint16_t adc_value = 0; // Gia tri ADC doc duoc
volatile uint16_t rx_data = 0;   // Du lieu nhan duoc tu master
volatile uint16_t tx_data = 0;   // Du lieu se gui tra ve cho master

/* ==================== Khai bao ham ==================== */
void GPIO_Init(void);
void delay_ms(uint32_t ms);
void SysTick_Init(void);
void ADC1_Config(void);
void PWM_Init(void);
void Update_PWM_From_ADC(void);
uint16_t ADC_Read(void);
void SPI1_Slave_Init(void);
void SPI1_IRQHandler(void);
void GPIO_Init_Slave(void);
void ADC1_Init(void);
uint16_t ADC1_Read(void);

/* ==================== HAM MAIN ==================== */
int main(void)
{
    SysTick_Init();          // Cau hinh SysTick tao ngat moi 1ms
    GPIO_Init();             // Khoi tao GPIO
    //ADC1_Config();           // Cau hinh ADC1 doc cam bien (PA0)
    PWM_Init();              // Cau hinh PWM TIM1
    __enable_irq();          // Cho phep ngat toan cuc
    delay_ms(100);
    
    GPIO_Init_Slave();       // Cau hinh GPIO cho SPI
    //ADC1_Init();             // Khoi tao ADC
    SPI1_Slave_Init();       // Khoi tao SPI1 trong che do Slave

    while (1)
    {
        // Cap nhat PWM theo gia tri ADC lien tuc
        //Update_PWM_From_ADC();
        delay_ms(10);         // Cap nhat moi 10ms
    }
}

/* ================= CAP NHAT PWM THEO ADC ================= */
void Update_PWM_From_ADC(void)
{
    adc_value = ADC_Read();  // Doc gia tri ADC (0 – 4095)
    // Dao nguoc logic: khi adc_value giam (V_IN giam) => duty tang => v_Out tang
   uint16_t duty = ((4095 - adc_value) * 50) / 4095;  // Chuyen doi ra phan tram duty (0-50%)
    TIM1->CCR1 = (duty * (TIM1->ARR + 1)) / 100;  // Cap nhat do rong xung PWM
    // Khi adc_value giam => duty tang => CCR1 tang => v_Out tang (boost converter)
}

/* ==================== GPIO ==================== */
void GPIO_Init(void)
{
    // Bat clock cho GPIOA, GPIOB, GPIOC, AFIO
    RCC->APB2ENR |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 0);
	
		AFIO->MAPR &= ~(0x7 << 24);
		AFIO->MAPR |=  (0x2 << 24);


    // Cau hinh PA0: dau vao analog (bien tro)
    GPIOA->CRL &= ~(0xF << (0 * 4)); // PA0 analog input
}

/* ================= Khoi tao SysTick ================= */
void SysTick_Init(void)
{
    SysTick->LOAD = 7200 - 1;        // 72MHz / 72000 = 1kHz (1ms)
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;  // Dung HCLK lam nguon
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;    // Bat ngat SysTick
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;     // Bat SysTick
}

// Ngat SysTick xay ra moi 1ms
void SysTick_Handler(void)
{
    msTicks++;
}

// Ham delay theo mili-giay
void delay_ms(uint32_t ms)
{
    uint32_t cur = msTicks;
    while ((msTicks - cur) < ms);
}

/* ================= Cau hinh ADC1 doc chan PA0 ================= */
void ADC1_Config(void)
{
    RCC->APB2ENR |= (1 << 9);  // Bat clock cho ADC1
ADC1->SQR1 &= ~(0xF << 20);    // Chi co 1 kenh chuyen doi
    ADC1->SQR3 &= ~(0x1F << 0);
ADC1->SQR3 |= (0 << 0);  // Chon kenh 0 tuong ung voi PA0
    // Thoi gian lay mau 239.5 chu ky ADC
    ADC1->SMPR1 &= ~(0x7 << (3 * (0 - 10)));
    ADC1->SMPR1 |= (7 << (3 * (0 - 10)));
    ADC1->CR2 |= (1 << 0);  // Bat ADC
    ADC1->CR2 |= (1 << 2);  // Hieu chuan ADC
    while (ADC1->CR2 & (1 << 2));  // Cho hieu chuan xong
}

/* ================= Khoi tao PWM TIM1 ================= */
void PWM_Init(void)
{
    RCC->APB2ENR |= (1 << 11);  // Bat clock cho TIM1
    RCC->APB2ENR |= (1 << 2);   // Bat clock cho GPIOA

    // --- Cau hinh PA8 lam TIM1_CH1 ---
    GPIOA->CRH &= ~(0xF << 0);
    GPIOA->CRH |= (0xB << 0);   // MODE8 = 11 (50MHz), CNF8 = 10 (AF push-pull)

    // --- Cau hinh bo dem TIM1 ---
    TIM1->PSC = 72 - 1;         // 72MHz / 72 = 1MHz
    TIM1->ARR = 50 - 1;         // => Tan so PWM = 20kHz
    TIM1->CCR1 = 25;            // Duty 50%
    TIM1->CCMR1 &= ~(0xFF << 0);
    TIM1->CCMR1 |= (6 << 4);    // OC1M = 110: PWM mode 1
    TIM1->CCMR1 |= (1 << 3);    // OC1PE: cho phep preload

    // --- Bat kenh chinh va kenh dao ---
    TIM1->CCER |= (1 << 0);     // CH1 enable
    TIM1->CCER |= (1 << 2);     // CH1N enable (nguoc)
    TIM1->CCER &= ~(1 << 1);    // CH1 khong dao
    TIM1->CCER &= ~(1 << 3);    // CH1N dao pha (nguoc voi CH1)

    // --- Cau hinh dead-time ---
    TIM1->BDTR &= ~(0xFF);
    TIM1->BDTR |= (72 & 0xFF);  // Dead-time = 1us
    TIM1->BDTR |= (1 << 15);    // MOE: cho phep ngo ra chinh
    TIM1->CR1 |= (1 << 7);      // Cho phep preload ARR
    TIM1->CR1 |= (1 << 0);      // Bat timer
}

/* ================= Doc gia tri ADC ================= */
uint16_t ADC_Read(void)
{
    ADC1->CR2 |= (1 << 0);      // Bat ADC
    ADC1->CR2 |= (1 << 22);     // Bat dau chuyen doi (SWSTART)
    while (!(ADC1->SR & (1 << 1)));  // Cho chuyen doi xong (EOC)
    return ADC1->DR & 0x0FFF;   // Tra ve ket qua 12-bit
}

/* ==================== KHOI TAO GPIO ==================== */
void GPIO_Init_Slave(void)
{
    // Bat clock cho GPIOA va AFIO
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

    // PA0: dau vao analog (ket noi voi bien tro hoac cam bien)
    GPIOA->CRL &= ~(0xF << (0 * 4));   // Dat ve che do analog (00)

    // Cau hinh cac chan SPI1:
    // PA4 = NSS  -> Input floating
    // PA5 = SCK  -> Input floating
    // PA6 = MISO -> Alternate function push-pull
    // PA7 = MOSI -> Input floating
    GPIOA->CRL &= ~(0xFFFF << 16);     // Xoa cau hinh cu cho PA4–PA7

    GPIOA->CRL |=  (0x4 << (4 * 4));   // PA4 input floating
    GPIOA->CRL |=  (0x4 << (5 * 4));   // PA5 input floating
    GPIOA->CRL |=  (0xB << (6 * 4));   // PA6 alternate function push-pull (MISO)
    GPIOA->CRL |=  (0x4 << (7 * 4));   // PA7 input floating (MOSI)
}

/* ==================== KHOI TAO ADC ==================== */
void ADC1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Bat clock cho ADC1
ADC1->SQR1 &= ~(0xF << 20);   // Cau hinh 1 kenh chuyen doi
    ADC1->SQR3 = 0;               // Chon kenh 0 (PA0)
ADC1->SMPR2 |= (0x7 << 0);    // Thoi gian lay mau dai (239.5 chu ky)

    ADC1->CR2 |= ADC_CR2_ADON;    // Bat ADC
    delay_ms(1);
    ADC1->CR2 |= ADC_CR2_CAL;     // Bat dau hieu chuan ADC
    while(ADC1->CR2 & ADC_CR2_CAL); // Doi cho den khi xong hieu chuan
}

// Ham doc mot mau ADC
uint16_t ADC1_Read(void)
{
    ADC1->CR2 |= ADC_CR2_ADON;         // Bat dau chuyen doi
    while(!(ADC1->SR & ADC_SR_EOC));   // Doi den khi chuyen doi xong (EOC=1)
    return ADC1->DR & 0x0FFF;          // Tra ve 12 bit du lieu ADC
}

/* ==================== KHOI TAO SPI SLAVE ==================== */
void SPI1_Slave_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Bat clock SPI1

    SPI1->CR1 = 0;  // Xoa cau hinh cu
    SPI1->CR2 = 0;

    SPI1->CR1 &= ~SPI_CR1_MSTR;        // Dat che do Slave
    SPI1->CR1 |= SPI_CR1_DFF;          // Su dung khung 16-bit
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA); // Che do CPOL=0, CPHA=0 (mode 0)
		SPI1->CR1 &= ~SPI_CR1_SSM; 
	  SPI1->CR1 &= ~SPI_CR1_SSI; 
    SPI1->CR2 |= SPI_CR2_RXNEIE;       // Cho phep ngat khi co du lieu nhan
    SPI1->CR1 |= SPI_CR1_SPE;          // Bat SPI (enable)

    NVIC_EnableIRQ(SPI1_IRQn);         // Cho phep ngat SPI1 trong NVIC
}

/* ==================== TRINH PHUC VU NGAT SPI ==================== */
void SPI1_IRQHandler(void)
{
    if (SPI1->SR & SPI_SR_RXNE)
    {
        rx_data = SPI1->DR; // Nh?n duty t? master (0–100)

        // Áp duty vào PWM
        TIM1->CCR1 = (rx_data * (TIM1->ARR + 1)) / 100;
    }
}


