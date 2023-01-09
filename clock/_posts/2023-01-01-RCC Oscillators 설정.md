---
layout: post
title: RCC Oscillators 설정 순서
tags: systick_sequence
sitemap: false
related_posts:
- systick/_posts/2023-01-08-RCC Clock Configuration 설정.md
---

STM32에서 Oscillator를 설정하는 과정을 확인해 보았다. 
<br /> SystemClock Config => HAL_RCC_OscConfig 함수가 호출되어 Oscillator가 설정이 된다. 호출 과정을 분석한 내용을 정리해 보았다.
<br /> Oscillator를 설정하기 위해 아래와 같이 설정 후 HAL_RCC_OscConfig(&RCC_OscInitStruct)를 호출 한다.
<br /> Clock 설정은 아래와 같이 세팅했다.
<br />![clock_setting](/assets/img/blog/clock_setting.png){: width="100%" height="100%"}
~~~
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
~~~

~~~
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
  /*------------------------------- HSE Configuration ------------------------*/
  // OscillatorType을 HSE(High Speed External 로 설정하였기 때문에 아래 조건에 성립 한다.)
  if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
  {
    ....
    /* When the HSE is used as system clock or clock source for PLL in these cases it is not allowed to be disabled */
    // RCC_CFGR(Clock configuration register) 레지스터 값은 리셋시 0으로 설정 되며 system clock이 HSE 또는 PLLCLK으로 설정 되어 있는지 확인
    // 1. RCC_CFGR 레지스터의 SWS 비트 값 확인
    if ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSE)
        || ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && (__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE)))
    {
      if ((__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET) && (RCC_OscInitStruct->HSEState == RCC_HSE_OFF))
      {
        return HAL_ERROR;
      }
    }
    else
    {
      /* Set the new HSE configuration ---------------------------------------*/
      // RCC_OscInitStruct.HSEState = RCC_HSE_ON; 
      // HSEState는 RCC_HSE_ON으로 설정 되어 있다.
      // 2.RCC->CR (Clock control register) 레지스터의 HSION 비트 값을으로 세팅 ENABLE
      // RCC->CR 레지스터에 HSION 비트 값을으로 세팅 ENABLE로 세팅을 해도 
      // RCC->CFGR 레지스터의 SWS의 비트 HSE oscillator used as system clock 로 세팅되지는 않는다.
      __HAL_RCC_HSE_CONFIG(RCC_OscInitStruct->HSEState);

      /* Check the HSE State */
      if (RCC_OscInitStruct->HSEState != RCC_HSE_OFF)
      {
        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSE is ready */
        // 3. RCC->CR 레지스터에서 HSERDY 비트 값이 0 인지 확인
        // __HAL_RCC_HSE_CONFIG에서 RCC->CR을 의 HSEON이 설정되어 HSERDY도 1로 설정 됨
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
        {
          if ((HAL_GetTick() - tickstart) > HSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
   }   
  ....

  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE)
  {
    /* Check if the PLL is used as system clock or not */
    // 4. RCC_CFGR 레지스터의 SWS 비트 값이 RCC_SYSCLKSOURCE_STATUS_PLLCLK 인지 확인
    // system clock으로 PLL Clock을 사용하고 있는지 확인
    if (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK)
    {
      // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
      if ((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
      {
        /* Check the parameters */
        assert_param(IS_RCC_PLLSOURCE(RCC_OscInitStruct->PLL.PLLSource));
        assert_param(IS_RCC_PLL_MUL(RCC_OscInitStruct->PLL.PLLMUL));

        /* Disable the main PLL. */
        // 5. RCC->CR 레지스터의 PLLON bit 값을 bit baning을 이용하여 Disable 한다.
        __HAL_RCC_PLL_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL is disabled */
        // 6. RCC->CR (Clock control register )레지스터의 PLLRDY 비트가 0인지 확인
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET)
        {
          if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }

        /* Configure the HSE prediv factor --------------------------------*/
        /* It can be written only when the PLL is disabled. Not used in PLL source is different than HSE */
        // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        if (RCC_OscInitStruct->PLL.PLLSource == RCC_PLLSOURCE_HSE)
        {
          /* Check the parameter */
          assert_param(IS_RCC_HSE_PREDIV(RCC_OscInitStruct->HSEPredivValue));
          /* Set PREDIV1 Value */
          // RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
          // 7. RCC->CFGR 레지스터에 RCC_HSE_PREDIV_DIV1 값을 write한다.
          __HAL_RCC_HSE_PREDIV_CONFIG(RCC_OscInitStruct->HSEPredivValue);
        }

        /* Configure the main PLL clock source and multiplication factors. */
        // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        // RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
        // RCC->CFGR 레지스터에 PLLSource, PLLMUL 값을 write 
        8. __HAL_RCC_PLL_CONFIG(RCC_OscInitStruct->PLL.PLLSource,
                             RCC_OscInitStruct->PLL.PLLMUL);
        /* Enable the main PLL. */
        // RCC->CR 레지스터의 PLLON bit를 bit baning을 이용하여 Enable 한다.
        // RCC->CR 레지스터에 PLLON 비트 값을으로 세팅 ENABLE로 세팅을 해도 
        // RCC->CFGR 레지스터의 SWS의 비트  PLL used as system clock 로 세팅되지는 않는다.
        9. __HAL_RCC_PLL_ENABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL is ready */
        // 10. RCC->CR의PLLRDY bit가 on 됐는지 확인
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  == RESET)
        {
          if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
    }
~~~

RCC_CFGR 레지스터 값은 아래 그림과 같이 Reset이 되면 0으로 설정이 된다.

<br /> ![rcc_cfgr_register](/assets/img/blog/rcc_cfgr_register.png){: width="500" height="500"}

~~~
1. if ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSE)
        || ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && (__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE)))

#define __HAL_RCC_GET_SYSCLK_SOURCE() ((uint32_t)(READ_BIT(RCC->CFGR,RCC_CFGR_SWS)))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define RCC                 ((RCC_TypeDef *)RCC_BASE)
#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000UL)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)
#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region */

typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;


} RCC_TypeDef;

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                     (2U)                              
#define RCC_CFGR_SWS_Msk                     (0x3UL << RCC_CFGR_SWS_Pos)        /*!< 0x0000000C */
#define RCC_CFGR_SWS                         RCC_CFGR_SWS_Msk                   /*!< SWS[1:0] bits (System Clock Switch Status) */

#define RCC_SYSCLKSOURCE_STATUS_HSI      RCC_CFGR_SWS_HSI            /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE      RCC_CFGR_SWS_HSE            /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK   RCC_CFGR_SWS_PLL            /*!< PLL used as system clock */

#define RCC_CFGR_SWS_HSI                     0x00000000U                       /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                     0x00000004U                       /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                     0x00000008U                       /*!< PLL used as system clock */

#define __HAL_RCC_GET_PLL_OSCSOURCE() ((uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLSRC)))

#define RCC_CFGR_PLLSRC_Pos                  (16U)                             
#define RCC_CFGR_PLLSRC_Msk                  (0x1UL << RCC_CFGR_PLLSRC_Pos)     /*!< 0x00010000 */
#define RCC_CFGR_PLLSRC                      RCC_CFGR_PLLSRC_Msk               /*!< PLL entry clock source */
~~~

~~~
 /* Set the new HSE configuration ---------------------------------------*/
// RCC_OscInitStruct.HSEState = RCC_HSE_ON; 
// HSEState는 RCC_HSE_ON으로 설정 되어 있다.
// 2 .RCC->CR 레지스터 세팅 (Clock control register)
__HAL_RCC_HSE_CONFIG(RCC_OscInitStruct->HSEState);

#define __HAL_RCC_HSE_CONFIG(__STATE__)                                     \
                    do{                                                     \
                      if ((__STATE__) == RCC_HSE_ON)                        \
                      {                                                     \
                        SET_BIT(RCC->CR, RCC_CR_HSEON);                     \
                      }                                                     \
                      else if ((__STATE__) == RCC_HSE_OFF)                  \
                      {                                                     \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEON);                   \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);                  \
                      }                                                     \
                      else if ((__STATE__) == RCC_HSE_BYPASS)               \
                      {                                                     \
                        SET_BIT(RCC->CR, RCC_CR_HSEBYP);                    \
                        SET_BIT(RCC->CR, RCC_CR_HSEON);                     \
                      }                                                     \
                      else                                                  \
                      {                                                     \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEON);                   \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);                  \
                      }                                                     \
                    }while(0U)

#define RCC_CR_HSEON_Pos                     (16U)                             
#define RCC_CR_HSEON_Msk                     (0x1UL << RCC_CR_HSEON_Pos)        /*!< 0x00010000 */
#define RCC_CR_HSEON                         RCC_CR_HSEON_Msk                  /*!< External High Speed clock enable */

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
~~~

<br />아래와 같이 RCC->CR 레지스터가 RCC->CR 의 HSEON을 Setting 후 HSERDY 값도 1로 변한 것을 알 수 있다.
<br /> ![rcc_cr_register](/assets/img/blog/rcc_cr_register.png){: width="500" height="500"}
<br />
<br /> ![rcc_cr_after_reset_value](/assets/img/blog/rcc_cr_after_reset_value.png){: width="500" height="500"}
<br />
<br /> ![rcc_cr_after_set_rcc_cr_set_hse](/assets/img/blog/rcc_cr_after_set_rcc_cr_set_hse.png){: width="500" height="500"}
<br />

~~~
  /* Wait till HSE is ready */
  // RCC->CR 레지스터에서 HSERDY가 0 인지 확인
  // __HAL_RCC_HSE_CONFIG에서 RCC->CR을 의 HSEON이 설정되어 HSERDY도 1로 설정 됨
  3. while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)

typedef enum 
{
  RESET = 0, 
  SET = !RESET
} FlagStatus, ITStatus;

#define RCC_FLAG_HSERDY      ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSERDY_Pos)) /*!< External High Speed clock ready flag */

#define RCC_CR_HSERDY_Pos                    (17U)                             
#define RCC_CR_HSERDY_Msk                    (0x1UL << RCC_CR_HSERDY_Pos)       /*!< 0x00020000 */
#define RCC_CR_HSERDY                        RCC_CR_HSERDY_Msk                 /*!< External High Speed clock ready flag */

/* Defines used for Flags */
#define CR_REG_INDEX                     ((uint8_t)1)
#define BDCR_REG_INDEX                   ((uint8_t)2)
#define CSR_REG_INDEX                    ((uint8_t)3)

#define __HAL_RCC_GET_FLAG(__FLAG__) (((((__FLAG__) >> 5U) == CR_REG_INDEX)?   RCC->CR   : \
                                      ((((__FLAG__) >> 5U) == BDCR_REG_INDEX)? RCC->BDCR : \
                                                                              RCC->CSR)) & (1U << ((__FLAG__) & RCC_FLAG_MASK)))

#define RCC_FLAG_MASK                    ((uint8_t)0x1F)                                                                       
~~~

~~~
4. __HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK

#define RCC_SYSCLKSOURCE_STATUS_HSI      RCC_CFGR_SWS_HSI            /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE      RCC_CFGR_SWS_HSE            /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK   RCC_CFGR_SWS_PLL            /*!< PLL used as system clock */

#define RCC_CFGR_SWS_HSI                     0x00000000U                       /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                     0x00000004U                       /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                     0x00000008U                       /*!< PLL used as system clock */

#define __HAL_RCC_GET_SYSCLK_SOURCE() ((uint32_t)(READ_BIT(RCC->CFGR,RCC_CFGR_SWS)))

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                     (2U)                              
#define RCC_CFGR_SWS_Msk                     (0x3UL << RCC_CFGR_SWS_Pos)        /*!< 0x0000000C */
#define RCC_CFGR_SWS                         RCC_CFGR_SWS_Msk                   /*!< SWS[1:0] bits (System Clock Switch Status) */

#define READ_BIT(REG, BIT)    ((REG) & (BIT))
~~~

~~~
5. __HAL_RCC_PLL_DISABLE()

#define RCC_PLL_NONE                      0x00000000U  /*!< PLL is not configured */
#define RCC_PLL_OFF                       0x00000001U  /*!< PLL deactivation */
#define RCC_PLL_ON                        0x00000002U  /*!< PLL activation */

#define __HAL_RCC_PLL_DISABLE()         (*(__IO uint32_t *) RCC_CR_PLLON_BB = DISABLE)

typedef enum 
{
  DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;

#define RCC_CR_PLLON_BB           ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_PLLON_BIT_NUMBER * 4U)))

#define RCC_CR_OFFSET_BB          (RCC_OFFSET + RCC_CR_OFFSET)

#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE) // 0x40021000 - 0x40000000

#define PERIPH_BB_BASE            0x42000000UL /*!< Peripheral base address in the bit-band region */

#define RCC_CR_OFFSET             0x00U

#define RCC_BASE                  (AHBPERIPH_BASE + 0x00001000UL). 	//  0x40000000 + 0x00020000 + 0x00001000

#define AHBPERIPH_BASE           (PERIPH_BASE + 0x00020000UL)

#define PERIPH_BASE               0x40000000UL /*!< Peripheral base address in the alias region */

#define RCC_PLLON_BIT_NUMBER      RCC_CR_PLLON_Pos

#define RCC_CR_PLLON_Pos                     (24U) 
~~~

<br /> STM32의 메모리 멥은 아래 그림과 같다.
<br /> ![bit_banding_memory_map](/assets/img/blog/bit_banding_memory_map.png){: width="500" height="500"}
<br />
<br /> STM32의 peripheral 멥은 아래 그림과 같다.
<br /> ![peripheral_bit_region](/assets/img/blog/peripheral_bit_region.png){: width="500" height="500"}
<br />
<br /> STM32 bit banding mapping 에 대한 설명
<br /> ![bit-banding-mapping](/assets/img/blog/bit-banding-mapping.png){: width="500" height="500"}

<br /> bit를 AND 또는 OR 연산자 없이 빠르게 변환 시키기 위해 bit banding을 사용한다.

<br /> 참고한 홈페이지 및 STM32 문서 링크
<br /> 1. [https://todayis.tistory.com/254](https://todayis.tistory.com/254)
<br /> 2. [https://developer.arm.com/documentation/ddi0337/h/programmers-model/bit-banding?lang=en](https://developer.arm.com/documentation/ddi0337/h/programmers-model/bit-banding?lang=en)
<br /> 3. [https://www.st.com/resource/en/programming_manual/pm0056-stm32f10xxx20xxx21xxxl1xxxx-cortexm3-programming-manual-stmicroelectronics.pdf](https://www.st.com/resource/en/programming_manual/pm0056-stm32f10xxx20xxx21xxxl1xxxx-cortexm3-programming-manual-stmicroelectronics.pdf)
<br /> 4. [https://todayis.tistory.com/254](https://todayis.tistory.com/254)
<br /> 5. [https://feelpass.tistory.com/entry/Cortex-M3-Bit-band](https://feelpass.tistory.com/entry/Cortex-M3-Bit-band)
<br /> 6. [https://scienceprog.com/bit-band-operations-with-arm-cortex-microcontrollers/](https://scienceprog.com/bit-band-operations-with-arm-cortex-microcontrollers/)

~~~
6. while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  != RESET)

#define RCC_FLAG_PLLRDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_PLLRDY_Pos)) /*!< PLL clock ready flag */

#define RCC_CR_PLLRDY_Pos                (25U)

#define __HAL_RCC_GET_FLAG(__FLAG__) (((((__FLAG__) >> 5U) == CR_REG_INDEX)?   RCC->CR   : \
                                      ((((__FLAG__) >> 5U) == BDCR_REG_INDEX)? RCC->BDCR : \
                                                                              RCC->CSR)) & (1U << ((__FLAG__) & RCC_FLAG_MASK)))
~~~   

~~~
7. __HAL_RCC_HSE_PREDIV_CONFIG(RCC_OscInitStruct->HSEPredivValue);

#define RCC_HSE_PREDIV_DIV1              0x00000000U

#define __HAL_RCC_HSE_PREDIV_CONFIG(__HSE_PREDIV_VALUE__) \
                  MODIFY_REG(RCC->CFGR,RCC_CFGR_PLLXTPRE, (uint32_t)(__HSE_PREDIV_VALUE__))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))


#define RCC_CFGR_PLLXTPRE_Pos                (17U)                             
#define RCC_CFGR_PLLXTPRE_Msk                (0x1UL << RCC_CFGR_PLLXTPRE_Pos)   /*!< 0x00020000 */
#define RCC_CFGR_PLLXTPRE                    RCC_CFGR_PLLXTPRE_Msk     

#define RCC_HSE_PREDIV_DIV1              0x00000000U

~~~

~~~
/* Configure the main PLL clock source and multiplication factors. */
// RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
// RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
// RCC->CFGR 레지스터에 PLLSource, PLLMUL 값을 write 
8. __HAL_RCC_PLL_CONFIG(RCC_OscInitStruct->PLL.PLLSource,
                      RCC_OscInitStruct->PLL.PLLMUL);

#define RCC_PLLSOURCE_HSE                    RCC_CFGR_PLLSRC            /*!< HSE clock selected as PLL entry clock source */
#define RCC_CFGR_PLLSRC_Pos                  (16U)                             
#define RCC_CFGR_PLLSRC_Msk                  (0x1UL << RCC_CFGR_PLLSRC_Pos)     /*!< 0x00010000 */
#define RCC_CFGR_PLLSRC                      RCC_CFGR_PLLSRC_Msk               /*!< PLL entry clock source */

#define RCC_PLL_MUL9                         RCC_CFGR_PLLMULL9

#define RCC_CFGR_PLLMULL9_Pos                (18U)                             
#define RCC_CFGR_PLLMULL9_Msk                (0x7UL << RCC_CFGR_PLLMULL9_Pos)   /*!< 0x001C0000 */
#define RCC_CFGR_PLLMULL9                    RCC_CFGR_PLLMULL9_Msk 

#define __HAL_RCC_PLL_CONFIG(__RCC_PLLSOURCE__, __PLLMUL__)\
          MODIFY_REG(RCC->CFGR, (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL),((__RCC_PLLSOURCE__) | (__PLLMUL__) ))

/*!< PLLMUL configuration */
#define RCC_CFGR_PLLMULL_Pos                 (18U)                             
#define RCC_CFGR_PLLMULL_Msk                 (0xFUL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x003C0000 */
#define RCC_CFGR_PLLMULL                     RCC_CFGR_PLLMULL_Msk     

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))
~~~

~~~
9. __HAL_RCC_PLL_ENABLE();

#define __HAL_RCC_PLL_ENABLE()          (*(__IO uint32_t *) RCC_CR_PLLON_BB = ENABLE) // Enable시 PLLRDY bit도 1로 세팅됨

typedef enum 
{
  DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;

#define RCC_CR_PLLON_BB           ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_PLLON_BIT_NUMBER * 4U)))

#define RCC_CR_OFFSET_BB          (RCC_OFFSET + RCC_CR_OFFSET)

#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE) // 0x40021000 - 0x40000000

#define PERIPH_BB_BASE            0x42000000UL /*!< Peripheral base address in the bit-band region */

#define RCC_CR_OFFSET             0x00U

#define RCC_BASE                  (AHBPERIPH_BASE + 0x00001000UL). 	//  0x40000000 + 0x00020000 + 0x00001000

#define AHBPERIPH_BASE           (PERIPH_BASE + 0x00020000UL)

#define PERIPH_BASE               0x40000000UL /*!< Peripheral base address in the alias region */

#define RCC_PLLON_BIT_NUMBER      RCC_CR_PLLON_Pos

#define RCC_CR_PLLON_Pos                     (24U) 
~~~

~~~
10. while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  == RESET)

#define     __IO    volatile             /*!< Defines 'read / write' permissions */

#define RCC_FLAG_PLLRDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_PLLRDY_Pos)) /*!< PLL clock ready flag */

#define RCC_CR_PLLRDY_Pos                (25U) 

/* Defines used for Flags */
#define CR_REG_INDEX                     ((uint8_t)1)
#define BDCR_REG_INDEX                   ((uint8_t)2)
#define CSR_REG_INDEX                    ((uint8_t)3)
                            
#define __HAL_RCC_GET_FLAG(__FLAG__) (((((__FLAG__) >> 5U) == CR_REG_INDEX)?   RCC->CR   : \
                                      ((((__FLAG__) >> 5U) == BDCR_REG_INDEX)? RCC->BDCR : \
                                                                              RCC->CSR)) & (1U << ((__FLAG__) & RCC_FLAG_MASK)))
~~~                                                                              