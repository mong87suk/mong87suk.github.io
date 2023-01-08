---
layout: post
title: RCC Clock Configuration 설정 순서
tags: systick_sequence
sitemap: false
related_posts:
- systick/_posts/2023-01-08-RCC Oscillators 설정.md
---

STM32에서 Clock Configuration을 설정하는 과정을 확인해 보았다. 
<br /> SystemClock Config => HAL_RCC_ClockConfig 함수가 호출되어 Clock Configuration이 설정이 된다. 호출 과정을 분석한 내용을 정리해 보았다.
<br /> Oscillator를 설정하기 위해 아래와 같이 설정 후 HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)를 호출 한다.
<br /> Clock 설정은 아래와 같이 세팅했다.
<br />![clock_setting](/assets/img/blog/clock_setting.png){: width="100%" height="100%"}
~~~
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  ...
  /** Initializes the CPU, AHB and APB buses clocks*/
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
~~~

~~~
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency)
{
  uint32_t tickstart;

  /* Check Null pointer */
  if (RCC_ClkInitStruct == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RCC_CLOCKTYPE(RCC_ClkInitStruct->ClockType));
  assert_param(IS_FLASH_LATENCY(FLatency));

  /* To correctly read data from FLASH memory, the number of wait states (LATENCY)
  must be correctly programmed according to the frequency of the CPU clock
    (HCLK) of the device. */

#if defined(FLASH_ACR_LATENCY)
  /* Increasing the number of wait states because of higher CPU frequency */
  // 1.FLASH->ACR , FLASH의 FLASH_ACR 레지스터의 LATENCY 비트 값을 확인 
  if (FLatency > __HAL_FLASH_GET_LATENCY())
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    // FLatency = FLASH_LATENCY_2
    // 2. FLASH->ACR 레지스터의 LATENCY 비트에 FLASH_LATENCY_2 값을 Write한다.
    __HAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
    // 3. FLASH->ACR의 LATENCY 비트 값이 FLatency 값과 같은지 확인
    if (__HAL_FLASH_GET_LATENCY() != FLatency)
  {
    return HAL_ERROR;
  }
}

#endif /* FLASH_ACR_LATENCY */
/*-------------------------- HCLK Configuration --------------------------*/
// 4. RCC_ClkInitStruct->ClockType 이 RCC_CLOCKTYPE_HCLK 값이 포함되어 있는지 확인
if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
  {
    /* Set the highest APBx dividers in order to ensure that we do not go through
    a non-spec phase whatever we decrease or increase HCLK. */
    // 5. RCC_ClkInitStruct->ClockType의 값에 RCC_CLOCKTYPE_PCLK1이 포함되어 있는지 확인
    if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
    {
      // 6. PPRE1: APB low-speed prescaler (APB1)
      // RCC->CFGR 레지스터의 PPRE1비트에 RCC_HCLK_DIV16 값을 Write한다.
      MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV16);
    }
    // 7. RCC_ClkInitStruct->ClockType의 값에 RCC_CLOCKTYPE_PCLK2이 포함되어 있는지 확인
    if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
    {
      // 8. RCC->CFGR 레지스터의 PPRE2비트에 RCC_HCLK_DIV16 값을 Write한다.
      MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (RCC_HCLK_DIV16 << 3));
    }

    /* Set the new HCLK clock divider */
    assert_param(IS_RCC_HCLK(RCC_ClkInitStruct->AHBCLKDivider));
    // 9. RCC->CFGR 레지스터의 HPRE 비트를 RCC_ClkInitStruct->AHBCLKDivider로 Write한다.
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);
  }

  /*------------------------- SYSCLK Configuration ---------------------------*/
  // 10. RCC_ClkInitStruct->ClockType에 RCC_CLOCKTYPE_SYSCLK이 포함되어 있는지 확인
  if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
  {
    assert_param(IS_RCC_SYSCLKSOURCE(RCC_ClkInitStruct->SYSCLKSource));

    /* HSE is selected as System Clock Source */
    // 11. RCC_ClkInitStruct->SYSCLKSource가 RCC_SYSCLKSOURCE_HSE인지 확인
    // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
    {
      /* Check the HSE ready flag */
      if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
      {
        return HAL_ERROR;
      }
    }
    /* PLL is selected as System Clock Source */
    // 12. RCC_ClkInitStruct->SYSCLKSource가 RCC_SYSCLKSOURCE_PLLCLK인지 확인
    // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    else if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)
    {
      /* Check the PLL ready flag */
      // 13. RCC->CR 레지스터의 PLLRDY 레지스터가 ON인지 확인
      if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
      {
        return HAL_ERROR;
      }
    }
    /* HSI is selected as System Clock Source */
    else
    {
      /* Check the HSI ready flag */
      if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
      {
        return HAL_ERROR;
      }
    }

    // 14. RCC_->CFGR 레지스터의 SW 비트에 RCC_ClkInitStruct->SYSCLKSource를 Write한다.
    // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    __HAL_RCC_SYSCLK_CONFIG(RCC_ClkInitStruct->SYSCLKSource);

    /* Get Start Tick */
    tickstart = HAL_GetTick();

    // 15. RCC->CFGR 레지스터의 SWS 비트가 RCC_SYSCLKSOURCE_PLLCLK 인지 확인
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != (RCC_ClkInitStruct->SYSCLKSource << RCC_CFGR_SWS_Pos))
    {
      if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  #if defined(FLASH_ACR_LATENCY)
  /* Decreasing the number of wait states because of lower CPU frequency */
  // 16.FLASH->ACR , FLASH의 FLASH_ACR 레지스터의 LATENCY 비트 값을 확인  
  if (FLatency < __HAL_FLASH_GET_LATENCY())
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
    if (__HAL_FLASH_GET_LATENCY() != FLatency)
   {
    return HAL_ERROR;
   }
}
#endif /* FLASH_ACR_LATENCY */

/*-------------------------- PCLK1 Configuration ---------------------------*/
// 17. RCC_ClkInitStruct->ClockType 에 RCC_CLOCKTYPE_PCLK1이 있는지 확인
if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
  {
    assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB1CLKDivider));
    //18. RCC->CFGR레지스터의 PPRE1 비트를 RCC_ClkInitStruct->APB1CLKDivider 값으로 Write한다..
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);
  }

  /*-------------------------- PCLK2 Configuration ---------------------------*/
  // 19. (RCC_ClkInitStruct->ClockType에 RCC_CLOCKTYPE_PCLK2이 있는지 확인
  if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
  {
    assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB2CLKDivider));
    // 20. RCC->CFGR 레지스터의 PPRE2 비트에 RCC_ClkInitStruct->APB2CLKDivider 값을 Write한다.
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct->APB2CLKDivider) << 3));
  }

  /* Update the SystemCoreClock global variable */
  // 21, 22, 23. SystemCoreClock = 72000000 >> 0
  // SystemCoreClock가 업데이트 된다.
  SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];

  /* Configure the source of time base considering new system clocks settings*/
  // 24. Systick을 다시 설정한다.
  HAL_InitTick(uwTickPrio);

  return HAL_OK;

~~~

~~~
stm32f103xb.h가 include 되는 과정

1. #include "stm32f1xx_hal.h"

2. #include "stm32f1xx_hal_conf.h"

#ifdef HAL_CORTEX_MODULE_ENABLED
3. #include "stm32f1xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

4. #include "stm32f1xx_hal_def.h"

5. #include "stm32f1xx.h"

#elif defined(STM32F103xB)
  6. #include "stm32f103xb.h"
~~~

~~~
stm32f103xb.h
1. if (FLatency > __HAL_FLASH_GET_LATENCY())

#define __HAL_FLASH_GET_LATENCY()           (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define FLASH                 ((FLASH_TypeDef *)FLASH_R_BASE)
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x00002000UL) /*!< Flash registers base address */
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)
#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region */

typedef struct
{
  __IO uint32_t ACR;
  __IO uint32_t KEYR;
  __IO uint32_t OPTKEYR;
  __IO uint32_t SR;
  __IO uint32_t CR;
  __IO uint32_t AR;
  __IO uint32_t RESERVED;
  __IO uint32_t OBR;
  __IO uint32_t WRPR;
} FLASH_TypeDef;

#define FLASH_ACR_LATENCY_Pos               (0U)                               
#define FLASH_ACR_LATENCY_Msk               (0x7UL << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000007 */
#define FLASH_ACR_LATENCY                   FLASH_ACR_LATENCY_Msk              /*!< LATENCY[2:0] bits (Latency) */
~~~

<br /> ![flash_acr_register.png](/assets/img/blog/flash_acr_register.png.png){: width="500" height="500"}
<br /> ![fcc_latency_value_after_reset.png](/assets/img/blog/fcc_latency_value_after_reset.png.png){: width="500" height="500"}

~~~
// FLatency = FLASH_LATENCY_2
2. __HAL_FLASH_SET_LATENCY(FLatency);

#define FLASH_LATENCY_2                     FLASH_ACR_LATENCY_1       /*!< FLASH Two Latency cycles */
#define FLASH_ACR_LATENCY_1                 (0x2UL << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000002 */
#define FLASH_ACR_LATENCY_Pos               (0U)

#define __HAL_FLASH_SET_LATENCY(__LATENCY__)    (FLASH->ACR = (FLASH->ACR&(~FLASH_ACR_LATENCY)) | (__LATENCY__))
#define FLASH_ACR_LATENCY                   FLASH_ACR_LATENCY_Msk              /*!< LATENCY[2:0] bits (Latency) */
#define FLASH_ACR_LATENCY_Msk               (0x7UL << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000007 */
#define FLASH_ACR_LATENCY_Pos               (0U) 
~~~

~~~
3. __HAL_FLASH_GET_LATENCY()
#define __HAL_FLASH_GET_LATENCY()           (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define FLASH_ACR_LATENCY                   FLASH_ACR_LATENCY_Msk              /*!< LATENCY[2:0] bits (Latency) */
#define FLASH_ACR_LATENCY_Msk               (0x7UL << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000007 */
#define FLASH_ACR_LATENCY_Pos               (0U) 
~~~

~~~
4. if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)

RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

#define RCC_CLOCKTYPE_SYSCLK             0x00000001U /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK               0x00000002U /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1              0x00000004U /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2              0x00000008U /*!< PCLK2 to configure */
~~~

~~~
5. if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2

#define RCC_CLOCKTYPE_SYSCLK             0x00000001U /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK               0x00000002U /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1              0x00000004U /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2              0x00000008U /*!< PCLK2 to configure */
~~~

<br /> ![rcc_cfgr_register](/assets/img/blog/rcc_cfgr_register.png){: width="500" height="500"}

~~~
6. MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV16);
// PPRE1: APB low-speed prescaler (APB1)
/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                   (8U)                              
#define RCC_CFGR_PPRE1_Msk                   (0x7UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000700 */
#define RCC_CFGR_PPRE1                       RCC_CFGR_PPRE1_Msk                /*!< PRE1[2:0] bits (APB1 prescaler) */

#define RCC_HCLK_DIV16                       RCC_CFGR_PPRE1_DIV16 /*!< HCLK divided by 16 */
#define RCC_CFGR_PPRE1_DIV16                 0x00000700U                       /*!< HCLK divided by 16 */

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

~~~

~~~
7. if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2

#define RCC_CLOCKTYPE_SYSCLK             0x00000001U /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK               0x00000002U /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1              0x00000004U /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2              0x00000008U /*!< PCLK2 to configure */
~~~

~~~
8. MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (RCC_HCLK_DIV16 << 3));

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                   (11U)                             
#define RCC_CFGR_PPRE2_Msk                   (0x7UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00003800 */
#define RCC_CFGR_PPRE2                       RCC_CFGR_PPRE2_Msk                /*!< PRE2[2:0] bits (APB2 prescaler) */

#define RCC_HCLK_DIV16                       RCC_CFGR_PPRE1_DIV16 /*!< HCLK divided by 16 */
#define RCC_CFGR_PPRE1_DIV16                 0x00000700U                       /*!< HCLK divided by 16 */

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))
~~~

~~~
9. MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

#define RCC_SYSCLK_DIV1                      RCC_CFGR_HPRE_DIV1   /*!< SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV1                   0x00000000U                       /*!< SYSCLK not divided */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos                    (4U)                              
#define RCC_CFGR_HPRE_Msk                    (0xFUL << RCC_CFGR_HPRE_Pos)       /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                        RCC_CFGR_HPRE_Msk                 /*!< HPRE[3:0] bits (AHB prescaler) */
~~~

~~~
10. if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)

RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

#define RCC_CLOCKTYPE_SYSCLK             0x00000001U /*!< SYSCLK to configure */
~~~

~~~
11. if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
    
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR_SW_HSE /*!< HSE selected as system clock */
#define RCC_CFGR_SW_HSE                      0x00000001U 
~~~

~~~
12. else if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)
 
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL /*!< PLL selected as system clock */
#define RCC_CFGR_SW_PLL                  0x00000002U                       /*!< PLL selected as system clock */
~~~

<br />![rcc_cr_register](/assets/img/blog/rcc_cr_register.png){: width="100%" height="100%"}
~~~
13. if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)

#define RCC_FLAG_PLLRDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_PLLRDY_Pos)) /*!< PLL clock ready flag */
#define CR_REG_INDEX                     ((uint8_t)1)
#define RCC_CR_PLLRDY_Pos                    (25U)                             

#define __HAL_RCC_GET_FLAG(__FLAG__) (((((__FLAG__) >> 5U) == CR_REG_INDEX)?   RCC->CR   : \
                                      ((((__FLAG__) >> 5U) == BDCR_REG_INDEX)? RCC->BDCR : \
                                                                              RCC->CSR)) & (1U << ((__FLAG__) & RCC_FLAG_MASK)))
#define RCC_FLAG_MASK                    ((uint8_t)0x1F)
~~~

~~~
14. __HAL_RCC_SYSCLK_CONFIG(RCC_ClkInitStruct->SYSCLKSource);

RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL /*!< PLL selected as system clock */

#define RCC_CFGR_SW_PLL                  0x00000002U     /*!< PLL selected as system clock */

#define __HAL_RCC_SYSCLK_CONFIG(__SYSCLKSOURCE__) \
                  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, (__SYSCLKSOURCE__))

#define RCC_CFGR_SW_Pos                      (0U)                              
#define RCC_CFGR_SW_Msk                      (0x3UL << RCC_CFGR_SW_Pos)         /*!< 0x00000003 */
#define RCC_CFGR_SW                          RCC_CFGR_SW_Msk                   /*!< SW[1:0] bits (System clock Switch) */

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))
~~~

<br />![rcc_cfgr_before_write_sw](/assets/img/blog/rcc_cfgr_before_write_sw.png){: width="100%" height="100%"}
<br />
<br />![rcc_cfgr_after_write_sw](/assets/img/blog/rcc_cfgr_after_write_sw.png){: width="100%" height="100%"}

~~~
15. while (__HAL_RCC_GET_SYSCLK_SOURCE() != (RCC_ClkInitStruct->SYSCLKSource << RCC_CFGR_SWS_Pos))

RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL /*!< PLL selected as system clock */
#define RCC_CFGR_SW_PLL                  0x00000002U                       /*!< PLL selected as system clock */

#define __HAL_RCC_GET_SYSCLK_SOURCE() ((uint32_t)(READ_BIT(RCC->CFGR,RCC_CFGR_SWS)))
#define RCC_CFGR_SWS_Pos                     (2U)                              
#define RCC_CFGR_SWS_Msk                     (0x3UL << RCC_CFGR_SWS_Pos)  /*!< 0x0000000C */
#define RCC_CFGR_SWS                         RCC_CFGR_SWS_Msk             /*!< SWS[1:0] bits (System Clock Switch Status) */
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
~~~

~~~
16. if (FLatency > __HAL_FLASH_GET_LATENCY())

#define __HAL_FLASH_GET_LATENCY()           (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))

#define FLASH_ACR_LATENCY_Pos               (0U)                               
#define FLASH_ACR_LATENCY_Msk               (0x7UL << FLASH_ACR_LATENCY_Pos)    /*!< 0x00000007 */
#define FLASH_ACR_LATENCY                   FLASH_ACR_LATENCY_Msk              /*!< LATENCY[2:0] bits (Latency) */
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
~~~

~~~
17. if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)

RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

#define RCC_CLOCKTYPE_PCLK1              0x00000004U /*!< PCLK1 to configure */
~~~

~~~
18. MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);

RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2  /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE1_DIV2              0x00000400U          /*!< HCLK divided by 2 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                   (8U)                              
#define RCC_CFGR_PPRE1_Msk                   (0x7UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000700 */
#define RCC_CFGR_PPRE1                       RCC_CFGR_PPRE1_Msk                /*!< PRE1[2:0] bits (APB1 prescaler) */

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))
~~~

~~~
19. if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)

RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

#define RCC_CLOCKTYPE_PCLK2              0x00000008U /*!< PCLK2 to configure */
~~~

~~~
20. MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct->APB2CLKDivider) << 3));

RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

#define RCC_HCLK_DIV1                    RCC_CFGR_PPRE1_DIV1  /*!< HCLK not divided */
#define RCC_CFGR_PPRE1_DIV1              0x00000000U                       /*!< HCLK not divided */
#define RCC_CFGR_PPRE1_DIV2              0x00000400U                       /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE1_DIV4              0x00000500U                       /*!< HCLK divided by 4 */

#define RCC_CFGR_PPRE2_Pos               (11U)                             
#define RCC_CFGR_PPRE2_Msk               (0x7UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00003800 */
#define RCC_CFGR_PPRE2                   RCC_CFGR_PPRE2_Msk                /*!< PRE2[2:0] bits (APB2 prescaler) */

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))
~~~

~~~
21. SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];

uint32_t HAL_RCC_GetSysClockFreq(void)
{
// RCC_CFGR2_PREDIV1SRC 정의 되어 있지 않음
#if defined(RCC_CFGR2_PREDIV1SRC)
  const uint8_t aPLLMULFactorTable[14] = {0, 0, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 13};
  const uint8_t aPredivFactorTable[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
#else
  const uint8_t aPLLMULFactorTable[16] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16};
// RCC_CFGR2_PREDIV1 정의 되어 있지 않음
#if defined(RCC_CFGR2_PREDIV1)
  const uint8_t aPredivFactorTable[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
#else
  const uint8_t aPredivFactorTable[2] = {1, 2};
#endif /*RCC_CFGR2_PREDIV1*/

#endif

  uint32_t tmpreg = 0U, prediv = 0U, pllclk = 0U, pllmul = 0U;
  uint32_t sysclockfreq = 0U;
// RCC_CFGR2_PREDIV1SRC 정의 되어 있지 않음
#if defined(RCC_CFGR2_PREDIV1SRC)
  uint32_t prediv2 = 0U, pll2mul = 0U;
#endif /*RCC_CFGR2_PREDIV1SRC*/

  tmpreg = RCC->CFGR;

  /* Get SYSCLK source -------------------------------------------------------*/
  // RCC->CFGR 레지스터의 SWS 비트 값을 확인
  switch (tmpreg & RCC_CFGR_SWS)
  {
    case RCC_SYSCLKSOURCE_STATUS_HSE:  /* HSE used as system clock */
    {
      sysclockfreq = HSE_VALUE;
      break;
    }
    // RCC->CFGR 레지스터의 SWS 비트는 PLLCLK로 설정되어 있음
    case RCC_SYSCLKSOURCE_STATUS_PLLCLK:  /* PLL used as system clock */
    {
      // #define RCC_CFGR_PLLMULL_Pos                 (18U)                             
      // #define RCC_CFGR_PLLMULL_Msk                 (0xFUL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x003C0000 */
      // #define RCC_CFGR_PLLMULL                     RCC_CFGR_PLLMULL_Msk              /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
      // (tmpreg & RCC_CFGR_PLLMULL) 의 값은    Hex:0x1c0000, Binary:111000000000000000000
      // ((tmpreg & RCC_CFGR_PLLMULL) >> RCC_CFGR_PLLMULL_Pos) 의 값은 Hex:0x7, Binary:111
      // aPLLMULFactorTable[(uint32_t)(tmpreg & RCC_CFGR_PLLMULL) >> RCC_CFGR_PLLMULL_Pos] 의 값은 Decimal:9
      // pllmul 은 9 이다.                 
      pllmul = aPLLMULFactorTable[(uint32_t)(tmpreg & RCC_CFGR_PLLMULL) >> RCC_CFGR_PLLMULL_Pos];

      // #define RCC_CFGR_PLLSRC_Pos                  (16U)                             
      // #define RCC_CFGR_PLLSRC_Msk                  (0x1UL << RCC_CFGR_PLLSRC_Pos)     /*!< 0x00010000 */
      // #define RCC_CFGR_PLLSRC                      RCC_CFGR_PLLSRC_Msk               /*!< PLL entry clock source */
      // RCC->CFGR 레지스터의 PLLSRC 비트의 값이 RCC_PLLSOURCE_HSI_DIV2인지 확인
      // PLLSRC 비트 값은 RCC_PLLSOURCE_HSE 로 설정 되어 있음
      if ((tmpreg & RCC_CFGR_PLLSRC) != RCC_PLLSOURCE_HSI_DIV2)
      {
// RCC_CFGR2_PREDIV1 정의 되어 있지 않음
#if defined(RCC_CFGR2_PREDIV1)
        prediv = aPredivFactorTable[(uint32_t)(RCC->CFGR2 & RCC_CFGR2_PREDIV1) >> RCC_CFGR2_PREDIV1_Pos];
#else
        //  #define RCC_CFGR_PLLXTPRE_Pos                (17U)                             
        // #define RCC_CFGR_PLLXTPRE_Msk                (0x1UL << RCC_CFGR_PLLXTPRE_Pos)   /*!< 0x00020000 */
        // #define RCC_CFGR_PLLXTPRE                    RCC_CFGR_PLLXTPRE_Msk             /*!< HSE divider for PLL entry */
        // (RCC->CFGR & RCC_CFGR_PLLXTPRE)의 값은 0
        // (uint32_t)(RCC->CFGR & RCC_CFGR_PLLXTPRE) >> RCC_CFGR_PLLXTPRE_Pos 의 값은 0
        // aPredivFactorTable[(uint32_t)(RCC->CFGR & RCC_CFGR_PLLXTPRE) >> RCC_CFGR_PLLXTPRE_Pos] 의 값은 1
        //prediv의 값은 1
        prediv = aPredivFactorTable[(uint32_t)(RCC->CFGR & RCC_CFGR_PLLXTPRE) >> RCC_CFGR_PLLXTPRE_Pos];
#endif /*RCC_CFGR2_PREDIV1*/
// RCC_CFGR2_PREDIV1SRC 정의 되어 있지 않음
#if defined(RCC_CFGR2_PREDIV1SRC)

        if (HAL_IS_BIT_SET(RCC->CFGR2, RCC_CFGR2_PREDIV1SRC))
        {
          /* PLL2 selected as Prediv1 source */
          /* PLLCLK = PLL2CLK / PREDIV1 * PLLMUL with PLL2CLK = HSE/PREDIV2 * PLL2MUL */
          prediv2 = ((RCC->CFGR2 & RCC_CFGR2_PREDIV2) >> RCC_CFGR2_PREDIV2_Pos) + 1;
          pll2mul = ((RCC->CFGR2 & RCC_CFGR2_PLL2MUL) >> RCC_CFGR2_PLL2MUL_Pos) + 2;
          pllclk = (uint32_t)(((uint64_t)HSE_VALUE * (uint64_t)pll2mul * (uint64_t)pllmul) / ((uint64_t)prediv2 * (uint64_t)prediv));
        }
        else
        {
          /* HSE used as PLL clock source : PLLCLK = HSE/PREDIV1 * PLLMUL */
          pllclk = (uint32_t)((HSE_VALUE * pllmul) / prediv);
        }

        /* If PLLMUL was set to 13 means that it was to cover the case PLLMUL 6.5 (avoid using float) */
        /* In this case need to divide pllclk by 2 */
        if (pllmul == aPLLMULFactorTable[(uint32_t)(RCC_CFGR_PLLMULL6_5) >> RCC_CFGR_PLLMULL_Pos])
        {
          pllclk = pllclk / 2;
        }
#else
        /* HSE used as PLL clock source : PLLCLK = HSE/PREDIV1 * PLLMUL */
        // #if !defined  (HSE_VALUE)
        //    #define HSE_VALUE    8000000U /*!< Value of the External oscillator in Hz */
        // #endif /* HSE_VALUE */
        // pllmul 는 9, prediv 는 1
        // (uint32_t)((HSE_VALUE  * pllmul) / prediv) 는 Decimal:72000000
        // pllclk 는 72000000
        pllclk = (uint32_t)((HSE_VALUE  * pllmul) / prediv);
#endif /*RCC_CFGR2_PREDIV1SRC*/
      }
      else
      {
        /* HSI used as PLL clock source : PLLCLK = HSI/2 * PLLMUL */
        pllclk = (uint32_t)((HSI_VALUE >> 1) * pllmul);
      }
      // sysclockfreq = 72000000
      sysclockfreq = pllclk;
      break;
    }
    case RCC_SYSCLKSOURCE_STATUS_HSI:  /* HSI used as system clock source */
    default: /* HSI used as system clock */
    {
      sysclockfreq = HSI_VALUE;
      break;
    }
  }
  return sysclockfreq;
}
~~~

~~~
22. AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos]

const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

#define RCC_CFGR_HPRE_Pos                    (4U)                              
#define RCC_CFGR_HPRE_Msk                    (0xFUL << RCC_CFGR_HPRE_Pos)       /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                        RCC_CFGR_HPRE_Msk                 /*!< HPRE[3:0] bits (AHB prescaler) */

(RCC->CFGR & RCC_CFGR_HPRE) 는 Decimal:0
(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos 는 Decimal:0
AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos] 는 Decimal:0
~~~

~~~
23. SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
    SystemCoreClock = 72000000 >> 0
~~~

~~~
24. SystTick 설정
__weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /* Configure the SysTick to have interrupt in 1ms time basis*/
  // Systick 분석 내용 참고
  if (HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)
  {
    return HAL_ERROR;
  }

  /* Configure the SysTick IRQ priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
    HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);
    uwTickPrio = TickPriority;
  }
  else
  {
    return HAL_ERROR;
  }

  /* Return function status */
  return HAL_OK;
}
~~~