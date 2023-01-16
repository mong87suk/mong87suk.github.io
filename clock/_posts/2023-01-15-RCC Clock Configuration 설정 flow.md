---
layout: post
title: RCC Clock Configuration 설정 flow
tags: systick_sequence
related_posts:
- 
---

<br />![clock_setting](/assets/img/blog/clock_setting.png){: width="100%" height="100%"}

1. FLASH_ACR(Flash access control register) 레지스터의 LATENCY 비트로  flash access time을 확인
   - These bits represent the ratio of the SYSCLK (system clock) period to the Flash access time.

2. FLASH_ACR레지스터의 LATENCY 비트로 flash access time을 세팅한다.

3. RCC_CFGR 레지스터의 PPRE1(APB1: APB Low-speed prescaler) 비트에 RCC_HCLK_DIV16을 세팅한다.

4. RCC_CFGR 레지스터의 PPRE2(APB2: APB high-speed prescaler)비트에 RCC_HCLK_DIV16을 세팅한다.

5. RCC_CFGR 레지스터의 HPRE(AHB prescaler)비트에 RCC_SYSCLK_DIV1 을 세팅한다. HCLK(AHB Clock) 비율을 설정 한다.
<br />   => AHB로 부터 APB1, APB2로 클락이 공급 된다. APB1, APB2로 클락을 RCC_HCLK_DIV16 최대로 줄 였을 비율로 
<br />      AHB 클락이 공급 되는지 확인하기 위함이다.

6. System Clock Source가 PLLCLK인지 확인 (RCC_ClkInitStruct->SYSCLKSource)

7. RCC_CR 레지스터의 PLLRDY 비트가  ON 확인

8. RCC_CFGR  레지스터의 SW(System clock Switch)비트에  RCC_SYSCLKSOURE_PLLCLK를 Write 한다.

9. RCC_CFGR  레지스터의 SWS(System clock switch status)비트가  RCC_SYSCLKSOURE_PLLCLK인지 확인

10. FLASH_ACR레지스터의 LATENCY 비트로  flash access time을 확인

11. RCC_CFGR 레지스터의 PPRE1(APB1: APB Low-speed prescaler, PCLK1) 비트에 RCC_ClkInitStruct->APB1CLKDivider을 세팅한다.

12. RCC_CFGR 레지스터의 PPRE2(APB2: APB high-speed prescaler, PCLK2)비트에 RCC_ClkInitStruct->APB2CLKDivider을 세팅한다.

13. RCC_CFGR 레지스터의 SWS 비트 값을 확인

14. RCC_CFGR 레지스터의 SWS 비트가 PLLCLK로 된 것을 확인

15. RCC_CFGR 레지스터의 PLLMULL 비트 값에 해당하는 배수의 값을 aPLLMULFActorTable 배열에서 찾은 후 pllmul에 대입한다. pllmul = 9

16. RCC_CFGR 레지스터의 PLLXTPRE 비트 값에 해당하는 배수의 값을 aPredivFactorTable 배열에서 찾은 후 prediv에 대입한다. prediv = 1

17. SystemClock  = (uint32_t)((HSE_VALUE  * pllmul) / prediv);   #define HSE_VALUE    8000000U  // 시스템 클락을 계산

18. 시스템 클락을 이용하여 아래와 같은 계산식을 이용하여 시스템 코어 클락을 계산한다.

19. RCC_CFGR 레지스터의  HPRE: AHB prescaler 값을 확인하여 SystemCoreClock을 계산

20. SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];

21. SystemCoreClock = 72000000 >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];

22. SystemCoreClock = 72000000 >> 0;








