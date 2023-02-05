---
layout: post
title: Set Interrupt Group 분석
tags: hal_init
related_posts:
- 
---

**HAL_Init에서 PriorityGrouping 설정하는 코드를 분석해 보았다.**

~~~
HAL_StatusTypeDef HAL_Init(void)
{
  /* Configure Flash prefetch */
#if (PREFETCH_ENABLE != 0)
#if defined(STM32F101x6) || defined(STM32F101xB) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F102x6) || defined(STM32F102xB) || \
    defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)

  /* Prefetch buffer is not available on value line devices */
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif
#endif /* PREFETCH_ENABLE */

  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
  HAL_InitTick(TICK_INT_PRIORITY);

  /* Init the low level hardware */
  HAL_MspInit();

  /* Return function status */
  return HAL_OK;
}
~~~

~~~
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  /* Check the parameters */
  assert_param(IS_NVIC_PRIORITY_GROUP(PriorityGroup));
  
  /* Set the PRIGROUP[10:8] bits according to the PriorityGroup parameter value */
  NVIC_SetPriorityGrouping(PriorityGroup);
}
~~~

<br /> ![stm32_System_control_block](/assets/img/blog/stm32_System_control_block.png){: width="500" height="500"}
<br /> ![scb_aircr_register](/assets/img/blog/scb_aircr_register.png){: width="500" height="500"}
<br /> ![binary_point](/assets/img/blog/binary_point.png){: width="500" height="500"}
<br /> ![interrupt_priority_groupping](/assets/img/blog/interrupt_priority_groupping.png){: width="500" height="500"}

~~~

#define NVIC_PRIORITYGROUP_4         0x00000003U /*!< 4 bits for pre-emption priority
                                                      0 bits for subpriority */
/**
  \brief   Set Priority Grouping
  \details Sets the priority grouping field using the required unlock sequence.
           The parameter PriorityGroup is assigned to the field SCB->AIRCR [10:8] PRIGROUP field.
           Only values from 0..7 are used.
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]      PriorityGroup  Priority grouping field.
 */
__STATIC_INLINE void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);             /* only values 0..7 are used          */

  reg_value  =  SCB->AIRCR;                                                   /* read old register configuration    */
  reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk)); /* clear bits to change               */
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                (PriorityGroupTmp << SCB_AIRCR_PRIGROUP_Pos) );               /* Insert write key and priority group */
  SCB->AIRCR =  reg_value;
}
~~~

~~~

#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */

#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

/* Memory mapping of Core Hardware */
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */


/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  __IM  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IOM uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __IOM uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __IOM uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IOM uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __IOM uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __IOM uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __IOM uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __IOM uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __IOM uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __IOM uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __IOM uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __IOM uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __IOM uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __IM  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __IM  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __IM  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __IM  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __IM  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
        uint32_t RESERVED0[5U];
  __IOM uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;
~~~

~~~
/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_Pos              16U                                            /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)            /*!< SCB AIRCR: VECTKEY Mask */

#define SCB_AIRCR_PRIGROUP_Pos              8U                                            /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */

~~~

인터럽트는 우선순위를 갖고 있으며 우선 순위는 IPR 레지스터의 Preemption Priority 와 Sub Priority 로 관리된다. 아래와 같이 5개의 그룹으로 나눠지며
그룹에 따라서 Preemption Priority 와 Sub Priority 의 개수가 변경된다.

<br /> ![binary_point](/assets/img/blog/binary_point.png){: width="500" height="500"}

위의 예시 코드는 0x03으로 그룹을 설정했기 때문에 첫번째 그룹으로 볼 수있다.

pre-emption group을 group priority로서 사용

pre-emption priority와 subpriority의 인터럽트 동작을 확인해 보았다.

pre-emption priority 는 ISR 간 선점 동작에 적용된다. 

subpriority 는 Pending 된 ISR 이 수행되는 우선순위만을 결정하며 선점 동작에는 영향을 미치지 않는다. 

아래의 예시 그림을 보면 Priority 는 pre-emption priority 이다. "A" ISR 이 우선순위가 가장 낮고, "C" ISR 이 우선순위가 가장 높다.  우
선 순위가 높은 ISR 이 프로세서를 선점하고 동작하는 Nested 형태의 동작을 확인할 수 있다.

<br /> ![nested_priority](/assets/img/blog/nested_priority.png){: width="500" height="500"}

아래의 예는 subpriority 동작의 예이다. pre-emption priority 는 동일하며 subpriority 만 다른 케이스이다. 

최초 A 가 수행되고 있는 시점에서 B 와 C 인터럽트 가 발생하더라도 우선순위가 동일하기 때문에 프로세서를 선점하지 못한다. 
A 가 완료되면 Pending 되고 있던 B 와 C 중에서 subpriority 가 높은 C 가 먼저 수행되고 이 후 B가 수행되는 시나리오를 확인할 수 있다.

<br /> ![sub_priority.png](/assets/img/blog/sub_priority.png){: width="500" height="500"}

참고 링크:

[https://www.st.com/resource/en/programming_manual/pm0056-stm32f10xxx20xxx21xxxl1xxxx-cortexm3-programming-manual-stmicroelectronics.pdf](https://www.st.com/resource/en/programming_manual/pm0056-stm32f10xxx20xxx21xxxl1xxxx-cortexm3-programming-manual-stmicroelectronics.pdf)

[https://m.blog.naver.com/eziya76/221428695204](https://m.blog.naver.com/eziya76/221428695204)

[https://www.youtube.com/watch?v=lxyfIY_Wzjg](https://www.youtube.com/watch?v=lxyfIY_Wzjg)

[https://www.youtube.com/watch?v=uFBNf7F3l60&t=2s](https://www.youtube.com/watch?v=uFBNf7F3l60&t=2s)

[https://www.youtube.com/watch?v=K0vmH2YGbOY](https://www.youtube.com/watch?v=K0vmH2YGbOY)

[https://www.amazon.com/Embedded-Cortex-M-Microcontrollers-Assembly-Language/dp/0982692676/][https://www.amazon.com/Embedded-Cortex-M-Microcontrollers-Assembly-Language/dp/0982692676/]



