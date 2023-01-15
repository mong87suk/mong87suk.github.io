---
layout: post
title: RCC Oscillators 설정 flow
tags: systick_sequence
sitemap: false
related_posts:
- 
---

<br />![clock_setting](/assets/img/blog/clock_setting.png){: width="100%" height="100%"}

1. 오실레이터 타입 확인

2. RCC_CFGR 레지스터의 SWS(System clock switch status) 비트 확인 (시스템 클락이 HSE 또는 PLL로 사용 중 인지 확인)

3. RCC_CR  레지스터의 HSEON(HSE clock enable) ENABLE (HSE 를 ENABLE 시킨다.)

4. RCC_CFGR 레지스터의 SWS(System clock switch status)  비트 확인 시스템 클락이 PLL로 사용 중 인지 확인

5. RCC_CR 레지스터의  PLLON(PLL enable) bit 값을 0 (Disable)로 세팅

6. RCC_CFGR 레지스터의  PLLXTPRE(HSE divider for PLL entry) bit 값을 0(HSE clock not divided)로 세팅

7. RCC_CFGR 레지스터의  PLLSRC(PLL entry clock source) bit 값을 1 (HSE oscillator clock selected as PLL input clock 으로 변경)

8. RCC_CFGR 레지스터의  PLLMUL(PLL multiplication factor) bit 값을 0111 (PLL input clock x 9) 로 write

9. RCC_CR 레지스터의 PLLON(PLL enable) bit 값을 1 로 세팅(PLL ON)