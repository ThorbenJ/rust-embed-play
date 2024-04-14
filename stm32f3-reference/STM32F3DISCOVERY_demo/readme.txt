/**
  @page Demo_Binary   Description of the STM32F3DISCOVERY Demo firmware's binary files
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the STM32F3DISCOVERY Demo firmware's binary files.
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics. All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license SLA0044,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                       http://www.st.com/SLA0044
  *
  ******************************************************************************
  @endverbatim

@par Example Description 

This demonstration firmware is based on STM32Cube. It helps you to discover
STM32 Cortex-M devices that can be plugged on a STM32 Discovery board.

- Connect the STM32F3DISCOVERY board to a PC with a 'USB type A to Mini-B' cable
through USB connector CN2 or CN1 to power the board. Then red led LD1 (PWR) and
LD2 (COM) light up.
- All 8 leds between B1 and B2 are blinking as chenillard.
- At reset when the Button B1 is Pressed more than 1 second the USB Test is executed, Othersiwse
the demo is launched.
- The USB test consist on moving the cursor in square path. The led is On corresponding to such direction.

- The following sequence describes the demo application:
	- Press User Button B1 (Button left corner of the board) then gyroscope MEMS sensor is
      enabled, move the board horizontally and observe the four LEDs blinking according to the motion
      direction.
	- Then if the you Press User Button B1 (Button left corner of the board) then Accelerometer MEMS 
	  sensor is enabled, move the board and observe the four LEDs blinking according to the acceleration.
	- Finally Pressing the User Button B1, enables the USB USER and convert the STM32F3-Discovery 
	  board as a standard mouse (LD3, LD6, LED7 and LED10 are ON). Connect a second USB type A/mini-B cable 
	  through the USB User connector and the PC, then move the board (horizontally and vertically)and see 
	  the mouse cursor moves according to the motion direction.
	- Pressing the Press User Button B1, to re launch the demo application.

@par Hardware and Software environment

  - This Demo runs on STM32F303xC devices.
  
  - This example has been tested with STMicroelectronics STM32F3DISCOVERY (MB1035) 
    and can be easily tailored to any other supported device and development board.

  - STM32F3DISCOVERY board Set-up
    - None.
      
@par How to use

You can use any in-system programming tool to reprogram the demonstration 
using these binary files, as described below:

 + Using "in-system programming tool" such as ST-Link Utility
    - Connect the STM32F3DISCOVERY board to a PC with a 'USB type A to Mini-B'
      cable through USB connector CN1 to power the board.
    - Make sure that the embedded ST-LINK/V2 is configured for in-system programming
      (both CN4 jumpers ON)
    - Use "STM32CubeF3_Demo_STM32F3-DISCOVERY.hex" binary with your preferred 
      in-system programming tool to reprogram the demonstration firmware 
      (ex. STM32 ST-LINK Utility, available for download from www.st.com).

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
