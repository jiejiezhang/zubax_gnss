/*
 * Copyright (C) 2014-2015  Zubax Robotics  <info@zubax.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#include "board.hpp"
#include <cstring>
#include <ch.hpp>
#include <hal.h>
#include <unistd.h>

/**
 * GPIO config for ChibiOS PAL driver
 */
const PALConfig pal_default_config = {
#if STM32_HAS_GPIOA
  {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR,
   VAL_GPIOA_ODR,   VAL_GPIOA_AFRL,   VAL_GPIOA_AFRH},
#endif
#if STM32_HAS_GPIOB
  {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR,
   VAL_GPIOB_ODR,   VAL_GPIOB_AFRL,   VAL_GPIOB_AFRH},
#endif
#if STM32_HAS_GPIOC
  {VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR, VAL_GPIOC_PUPDR,
   VAL_GPIOC_ODR,   VAL_GPIOC_AFRL,   VAL_GPIOC_AFRH},
#endif
#if STM32_HAS_GPIOD
  {VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR, VAL_GPIOD_PUPDR,
   VAL_GPIOD_ODR,   VAL_GPIOD_AFRL,   VAL_GPIOD_AFRH},
#endif
#if STM32_HAS_GPIOE
  {VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR, VAL_GPIOE_PUPDR,
   VAL_GPIOE_ODR,   VAL_GPIOE_AFRL,   VAL_GPIOE_AFRH},
#endif
#if STM32_HAS_GPIOF
  {VAL_GPIOF_MODER, VAL_GPIOF_OTYPER, VAL_GPIOF_OSPEEDR, VAL_GPIOF_PUPDR,
   VAL_GPIOF_ODR,   VAL_GPIOF_AFRL,   VAL_GPIOF_AFRH},
#endif
#if STM32_HAS_GPIOG
  {VAL_GPIOG_MODER, VAL_GPIOG_OTYPER, VAL_GPIOG_OSPEEDR, VAL_GPIOG_PUPDR,
   VAL_GPIOG_ODR,   VAL_GPIOG_AFRL,   VAL_GPIOG_AFRH},
#endif
#if STM32_HAS_GPIOH
  {VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR, VAL_GPIOH_PUPDR,
   VAL_GPIOH_ODR,   VAL_GPIOH_AFRL,   VAL_GPIOH_AFRH},
#endif
#if STM32_HAS_GPIOI
  {VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR, VAL_GPIOI_PUPDR,
   VAL_GPIOI_ODR,   VAL_GPIOI_AFRL,   VAL_GPIOI_AFRH}
#endif
};

/// Provided by linker
const extern std::uint8_t DeviceSignatureStorage[];

namespace board
{


#define PAL_STM32_ALTERNATE(n)          ((n) << 7U)

#define PAL_MODE_ALTERNATE(n)           (PAL_STM32_MODE_ALTERNATE |         \
                                         PAL_STM32_ALTERNATE(n))

#if 0
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
	palWritePad(GPIOF, 9, 0);
	palWritePad(GPIOF, 10, 1);
	chThdSleepMilliseconds(500);
	palWritePad(GPIOF, 9, 1);
	palWritePad(GPIOF, 10, 0);
	chThdSleepMilliseconds(500);
  }
}
#endif

os::watchdog::Timer init(unsigned wdt_timeout_ms)
{
    /*
     * OS
     */
    halInit();
    chSysInit();
    sdStart(&STDOUT_SD, nullptr);

#if 0
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
#endif

    /*
     * Watchdog
     */
    os::watchdog::init();
    os::watchdog::Timer wdt;
    wdt.startMSec(wdt_timeout_ms);

    /*
     * Version check
     */
    const auto hw_ver = detectHardwareVersion();
#if 0
    if (hw_ver.major != 2)
    {
        chibios_rt::System::halt("BAD HW");
    }

    if (hw_ver.minor != 2)
    {
        chibios_rt::System::halt("BAD HW");
    }
#endif

    /*
     * Hardware
     */
    setCANLed(0, false);        // Default state
    setCANLed(1, false);

    {
        os::CriticalSectionLocker locker;

        RCC->APB1ENR  |=  RCC_APB1ENR_CAN1EN;           // CAN1
        RCC->APB1RSTR |=  RCC_APB1RSTR_CAN1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
        // CAN2 is not used
    }

    /*
     * Prompt
     */
    os::lowsyslog(PRODUCT_NAME_STRING " Bootloader %d.%d %d.%d.%08x / %s\n",
                  hw_ver.major, hw_ver.minor,
                  BL_VERSION_MAJOR, BL_VERSION_MINOR, GIT_HASH,
                  watchdogTriggeredLastReset() ? "WDTRESET" : "OK");

    return wdt;
}

void setCANLed(unsigned iface_index, bool state)
{
    switch (iface_index)
    {
    case 0:
    {
        palWritePad(GPIO_PORT_LED_CAN1, GPIO_PIN_LED_CAN1, state);
        break;
    }
    case 1:
    {
        palWritePad(GPIO_PORT_LED_CAN2, GPIO_PIN_LED_CAN2, state);
        break;
    }
    default:
    {
        break;
    }
    }
}

void setStatusLed(bool state)
{
    palWritePad(GPIO_PORT_LED_STATUS, GPIO_PIN_LED_STATUS, state);
}

void restart()
{
    NVIC_SystemReset();

    for (;;) { }    // No return
}

void bootApplication()
{
    // Loading and validating the application entry point
    const unsigned stacktop   = *reinterpret_cast<unsigned*>(FLASH_BASE + APPLICATION_OFFSET);
    const unsigned entrypoint = *reinterpret_cast<unsigned*>(FLASH_BASE + APPLICATION_OFFSET + 4);

    if ((stacktop <= SRAM_BASE) ||
        (stacktop > (SRAM_BASE + 1024 * 1024)))
    {
        chibios_rt::System::halt("STACKTOP");
    }

    if ((entrypoint < (FLASH_BASE + APPLICATION_OFFSET)) ||
        (entrypoint >= (FLASH_BASE + getFlashSize())))
    {
        chibios_rt::System::halt("ENTRYPOINT");
    }

    // We cordially extend our thanks to David Sidrane and Ben Dyer, whose ideas have somewhat inspired this thing.
    asm volatile("cpsid i");

    // Deinit all peripherals that may have been used
    // It is crucial to disable ALL peripherals, else a spurious interrupt will crash the application
    RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST | RCC_APB1RSTR_CAN2RST | RCC_APB1RSTR_USART3RST;
    RCC->AHB1RSTR |= RCC_AHB2RSTR_OTGFSRST;

    // Kill the sys tick
    SysTick->CTRL = 0;

    // Update the vector table location
    asm volatile("dsb");
    asm volatile("isb");
    SCB->VTOR = FLASH_BASE + APPLICATION_OFFSET;
    asm volatile("dsb");

    // Let's roll!
    asm volatile("msr msp, %[stacktop]          \n"
                 "bx       %[entrypoint]        \n"
                 :: [stacktop] "r"(stacktop), [entrypoint] "r"(entrypoint):);

    for (;;) { }        // Noreturn
}

UniqueID readUniqueID()
{
    UniqueID out_bytes;
    std::memcpy(out_bytes.data(), reinterpret_cast<const void*>(0x1FFFF7E8), std::tuple_size<UniqueID>::value);
    return out_bytes;
}

bool tryReadDeviceSignature(DeviceSignature& out_sign)
{
    std::memcpy(out_sign.data(), &DeviceSignatureStorage[0], std::tuple_size<DeviceSignature>::value);

    bool valid = false;
    for (auto x : out_sign)
    {
        if (x != 0xFF && x != 0x00)          // All 0xFF/0x00 is not a valid signature, it's empty storage
        {
            valid = true;
            break;
        }
    }

    return valid;
}

HardwareVersion detectHardwareVersion()
{
    auto v = HardwareVersion();

    v.major = HW_VERSION_MAJOR;

    if (palReadPad(GPIO_PORT_HWID, GPIO_PIN_HWID_BIT0))
    {
        v.minor += 1;
    }

    if (!palReadPad(GPIO_PORT_HWID, GPIO_PIN_HWID_BIT1_INVERSE))
    {
        v.minor += 2;
    }

    if (palReadPad(GPIO_PORT_HWID, GPIO_PIN_HWID_BIT2))
    {
        v.minor += 4;
    }

    return v;
}

} // namespace board


namespace os
{

void applicationHaltHook()
{
    // Panic indication
    board::setStatusLed(true);
    board::setCANLed(0, true);
    board::setCANLed(1, true);
}

}

/*
 * Early init from ChibiOS
 */
extern "C"
{

void __early_init(void)
{
    stm32_clock_init();
}

#define PAL_MODE_STM32_ALTERNATE_PUSHPULL   16
#define PAL_MODE_STM32_ALTERNATE_OPENDRAIN  17
void boardInit(void)
{
#if 0
    uint32_t mapr = AFIO->MAPR;
    mapr &= ~AFIO_MAPR_SWJ_CFG; // these bits are write-only

    // Enable SWJ only, JTAG is not needed at all:
    mapr |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    AFIO->MAPR = mapr | AFIO_MAPR_CAN_REMAP_REMAP2;
#endif

	palSetPadMode(GPIOF, 9, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);	//led
	palSetPadMode(GPIOF, 10, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);

	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7)); //uart
  	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
	
    /*
     * Enabling the CAN controllers, then configuring GPIO functions for CAN_TX.
     * Order matters, otherwise the CAN_TX pins will twitch, disturbing the CAN bus.
     * This is why we can't perform this initialization using ChibiOS GPIO configuration.
     *
     * NOTE: Check this - the problem may only appear when CAN pin remapping is used,
     *       because ChibiOS initializes AFIO after GPIO.
     */
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    palSetPadMode(GPIOB, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    // Enabling CAN2 too, in order to avoid bus disruptions
    RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
    palSetPadMode(GPIOB, 13, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
}

}
