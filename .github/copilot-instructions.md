# STM32F407 RTC Alarm Clock Project

## Project Overview
This is an ECE 330L embedded systems final project implementing an RTC-based alarm clock on the STM32F407VG Discovery board with a custom peripheral board featuring 8x 7-segment displays. The system displays date/time, allows setting clock and alarm via potentiometer/buttons, and plays audio alarms.

## Hardware Architecture

**Target MCU:** STM32F407VGT6 (ARM Cortex-M4, 168MHz, 1MB Flash, 192KB RAM)

**Key Peripherals Used:**
- **RTC (Real-Time Clock):** LSI oscillator (32kHz), prescalers set for 1Hz calendar tick (sync=258, async=127)
- **GPIO:** Port D (audio + 4 LEDs), Port E (7-segment displays), Port C (buttons/switches), Port A (analog inputs)
- **ADC1:** Potentiometer input on PA0-PA3 for setting time/alarm values
- **TIM7:** 250kHz → 125kHz PWM for audio tone generation and LED dimming (PSC=199, ARR=1)
- **7-Segment Displays:** 8 multiplexed digits on Port E (PE0-PE7 segments, PE8-PE15 digit select)

## Code Organization

```
Core/Src/main.c         - Main application logic, RTC init, mode switching
Core/Src/seg7.c         - 7-segment display driver (character lookup + output)
Core/Src/stm32f4xx_it.c - Interrupt handlers (SysTick, TIM7, RTC Alarm)
Core/Inc/main.h         - Music note frequencies (C0-B6), note durations, Music struct
Core/Inc/seg7.h         - Character constants (CHAR_0-CHAR_Z, SPACE, DASH, etc.)
```

## Critical RTC Initialization Sequence

**Must follow this order strictly (see `instructions.txt` for rationale):**
1. Enable RTC register access: `PWR->CR |= (1 << 8)` 
2. Unlock write protection: Write `0xCA` then `0x53` to `RTC->WPR`
3. Enter init mode: Set `RTC->ISR |= (1 << 7)`, poll `INITF` bit (bit 6)
4. Configure clock source: `RCC->BDCR` bits 8-9 = `10` (LSI), bit 15 = `1` (enable)
5. Set prescalers: `RTC->PRER = 0x007F0102` (sync=258, async=127)
6. Configure time/date in `RTC->TR`/`RTC->DR` using BCD format
7. Exit init mode: Clear `INIT` bit in `RTC->ISR`

**Alarm Setup:** Disable alarm (`RTC->CR` clear ALRAE/ALRBIE), poll ALRAWF/ALRBWF, program registers, re-enable alarm.

## Key Conventions

### BCD Time/Date Format
- `RTC->TR` (time): bits [21:20]=hour tens, [19:16]=hour ones, [14:12]=min tens, [11:8]=min ones, [6:4]=sec tens, [3:0]=sec ones
- `RTC->DR` (date): bits [15:13]=day-of-week (1=Mon), [12]=month tens, [11:8]=month ones, [5:4]=date tens, [3:0]=date ones

### 7-Segment Display Protocol
- `Seven_Segment(uint32_t HexValue)`: Displays 8 hex digits (4 bits each, nibble = one digit)
- Character encoding: Use `seg7.h` constants (e.g., `CHAR_H`, `CHAR_E`, `CHAR_L`, `CHAR_O`) shifted into position
- Example: `(CHAR_M) | (CHAR_O << 4) | (CHAR_N << 8)` displays "MON" on digits 0-2

### Music System
- **Music struct** (in `main.h`): `{int note, int size, int tempo, int space, char end}`
- `note`: Timer count for pitch (e.g., `A4=142` → 440Hz), defined in `main.h` as `C0` through `B6`
- `size`: Note duration divisor (`quarter=4`, `_8th=8`, `whole=1`)
- `tempo`: Base timing unit (typical: 1400)
- `space`: Silent gap after note (10 = short)
- `end`: 1 = last note in song array
- Tone generation: TIM7 interrupt toggles `GPIOD->ODR` bit 0 at note frequency

### Timing Architecture
- **TIM7 ISR (125kHz):** Music playback, PWM LED dimming (via `ramp` counter), marquee animation
- **SysTick ISR (1kHz):** RTC alarm polling (check `RTC->ISR` bit 8), HAL tick, vibrato effect

## User Interaction Modes

### Time Display Mode (PC0 = LOW)
Cycles through 3 pages on 7-segment display:
1. Time: `HH:MM:SS` (24-hour format)
2. Day of week: 3-letter abbreviation (`MON`, `TUE`, etc.)
3. Date: `MM:DD:YY`

### Time/Date Setting Mode (PC0 = HIGH)
- Potentiometer (ADC1) sets value for current field
- Fields: Hour → Minute → Second → Day-of-Week → Month → Date
- PC10 button: Advance to next field (with 200ms debounce)
- Values written directly to `RTC->TR`/`RTC->DR` with BCD conversion

## Build System
- **IDE:** STM32CubeIDE 1.19.0
- **Build Output:** `Debug/` directory with `.list`, `.elf`, `.map` files
- **Linker Scripts:** `STM32F407VGTX_FLASH.ld` (main), `STM32F407VGTX_RAM.ld` (debug)
- `.ioc` file: STM32CubeMX configuration (regenerate HAL init code via Tools → Code Generation)

## Common Pitfalls
- **RTC not running:** Forgot to set `PWR->CR` bit 8 before accessing RTC registers
- **Incorrect time:** BCD format required (not binary) - convert with `tens*10 + ones`
- **Alarm not triggering:** Enable ALRAE in `RTC->CR`, check ISR bit 8 in SysTick (not using EXTI17)
- **7-seg garbled:** Missing latch pulse (`GPIOE->ODR |= 0xFF00` after character write)
- **Music stuttering:** `INDEX` not reset to 0 after song ends, or `end` flag not set on last note

## Testing/Debug Patterns
- Use onboard LEDs (LD3-LD6 on PD12-PD15) for state indication during debugging
- Monitor RTC registers via live expressions: `RTC->TR`, `RTC->DR`, `RTC->ISR`
- HAL_Delay() available but avoid in ISRs - use counters like `Delay_counter` in SysTick instead
