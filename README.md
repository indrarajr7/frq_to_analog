# STM32 Frequency-to-Analog Converter

STM32 HAL firmware that measures an incoming pulse frequency using **TIM1 Input Capture** and converts the measured frequency into a **0–3.3 V analog output** through an external I2C DAC.

The output scaling is selected using GPIO range-selection inputs.

## Features

- Frequency measurement using **TIM1 Channel 1 Input Capture**
- Rising-edge pulse measurement
- Frequency-to-voltage conversion
- External **12-bit I2C DAC**
- 0–3.3 V output scaling
- Selectable frequency ranges:
  - 500 Hz
  - 1 kHz
  - 1.5 kHz
  - 2 kHz
  - 2.5 kHz
  - 5 kHz
  - Custom range (currently 276 Hz)
- Input signal LED indication
- TIM16 periodic timeout/status handling
- Developed using **STM32 HAL / STM32CubeMX generated project structure**

## How It Works

The firmware measures the time between two consecutive rising edges of the input signal.

```text
Input Pulse Signal
        │
        ▼
TIM1 CH1 Input Capture
        │
        ▼
Measure Period
        │
        ▼
Calculate Frequency (pps)
        │
        ▼
Select Frequency Range
        │
        ▼
Scale Frequency → 0–3.3 V
        │
        ▼
I2C DAC
        │
        ▼
Analog Output
```

## Frequency-to-Voltage Mapping

For a selected maximum frequency `FMAX`:

```text
VOUT = 3.3 × Frequency / FMAX
```

The output is limited to 3.3 V when the measured frequency exceeds the selected range.

For example, for the 1 kHz range:

| Input Frequency | Output Voltage |
|---:|---:|
| 0 Hz | 0.000 V |
| 250 Hz | 0.825 V |
| 500 Hz | 1.650 V |
| 750 Hz | 2.475 V |
| 1000 Hz | 3.300 V |
| >1000 Hz | 3.300 V |

## Supported Ranges

| Selection Input | Maximum Frequency | Output |
|---|---:|---|
| `_500_HZ_Pin` | 500 Hz | 0–500 Hz → 0–3.3 V |
| `_1000_HZ_Pin` | 1000 Hz | 0–1000 Hz → 0–3.3 V |
| `_1500_HZ_Pin` | 1500 Hz | 0–1500 Hz → 0–3.3 V |
| `_2000_HZ_Pin` | 2000 Hz | 0–2000 Hz → 0–3.3 V |
| `_2500_HZ_Pin` | 2500 Hz | 0–2500 Hz → 0–3.3 V |
| `_5000_HZ_Pin` | 5000 Hz | 0–5000 Hz → 0–3.3 V |
| `CUSTOM_FREQ_Pin` | 276 Hz* | 0–276 Hz → 0–3.3 V |

> **Note:** The custom range is currently hard-coded to **276 Hz** in `freq_map()`. Change this value to match the required application.

## DAC Interface

The firmware communicates with the external DAC using **I2C1**.

The DAC is treated as a **12-bit device**:

```text
DAC code: 0 ... 4095
Voltage : 0 ... 3.3 V
```

Voltage conversion in the firmware:

```c
value = ((4095 * volt) / 3.3);
```

The generated 12-bit value is transmitted as two bytes:

```c
frame[0] = (value >> 8) & 0xFF;
frame[1] = value & 0xFF;
```

The source uses the I2C address:

```c
0xC2
```

### DAC Functions

```c
uint8_t dac_set_val(float volt);
uint8_t dac_set_with_range(uint32_t val, uint32_t range);
```

`dac_set_val()` sets the requested output voltage.

`dac_set_with_range()` converts a value within a specified range directly to the corresponding 12-bit DAC value.

## Frequency Measurement

TIM1 is configured for input capture on Channel 1.

Important configuration:

```c
Prescaler       = 1000
Period          = 65535
Counter Mode    = Up
Capture Edge    = Rising
Capture Prescaler = DIV1
Input Filter    = 0
```

Two capture values are used:

```c
c1
c2
```

The timer difference is calculated with overflow handling:

```c
if(c2 >= c1) {
    intvl = c2 - c1;
}
else {
    intvl = c2 + (65536 - c1);
}
```

The resulting frequency is stored in:

```c
volatile uint32_t pps;
```

The capture callback is:

```c
HAL_TIM_IC_CaptureCallback()
```

## Signal LED

The input-signal LED is controlled using:

```c
INP_SIG_LED_GPIO_Port
INP_SIG_LED_Pin
```

When a TIM1 capture interrupt occurs:

```c
led_flag = 1;
```

TIM16 periodically processes the flag.

The current behavior is:

- `led_flag == 1` → LED ON
- `led_flag == 2` → LED toggled
- `led_flag == 0` → LED OFF and `pps = 0`

This provides a basic indication of whether the input pulse signal is being received.

## TIM16

TIM16 is used as a periodic interrupt timer.

Configuration:

```c
Prescaler = 47999
Period    = 999
```

With a 48 MHz system clock, this is intended to generate approximately a **1 second periodic interrupt**.

Started with:

```c
HAL_TIM_Base_Start_IT(&htim16);
```

Callback:

```c
HAL_TIM_PeriodElapsedCallback()
```

## System Clock

The firmware configures the MCU to run from the internal HSI oscillator with PLL.

Main configuration:

```text
HSI              : ON
PLL              : ON
PLL Multiplier   : 12
System Clock     : 48 MHz
AHB Divider      : 1
APB1 Divider     : 1
```

The application defines:

```c
#define CLK_FREQ  (48000000UL)
#define REF_VLT   (3.30F)
```

## GPIO Configuration

The frequency-selection inputs are configured as digital inputs:

```c
GPIO_MODE_INPUT
GPIO_NOPULL
```

Selection logic is **active LOW**.

Example:

```c
if(!HAL_GPIO_ReadPin(_500_HZ_GPIO_Port, _500_HZ_Pin))
```

So the corresponding frequency-selection pin must be LOW to select that range.

The DAC control pin and input-signal LED pin are configured as push-pull outputs.

## Startup Sequence

During startup:

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_I2C1_Init();
MX_TIM16_Init();
MX_TIM1_Init();
```

Then the application starts the timers:

```c
HAL_TIM_Base_Start_IT(&htim16);
HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
```

The DAC output is initially set to 0 V.

After initialization, the application continuously executes:

```c
while (1)
{
    freq_map();
}
```

## Main Control Logic

`freq_map()` checks the selected range and calculates the DAC output.

Example for 500 Hz:

```c
if(pps > 500)
{
    dac_set_val(3.3);
}
else
{
    float volt = ((REF_VLT * pps) / 500);
    dac_set_val(volt);
}
```

The same principle is used for all other configured ranges.

When no range-selection input is active:

```c
dac_set_val(0);
```

## Project Structure

A typical STM32CubeIDE project using this firmware will contain:

```text
Project/
├── Core/
│   ├── Inc/
│   │   └── main.h
│   └── Src/
│       └── main.c
├── Drivers/
├── *.ioc
└── README.md
```

The main application logic is implemented in:

```text
Core/Src/main.c
```

## Build

Open the STM32CubeIDE project and build it normally.

Alternatively, use the project toolchain configured for the target MCU.

Typical STM32CubeIDE flow:

```text
Open Project
    ↓
Build Project
    ↓
Flash / Debug
```

## Configuration

Before using the firmware on a new hardware design, verify:

1. MCU system clock is actually 48 MHz.
2. TIM1 input signal is connected to Channel 1.
3. DAC I2C address matches the hardware.
4. DAC expects the transmitted 2-byte data format.
5. DAC output/reference is compatible with the required 0–3.3 V range.
6. Frequency-selection GPIO logic is active LOW.
7. Custom frequency value in `freq_map()` matches the actual requirement.
8. Timer prescaler and frequency calculation match the actual TIM1 timer clock.

## Important Notes

### Timer Frequency Calculation

The frequency calculation depends on the actual TIM1 timer clock and prescaler configuration.

The current source uses:

```c
#define CLK_FREQ (48000000UL)
```

and calculates the frequency from the captured timer interval.

If the MCU clock tree or TIM1 clock configuration changes, the calculation should be rechecked.

### DAC I2C Address

The source passes:

```c
0xC2
```

to:

```c
HAL_I2C_Master_Transmit()
```

Verify the address format required by the particular STM32 HAL version and DAC device. STM32 HAL APIs commonly expect the 7-bit address shifted left by one bit.

### Frequency Measurement Range

The timer counter is 16-bit:

```c
Period = 65535
```

Very low input frequencies can require more timer counts than the configured counter can represent. For lower-frequency signals, consider changing the timer prescaler, using overflow counting, or using a different measurement method.

## Application

This firmware is suitable for applications where an incoming pulse/frequency signal needs to be converted into a proportional analog control signal, such as:

- Frequency-to-voltage conversion
- Fan / motor control interfaces
- Industrial control signals
- Sensor signal conditioning
- Compressor or equipment control interfaces
- Legacy systems requiring an analog input instead of a pulse input

## License

The source project contains the standard STM32Cube-generated license/header notice. Refer to the project's `LICENSE` file for the applicable license terms.
