# Threading

What you can safely call from more than one FreeRTOS task. The HAL creates **no tasks of its own**: every call runs
in the task that makes it. Periodic work, such as LED animation steps, is driven by your code.

## Locks

Each manager (except `NvsManager`) has one `RtosMutex` from `hf-utils-rtos-wrap`. `EnsureInitialized()` uses a
double-checked pattern: an atomic "initialised" flag is read first, then the lock is taken and the flag checked again.

| Manager | Locked operations | Not locked |
| --- | --- | --- |
| `CommChannelsManager` | `EnsureInitialized()`, `GetSystemDiagnostics()`, `Deinitialize()` | Bus and device accessors (`GetSpiDevice()`, `GetI2cDevice()`, `GetUartBus()`, `GetCanBus()`) and the runtime I2C calls (`CreateI2cDevice()`, `I2cProbeAddress()`, `I2cScanBus()`, `RemoveI2cDevice()`, `I2cResetBus()`) |
| `NvsManager` | none | everything; the `BaseNvs` namespaces do their own work |
| `GpioManager` | `EnsureInitialized()`, `Deinitialize()`, `GetSystemDiagnostics()`, TMC9660 bridge registration | every per-pin call (`Set()`, `Read()`, `Toggle()`, `SetDirection()`, ...) |
| `MotorController` | device creation and deletion, `handler()`, diagnostics, motor mechanics, load-frame velocity and position | `setMotorVelocity()` / `getMotorVelocity()` / `setMotorPosition()` / `getMotorPosition()` themselves (they go through the handler) |
| `AdcManager` | `EnsureInitialized()`, `Deinitialize()`, `ReadVoltage()`, `ReadRaw()` | `Get()`, `Contains()`, `Size()` |
| `ImuManager` | device creation and lookup, interrupt enable / disable, diagnostics | `WaitForInterrupt()`, `GetSensor()` |
| `EncoderManager` | device creation and lookup, `ReadAllAngles()`, `ReadAllVelocities()`, `CheckAllDevicesHealth()` | single-device reads (`ReadAngle()`, `ReadAngleDegrees()`, `ReadVelocityRPM()`, `ReadDiagnostics()`) and `SetZeroPosition()` |
| `TemperatureManager` | `EnsureInitialized()`, `Deinitialize()`, `ReadTemperatureCelsius()` | the convenience reads call `ReadTemperatureCelsius()`, so they are serialised too |
| `LedManager` | colour, brightness and animation calls | `DumpStatistics()` |
| `Vortex` facade | none | `EnsureInitialized()`, `Shutdown()`; diagnostics read atomics |

## What that means for your code

1. **Initialise from one task.** `Vortex::EnsureInitialized()` has no lock of its own: call it once from
   `app_main()` before you start other tasks. The same holds for `Shutdown()`.
2. **ADC, temperature, LED and motor configuration calls** may be made from several tasks; the manager serialises
   them. A call can block while another task holds the lock (for example during an SPI or UART transfer).
3. **GPIO**: give each pin one owner task, or add your own lock. Two tasks driving the same pin through
   `GpioManager` are not serialised, and the operation counters are plain integers.
4. **SPI2 is shared** by the TMC9660 (when on SPI), the AS5047U and the external slots. Keep each device to one task,
   and remember the TMC9660 SPI mux: while it is selected, the TMC9660 sees the SPI2 traffic.
5. **Encoder reads** of one device from several tasks: add your own lock, or read all devices with
   `ReadAllAngles()`, which takes the manager lock.
6. **LED animations** advance only when you call `vortex.leds.UpdateAnimation()`; call it from a periodic task
   about every 50 ms.

## Stack size

`Vortex::EnsureInitialized()` brings up the TMC9660 bootloader, the BNO08x SH-2 stack and the sensors with deep call
chains. The examples set `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` in
[`sdkconfig.defaults`](../../examples/esp32/sdkconfig.defaults); keep at least that much if you initialise from
`app_main()`.
