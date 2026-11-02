# Threading

What you can safely call from more than one FreeRTOS task. The HAL creates **no tasks of its own**: every call runs
in the task that makes it. Periodic work, such as LED animation steps, is driven by your code.

## Locks

**Rule: every chip driver is reached through its handler's `visitDriver()`**, which holds the handler mutex for the
call. The managers never use the raw accessors that bypass it (`GetSensor()`, `driverViaSpi()`, `driverViaUart()`,
`GetDriver()`, `GetStrip()`, `GetAnimator()`); `tools/lint/lint_hal_boundary.sh` fails CI if one appears in
`lib/api` or `lib/managers`. RTOS primitives come only from hf-utils-rtos-wrap (`RtosMutex`, `SignalSemaphore`,
`os_delay_msec()`).

Lock order, when one manager calls another while holding its own lock: `Vortex` lifecycle lock → any manager;
`MotorController` → `GpioManager` (the TMC9660 SPI mux line is a PCAL95555 pin); `ImuManager` →
`CommChannelsManager` (runtime I2C devices); and a manager's lock → its handler's lock. Nothing takes them in the
other order.

Each manager (except `NvsManager`) has one `RtosMutex` from `hf-utils-rtos-wrap`. `EnsureInitialized()` uses a
double-checked pattern: an atomic "initialised" flag is read first, then the lock is taken and the flag checked again.

| Manager | Locked operations | Not locked |
| --- | --- | --- |
| `CommChannelsManager` | everything: `EnsureInitialized()`, `Deinitialize()`, diagnostics, the bus and device accessors (`GetSpiDevice()`, `GetI2cDevice()`, `GetUartBus()`, `GetCanBus()`) and the runtime I2C calls (`CreateI2cDevice()`, `I2cProbeAddress()`, `I2cScanBus()`, `RemoveI2cDevice()`, `I2cResetBus()`) | `IsInitialized()`, `GetLastError()` (atomics) |
| `NvsManager` | none | everything; the `BaseNvs` namespaces do their own work |
| `GpioManager` | `EnsureInitialized()`, `Deinitialize()`, `GetSystemDiagnostics()`, TMC9660 bridge registration, every per-pin call (`Set()`, `Read()`, `Toggle()`, `SetDirection()`, interrupts, ...) and `Get()` | `DebugReadPcal95555Pin()`, `DebugReadPcal95555InputSnapshot()` (bring-up only) |
| `MotorController` | device creation and deletion, `handler()`, diagnostics, motor mechanics, load-frame velocity and position, TMCL SPI timing and statistics, and `visitDriver()` (device lock for the whole visit, then the handler lock), so every motor-frame call too | `IsInitialized()`, `GetLastError()` (atomics) |
| `AdcManager` | everything: init, reads (lookup included), `Get()`, `Contains()`, `Size()` | `IsInitialized()`, `GetLastError()` (atomics) |
| `ImuManager` | device creation and lookup, interrupt enable / disable, diagnostics, `VisitSensor()` (manager lock, then the handler lock) | `WaitForInterrupt()` (it blocks on the interrupt signal, not on the lock) |
| `EncoderManager` | device creation and lookup, every read (`ReadAngle()`, `ReadAngleDegrees()`, `ReadVelocityRPM()`, `ReadDiagnostics()`, `ReadFieldStrength()`, `ReadAll*()`), `SetZeroPosition()`, `CheckAllDevicesHealth()`, `VisitDriver()`; the SPI exchange also holds the handler lock | `IsInitialized()`, counters (atomics) |
| `TemperatureManager` | `EnsureInitialized()`, `Deinitialize()`, `ReadTemperatureCelsius()` | the convenience reads call `ReadTemperatureCelsius()`, so they are serialised too |
| `LedManager` | colour, brightness and animation calls | `DumpStatistics()` |
| `Vortex` facade | `EnsureInitialized()`, `Shutdown()`, `GetSystemDiagnostics()`, `CollectManagerHealth()`, `GetFailedComponents()`, `GetSystemWarnings()` | `IsInitialized()`, `GetComponentInitializationStatus()` (atomics) |

## What that means for your code

1. **Initialise once, early.** `Vortex::EnsureInitialized()` and `Shutdown()` take the facade's lifecycle lock, so a
   second task that calls `EnsureInitialized()` waits for the first and then returns its result. Still call it from
   `app_main()` before you start other tasks: it blocks for the whole bring-up.
2. **ADC, temperature, LED and motor configuration calls** may be made from several tasks; the manager serialises
   them. A call can block while another task holds the lock (for example during an SPI or UART transfer).
3. **GPIO** calls are serialised by the manager lock, so two tasks cannot interleave a configuration change on the
   same pin. A read-modify-write of your own (read, decide, write) still needs one owner task.
4. **SPI2 is shared** by the TMC9660 (when on SPI), the AS5047U and the external slots. Keep each device to one task,
   and remember the TMC9660 SPI mux: while it is selected, the TMC9660 sees the SPI2 traffic.
5. **Encoder and IMU reads** may come from several tasks; each holds the manager lock, so a device cannot be
   deleted under it, and the handler lock for the bus exchange. For a driver call the manager does not wrap, use
   `EncoderManager::VisitDriver()` or `ImuManager::VisitSensor()`; there is no raw driver pointer to hold on to.
6. **LED animations** advance only when you call `vortex.leds.UpdateAnimation()`; call it from a periodic task
   about every 50 ms.

## Stack size

`Vortex::EnsureInitialized()` brings up the TMC9660 bootloader, the BNO08x SH-2 stack and the sensors with deep call
chains. The examples set `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` in
[`sdkconfig.defaults`](../../examples/esp32/sdkconfig.defaults); keep at least that much if you initialise from
`app_main()`.
