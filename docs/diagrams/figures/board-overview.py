"""The Vortex V1 board as the software sees it: the ESP32-C6, each bus, and the chips on it.

Reads: The ESP32-C6 on the left runs all the software. Each arrow to the right is one bus or signal, labelled with
the GPIOs it uses: SPI2 to the TMC9660 motor controller and the AS5047U encoder (plus two external chip selects),
a UART that is the TMC9660's default host link, I2C to the PCAL95555 I/O expander and the BNO08x IMU, TWAI to the
CAN transceiver, and one data line to the WS2812 status LED. The PCAL95555 drives the TMC9660 control lines (driver
enable, reset, wake, SPI mux) and the IMU's boot and reset lines, and reads their fault and interrupt outputs.
The layout is drawn by hand; every GPIO number, expander bit, chip select, SPI mode and I2C address is read from
the functional pin map and the comms manager, so a pin change rebuilds this figure.
"""

import hal_extract as hx
from figkit import Fig

SOURCES = []


def data():
    p, b = hx.pins(), hx.buses()
    return {
        "gpio": {g["id"]: g for g in p["gpio"]},
        "spi": b["spi"],
        "i2c": b["i2c"],
        "sources": p["sources"][:1] + b["sources"][:2],
    }


def build(data):
    g = data["gpio"]

    def io(pin_id: str) -> str:
        e = g[pin_id]  # KeyError here means the pin map changed: update the figure
        return f"GPIO{e['pin']}" if e["chip"] == "ESP32_INTERNAL" else f"P{e['pin'] // 8}_{e['pin'] % 8}"

    spi_mode = {d["id"]: d["mode"] for d in data["spi"]}
    i2c = {d["id"]: d["address"] for d in data["i2c"]}
    fig = Fig(
        "board-overview",
        "Vortex V1: buses and chips",
        alt="Block diagram of the Vortex V1 board: the ESP32-C6 connects over SPI2 to the TMC9660 and AS5047U, over UART "
        "to the TMC9660, over I2C to the PCAL95555 expander and BNO08x IMU, over TWAI to the CAN transceiver and to a "
        "WS2812 LED; the PCAL95555 drives the TMC9660 and IMU control lines.",
        width=1200,
        kicker="Hardware",
        subtitle="Every GPIO, expander pin, SPI mode and I2C address below is read from the pin map and the comms "
        "manager.",
        legend=[("brand", "MCU"), ("blue", "motor"), ("teal", "sensing"), ("amber", "I/O and control"),
                ("green", "communication")],
    )
    W = fig.body_w
    mcu = fig.card(0, 0, 230, "ESP32-C6", ["RISC-V MCU", "runs the HAL and your app",
                                         f"USB-JTAG on {io('JTAG_USB_D_N')}/{io('JTAG_USB_D_P')}"],
                   tone="brand", accent=True, h=640)
    dx, dw = 600, W - 600 - 90
    tmc = fig.card(dx, 0, dw, "TMC9660", ["motor controller with gate drivers", "TMCL over UART or SPI"],
                   tone="blue", h=96)
    enc = fig.card(dx, 120, dw, "AS5047U", ["magnetic angle encoder"], tone="teal", h=70)
    ext = fig.card(dx, 210, dw, "External SPI slots 1 and 2", ["chip selects on the header"], tone="slate", h=70,
                   dashed=True)
    pcal = fig.card(dx, 300, dw, "PCAL95555", [f"16-bit I/O expander, I2C {i2c['PCAL9555_GPIO_EXPANDER']}"],
                    tone="amber", h=70)
    imu = fig.card(dx, 390, dw, "BNO08x IMU", [f"9-axis IMU, I2C {i2c['BNO08X_IMU']}"], tone="teal", h=70)
    can = fig.card(dx, 480, dw, "CAN transceiver", ["standby via PCAL95555"], tone="green", h=70)
    led = fig.card(dx, 570, dw, "WS2812", ["RGB status LED"], tone="amber", h=70)

    def bus(target, y, label, tone, t=0.5, dashed=False):
        fig.edge([(mcu.r + 4, y), (target.x - 4, y)], tone=tone, label=label, label_at=0.5, dashed=dashed)

    bus(tmc, 30, f"SPI2 mode {spi_mode['TMC9660_MOTOR_CONTROLLER']}, CS {io('SPI2_CS_TMC9660')}", "blue")
    bus(tmc, 72, f"UART RX {io('UART_RXD')} / TX {io('UART_TXD')} (default link)", "blue")
    bus(enc, 155, f"SPI2 mode {spi_mode['AS5047U_POSITION_ENCODER']}, CS {io('SPI2_CS_AS5047')}", "teal")
    bus(ext, 245, f"SPI2, CS {io('EXT_GPIO_CS_1')} and {io('EXT_GPIO_CS_2')}", "slate", dashed=True)
    bus(pcal, 325, f"I2C SDA {io('I2C_SDA')} / SCL {io('I2C_SCL')}", "amber")
    bus(pcal, 352, f"interrupt {io('I2C_PCAL95555_INT')} (active low)", "amber")
    bus(imu, 425, f"I2C SDA {io('I2C_SDA')} / SCL {io('I2C_SCL')}", "teal")
    bus(can, 515, f"TWAI TX {io('TWAI_TX')} / RX {io('TWAI_RX')}", "green")
    bus(led, 605, f"RMT data {io('WS2812_LED_DAT')}", "amber")
    fig.text(mcu.r + 18, 0 + 12, f"SPI2: SCK {io('SPI2_SCK')}, MOSI {io('SPI2_MOSI')}, MISO {io('SPI2_MISO')}",
             size=fig.k["small"], color="muted")
    # Control lines from the expander, in the right-hand gutter.
    gx = dx + dw + 44
    fig.edge([(pcal.r + 4, pcal.cy), (gx, pcal.cy), (gx, tmc.cy), (tmc.r + 4, tmc.cy)], tone="amber", dashed=True)
    fig.edge([(pcal.r + 4, pcal.cy + 14), (gx - 16, pcal.cy + 14), (gx - 16, imu.cy), (imu.r + 4, imu.cy)],
             tone="amber", dashed=True)
    note_y = 670
    fig.note(0, note_y, W,
             f"TMC9660 control lines on the PCAL95555: DRV_EN {io('PCAL_TMC_DRV_EN')}, RST_CTRL "
             f"{io('PCAL_TMC_RST_CTRL')}, WAKE_CTRL {io('PCAL_TMC_WAKE_CTRL')}, SPI_COMM_EN {io('PCAL_TMC_SPI_COMM_EN')} "
             f"(SPI mux, active low), FAULT_STATUS {io('PCAL_TMC_FAULT_STATUS')}. IMU: BOOT {io('PCAL_IMU_BOOT')}, INT "
             f"{io('PCAL_IMU_INT')}, RST {io('PCAL_IMU_RST')}. CAN standby: {io('PCAL_CAN_HS_STB_OP')}. "
             f"Power good: {io('PCAL_PWR_GOOD')}.",
             tone="amber", title="Expander pins")
    return fig
