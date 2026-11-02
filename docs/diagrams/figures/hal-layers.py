"""The layers of the HAL, top to bottom, with what is in each one today.

Reads: From top to bottom, each layer only calls the one under it. Your application asks the facade singleton for
the managers. The managers (one per board function) are MCU-free: they own the hf-core handlers, which wrap one chip
driver each, and they get every bus, MCU pin, NVS namespace and the status LED from the board binding. The binding
is the only HAL code that names an ESP32 type; it constructs the ESP32 implementations of the interface classes,
which sit on ESP-IDF. Drivers talk to hardware only through those interface classes. Every name in a band is read
from the code: the managers from the facade's member references, the binding's classes from its #include lines,
the handlers, drivers and interface classes from the managers' and handlers' #include lines.
"""

import hal_extract as hx
from hardfoc_mkdocs_theme.figkit import Fig


def data():
    f, d, b = hx.facade(), hx.dependencies(), hx.binding()
    handlers = sorted({h for m in d["managers"].values() for h in m["handlers"]})
    handler_classes = sorted(
        p.stem for h in handlers for p in (hx.ROOT / hx.BOARD["handlers_dir"] / h).glob("*Handler.h")
    )
    drivers = sorted({x for h in handlers for x in d["handler_drivers"].get(h, [])}
                     | {x for m in d["managers"].values() for x in m["drivers"]})
    iface = sorted({i for m in d["managers"].values() for i in m["interfaces"] if i.startswith("Base")})
    return {
        "board": hx.BOARD["name"],
        "mcu": hx.BOARD["mcu"],
        "facade": hx.BOARD["facade_class"],
        "managers": [m["class"] for m in f["members"]],
        "handlers": handler_classes,
        "drivers": drivers,
        "interfaces": iface,
        "binding": b["mcu_classes"] + sorted(
            q.stem for h in b["platform_handlers"] for q in (hx.ROOT / hx.BOARD["handlers_dir"] / h).glob("*Handler.h")
        ),
        "binding_dir": b["platform"],
        "sources": f["sources"][:1] + d["sources"] + b["sources"][2:],
    }


def build(data):
    fig = Fig(
        "hal-layers",
        f"{data['board']} HAL, layer by layer",
        alt=f"Layer stack of the {data['board']} HAL: application, the {data['facade']} facade, "
        f"{len(data['managers'])} MCU-free managers, the board binding that constructs the {data['mcu']} "
        f"peripherals, {len(data['handlers'])} hf-core handlers, {len(data['drivers'])} chip drivers, and the "
        f"interface wrap on ESP-IDF.",
        width=1200,
        kicker="Architecture",
        subtitle="Each layer calls only the one below it. Every name is read from the code.",
        profile="overview",
    )
    W = fig.body_w
    label_w = 300
    bands = [
        ("Your application", "app_main()", ["your tasks", "examples/esp32"], "slate"),
        ("Facade", f"{data['facade']}::GetInstance()", ["EnsureInitialized()", "diagnostics"], "brand"),
        ("Managers (MCU-free)", "lib/managers", data["managers"], "blue"),
        ("Board binding", data["binding_dir"].replace("lib/managers/", ""), data["binding"], "slate"),
        ("hf-core handlers", "lib/core/handlers", data["handlers"], "teal"),
        ("Chip drivers", "hf-core-drivers", [f"{d}-driver" for d in data["drivers"]], "violet"),
        ("Interface wrap", f"ESP-IDF on {data['mcu']}", data["interfaces"], "amber"),
    ]
    y = 0
    gap = 22
    prev = None
    for title, sub, items, tone in bands:
        # Measure the chip rows first, so the band fits them.
        rows_h = _chip_height(fig, W - label_w - 36, items)
        h = max(rows_h + 36, 92)
        fig.rect(0, y, W, h, fill=f"{tone}.tint", stroke=f"{tone}.soft", sw=1.2, layer="region", rx=16)
        fig.rect(0, y + 10, 6, h - 20, fill=f"{tone}.strong", stroke=None, rx=3, layer="node")
        fig.text(26, y + h / 2 - 4, title, size=fig.k["card_title"], weight=700, color=f"{tone}.text",
                 bg=f"{tone}.tint")
        fig.text(26, y + h / 2 + 22, sub, size=fig.k["small"], color=f"{tone}.text", bg=f"{tone}.tint", mono=True)
        fig.chips(label_w, y + (h - rows_h) / 2, W - label_w - 20, items, tone=tone, on=f"{tone}.tint")
        if prev is not None:
            fig.edge([(W - 60, prev), (W - 60, y - 2)], tone="rail")
        prev = y + h + 2
        y += h + gap
    return fig


def _chip_height(fig, w, items):
    h = fig.k["pill"] + 10
    rows, cx = 1, 0.0
    for s in items:
        pw = fig.pill_width(s)
        if cx > 0 and cx + pw > w:
            rows, cx = rows + 1, 0.0
        cx += pw + 8
    return rows * h + (rows - 1) * 8
