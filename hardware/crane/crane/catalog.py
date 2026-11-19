"""The purchasing catalogue: what each bought item is, costs and weighs.

Pattern: Flyweight.
    One CatalogItem per kind of thing you buy, shared by every placement of it.
    Sixteen V-wheels are sixteen PartNodes pointing at one CatalogItem. Why:
    price, mass and description are stored once, so changing a price changes
    every BOM line that uses it, and the tree stays light.

Prices are rough USD street prices for planning, not quotes. Check suppliers.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CatalogItem:
    key: str
    description: str
    unit: str            # "ea", "m", "kg", "set"
    price_usd: float     # per unit
    mass_kg: float = 0.0  # per unit (per metre for "m")
    section: str = "Structure and motion"


_ITEMS = [
    # Structure and motion
    CatalogItem("extrusion_2020", "V-slot aluminium extrusion 2020", "m", 6.0, 0.55),
    CatalogItem("extrusion_2040", "V-slot aluminium extrusion 2040 (floor rail)", "m", 12.0, 1.0),
    CatalogItem("v_wheel", "V-slot wheel, Delrin, 625 bearings, with eccentric spacer", "ea", 1.6, 0.012),
    CatalogItem("t8_screw", "T8 lead screw, 8 mm lead", "m", 18.0, 0.40),
    CatalogItem("t8_nut", "T8 anti-backlash nut", "ea", 4.0, 0.015),
    CatalogItem("kfl08", "KFL08 flange bearing (screw top support)", "ea", 2.0, 0.03),
    CatalogItem("coupler_5_8", "Flexible coupler 5 mm to 8 mm", "ea", 2.0, 0.01),
    CatalogItem("mgn12_rail", "MGN12 linear rail (telescopic fork stages)", "m", 30.0, 0.65),
    CatalogItem("alu_flat_25x6", "Aluminium flat bar 25 x 6 mm (fork middle stage)", "m", 8.0, 0.405),
    CatalogItem("mgn12h_block", "MGN12H carriage block", "ea", 7.0, 0.045),
    CatalogItem("bearing_625", "625ZZ bearing (guide rollers)", "ea", 0.8, 0.005),
    CatalogItem("gt2_belt", "GT2 6 mm belt", "m", 2.0, 0.0),
    CatalogItem("gt2_pulley", "GT2 20T pulley, 5 mm bore", "ea", 2.0, 0.008),
    CatalogItem("gt2_idler", "GT2 toothless idler, 5 mm bore", "ea", 1.5, 0.006),
    CatalogItem("drag_chain_10x15", "Cable drag chain 10 x 15 mm", "m", 6.0, 0.10),
    CatalogItem("fastener_kit", "M3/M4/M5 screw kit + T-nuts", "set", 30.0),
    CatalogItem("heat_insert_m3", "M3 heat-set insert", "ea", 0.05),
    CatalogItem("water_tub", "Food-grade shallow tub, about 440 x 440 x 70 mm", "ea", 15.0, 0.6),
    CatalogItem("petg", "PETG filament", "kg", 20.0, 1.0),
    # Motion electronics and power
    CatalogItem("pi5", "Raspberry Pi 5, 8 GB, active cooler, 64 GB card", "ea", 105.0, 0.06, "Compute, electronics, power"),
    CatalogItem("skr_pico", "BTT SKR Pico (RP2040, 4x TMC2209)", "ea", 35.0, 0.04, "Compute, electronics, power"),
    CatalogItem("nema17", "NEMA17 stepper, 48 mm (rail Y, mast Z)", "ea", 13.0, 0.36, "Compute, electronics, power"),
    CatalogItem("nema14", "NEMA14 round stepper, short (arm X)", "ea", 15.0, 0.15, "Compute, electronics, power"),
    CatalogItem("psu_24v", "24 V 150 W sealed power brick", "ea", 35.0, 0.6, "Compute, electronics, power"),
    CatalogItem("buck_5v", "Buck converter 24 V to 5 V 5 A (Pi)", "ea", 12.0, 0.02, "Compute, electronics, power"),
    CatalogItem("gfci", "GFCI plug or RCD adapter (water nearby)", "ea", 25.0, 0.0, "Compute, electronics, power"),
    CatalogItem("estop", "E-stop mushroom switch, NC", "ea", 8.0, 0.05, "Compute, electronics, power"),
    CatalogItem("wiring_kit", "Silicone wire, ferrules, Wago, fuses, PG7 glands", "set", 35.0, 0.0, "Compute, electronics, power"),
    CatalogItem("microswitch", "Microswitch endstop (home Y, Z, X)", "ea", 1.5, 0.003, "Compute, electronics, power"),
    # Sensors (trimmed: one camera + wireless water nodes)
    CatalogItem("pi_camera3_wide", "Raspberry Pi Camera Module 3 Wide (NoIR if you add IR light)", "ea", 35.0, 0.004, "Sensors"),
    CatalogItem("csi_cable_1m", "CSI ribbon cable for Pi 5, 1 m", "ea", 6.0, 0.01, "Sensors"),
    CatalogItem("csi_hdmi_extender", "CSI-to-HDMI extender pair + 3 m slim HDMI cable (hook camera, through the drag chains)", "ea", 25.0, 0.08, "Sensors"),
    CatalogItem("esp32c3", "ESP32-C3 SuperMini (Wi-Fi water sensor node)", "ea", 4.0, 0.003, "Sensors"),
    CatalogItem("ds18b20", "DS18B20 waterproof temperature probe", "ea", 3.0, 0.03, "Sensors"),
    CatalogItem("tds_probe", "Gravity-style analog TDS probe + board (waste proxy)", "ea", 12.0, 0.04, "Sensors"),
    CatalogItem("ph_probe", "Gravity-style analog pH probe + board", "ea", 40.0, 0.05, "Sensors"),
    CatalogItem("cell_18650", "18650 cell + holder + TP4056 USB-C charger", "set", 8.0, 0.05, "Sensors"),
]

CATALOG: dict[str, CatalogItem] = {i.key: i for i in _ITEMS}


def item(key: str) -> CatalogItem:
    try:
        return CATALOG[key]
    except KeyError as e:
        raise KeyError(f"{key!r} is not in the catalogue (crane/catalog.py)") from e
