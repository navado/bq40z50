# General
Backport of https://github.com/tibms/kernel-4.4/tree/release/drivers/power/bq40z50 from Linux-4.4 to 3.15.

Main differences from the source:
- Removed not supported values
- Added more documented values
- Code cleanup and less locking
- less loging at info level
## Supported readings:
```bash
# cat /sys/class/power_supply/BQ40Z50/uevent
POWER_SUPPLY_NAME=BQ40Z50
POWER_SUPPLY_PRESENT=1
POWER_SUPPLY_VOLTAGE_NOW=7495000                    uV
POWER_SUPPLY_CURRENT_NOW=677000                     uA Negative means charging
POWER_SUPPLY_CAPACITY=64                            %
POWER_SUPPLY_CAPACITY_LEVEL=Normal
POWER_SUPPLY_TEMP=204                               0.1°C
POWER_SUPPLY_TIME_TO_EMPTY_NOW=579                  Sec
POWER_SUPPLY_TIME_TO_FULL_NOW=-1                    Discharging
POWER_SUPPLY_CHARGE_FULL=10521000                   uAh Calibrated at the factory
POWER_SUPPLY_CHARGE_FULL_DESIGN=12000000            uAh
POWER_SUPPLY_CYCLE_COUNT=0
POWER_SUPPLY_TECHNOLOGY=Li-poly
POWER_SUPPLY_VOLTAGE_MAX_DESIGN=7260000             uV
POWER_SUPPLY_VOLTAGE_MIN_DESIGN=7260000             uV
POWER_SUPPLY_SERIAL_NUMBER=0001
POWER_SUPPLY_CONSTANT_CHARGE_VOLTAGE_MAX=8200000    uV
POWER_SUPPLY_CONSTANT_CHARGE_CURRENT_MAX=500000     uA
POWER_SUPPLY_STATUS=Discharging
POWER_SUPPLY_CURRENT_AVG=677000                     uA

```
# DTS
You'll need to add something like this in DTS file for your board
```
	battery@0B{
		compatible = "ti,bq40z50";
		reg = <0x0B>;
	};
```
