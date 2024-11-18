# Frodo Power Module

The battery boards for the FRoDO-rockets

## Version 1

@todo

## Version 2

<img src="assets/69fc17f84e49772f28676bf87e165e84bd7ea957.jpg" title="" alt="battery-board" width="271">

The newest version of the battery developed for the FRoDO-M-rocket 

The battery module contains two lithium-ion cells in series, mounted on an *SRAD* *PCB* using a Keystone connector, allowing the replacement of battery cells. 

The battery features a linear charging IC supporting up to 700 mA of charge current and performs balancing of the cells, powered by the charge bus through a reverse current protection diode and a PTC fuse.

### Components

- 2x 3000 mA h Sony VTC6 18650 lithium-ion cells in series
- for other components see https://github.com/tudsat-rocket/frodo_power_module/blob/main/frodo_power_module.csv

### Usage

| Connector   | Purpose                                             |
| ----------- | --------------------------------------------------- |
| FC          | connect the flight computer. Outputs 8v with max ?a |
| SW          | Switch to turn the battery board on and off         |
| FC-CAN (2x) | CAN-bus for communication                           |
| BUS-A       |                                                     |
| BUS-B       |                                                     |
