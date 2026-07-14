# Hardware

The analog hardware is responsible for controlling the electrochemical cell and conditioning the acquired signals before digitization by the ESP32.

## Main Components

| Component | Function |
|-----------|----------|
| ESP32 | Main controller and data acquisition |
| TL084 | Analog feedback control loop |
| AD620 | Differential voltage measurement (WE–RE) |
| Ag/AgCl Electrode | Reference electrode |
| Graphite Electrode | Counter electrode |
| PAN Membrane Electrode | Working electrode |
| ±12 V Supply | Analog power supply |
| RC Filter | PWM smoothing |
| Voltage Reference | ADC offset generation |

## Analog Front-End

The analog stage performs three primary functions:

- Potential control
- Differential voltage measurement
- Signal conditioning for the ESP32 ADC

The control loop maintains the desired potential between the working and reference electrodes while driving the counter electrode.

## Electrochemical Cell

The system uses a conventional three-electrode configuration:

- Working Electrode (WE)
- Reference Electrode (RE)
- Counter Electrode (CE)

This architecture provides accurate electrochemical measurements and minimizes polarization effects during experiments.
