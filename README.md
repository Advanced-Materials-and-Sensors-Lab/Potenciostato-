### Open-Source Low-Cost Potentiostat Based on ESP32 for Electrochemical Sensing

<p align="center">
  <img src="docs/images/biopot_banner.png" alt="BioPot" width="800"/>
</p>

<p align="center">

![ESP32](https://img.shields.io/badge/ESP32-Microcontroller-red)
![Python](https://img.shields.io/badge/Python-3.10+-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Research-orange)

</p>

---

## Overview

Is an open-source, low-cost potentiostat designed for electrochemical sensing applications. The system combines an **ESP32**, an analog signal conditioning circuit, and a Python-based graphical interface to perform electrochemical measurements in real time.

The potentiostat is specifically optimized for **high-impedance working electrodes**, such as **polyacrylonitrile (PAN) membrane sensors**, enabling the measurement of currents in the **nanoampere range**.

The project aims to provide an accessible platform for researchers, students, and developers interested in electrochemical instrumentation.

---

## Features

- Three-electrode potentiostat architecture
- Analog feedback control loop
- Differential voltage measurement (WE–RE)
- ESP32-based waveform generation
- Real-time data acquisition
- Cyclic Voltammetry (CV)
- Linear Sweep Voltammetry (LSV)
- Chronoamperometry (CA)
- CSV data export
- Python graphical interface
- Open-source hardware and software

---

# System Architecture

```
             Python GUI
                  │
          USB / Bluetooth
                  │
              ESP32 MCU
      PWM + ADC Acquisition
                  │
          Analog Front-End
      ┌─────────────────────┐
      │      TL084          │
      │ Control Amplifier   │
      └─────────────────────┘
                  │
                  CE
                  │
          Electrochemical Cell
        WE ───── RE ───── CE
                  │
            AD620 Instrumentation
             Amplifier (WE-RE)
                  │
               ESP32 ADC
```

---

# Hardware

Main components:

| Component | Description |
|------------|------------|
| ESP32 | Main controller |
| TL084 | Analog control loop |
| AD620 | Instrumentation amplifier |
| Ag/AgCl | Reference electrode |
| Graphite | Counter electrode |
| PAN membrane | Working electrode |
| ±12 V Supply | Analog power |

---
# Electrochemical Techniques

Currently supports:

- Cyclic Voltammetry (CV)
- Linear Sweep Voltammetry (LSV)
- Chronoamperometry (CA)

Additional techniques can be incorporated in future versions.

---

# Python Interface

The desktop application provides:

- Real-time voltage monitoring
- Real-time current monitoring
- Voltammogram visualization
- Experiment configuration
- Automatic CSV export
- Serial communication with ESP32

---

# Applications

Can be used for:

- Heavy metal detection
- Electrochemical sensors
- Biosensors
- Environmental monitoring
- Academic laboratories
- Research projects
- Educational purposes

---

# Future Improvements

- Electrochemical Impedance Spectroscopy (EIS)
- Differential Pulse Voltammetry (DPV)
- Square Wave Voltammetry (SWV)
- Bluetooth Low Energy
- Wi-Fi connectivity
- Cloud data logging
- External high-resolution ADC
- Automatic calibration

---

# Citation

If you use BioPot in your research, please cite:

```
E. Torres,
BioPot: Open-Source Low-Cost Potentiostat Based on ESP32,
GitHub Repository, 2026.
```

---

# License

This project is released under the **MIT License**.

---

# Author

**Evelin Torres**

Research project focused on low-cost electrochemical instrumentation for environmental sensing.

---

# Acknowledgments

Special thanks to the open-source community and everyone contributing to affordable scientific instrumentation.

---

⭐ If you find this project useful, consider giving it a **Star** on GitHub.
