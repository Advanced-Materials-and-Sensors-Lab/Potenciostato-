# Potenciostato

## Open-Source ESP32-Based Potentiostat with Desktop Software, Mobile App and Machine Learning for Electrochemical Analysis

> **An integrated electrochemical platform combining custom hardware, ESP32 firmware, a Python desktop application, a Flutter mobile application, and Machine Learning for electrochemical sensing.**

---
<p align="center">

![ESP32](https://img.shields.io/badge/ESP32-Microcontroller-red)
![Python](https://img.shields.io/badge/Python-3.10+-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Research-orange)

</p>
## Overview

This repository contains the complete development of an open-source potentiostat designed for electrochemical sensing applications.

The project integrates:

- Custom analog potentiostat hardware
- ESP32 firmware
- Python desktop application
- Flutter mobile application
- Machine Learning models
- Real-time electrochemical acquisition
- Data visualization and export

The platform is optimized for high-impedance electrochemical sensors and supports nanoampere-level measurements.

---

# Key Features

### Hardware

- Three-electrode architecture (WE, RE, CE)
- Analog feedback control
- TL084 control amplifier
- AD620 instrumentation amplifier
- Signal conditioning
- ±12 V analog supply

### Firmware

- ESP32-based control
- PWM waveform generation
- ADC acquisition
- USB Serial communication
- Bluetooth communication

### Desktop Software

- Python + PyQt
- Real-time plots
- CSV export
- Experiment configuration

### Mobile Application

- Flutter
- Bluetooth connectivity
- Portable experiment monitoring

### Machine Learning

- Dataset generation
- Model training
- PKL export
- ONNX export
- Electrochemical prediction

---

# System Architecture

```text
                 Python Desktop Application
                          │
                   USB / Bluetooth
                          │
                     ESP32 Firmware
          PWM Generation + ADC Acquisition
                          │
               Analog Potentiostat Circuit
        ┌────────────────────────────────────┐
        │ TL084 + AD620 +Signal Conditioning │
        └────────────────────────────────────┘
                          │
             Electrochemical Cell (WE-RE-CE)
                          │
                Electrochemical Measurements
                          │
                 Machine Learning Models
                          │
                Flutter Mobile Application
```

---

## Electrochemical Cell

- Working Electrode (WE)
- Reference Electrode (RE)
- Counter Electrode (CE)

---

# Electrochemical Techniques

- Cyclic Voltammetry (CV)
- Linear Sweep Voltammetry (LSV)
- Chronoamperometry (CA)

---

# Repository Structure

```
Potenciostato/
│
├── Firmware/
│     ESP32 source code
│
├── Hardware/
│     Schematics
│     PCB
│   
├── Python_GUI/
│     Desktop Application
│
├── Flutter_App/
│     Android Application
│
├── Machine_Learning/
│     Dataset
│     Training
│     Models
│     ONNX
│     PKL
│
├── Documentation/
│
└── README.md
```

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

# Installation

1. Upload the firmware to the ESP32.
2. Connect the analog potentiostat hardware.
3. Install the Python dependencies.
4. Launch the desktop application.
5. Pair the Flutter app (optional).
6. Run CV, LSV or CA experiments.

---

# Applications

- Electrochemical sensing
- Heavy metal detection
- Environmental monitoring
- Biosensors
- Academic research
- Teaching laboratories

# Citation

If you use BioPot in your research, please cite:

```
E. Torres,
BioPot: Open-Source Low-Cost Potentiostat Based on ESP32,
GitHub Repository, 2026.
```

---


# License

MIT License

---

# Author

**Evelin Torres**

Open-source development of an ESP32-based potentiostat integrating electronics, embedded systems, electrochemistry, desktop software, mobile applications, and Machine Learning.

---
# Acknowledgments

Special thanks to the open-source community and everyone contributing to affordable scientific instrumentation.

---

⭐ If you find this project useful, consider giving it a **Star** on GitHub.
