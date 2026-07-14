# Potenciostato

## Open-Source ESP32-Based Potentiostat with Desktop Software, Mobile App and Machine Learning for Electrochemical Analysis

> **An integrated electrochemical platform combining custom hardware, ESP32 firmware, a Python desktop application, a Flutter mobile application, and Machine Learning for electrochemical sensing.**

---

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
        ┌─────────────────────────────────┐
        │ TL084 + AD620 + Signal Conditioning │
        └─────────────────────────────────┘
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

# Hardware

## Main Components

| Component | Function |
|-----------|----------|
| ESP32 | Main controller |
| TL084 | Potential control loop |
| AD620 | Differential WE–RE measurement |
| Ag/AgCl | Reference electrode |
| Graphite | Counter electrode |
| PAN Membrane | Working electrode |
| RC Filter | PWM smoothing |
| Voltage Reference | ADC offset |
| ±12 V Supply | Analog power |

## Analog Front-End

The analog front-end controls the electrochemical cell, maintains the desired potential, measures the WE–RE voltage, and conditions signals for the ESP32 ADC.

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

```text
Potenciostato/
├── Firmware/
├── Hardware/
│   ├── Schematics/
│   ├── PCB/
│   └── BOM/
├── Python_GUI/
├── Flutter_App/
├── Machine_Learning/
├── Documentation/
├── docs/
└── README.md
```

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

---

# Roadmap

- [x] Cyclic Voltammetry
- [x] Linear Sweep Voltammetry
- [x] Chronoamperometry
- [x] Python GUI
- [x] Flutter App
- [x] Machine Learning
- [ ] DPV
- [ ] SWV
- [ ] EIS
- [ ] Wi-Fi synchronization

---

# License

MIT License

---

# Author

**Evelin Torres**

Open-source development of an ESP32-based potentiostat integrating electronics, embedded systems, electrochemistry, desktop software, mobile applications, and Machine Learning.
