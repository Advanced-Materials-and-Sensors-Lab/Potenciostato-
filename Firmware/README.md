## Firmware (ESP32 Potentiostat Controller)(Codigo C++)

The ESP32 firmware acts as the main control unit of the potentiostat system. It manages PWM signal, electrochemical signal acquisition, data processing, and communication with the desktop/mobile applications.

### Main Functions

- **Potential waveform generation**
  - Generates the excitation signal through PWM output (GPIO25).
  - PWM frequency: 5 kHz.
  - 8-bit resolution (0–255 duty cycle).
  - The PWM signal is filtered and converted into an analog control voltage for the potentiostat circuit.
  - Programmable electrochemical range:
  
    \[
    -2 Volts a 2 Volts
    \]

- **Electrochemical signal acquisition**
  - ADC34 measures the generated control voltage.
  - ADC35 acquires the transimpedance amplifier (TIA) output from the working electrode (WE) measurement path.
  - ADC configuration:
    - Resolution: 12 bits
    - Attenuation: 11 dB
    - Voltage range: 0–3.3 V

- **Current conversion**
  
  The firmware converts the TIA voltage into electrode current using the reference voltage and feedback resistance


  Configuration:

```cpp
VrefTIA = 0.91 V
RfTIA = 10 kΩ
