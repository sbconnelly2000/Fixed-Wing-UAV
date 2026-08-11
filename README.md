# Fixed-Wing UAV Autopilot & Flight Control System

An autonomous delta-wing Unmanned Aerial Vehicle (UAV) flight controller developed in C++ for the ESP32 platform. Designed for multi-axis flight stability, the system integrates real-time Proportional-Derivative (PD) control loops, sensor fusion via I2C and SPI interfaces, dynamic pressure airspeed estimation, elevon actuation mixing, non-blocking UBX protocol GNSS navigation, and high-frequency SD card telemetry logging.

---

## Technical Features

* **Real-Time PD Control Architecture:** Multi-axis attitude stabilization loop operating on absolute Euler orientation vectors and angular velocity feedback.
* **Elevon Mixer & PWM Driver:** Embedded elevon control mixing algorithm translating combined roll and pitch stability vectors into clamped servo PWM commands.
* **Dynamic Airspeed Estimation:** Calculates true airspeed in real time using differential pressure sensor readings and barometric density calibration.
* **Environmental Density Calibration:** Pre-flight initialization routine taking barometric pressure and temperature samples to compute accurate air density ($\rho$).
* **High-Rate Telemetry Engine:** Non-blocking u-blox GNSS integration over serial combined with 10 Hz telemetry logging of 17 flight parameters to onboard SD storage.

---

## System Hardware Architecture

The system's physical component placement and power distribution were configured to maintain center-of-gravity (CG) stability, minimize parasitic wiring weight, and optimize flight endurance.

| Hardware Component | Protocol / Interface | Function |
| :--- | :--- | :--- |
| **Microcontroller** | ESP32 Board | Main autopilot executing control loops, telemetry logging, and PWM generation |
| **IMU Sensor** | Adafruit BNO055 (I2C `0x28`) | 9-DOF orientation tracking (Euler angles and angular body rates) |
| **Barometric Sensor** | Adafruit BMP280 (SPI) | Ambient air temperature and pressure sampling for air density calibration |
| **Airspeed Indicator** | Analog Pitot Differential Pressure Sensor (`A0`) | Measure dynamic pressure differential for airspeed calculation |
| **GNSS Module** | u-blox GNSS (UART1 @ 38400 Baud) | Latitude, longitude, altitude, and ground speed tracking using UBX frame format |
| **Data Logging** | SPI MicroSD Storage (`GPIO 21`) | High-frequency telemetry logging (`/Flight_Data.csv`) |
| **Flight Actuators** | Servo Motors (`GPIO D1`, `GPIO D2`) | Elevon control surface position control |

---

## Control Theory & Algorithmic Design

### 1. Barometric Density Calibration & Airspeed Calculation
Prior to launch, the system samples local ambient pressure ($P$) and ambient temperature ($T$) across 100 iterations to derive atmospheric density ($\rho$) using the ideal gas law:

$$\rho = \frac{P_{\text{avg}}}{(T_{\text{avg}} + 273.15) \cdot R}$$

Where $R = 287.05 \text{ J/(kg}\cdot\text{K)}$. 

During flight, the analog pressure differential across the Pitot probe ($\Delta P$) is converted from raw ADC voltage to kPa. True airspeed ($v$) is calculated using Bernoulli’s equation:

$$v = \sqrt{\frac{2 \cdot \Delta P}{\rho}}$$

### 2. PD Attitude Control Loop
The flight controller computes error between target reference states and feedback states derived from the BNO055 IMU. Control outputs for roll and pitch stability combine proportional angle error and derivative body rate feedback:

$$\text{Roll}_{\text{cmd}} = K_p \cdot (\theta_{\text{target}} - \theta_{\text{roll}}) + K_d \cdot \omega_{\text{gyro,y}} + \theta_{\text{roll}}$$

$$\text{Pitch}_{\text{cmd}} = K_p \cdot (\theta_{\text{target}} - \theta_{\text{pitch}}) + K_d \cdot \omega_{\text{gyro,z}} + \theta_{\text{pitch}}$$

### 3. Elevon Control Surface Mixing & Servo Mapping
Because delta-wing aircraft combine roll and pitch actuation into shared elevons, control commands are mixed mathematically prior to servo output mapping:

$$\delta_{\text{right}} = \text{Pitch}_{\text{cmd}} - \text{Roll}_{\text{cmd}}$$

$$\delta_{\text{left}} = \text{Pitch}_{\text{cmd}} + \text{Roll}_{\text{cmd}}$$

The mixed values are converted into pulse widths (microseconds) and constrained within hard mechanical bounds to avoid physical binding:

$$\text{PWM}_{\text{right}} = \text{Clamp}\left(-26.154 \cdot \delta_{\text{right}} + 1510, \ 500, \ 2500\right)$$

$$\text{PWM}_{\text{left}} = \text{Clamp}\left(29.615 \cdot \delta_{\text{left}} + 1580, \ 1000, \ 2200\right)$$
## Flight Testing & Control Loop Tuning

The aircraft underwent systematically recorded flight testing to evaluate open-loop flight dynamics and closed-loop autonomous stability. Flight telemetry was logged continuously at 10 Hz.

```text
Flight Log Parameters (Recorded at 10 Hz):
Time, Heading, Roll, Pitch, Accel (X,Y,Z), Mag (X,Y,Z), Gyro (X,Y,Z), Latitude, Longitude, Altitude, Airspeed
```

### Critical Flight Performance Insights
* **Roll Dynamics & Overshoot:** Initial autonomous flight tests revealed severe roll oscillations and dramatic rolling movements caused by an over-aggressive roll controller gain setting. Downscaling the gain mitigated limit cycling.
* **Pitch Dynamics & Nose-Dive Recovery:** Flight 1 resulted in an immediate nose-dive upon launch due to an untuned baseline pitch gain. Re-tuning the pitch loop gain resolved the pitch instability and allowed level flight hold in subsequent flights.
* **Airframe Asymmetry & Loitering Behavior:** Aerodynamic drag asymmetries caused by physical misalignment of the left wing introduced persistent yaw-roll coupling, forcing the plane into repeating looping flight trajectories[cite: 2]. This required manual structural trim adjustments and servo offset recalibration[cite: 2].
