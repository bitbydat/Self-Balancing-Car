<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/bitbydat/Self-Balancing-Car">
    <img src="images/car2.jpg" alt="Logo" width="600" height="600">
  </a>

<h3 align="center">Two-Wheeled Self-Balancing Robot</h3>

  <p align="center">
    A little DIY project. It falls more than it stands, but still looks pretty cool.
    <br />
    <br />
    <a href="https://github.com/hungdaqq/Smarthome-IoT/issues">Report Bug</a>
    ·
    <a href="https://github.com/hungdaqq/Smarthome-IoT/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

<p align="justify">
The objective of this project is to build a two-wheeled self-balancing car using a accelerometer sensor and a PID control algorithm to control the motors via PWM signals. Additionally, this car can receive commands from a mobile app via Bluetooth to move forward, backward, turn left, or turn right. </p>

<!-- GETTING STARTED -->
## Getting Started
<br />
<div align="center">
  <a href="https://github.com/bitbydat/Self-Balancing-Car">
    <img src="images/diagram3.png" alt="Logo" width="500" height="450">
  </a>
</div>
<p align="justify">
I chose the MPU6050 accelerometer and gyroscope sensor to calculate the tilt angle of the car, then applied a Kalman filter to smooth the data. The tilt angle, angular velocity, and control commands received from the HC-05 module are used as inputs to the PID controller. The output of the PID controller is used to generate PWM voltage signals to control the two DC motors. </p>

### Prerequisites
Before getting started with this project, you will need to prepare some stuffs:
| Hardware | Reason for Selection |
|----------|----------|
|x01 STM32F103C8 microcontroller| A low-cost and high-performance 32-bit microcontroller, featuring I2C and UART interfaces |
|x01 MPU6050 accelerometer and gyroscope sensor| Combines a 3‑axis accelerometer and a 3‑axis gyroscope. Operates at 3.3–5V with an I2C interface (up to 400 kHz clock speed)|
|x01 DRV8833 motor driver| Dual H‑Bridge motor driver capable of driving two DC geared motors. Input voltages range of 3V-10V and provides output current up to 1.5A RMS|
|x02 DC gear motors (encoders optional but recommended)| Operating voltage 5V–7V. No‑load current ≤ 0.5A and speed ranges from 100–200 RPM @5V |
|x01 LM2596 DC-DC converter| Wide input voltage range: 4.5–30V and up to 3A output current|
|x02 18650 batteries| Two cells in series provide 7.4–8.1 V, suitable for LM2596 input to maintain a 5V output|
|x01 robot chassis with wheels| Optional design depending on project scope|

<div align="center">
  <a href="https://github.com/bitbydat/Self-Balancing-Car">
    <img src="images/flowchartv2.png" alt="Logo" width="600" height="800">
  </a>
</div>



### Installation

1. Clone the repo:
   ```sh
   git clone https://github.com/hungdaqq/Smarthome-IoT.git
   ```
2. Create a Thingsboard account at https://demo.thingsboard.io/ and login to use Thingsboard Live Demo server. Then go to [Thingsboard](https://github.com/hungdaqq/Smarthome-IoT/tree/main/Thingsboard) for a quick overview of this open-source IoT platform.
3. Follow the instructions to install [Thingsboard Edge CE](https://thingsboard.io/docs/user-guide/install/edge/installation-options/) v3.4.3 and provision your Edge to the Server.
4. Get and install the ThingsBoardLive on [App Store](https://apps.apple.com/us/app/thingsboard-live/id1594355695) or [Google Play](https://play.google.com/store/apps/details?id=org.thingsboard.demo.app&hl=vi&gl=US).
5. Please refer to [ESP8266](https://github.com/hungdaqq/Smarthome-IoT/tree/main/ESP8266) for setting up micro controllers programming evironment and [Features](https://github.com/hungdaqq/Smarthome-IoT/tree/main/Features) for ThingsBoard configuration as well as connecting the electronics components in accordance with the hardware schema.
6. (Optional) Follow the instructions to install [Thingsboard IoT Gateway](https://thingsboard.io/docs/iot-gateway/installation/) v2.9 on your Raspberry Pi or PC if you want integrate devices that are connected to legacy and third-party systems with ThingsBoard IoT platform. For example: external MQTT brokers, OPC-UA servers, Sigfox Backend, Modbus slaves or CAN nodes.

<!-- USAGE EXAMPLES -->
## Usage

1. Start Edge service by running the script:
   ```sh
   sudo service tb-edge start
   ```
2. Access the user interface by opening a web browser and navigating to https://demo.thingsboard.io/home or using ThingsBoardLive mobile application.
3. Use the interface to control the devices and other features in your home.


<!-- ROADMAP -->
## Roadmap and Future work

- [x] Indoor temperature monitoring
- [x] Outdoor temperature, humidity monitoring (with OpenWeather API)
- [x] Light and Household applicances control
- [x] Create alarms, send emails and notifications.
- [ ] Power consumption and charging monitoring
- [ ] Devices claming (QR code)
- [ ] Data analytics with Trendz

<!-- CONTRIBUTING -->
## Contributing

I welcome contributions to this project. If you have suggestions, improvements, or new ideas, feel free to open an issue or submit a pull request. Your input is greatly appreciated and helps me improve.

<!-- CONTACT -->
## Contact

Dat Duy Nguyen - datndng01@gmail.com

Project Link: [https://github.com/bitbydat/Self-Balancing-Car/](https://github.com/bitbydat/Self-Balancing-Car/)

</p>
