## 2. ESP32 Pin Assignment Table

| Pin / Function | Net Name | MCU Pin | Notes |
| :---: | :--- | :---: | :--- |
| **ENA** | `MOT_ENA` | GPIO 9 | PWM Speed Control (Motor A) |
| **IN1** | `MOT_IN1` | GPIO 10 | Direction Input 1 (Motor A) |
| **IN2** | `MOT_IN2` | GPIO 11 | Direction Input 2 (Motor A) |
| **IN3** | `MOT_IN3` | GPIO 12 | Direction Input 3 (Motor B) |
| **IN4** | `MOT_IN4` | GPIO 13 | Direction Input 4 (Motor B) |
| **ENB** | `MOT_ENB` | GPIO 14 | PWM Speed Control (Motor B) |
| **LiDAR TX** | `LIDAR_TX` | RX / GPIO 3 | Serial In (ESP $\leftarrow$ LiDAR TX) |
| **LiDAR RX** | `LIDAR_RX` | TX / GPIO 1 | Serial Out (ESP $\rightarrow$ LiDAR RX) |
| **5V In** | `VCC_5V` | `5V` / `VIN` | Logic Power In (from 5V Buck-Boost) |
| **GND** | `GND` | `GND` | Common Ground Reference |

---

## 3. LiDAR Sensor Connector (4-Pin)

| Pin | Net Name | MCU Pin | Notes |
| :---: | :--- | :---: | :--- |
| **1** | `VCC_5V` | — | $5\text{ V}$ Power from Buck-Boost |
| **2** | `LIDAR_TX` | RX (GPIO 3) | Signal Out (LiDAR TX $\rightarrow$ ESP RX) |
| **3** | `LIDAR_RX` | TX (GPIO 1) | Signal In (ESP TX $\rightarrow$ LiDAR RX) |
| **4** | `GND` | — | Ground Reference |

---

## 4. L298N Driver Terminals & Connections

| Terminal | Net Name | Connected To | Notes |
| :---: | :--- | :---: | :--- |
| **12V / VMS** | `12V_MOT` | 12V Buck-Boost | Motor power input ($12\text{ V}$) |
| **GND** | `GND` | Common Ground | Common ground rail |
| **5V Logic** | `VCC_5V` | 5V Buck-Boost | Logic supply (**remove onboard 5V jumper**) |
| **ENA** | `MOT_ENA` | ESP GPIO 9 | Remove jumper, drive via PWM |
| **IN1** | `MOT_IN1` | ESP GPIO 10 | Motor A direction |
| **IN2** | `MOT_IN2` | ESP GPIO 11 | Motor A direction |
| **IN3** | `MOT_IN3` | ESP GPIO 12 | Motor B direction |
| **IN4** | `MOT_IN4` | ESP GPIO 13 | Motor B direction |
| **ENB** | `MOT_ENB` | ESP GPIO 14 | Remove jumper, drive via PWM |
| **OUT1 / OUT2**| `MOT_A_OUT`| Motor 1 (Left) | $12\text{ V}$ H-Bridge output |
| **OUT3 / OUT4**| `MOT_B_OUT`| Motor 2 (Right)| $12\text{ V}$ H-Bridge output |
