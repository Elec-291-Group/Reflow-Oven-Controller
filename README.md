# 8052 Oven Reflow Controller

A sophisticated Oven Reflow Soldering Controller implemented entirely in **8052 Assembly**. This project features a high-performance, non-blocking cooperative multitasking architecture, a pub-sub communication pattern, and a dual-interaction system (Hardware & GUI).

## 🛠 Architectural Design

* **Pure Assembly Implementation**: 100% of the control logic, mathematical routines, and peripheral drivers are written in 8052 assembly for deterministic, real-time performance.
* **Cooperative Multitasking**: Uses a high-speed main loop to poll multiple Finite State Machines (FSMs), ensuring that temperature sensing, UI updates, and safety checks happen concurrently without blocking the processor.
* **Pub-Sub Interaction Pattern**: Utilizes bit-addressable memory (BSEG) as a "signal bus." Modules "publish" status flags (e.g., temperature reached, emergency stop), and independent "subscriber" modules (like the PWM driver or Buzzer) react to these signals.
* **Non-Blocking I/O**: Implements a `Serial_RX_Pump` and state-machine-based debouncing (`PBx_DEB`) to ensure the system remains responsive to user input at all times.
* ![Reflow FSM Diagram]("D:\下载\Oven Controller Software Architecture (2).png")

## 🚀 Key Features

* **Precision Thermal Profiling**:
* **7-Stage Control**: Manages the full cycle: *Preheat (Ramp to Soak), Soaking, Ramp to Reflow, Reflow, Cooling, and Process Completion.*
* **P-Control Power Output**: Utilizes Proportional control logic during the reflow stage to maintain precise temperature curves and prevent overshoot.
* **Soft-PWM**: Drives a Solid State Relay (SSR) via software-generated PWM for granular heater control.


* **Dual-Interaction System**:
* **Hardware Interface**: 4x4 Matrix Keypad for parameter entry, 1602 LCD for status messages, and 7-segment displays for live temperature/time tracking.
* **Host GUI**: Full remote control and telemetry via UART (57600 Baud). Users can set parameters, start/stop the process, and monitor live data from a PC.


* **Automated Peripherals**:
* **Automated Cooling**: Triggers an external processor (Arduino) via GPIO during the cooling phase to activate fans.
* **Remote Alerts**: Automatically signals the host to send an email notification once the soldering process is successfully completed.


* **Safety Engine**: Includes a `Safety_Check_TC` module that continuously monitors thermocouple integrity, triggering an immediate emergency shutdown if sensor failure is detected.

## 🔌 Hardware Mapping

| Peripheral | Pin | Function |
| --- | --- | --- |
| **SSR Heater** | `P1.3` | PWM Output for power control |
| **Cooling Fan** | `P4.0` | GPIO trigger for external fan/Arduino |
| **Buzzer** | `P1.5` | Audio alerts for state transitions |
| **UART** | `TX/RX` | 57600 Baud communication with Host GUI |
| **LCD Data** | `P0.x` | 8-bit parallel display interface |

## 📁 Core Logic (Source Reference)

* `Control_FSM`: The master logic controller governing the reflow state machine.
* `SEC_FSM`: Precise non-blocking timekeeper for soak and reflow durations.
* `proportional_power_control`: Computes heater duty cycle based on thermal error.
* `Serial_Process_Line`: Command parser for incoming GUI instructions.
* `Read_Thermocouple`: High-frequency ADC sampling for temperature monitoring.

## 🔧 Setup & Usage

1. **Configure**: Set your desired Soak/Reflow temperatures and durations via the Keypad or GUI.
2. **Start**: Initiate the process using the physical "Start" button or the GUI command.
3. **Monitor**: The system provides live feedback on the LCD (current state) and 7-segment displays (temperature).
4. **Stop**: An emergency stop is available via hardware button or GUI at any time.

---

*Developed for the Intel 8052 (DE10-Lite FPGA platform).*
