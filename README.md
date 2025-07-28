# FRC Robot Simulation with WPILib

![MIT License](https://img.shields.io/badge/License-MIT-blue.svg)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![WPILib](https://img.shields.io/badge/WPILib-2025-brightgreen)

## Overview

This repository contains a Python-based simulation of a FIRST Robotics Competition (FRC) robot using the WPILib library. The simulation is designed to facilitate learning and experimentation with FRC robot programming, focusing on both **autonomous** and **teleoperated** modes. It features a differential drivetrain controlled via a joystick in teleoperated mode and programmable autonomous trajectories. The simulation integrates with **NetworkTables** and **SmartDashboard** for real-time visualization using **Glass**.

The project consists of two main scripts:
- **`robot.py`**: Implements the robot's control logic, including motor configuration, joystick input, and autonomous mode behaviors.
- **`robot_sim.py`**: Provides the simulation framework, managing the robot's pose (position and orientation) and visualizing it on a 2D field via Field2d in Glass.

This project is ideal for FRC teams, students, and educators looking to practice and test robot algorithms without physical hardware.

## Features

- **Differential Drivetrain**: Simulates a robot with four motors (two per side) using WPILib's `DifferentialDrive` class.
- **Teleoperated Mode**: Control the robot using a joystick (Xbox controller) with speed and rotation limits for smooth operation.
- **Autonomous Modes**: Students can practice creating their own autonomous algorithms. Some examples are included:
  - Forward movement for 1, 2, or 3 seconds.
  - Square path (four sides with 90-degree turns).
  - Zigzag path using sinusoidal rotation.
  - Circular path with constant turning.
- **Simulation Visualization**: Tracks and displays the robot's pose (x, y, heading) on a 2D field using Field2d in Glass.
- **NetworkTables Integration**: Sends real-time data (e.g., position, time, command state) to SmartDashboard for monitoring.
- **Command Simulation**: Simulates generic mechanism commands (e.g., ON/OFF) triggered by joystick buttons, with feedback in NetworkTables.

## Requirements

- **Python 3.8 or higher**: Ensure Python is installed on your system.
- **WPILib Suite**: Install the WPILib Python libraries and tools for FRC development.
- **Glass**: WPILib's simulation GUI for visualizing the robot's pose and NetworkTables data.

## Installation

1. **Install Python**:
   - Download and install Python 3.8+ from [python.org](https://www.python.org/downloads/).
   - Ensure `pip` is available.

2. **Install WPILib**:
   - Follow the official WPILib installation guide: [WPILib Installation Instructions](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).
   - Install the WPILib Python packages.

3. **Clone the Repository**:
   - Clone this repository to your local machine:
     ```bash
     git clone https://github.com/your-username/frc-robot-simulation.git
     cd frc-robot-simulation
     ```
## Usage

1. **Prepare the Environment**:
   - Ensure all dependencies are installed and Python is configured with WPILib.
   - Connect a joystick or Xbox controller if testing teleoperated mode.

2. **Run the Simulation**:
   - Access the project folder and execute the main script to start the robot simulation:
     ```bash
     py -3 -m robotpy sim
     ```
3. **Open Glass**:
   - Launch Glass (available in the WPILib tools) to visualize the robot's movement on the 2D field.
   - The robot's pose (x, y, heading) and other data (e.g., time, command state) will be displayed in real-time.

4. **Select Autonomous Mode**:
   - Open **SmartDashboard** or use Glass to access the `Auto Selector` menu.
   - Choose from the available autonomous modes, created via script.

5. **Test Teleoperated Mode**:
   - Use a joystick or Xbox controller to control the robot.
   - Axis 1 (Y-axis) controls forward/backward movement, and Axis 4 (X-axis) controls rotation.
   - Press button 3 (Xbox X button) to toggle a simulated mechanism (ON/OFF state displayed in NetworkTables).
   - Other mechanisms could be added via code

## Notes

- **Simulation Limitations**: This is a simplified simulation and does not include sensor feedback or complex physics. Future improvements may address these (see TODO in `robot_sim.py`).
- **Customization**: Extend the autonomous modes in `robot_sim.py` by modifying the `autonomousPeriodic` method to create new trajectories or behaviors.
- **Debugging**: Monitor NetworkTables data (e.g., `X`, `Y`, `Heading`, `Time`, `Command State`) in Glass or SmartDashboard for debugging.
- **Hardware-Free**: This simulation runs entirely in software, making it ideal for testing without a physical robot.

## TODO

- Implement sensor simulation (e.g., encoders, gyroscopes) for more realistic feedback.
- Enhance physics model for more accurate movement simulation.
- Add support for additional mechanisms and commands.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Author

- **Diego Souza**  
  Created on July 26, 2025

## Contributing

Contributions are welcome! Feel free to submit issues or pull requests to improve the simulation or add new features.

1. Fork the repository.
2. Create a new branch (`git checkout -b feature/your-feature`).
3. Commit your changes (`git commit -m 'Add your feature'`).
4. Push to the branch (`git push origin feature/your-feature`).
5. Open a pull request.
