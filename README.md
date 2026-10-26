# Pure Pursuit

A ROS 2 implementation of the **Pure Pursuit path tracking algorithm** for autonomous robot navigation.

The robot follows a predefined path by selecting a **lookahead point** ahead of its current position and calculating the steering/angular velocity required to move toward that point.

### Features

* Pure Pursuit path tracking
* Custom path support
* ROS 2 implementation
* RViz visualization
* Tested with curved and closed-loop paths

### Demo

https://github.com/## My DIY TurtleBot

<p align="center">
  <img width="70%" alt="front" src="https://github.com/user-attachments/assets/af74a811-5fc8-496f-8e4e-a2a0e28bcad0" />
  <img width="70%" alt="side" src="https://github.com/user-attachments/assets/b89b84fe-370c-4311-8541-b0b16f54ac22" />
  <img width="70%" alt="side2" src="https://github.com/user-attachments/assets/9694ad78-e866-4807-b734-111e5fe8ed54" />
  <img width="70%" alt="back" src="https://github.com/user-attachments/assets/4f439928-50c0-4767-a224-44b347462360" />
  <img width="70%" alt="above" src="https://github.com/user-attachments/assets/d60ee4f6-c0de-446b-9f55-3a549678e082" />
</p>user-attachments/assets/eb726cba-1a12-4e8d-a90c-9bfdb90afd55

### Project Structure

```text
pure_pursuit/
├── src/        # Pure Pursuit implementation
├── rviz/       # RViz configuration
├── CMakeLists.txt
└── package.xml
```

### Concept

```text
Robot → Find lookahead point → Calculate curvature → Generate velocity → Follow path
```

This project was built to understand and implement **path tracking for mobile robots using ROS 2**.
