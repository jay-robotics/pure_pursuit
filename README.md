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

https://github.com/user-attachments/assets/eb726cba-1a12-4e8d-a90c-9bfdb90afd55

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
