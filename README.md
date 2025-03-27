# PID Control Challenge

This repository contains a basic simulation of an inverted pendulum. In this challenge you will deploy a pid controller in this simulation, to keep the pendulum in a desired angle.

## Objective

The goal of this challenge is to implement a PID (Proportional–Integral–Derivative) controller that stabilizes the pendulum in an upright position defined by a reference angle.

## What Is Provided

- A visual simulation built with `pygame`.
- The physics and rendering of the pendulum.
- A partially implemented `InvertedPendulum` class.
- The dynamic model of the system.

## What You Need to Do

In the `InvertedPendulum` class, complete the `pid_control()` method so that it returns the control torque required to move the pendulum toward the reference angle `theta_ref`.

### Method to Complete

```python
def pid_control(self):
    error = self.theta - self.theta_ref

    # Deploy here your pid controller equation.
    # Use the variable 'error' to calculate the torque.

    torque = 0.0  # <- implement this using kp, ki, kd
    return torque
```

You must compute the control output `torque` using the PID gains:

- `kp` for the proportional term  
- `ki` for the integral term  
- `kd` for the derivative term

## System Dynamics

The angular acceleration of the pendulum is computed as:

$$
\ddot{\theta} = \frac{g}{L} \sin(\theta) + \frac{1}{mL^2} \cdot \tau - \frac{b}{mL^2} \cdot \dot{\theta}
$$

Where:
- \( \theta \): angular position (rad)  
- \( \dot{\theta} \): angular velocity (rad/s)  
- \( \ddot{\theta} \): angular acceleration (rad/s²)  
- \( \tau \): control torque (Nm)  
- \( g \): gravitational acceleration (9.81 m/s²)  
- \( L \): length of the pendulum (m)  
- \( m \): mass of the pendulum (kg)  
- \( b \): damping coefficient  

This equation is already implemented in the simulation. You should use it to understand how the system responds to the control torque.

## Visualization

- **Black line**: current position of the pendulum.
- **Red line**: desired reference position (`theta_ref`).

Use the visualization to verify whether your PID controller is stabilizing the pendulum at the desired angle.

## How to Run

Install dependencies:

```bash
pip install pygame
```

Run the simulation:

```bash
python main.py
```

> Replace `main.py` with the actual filename if different.

## Tips

- Carefully tune your PID gains to avoid oscillations or instability.
- Use small time steps for better numerical stability.
- Watch how the system responds over time to adjust your controller.

Good luck!