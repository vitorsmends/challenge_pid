import pygame
import numpy as np
import sys


class InvertedPendulum:
    def __init__(self, dt=0.02):
        self.g = 9.81
        self.L = 1.0
        self.m = 1.0
        self.b = 0.1

        self.dt = dt
        self.theta = 0.2
        self.theta_dot = 0.0
        self.theta_ref = np.pi * 0.7

        self.kp = -30.0
        self.ki = -10.0
        self.kd = -5.0
        self.integral_error = 0.0
        self.prev_error = 0.0

    def pid_control(self):
        error = self.theta - self.theta_ref

        # Deploy here your pid controller equation.
        # Use the variable 'error' to calculate the torque.

        torque = 0.0  # <- implement this using kp, ki, kd
        return torque

    def step(self):
        torque = self.pid_control()
        theta_ddot = (self.g / self.L) * np.sin(self.theta) + \
                     (1 / (self.m * self.L ** 2)) * torque - \
                     (self.b / (self.m * self.L ** 2)) * self.theta_dot

        self.theta_dot += theta_ddot * self.dt
        self.theta += self.theta_dot * self.dt


class PendulumApp:
    def __init__(self, sim: InvertedPendulum):
        pygame.init()
        self.width = 600
        self.height = 400
        self.origin = (self.width // 2, self.height // 2)
        self.length_px = 150

        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Inverted Pendulum Simulation")
        self.clock = pygame.time.Clock()
        self.sim = sim

    def run(self):
        running = True
        while running:
            self.clock.tick(int(1 / self.sim.dt))

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            self.sim.step()
            self.render()

        pygame.quit()
        sys.exit()

    def render(self):
        self.screen.fill((255, 255, 255))

        x = self.length_px * np.sin(self.sim.theta)
        y = self.length_px * np.cos(self.sim.theta)
        end_pos = (int(self.origin[0] + x), int(self.origin[1] - y))

        x_ref = self.length_px * np.sin(self.sim.theta_ref)
        y_ref = self.length_px * np.cos(self.sim.theta_ref)
        ref_pos = (int(self.origin[0] + x_ref), int(self.origin[1] - y_ref))

        pygame.draw.line(self.screen, (0, 0, 0), self.origin, end_pos, 5)
        pygame.draw.circle(self.screen, (0, 0, 255), end_pos, 10)

        pygame.draw.line(self.screen, (255, 0, 0), self.origin, ref_pos, 2)

        pygame.display.flip()


if __name__ == "__main__":
    sim = InvertedPendulum()
    app = PendulumApp(sim)
    app.run()
