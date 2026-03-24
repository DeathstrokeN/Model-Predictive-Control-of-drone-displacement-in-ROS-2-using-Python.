"""Core MPC implementation for drone displacement control.

This file contains a reusable linear MPC controller for a 3D point-mass drone.
The model is intentionally simple:

    p_dot = v
    v_dot = u

where:
- p is position in x, y, z
- v is velocity in x, y, z
- u is commanded acceleration in x, y, z

The controller solves a quadratic program with CVXPY.
"""

from __future__ import annotations

import numpy as np
import cvxpy as cp


class DroneMPC:
    """Finite-horizon linear MPC for 3D translational motion."""

    def __init__(
        self,
        dt: float = 0.1,
        horizon: int = 20,
        q_position: float = 20.0,
        q_velocity: float = 2.0,
        r_acceleration: float = 0.1,
        p_terminal: float = 25.0,
        max_acceleration: float = 2.5,
        max_velocity: float = 3.0,
    ) -> None:
        self.dt = dt
        self.N = horizon
        self.max_acceleration = max_acceleration
        self.max_velocity = max_velocity

        # State is [x, y, z, vx, vy, vz]^T.
        self.nx = 6
        self.nu = 3

        self.A, self.B = self._build_discrete_model(dt)

        # Cost matrices.
        self.Q = np.diag([
            q_position, q_position, q_position,
            q_velocity, q_velocity, q_velocity,
        ])
        self.R = np.diag([
            r_acceleration, r_acceleration, r_acceleration,
        ])
        self.P = np.diag([
            p_terminal, p_terminal, p_terminal,
            q_velocity, q_velocity, q_velocity,
        ])

        # Build the optimization problem once, then update parameters online.
        self._build_problem()

    @staticmethod
    def _build_discrete_model(dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Return the discrete-time matrices A and B for a 3D double integrator."""
        A_axis = np.array([
            [1.0, dt],
            [0.0, 1.0],
        ])
        B_axis = np.array([
            [0.5 * dt * dt],
            [dt],
        ])

        A = np.zeros((6, 6))
        B = np.zeros((6, 3))

        # Build the full 3D block-diagonal model.
        # States are ordered as [x, y, z, vx, vy, vz].
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[4, 4] = 1.0
        A[5, 5] = 1.0

        A[0, 3] = dt
        A[1, 4] = dt
        A[2, 5] = dt

        B[0, 0] = 0.5 * dt * dt
        B[1, 1] = 0.5 * dt * dt
        B[2, 2] = 0.5 * dt * dt
        B[3, 0] = dt
        B[4, 1] = dt
        B[5, 2] = dt

        return A, B

    def _build_problem(self) -> None:
        """Create a parametric quadratic program.

        Using CVXPY Parameters lets us avoid rebuilding the problem at each time step.
        """
        self.x0_param = cp.Parameter(self.nx)
        self.xref_param = cp.Parameter(self.nx)

        self.X = cp.Variable((self.nx, self.N + 1))
        self.U = cp.Variable((self.nu, self.N))

        objective = 0
        constraints = []

        constraints.append(self.X[:, 0] == self.x0_param)

        for k in range(self.N):
            # Dynamics constraint.
            constraints.append(self.X[:, k + 1] == self.A @ self.X[:, k] + self.B @ self.U[:, k])

            # Input bounds.
            constraints.append(self.U[:, k] <= self.max_acceleration)
            constraints.append(self.U[:, k] >= -self.max_acceleration)

            # Velocity bounds.
            constraints.append(self.X[3:6, k] <= self.max_velocity)
            constraints.append(self.X[3:6, k] >= -self.max_velocity)

            # Tracking + effort cost.
            x_err = self.X[:, k] - self.xref_param
            objective += cp.quad_form(x_err, self.Q)
            objective += cp.quad_form(self.U[:, k], self.R)

        terminal_error = self.X[:, self.N] - self.xref_param
        objective += cp.quad_form(terminal_error, self.P)
        constraints.append(self.X[3:6, self.N] <= self.max_velocity)
        constraints.append(self.X[3:6, self.N] >= -self.max_velocity)

        self.problem = cp.Problem(cp.Minimize(objective), constraints)

    def solve(self, x0: np.ndarray, xref: np.ndarray) -> np.ndarray:
        """Solve the MPC problem and return the first optimal control input.

        If the solver fails, return a safe zero command.
        """
        self.x0_param.value = x0
        self.xref_param.value = xref

        try:
            self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
        except Exception:
            return np.zeros(self.nu)

        if self.U.value is None:
            return np.zeros(self.nu)

        return np.array(self.U[:, 0].value).reshape(-1)
