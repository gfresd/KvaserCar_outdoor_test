import numpy as np

class KalmanFilter:
    def __init__(self, time_to_sec_factor):
        self.x = np.array([[0.0], [0.0], [0.0]])   # [position, velocity, acceleration]
        self.P = np.eye(3) * 0.1                   # Initial covariance
        self.Q = np.eye(3) * 0.01                  # Process noise covariance trust in the model
        self.Rv = np.array([[0.5]])                # Velocity noise covariance trust in the obsveration
        self.Hv = np.array([[0, 1, 0]])            # Velocity-only observation
        self.time_factor = time_to_sec_factor      # to transform input delta time in seconds

    def predict(self, delta_t):
        F = np.array([
            [1, delta_t, 0.5 * delta_t ** 2],
            [0, 1, delta_t],
            [0, 0, 1]
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update_velocity(self, z):
        y = z - self.Hv @ self.x
        S = self.Hv @ self.P @ self.Hv.T + self.Rv
        K = self.P @ self.Hv.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ self.Hv) @ self.P

    def get_state(self):
        return self.x

    def get_predicted_acc(self) -> float:
        return self.x[2][0]

    def predict_acc(self, delta_t, velocity:float):
        self.predict(delta_t * self.time_factor)
        self.update_velocity(np.array([[velocity]]))
        return self.get_predicted_acc()

# Example Usage
if __name__ == "__main__":
    kf = KalmanFilter(0.001)

    # Simulated data
    observations = [
    (3.86, 283),
    (5.66028, 600-283),
    (11.8472, 1260 - 600)
    ]
    for obs, delta_t in observations:
        kf.predict(delta_t*kf.time_factor)
        kf.update_velocity(np.array([[obs]]))
        print(f"Time Step: {delta_t} | State Estimate: {kf.get_state().flatten()}")
        print(kf.get_predicted_acc())

    kf = KalmanFilter(0.001)
    a = kf.get_predicted_acc()
    print(a)
