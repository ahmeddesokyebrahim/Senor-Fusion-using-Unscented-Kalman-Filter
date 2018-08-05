# **Sensor Fusion using Unscented Kalman Filter**
---

**In this project, I have worked on the development of Unscented Kalman Filter (UKF) to fuse noisy data from Lidar and Radar sensors to track a dynamic object .**

## Algorithm Architecture:
---
  - Initialization
  - Prediction Step
    1. Generate Sigma Points
    2. Predict Sigma Points
    3. Predict Mean and Covariance
  - Update Step
    1. Predict Measurement
    2. Update state

For algorithm implementation, please visit the ukf.cpp inside the src folder.

## Conclusion
---
  * The UKF is tracking a dynamic object position and velocity and fusing data between two sensors efficiently.
  * The UKF outperforms the Extended Kalman Filter (EKF) for non-linear models as the EKF used Jacobian matrix for linearization while UKF uses Constant Turn Rate and Velocity Magnitude (CTRV) model which assumes the object can move in straight lines and in turns also and used the unscented transformation with sigma points to represent the whole Gaussian distribution.
  * The perfomance ouput with RMSE values : [X= 0.07, Y, 0.08, VX= 0.34, VY = 0.23]
