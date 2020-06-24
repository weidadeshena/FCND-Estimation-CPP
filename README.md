# FCND-Estimation-CPP Writeup

## Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data
Data recorded in config/log/Graph1.txt (GPS X data) and config/log/Graph2.txt (Accelerometer X data) are processed using np.loadtxt and np.std

## Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.
First, convert the roll, pitch yaw angle to a quaternion.
```
Quaternion<float> qt = Quaternion<float>::FromEuler123_RPY(rollEst,pitchEst, ekfState(6));
```
Then, integrate q:
```
q.IntegrateBodyRate(gyro, dtIMU);
```
Finally, the roll, pitch, yaw angle can be extracted from the quaternion.
```
  float predictedPitch = q.Pitch();
  float predictedRoll = q.Roll();
  ekfState(6) = q.Yaw();
```

## Implement all of the elements of the prediction step for the estimator.
We start with a perfect IMU sensor. Assuming small dt, we can use direct euler integration for x,y,z
```
  predictedState(0) = curState(0) + dt * curState(3);
  predictedState(1) = curState(1) + dt * curState(4);
  predictedState(2) = curState(2) + dt * curState(5);
```
The measurement from accelerometer is converted from body frame to world frame as
```
  V3F accel_w = attitude.Rotate_BtoI(accel);
```
And velocity update
```
  predictedState(3) = curState(3) + dt * accel_w.x;
  predictedState(4) = curState(4) + dt * accel_w.y;
  predictedState(5) = curState(5) + dt * accel_w.z - dt * CONST_GRAVITY;
```
Then we move on to a noisy sensor. Implementing the state transition model in the paper, we get the g' matrix as
```
  gPrime(0,3) = dt;
  gPrime(1,4) = dt;
  gPrime(2,5) = dt;
  gPrime(3,6) = (RbgPrime(0)*accel).sum()*dt;
  gPrime(4,6) = (RbgPrime(1)*accel).sum()*dt;
  gPrime(5,6) = (RbgPrime(2)*accel).sum()*dt;
```
Then we can update covariance as
```
ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
```

## Implement the magnetometer update.
For the h prime matrix, only the last term is 1 and the rest is 0. The difference between estimated and measure can be calculated as
```
float difference = z(0) - zFromX(0);
```
Then, the difference is normalised.
```
  if (difference > F_PI) {
      zFromX(0) += 2.f*F_PI;
  } else if (difference < -F_PI) {
      zFromX(0) -= 2.f*F_PI;
  }
```

## Implement the GPS update.
Set the h prime matrix, then the update can be calculated by
```
zFromX = hPrime * ekfState;
```

