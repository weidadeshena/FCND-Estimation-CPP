import numpy as np 

GPS_data = np.loadtxt('../config/log/Graph1.txt', delimiter=',', skiprows=1, usecols=1)
IMU_data = np.loadtxt('../config/log/Graph2.txt', delimiter=',', skiprows=1, usecols=1)

print("GPS std: ", np.std(GPS_data))
print("IMU std: ", np.std(IMU_data))
