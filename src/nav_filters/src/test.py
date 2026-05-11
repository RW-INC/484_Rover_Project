import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import chi2
from scipy.spatial.transform import Rotation as R

estimations = np.loadtxt("output.csv", delimiter=",",usecols=np.linspace(0, 25, 25, dtype=int))

x = estimations[:, 0]
y = estimations[:, 1]
quat = estimations[:, 6:10]
eul = R.from_quat(quat).as_euler('xyz', degrees=False)

yaw = eul[:,2] % (2* np.pi) + np.pi

plt.figure()
plt.plot(y)

plt.figure()
plt.plot(np.sin(yaw), np.cos(yaw))
plt.title("Yaw")
plt.axis('equal')

plt.show()

