import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

log_file = pd.read_csv('../../../output.csv')

data = log_file.to_numpy()

t = data[:,0]
wroll = data[:,1]
wptch = data[:,2]
wyaw = data[:,3]
roll = data[:,4]
ptch = data[:,5]
yaw = data[:,6]

fig,ax = plt.subplots(3,1)
ax[0].plot(t,roll)
ax[1].plot(t,ptch)
ax[2].plot(t,yaw)

print(f"Roll bias mean: {np.mean(roll)}")
print(f"Pitch bias mean: {np.mean(ptch)}")
print(f"Yaw bias mean: {np.mean(yaw)}")

dt = np.mean(np.diff(t))

X_roll = np.fft.fft(wroll - np.mean(wroll))
X_ptch = np.fft.fft(wptch - np.mean(wptch))
X_yaw  = np.fft.fft(wyaw - np.mean(wyaw))

freqs = np.fft.fftfreq(len(wroll), d=dt)
mask = freqs >= 0

fig,ax = plt.subplots(3,1)
ax[0].plot(freqs[mask], np.abs(X_roll)[mask])
ax[1].plot(freqs[mask], np.abs(X_ptch)[mask])
ax[2].plot(freqs[mask], np.abs(X_yaw)[mask])

ax[0].set_title("Roll FFT")
ax[1].set_title("Pitch FFT")
ax[2].set_title("Yaw FFT")


plt.show()