import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import chi2


filetype = 'trans'
if filetype == 'rot':
    n = 3
else:
    n = 5

innovations = filetype + "_innovations.csv"
residuals = filetype + "_residuals.csv"

innovations = np.loadtxt(innovations, delimiter=",")
residuals = np.loadtxt(residuals, delimiter=",")


nis = []

for i in range(len(residuals)): 
    res = residuals[i]
    inno = innovations[(len(res)*i):(len(res)*i+len(res))]
    nis.append(res.T @ np.linalg.inv(inno) @ res)
anis = []
window = 1
for i in range(np.int32(np.floor(len(nis) / window))):
    anis.append(np.mean(nis[(window * i):(window * i + window)]))
anis = np.array(anis)
#if filetype == 'rot':
#    anis = np.array(anis) * (180 / np.pi)**2

plt.figure()
alpha = 0.05
lower_bound = chi2.ppf(alpha/2, n) / n
upper_bound = chi2.ppf(1 - alpha/2, n) / n
plt.fill_between(np.linspace(0, len(anis), len(anis)),  lower_bound, upper_bound, alpha=0.2)
plt.plot(anis)
#plt.ylim([0, 20])
plt.show()




