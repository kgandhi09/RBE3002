#Plotting Histogram with normal distribution
import csv
import numpy as np
import math
from scipy.stats import norm
import matplotlib.pyplot as plt
from hw4_p1 import calc_mean_var

csv_path = 'RBE3002_D20_homework_4 - Vnav_Data.csv'
file = csv.reader(open(csv_path, 'r'))
data = calc_mean_var(file)

sorted_x = data[6]
sorted_x.sort()

normal = norm.pdf(sorted_x, np.mean(sorted_x), np.std(sorted_x))

plt.hist(sorted_x, normed=True)
plt.plot(sorted_x, normal, 'r')
plt.show()