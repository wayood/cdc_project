#!/usr/bin/env python

import csv
import matplotlib.pyplot as plt
import numpy as np

io=sum([1 for _ in open('demo3.csv')])
x = np.empty(shape=(0), dtype=np.float64)
y = np.empty(shape=(0), dtype=np.float64)
with open('./demo3.csv') as f:
   counter = 0
   reader = csv.reader(f)        
   line = [row for row in reader]
   for i in range(1,io):
      x = np.append(x, np.array(line[i][6]))
      y = np.append(y, np.array(line[i][5]))
x = list(x)
y = list(y)
plt.plot(x,y)
plt.xlim(-2,2)
plt.plot(0,1)
plt.grid()
plt.show()

      
        
