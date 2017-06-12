# This Python file uses the following encoding: utf-8
#
# Name: plotvector.py
# 
# This script is used to plot the energy and error per iteration produced during the TVL1 or TVL2 optimization
# 
# Author: Roger Marí
# Universitat Pompeu Fabra, Barcelona
# 2017
# 

import numpy as np
import matplotlib.pyplot as plt
import sys

y = np.loadtxt(sys.argv[1])
x = [i+1 for i in range(y.size)]

fig = plt.figure()
fig.suptitle(sys.argv[5],fontsize=28)
ax1 = fig.add_subplot(111)
ax1.set_ylabel(sys.argv[3],fontsize=20)
ax1.set_xlabel(sys.argv[4],fontsize=20)
ax1.plot(x,y, c='b')

fig.savefig(sys.argv[2])
