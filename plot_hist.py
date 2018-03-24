#!/usr/bin/env python

'''
Plot results of nographics run
'''

import numpy as np
import matplotlib.pyplot as plt

with open('stats.txt') as f:
    good = map(int, f.readline().split(', '))
    bad = map(int, f.readline().split(', '))
    scores = map(int, f.readline().split(', '))

total_counts = [0 for i in range(11)]
for i in range(len(good)):
    total_counts[good[i] + bad[i]] += 1

good_counts = [0 for i in range(11)]
for count in good:
    good_counts[count] += 1

bad_counts = [0 for i in range(11)]
for count in bad:
    bad_counts[count] += 1

plt.subplot(131)
plt.title('Exits on the green side')
plt.bar(range(0, 11), good_counts, 1.0, align='center')

plt.subplot(132)
plt.title('Exits on bad sides')
plt.bar(range(0, 11), bad_counts, 1.0, align='center')

plt.subplot(133)
plt.title('Exits on all sides')
plt.bar(range(0, 11), total_counts, 1.0, align='center')

plt.show()
