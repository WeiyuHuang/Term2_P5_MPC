import numpy as np
import math

px = -43.6023
py = 106.728

ptsx = [-32.16173,-43.49173,-61.09,-78.29172,-93.05002,-107.7717]
ptsy = [113.361,105.941,92.88499,78.73102,65.34102,50.57938]

psi = 3.73281

next_x_vals = []
next_y_vals = []

for i in range(len(ptsx)):
	next_x_vals.append( math.cos(psi) * (ptsx[i] - px) + math.sin(psi) * (ptsy[i] - py));
	next_y_vals.append(-math.sin(psi) * (ptsx[i] - px) + math.cos(psi) * (ptsy[i] - py));

print('next_x_vals is', next_x_vals)
print('next_y_vals is', next_y_vals)