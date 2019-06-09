import numpy as np
import matplotlib.pyplot as plt
import sys
from astropy.modeling import models, fitting

# Generate data_1
np.random.seed(0)
x1 = np.linspace(-0, 1., 100)
y1 = np.exp(-(x1 - 0)**2 / 0.2**2)
# y1 += np.random.normal(0., 0.1, x1.shape)
# Fit the data using a Gaussian
g1_init = models.Gaussian1D(amplitude=1., mean=0, stddev=0.2)
fit_g1 = fitting.LevMarLSQFitter()
g1 = fit_g1(g1_init, x1, y1)

# Generate data_2
y2 = np.exp(-(x1 - 0.5)**2 / 0.2**2)
# Fit the data using a Gaussian
g2_init = models.Gaussian1D(amplitude=1., mean=0.5, stddev=0.2)
fit_g2 = fitting.LevMarLSQFitter()
g2 = fit_g2(g2_init, x1, y2)

# Generate data_2
y3 = np.exp(-(x1 - 1.0)**2 / 0.2**2)
# Fit the data using a Gaussian
g3_init = models.Gaussian1D(amplitude=1., mean=1.0, stddev=0.2)
fit_g3 = fitting.LevMarLSQFitter()
g3 = fit_g3(g3_init, x1, y3)


# Plot the data with the best-fit model
plt.figure(figsize=(8,5))

plt.plot(x1, g1(x1), label='$\phi_1$')
plt.plot(x1, g2(x1), label='$\phi_2$')
plt.plot(x1, g3(x1), label='$\phi_3$')

# plt.xlabel('Position')
# plt.ylabel('Flux')
plt.legend(loc='center left')
plt.show()
