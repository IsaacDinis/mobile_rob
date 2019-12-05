import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

# mapA3 = np.kron(np.random.choice([0., 1.], [13, 9]), np.ones((3, 3)))
raw = np.random.choice([0., 1.], [38, 27])
mapA0 = np.kron(raw, np.ones((3, 3)))

plt.imshow(mapA0, cmap='gray')
plt.show()
# %%
# map_to_printA3 = np.kron(raw, np.ones((118, 118)))
map_to_printA0 = np.kron(raw, np.ones((354, 354)))

# %%
# plt.imsave("maptest.png", mapA0, cmap='gray')
plt.imsave("maptest_big.jpg", map_to_printA0, cmap='gray', dpi=300)


