import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

mapA3 = np.kron(np.random.choice([0., 1.], [13, 9]), np.ones((3, 3)))
plt.imshow(mapA3, cmap='gray')
plt.show()
# %%
mapA3_to_print = np.kron(mapA3, np.ones((118, 118)))

# %%
# plt.imsave("mapA3.png", mapA3, cmap='gray')
# plt.imsave("mapA3_to_print.png", mapA3_to_print, cmap='gray', dpi=300)


