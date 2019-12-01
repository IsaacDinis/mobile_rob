import numpy as np
import json
import time

import utils as ut
from Thymio_custom import Thymio

# connect to the Thymio
thymio = Thymio.serial(port="COM14", refreshing_rate=0.5)

ok = False
yy, zz = [], []
while not ok or len(yy) == 0 or len(zz) == 0:  # wait for thymio to be ready
    time.sleep(0.5)
    try:
        yy = thymio["event.args"]
        zz = thymio["prox.ground.delta"]
    except KeyError:
        time.sleep(1)
    else:
        ok = True
# %%
map_l = []
map_r = []
# %% Press on center button several times for averaging
# execute for each color line, from dark to light

raw = thymio["event.args"]
sensors_l = np.array([raw[i] for i in range(len(raw)) if (i+1) % 2])
sensors_r = np.array([raw[i] for i in range(len(raw)) if i % 2])

map_l.append(sensors_l.sum()/(sensors_l > 0).sum())
map_r.append(sensors_r.sum()/(sensors_r > 0).sum())
# %%
print(map_l)
print(map_r)
# %%
config_filename = 'data\\config_TP465.json'
print('Writing calibration tables to', config_filename)
with open(config_filename, 'w') as outfile:
    json.dump({'left': map_l, 'right': map_r}, outfile)

