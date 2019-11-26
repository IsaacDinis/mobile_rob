import os
import glob
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from particle_filter import *
import utils as ut


# %% support functions
def sigm(x, s): return 1. / (1. + np.exp(-x * s))


def response_function(v): return sigm(v - c_factor, s_factor) * m_factor + a_factor


def unitToSensor(value, table):
    assert len(table) == 17
    tableBin = int(value * 16)
    r = value - (1. / 16.) * tableBin
    if tableBin == 16:
        return table[16]
    return float(table[tableBin]) * (1. - r) + float(table[tableBin + 1]) * r


# %% Main


if True:  # TODO calibration of Thymio still to be done
    # ... generate configuration on the fly
    c_factor = 0.44  # taken from Enki::Thymio2
    s_factor = 9.  # taken from Enki::Thymio2
    m_factor = 884.  # taken from Enki::Thymio2
    a_factor = 60.  # taken from Enki::Thymio2

    # fill the table
    # calib_table = map(response_function, np.linspace(0, 1, 17))
    calib_table = [response_function(i) for i in np.linspace(0, 1, 17)]
    config = {'left': calib_table, 'right': calib_table}
# else:
#     # ... otherwise load config file
#     config_filename = 'config.json'
#     with open(config_filename) as infile:
#         config = json.load(infile)

ground_map = np.flipud(mpimg.imread('data\\map-xhalf-yhalf.png').astype(float))
# mpimg.imread('C:\\Users\\utilisateur\\Documents\\git\\mobile_rob\\data\\map-xhalf-yhalf.png').astype(float))
# imgplot = plt.imshow(np.uint8(ground_map * 255))
# plt.show()

# load stuff
vUnitToSensor = np.vectorize(unitToSensor, excluded=[1])
ground_map_left = vUnitToSensor(np.transpose(ground_map), config['left'])  # should be normalized
ground_map_right = vUnitToSensor(np.transpose(ground_map), config['right'])
x = 31
y = 0
th = np.pi/2.
loc = MonteCarlo(ground_map_left, ground_map_right, particles_count=2000, sigma_obs=150., prop_uniform=0.,
                 alpha_xy=0.1, alpha_theta=0.1,  state_init=[x, y, th])
# %%
# colonne pour x = 31
# sensor_values = np.flipud(np.array([1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0])) * 1000

# ests_x = []
# ests_y = []

# remove previous drawings
save_dir = "output\\particles_"
for fl in glob.glob(save_dir+"*"):
    os.remove(fl)

for i in range(25):
    # # odometry
    dx_local = 3  # float(values[2]) * 0.01 * 0.1 # input to the localizer is in cm
    dy_local = 0
    dth_local = 0  # float(values[4]) * math.pi / 32767. # input to the localizer is in radian

    x += dx_local * np.cos(th) - dy_local * np.sin(th)
    y += dx_local * np.sin(th) + dy_local * np.cos(th)
    th += dth_local

    rot = ut.rot_mat2(th)
    left_sensor_pos = rot.dot([7.2, 1.1]) + np.asarray([x, y])
    right_sensor_pos = rot.dot([7.2, -1.1]) + np.asarray([x, y])

    sensor_left = ground_map_left[ut.xyW2C(left_sensor_pos[0]),
                                  ut.xyW2C(left_sensor_pos[1])]  # input to the localizer is within 0 to 1000
    sensor_right = ground_map_right[ut.xyW2C(right_sensor_pos[0]),
                                    ut.xyW2C(right_sensor_pos[1])]  # input to the localizer is within 0 to 1000

    # if dx_local != 0.0 or dy_local != 0.0 or dth_local != 0.0:
    print("sensor_left {0:.2f}, sensor_right {1:.2f}, x{2:.2f}, y{3:.2f}".format(sensor_left, sensor_right, x, y))
    print(["{:0.2f}".format(x) for x in loc.estimated_particle])
    print("----------------", i)

    # odom_plot.set_xdata(numpy.append(odom_plot.get_xdata(), x))
    # odom_plot.set_ydata(numpy.append(odom_plot.get_ydata(), y))

    # localization
    start_time = time.time()
    loc.apply_command(dx_local, dy_local, dth_local)
    loc.apply_obs_and_resample(sensor_left, sensor_right)
    loc.estimated_particle, confidence = loc.estimate_state()
    duration = time.time() - start_time
    print("Time end of loop = ", duration)
    time = time.time()

    if True:  # plot or not??
        loc.dump_PX(save_dir + str(i), gt_x=x, gt_y=y, gt_theta=th)
        print("duration_plot = ", time.time() - duration)

    # ests_x.append(est_x)
    # ests_y.append(est_y)


# %%
input("bla")
