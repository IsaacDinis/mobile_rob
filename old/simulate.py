import os
import glob
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from particle_filter import *
import utils as ut


# support functions
def sigm(x, s): return 1. / (1. + np.exp(-x * s))


def response_function(v): return sigm(v - c_factor, s_factor) * m_factor + a_factor


def unitToSensor(value, table):
    assert len(table) == 17
    tableBin = int(value * 16)
    r = value - (1. / 16.) * tableBin
    if tableBin == 16:
        return table[16]
    return float(table[tableBin]) * (1. - r) + float(table[tableBin + 1]) * r



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


x = 30
y = 0
th = 3*np.pi/4.
alpha_theta = 0.1
alpha_xy = 0.1
# setting all the parameters
loc = MonteCarlo(ground_map_left, ground_map_right, particles_count=100000, sigma_obs=150., prop_uniform=0.,
                 alpha_xy=alpha_xy, alpha_theta=alpha_theta,  state_init=[x, y, th])

# remove previous outputs
save_dir = "output\\particles_"
for fl in glob.glob(save_dir+"*"):
    os.remove(fl)

a, b, c = 7, 2, 15
dxs = [3.]*a + [0.]*b + [3.]*c
dys = [0.]*a + [0.]*b + [0.]*c
dths = [0.]*a + [-np.pi/4.]*b + [0.]*c

for i in range(a + b + c):
    print("----------------------", i)
    # # odometry    dx_local = dxs[i]  # float(values[2]) * 0.01 * 0.1 # input to the localizer is in cm
    dy_local = dys[i]
    dth_local = dths[i]  # float(values[4]) * math.pi / 32767. # input to the localizer is in radian:

    x += dx_local * np.cos(th) - dy_local * np.sin(th)
    y += dx_local * np.sin(th) + dy_local * np.cos(th)
    th += dth_local

    if True:  # odometry noise on?
        norm_xy = np.sqrt(dx_local ** 2 + dy_local ** 2)
        e_theta = alpha_theta * abs(dth_local) + np.radians(0.25)
        e_xy = alpha_xy * norm_xy + 0.01
        x += float(np.random.normal(0., e_xy, 1))
        y += float(np.random.normal(0., e_xy, 1))
        th += float(np.random.normal(0., e_theta, 1))

    rot = ut.rot_mat2(th)
    left_sensor_pos = rot.dot([7.2, 1.1]) + np.asarray([x, y])
    right_sensor_pos = rot.dot([7.2, -1.1]) + np.asarray([x, y])

    if ut.is_in_bound(ground_map_left.shape, np.array([x, y])):
        sensor_left = ground_map_left[ut.xyW2C(left_sensor_pos[0]),
                                      ut.xyW2C(left_sensor_pos[1])]  # input to the localizer is within 0 to 1000
        sensor_right = ground_map_right[ut.xyW2C(right_sensor_pos[0]),
                                        ut.xyW2C(right_sensor_pos[1])]  # input to the localizer is within 0 to 1000
        print("sensor_left {0:.2f}, sensor_right {1:.2f},".format(sensor_left, sensor_right))
        print("Real state        x {0:.2f}  y {1:.2f} th {2:.2f}".format(x, y, th))
    else:
        print("Coordinates ", x, y, " are out of the map!")
        exit(1)

    # if dx_local != 0.0 or dy_local != 0.0 or dth_local != 0.0:


    # odom_plot.set_xdata(numpy.append(odom_plot.get_xdata(), x))
    # odom_plot.set_ydata(numpy.append(odom_plot.get_ydata(), y))

    # localization
    start_time = time.time()
    loc.apply_command(dx_local, dy_local, dth_local)
    loc.apply_obs_and_resample(sensor_left, sensor_right)
    estimated_particle, confidence = loc.estimate_state()
    duration = time.time() - start_time

    print("Estimated state: ", ["{:0.2f}".format(x) for x in loc.estimated_particle])
    print("Confidence:", confidence)

    print("Duration algo: {} ms".format(round(1000*duration)))
    start_time = time.time()

    if True:  # plot or not
        loc.plot_state(save_dir + str(i), gt_x=x, gt_y=y, gt_theta=th,
                    plot_sens_pos=True, map_back=ground_map, num_particles=500)
        print("Duration plot: {} ms".format(round(1000*(time.time()-start_time))))

    # ests_x.append(est_x)
    # ests_y.append(est_y)

