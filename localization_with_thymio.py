import os
import glob
import numpy as np
import time
import json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from particle_filter import *
import utils as ut
import Thymio_custom
import global_controller


def unitToSensor(value, table):
    assert len(table) == 17
    tableBin = int(value * 16)
    r = value - (1. / 16.) * tableBin
    if tableBin == 16:
        return table[16]
    return float(table[tableBin]) * (1. - r) + float(table[tableBin + 1]) * r


def read_odometry(thymio):
    sensors = thymio["prox.ground.delta"]
    dx, dy, dtheta = thymio.delta_x, thymio.delta_y, thymio.delta_th

    read_reset_times.append(["{:6.2f}".format(time.time()-thymio.start_t)])

    thymio.delta_x, thymio.delta_y, thymio.delta_th = 0., 0., 0.
    return sensors[0], sensors[1], dx, dy, dtheta


map_file = 'data\\mapA0.png'
save_dir = "output\\particles_"

# connect to the Thymio
thymio = Thymio_custom.Thymio.serial(port="COM14", refreshing_rate=0.1)

config_filename = 'data\\config_TP465.json'
with open(config_filename) as infile:
    config = json.load(infile)

ground_map = np.flipud(mpimg.imread(map_file).astype(float))[:, :, 0]
vUnitToSensor = np.vectorize(unitToSensor, excluded=[1])
ground_map_left = vUnitToSensor(np.transpose(ground_map), config['left'])  # should be normalized
ground_map_right = vUnitToSensor(np.transpose(ground_map), config['right'])

x = 6.
y = 3.
theta = np.pi/2.
read_reset_times = []

# setting all the parameters   sigma_obs=150.
loc = MonteCarlo(ground_map_left, ground_map_right, particles_count=200000, sigma_obs=150., prop_uniform=0,
                 alpha_xy=0.1, alpha_theta=0.1,  state_init=[x, y, theta])

path = [np.array([x, y]), np.array([45, 45]), np.array([6, 60])]
glob_ctrl = global_controller.GlobalController(path)

# remove previous output plots
for fl in glob.glob(save_dir+"*"):
    os.remove(fl)

loc.plot_state(base_filename=save_dir+str(0), map_back=ground_map, num_particles=50)

Thymio_custom.wait_init(thymio)
Thymio_custom.reset_thymio(thymio)


# time.sleep(1)
# printed = False
T = 1.5
i = 1
d_reck = np.array([x, y, theta])
sum_dx = 0
start_time = 0
thymio.start_t = time.time()
estimated_particle = [0]*3
while glob_ctrl.state is not "reachedGoal":  # i < 30:

    if time.time() - start_time > T:
        print("----------------------", i, "t{:0.2f}".format(time.time()-thymio.start_t))
        sensor_left, sensor_right, dx, dy, dth = read_odometry(thymio)
        sum_dx += dx
        # odometry alone
        norm_xy = np.sqrt(dx ** 2 + dy ** 2)
        d_reck[0:2] += (ut.rot_mat2(d_reck[2]) @ np.asarray([dx, dy]).T).T
        d_reck[2] += dth

        # localization
        start_time = time.time()
        loc.apply_command(dx, dy, dth)
        loc.apply_obs_and_resample(sensor_left, sensor_right)
        estimated_particle, confidence = loc.estimate_state()
        duration = time.time() - start_time

        # print("Ground values:", sensor_left, sensor_right)
        print("Odometry: {:0.2f} {:0.2f} {:0.2f}".format(dx, dy, dth))
        print("Dead reckoning: {:0.2f} {:0.2f} {:0.2f}".format(d_reck[0], d_reck[1], d_reck[2])+",  sum dx:", sum_dx)
        # print("Estimated state: ", ["{:0.2f}".format(x) for x in loc.estimated_particle])
        print("Confidence:", confidence)

        plot_time = time.time()
        if True:  # plot or not
            loc.plot_state(base_filename=save_dir+str(i), map_back=ground_map,
                           num_particles=50, gt=d_reck, sens=[sensor_left, sensor_right], path=path)
        print("Duration algo, plot : {} , {} ms".format(round(1000*duration), round(1000 * (time.time() - plot_time))))

        # if i == 1:
        #     thymio.set_var("motor.left.target", 80)
        #     thymio.set_var("motor.right.target", 80)
        i += 1

        # if sum_dx >= 21. and not printed:
        #     print("Motors stopped")
        #     thymio.set_var("motor.left.target", 0)
        #     thymio.set_var("motor.right.target", 0)
        #     printed = True
    else:
        glob_ctrl.followPath(estimated_particle[0:2], estimated_particle[2], thymio, "NavGlobal")
        time.sleep(0.1)  # necessary ?
    # except:
    #     thymio.set_var("motor.target.left", 0)
    #     thymio.set_var("motor.target.right", 0)
#     lost = np.sum(np.asarray(thymio.debug_odom), 0);
thymio.set_var("motor.left.target", 0)
thymio.set_var("motor.right.target", 0)

print("past dl, past dr:", thymio.past_dl, thymio.past_dr)
# with open("output\\d_odom.txt", "w")as f:  # debug
#     for st in thymio.deltas_and_time:
#         print(st, file=f)
pass
