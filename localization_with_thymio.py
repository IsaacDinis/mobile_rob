import os
import glob
import numpy as np
import time
import json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from particle_filter import *
import utils as ut
from Thymio_custom import Thymio

# support functions
# def sigm(x, s): return 1. / (1. + np.exp(-x * s))
#
# def response_function(v): return sigm(v - c_factor, s_factor) * m_factor + a_factor

def unitToSensor(value, table):
    assert len(table) == 17
    tableBin = int(value * 16)
    r = value - (1. / 16.) * tableBin
    if tableBin == 16:
        return table[16]
    return float(table[tableBin]) * (1. - r) + float(table[tableBin + 1]) * r

# def read_and_reset_odometry(thymio):  # with research code on thymio
#     sensors = thymio["prox.ground.delta"]
#     table = thymio["event.args"]
#     dx, dy, dtheta = table[0:3]
#     if dx > 2**15:
#         dx -= 2**16
#     if dy > 2**15:
#         dy -= 2**16
#     if dtheta > 2**15:
#         dtheta -= 2**16
#
#     thymio.set_var("d_x", 0)
#     thymio.set_var("d_y", 0)
#     thymio.set_var("d_theta", 0)
#     return sensors[0], sensors[1], dx/1000, dy/1000, dtheta * np.pi/2**15

def read_and_reset_odometry(thymio, reset=False):
    sensors = thymio["prox.ground.delta"]
    dx, dy, dtheta = thymio.delta_x, thymio.delta_y, thymio.delta_th

    if reset:
        thymio.delta_x, thymio.delta_y, thymio.delta_th = 0., 0., 0.
        thymio.past_dl, thymio.past_dr = 0., 0.
        thymio.set_var("dist_left", 0)
        thymio.set_var("dist_right", 0)
        read_reset_times.append(["{:6.2f}".format(time.time()-thymio.start_t)])

    return sensors[0], sensors[1], dx, dy, dtheta

map_file = 'data\\mapA3.png'
save_dir = "output\\particles_"

# connect to the Thymio
thymio = Thymio.serial(port="COM14", refreshing_rate=0.1)

# if True:
#     # ... generate configuration on the fly
#     c_factor = 0.44  # taken from Enki::Thymio2
#     s_factor = 9.  # taken from Enki::Thymio2
#     m_factor = 884.  # taken from Enki::Thymio2
#     a_factor = 60.  # taken from Enki::Thymio2
#
#     calib_table = [response_function(i) for i in np.linspace(0, 1, 17)]
#     config = {'left': calib_table, 'right': calib_table}
# else:
    # ... otherwise load config file
config_filename = 'data\\config_TP465.json'
with open(config_filename) as infile:
    config = json.load(infile)

ground_map = np.flipud(mpimg.imread(map_file).astype(float))[:,:,0]
# load stuff
vUnitToSensor = np.vectorize(unitToSensor, excluded=[1])
ground_map_left = vUnitToSensor(np.transpose(ground_map), config['left'])  # should be normalized
ground_map_right = vUnitToSensor(np.transpose(ground_map), config['right'])

x = 6.
y = 3.
theta = np.pi/2.
read_reset_times = []

# setting all the parameters   sigma_obs=150.
loc = MonteCarlo(ground_map_left, ground_map_right, particles_count=200000, sigma_obs=300., prop_uniform=0,
                 alpha_xy=0.1, alpha_theta=0.1,  state_init=[x, y, theta])

# remove previous outputs
for fl in glob.glob(save_dir+"*"):
    os.remove(fl)

loc.plot_state(base_filename=save_dir+str(0), plot_sens_pos=True, map_back=ground_map, num_particles=50)

ok = False
yy, zz = [], []
while not ok or len(yy) == 0 or len(zz) == 0:
    time.sleep(0.5)
    try:
        yy = thymio["event.args"]
        zz = thymio["prox.ground.delta"]
    except KeyError:
        time.sleep(1)
    else:
        ok = True

thymio.start_t = time.time()
# time.sleep(1)
printed = False
T = 1.5
start_time = 0
i = 1
d_reck = np.array([x, y, theta])
sum_dx = 0
while i < 15:

    if time.time() - start_time > T:
        print("----------------------", i, "t{:0.2f}".format(time.time()-thymio.start_t))
        sensor_left, sensor_right, dx, dy, dth = read_and_reset_odometry(thymio, reset=True)
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
        # print("Confidence:", confidence)

        # print("Duration algo: {} ms".format()
        plot_time = time.time()

        if True:  # plot or not
            loc.plot_state(base_filename=save_dir+str(i), plot_sens_pos=True,
                           map_back=ground_map, num_particles=50, gt=d_reck)
        print("Duration algo, plot : {} , {} ms".format(round(1000*duration), round(1000 * (time.time() - plot_time))))

        i += 1
        if i == 2:
            thymio.set_var("motor.left.target", 80)
            thymio.set_var("motor.right.target", 80)
        # if sum_dx >= 21. and not printed:
        #     print("Motors stopped")
        #     thymio.set_var("motor.left.target", 0)
        #     thymio.set_var("motor.right.target", 0)
        #     printed = True
    else:
        pass
        time.sleep(0.1)
    # except:
    #     thymio.set_var("motor.target.left", 0)
    #     thymio.set_var("motor.target.right", 0)
#     lost = np.sum(np.asarray(thymio.debug_odom), 0);
thymio.set_var("motor.left.target", 0)
thymio.set_var("motor.right.target", 0)

# with open("output\\d_odom.txt", "w")as f:  # debug
#     for st in thymio.deltas_and_time:
#         print(st, file=f)
pass
