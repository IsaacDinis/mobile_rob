import os
import glob
import numpy as np
import time
import json
import matplotlib.image as mpimg

from particle_filter import *
import utils as ut
import Thymio_custom
import global_controller
import Pathplanning
from local_avoidance import local_avoidance

def read_odometry(thymio):
    sensors = thymio["prox.ground.delta"]
    dx, dy, dtheta = thymio.delta_x, thymio.delta_y, thymio.delta_th

    # read_reset_times.append(["{:6.2f}".format(time.time()-thymio.start_t)])

    thymio.delta_x, thymio.delta_y, thymio.delta_th = 0., 0., 0.
    return sensors[0], sensors[1], dx, dy, dtheta


map_file = 'data\\mapA0.png'
save_dir = "output\\particles_"


config_filename = 'data\\config_TP465.json'
with open(config_filename) as infile:
    config = json.load(infile)

ground_map = np.flipud(mpimg.imread(map_file).astype(float))[:, :, 0]
vUnitToSensor = np.vectorize(ut.unit_to_sensor, excluded=[1])
ground_map_left = vUnitToSensor(np.transpose(ground_map), config['left'])
ground_map_right = vUnitToSensor(np.transpose(ground_map), config['right'])

# x = 6.
# y = 3.
# theta = np.pi/2.
# path = [np.array([x, y]), np.array([45, 45]), np.array([10, 60]), np.array([40, 20])]  # fake path
# read_reset_times = []

x, y, theta, goal, obsList = Pathplanning.take_picture_to_init(margeObs=8.5, cam_capture=0)

path = Pathplanning.find_path([x, y], goal, obsList, plotFlag=True)

path = [np.array(tup) for tup in path]
glob_ctrl = global_controller.GlobalController(path, tubeTol=4, outOfTubeAvancementTarget=3, noTurningDistance=3 )

# connect to the Thymio
thymio = Thymio_custom.Thymio.serial(port="COM10", refreshing_rate=0.1, global_controller=glob_ctrl)


# setting up all the parameters
loc = MonteCarlo(ground_map_left, ground_map_right, particles_count=150000, sigma_obs=150., prop_uniform=0,
                 alpha_xy=0.1, alpha_theta=0.1,  state_init=[x, y, theta])

# remove previous output plots
for fl in glob.glob(save_dir+"*"):
    os.remove(fl)

loc.plot_state(base_filename=save_dir+str(0), map_back=ground_map, num_particles=50, path=path)

Thymio_custom.wait_init(thymio)
Thymio_custom.reset_thymio(thymio)

# %%
# time.sleep(1)
T = 1.5
i = 1
d_reck = np.array([x, y, theta])  # dead_reckoning --> debug
sum_dx = 0  # --> debug
start_time = 0
thymio.start_t = time.time()
est_pos = [0] * 3
while glob_ctrl.state is not "reachedGoal":  # i < 30

    # if time.time() - start_time > T:
    print("----------------------", i, "t{:0.2f}".format(time.time()-thymio.start_t))
    sensor_left, sensor_right, dx, dy, dth = read_odometry(thymio)
    sum_dx += dx

    # localization
    start_time = time.time()
    loc.apply_command(dx, dy, dth)
    loc.apply_obs_and_resample(sensor_left, sensor_right)
    est_pos, confidence = loc.estimate_state()
    duration = time.time() - start_time

    # print("Ground values:", sensor_left, sensor_right)
    print("Odometry: {:0.2f} {:0.2f} {:0.2f}".format(dx, dy, dth))
    # print("Dead reckoning: {:0.2f} {:0.2f} {:0.2f}".format(d_reck[0], d_reck[1], d_reck[2])+",  sum dx:", sum_dx)
    print("Estimated state: {:0.2f} {:0.2f} {:0.2f}".format(est_pos[0], est_pos[1], est_pos[2]))
    if confidence < 0.7:
        print("WARNING LOW CONFIDENCE:", confidence)
    else:
        print("Confidence:", confidence)

    plot_time = time.time()
    if True:  # plot or not
        # odometry alone --> debug
        d_reck[0:2] += (ut.rot_mat2(d_reck[2]) @ np.asarray([dx, dy]).T).T
        d_reck[2] += dth

        loc.plot_state(base_filename=save_dir+str(i), map_back=ground_map,
                       num_particles=50, odom=d_reck, sens=[sensor_left, sensor_right], path=path)
    print("Duration algo, plot : {} , {} ms".format(round(1000*duration), round(1000 * (time.time() - plot_time))))

    glob_ctrl.followPath(est_pos[0:2], est_pos[2], thymio, thymio.nav_flag)
    if thymio.nav_flag == "local":
        thymio.set_var_array("leds.top", [255, 255, 0])  # yellow
    elif glob_ctrl.state == "start":  # everything is going well
        thymio.set_var_array("leds.top", [0, 255, 0])  # green
    elif thymio.nav_flag == "global":  # coming back to the planned path
        thymio.set_var_array("leds.top", [0, 0, 255])  # blue

    if thymio.nav_flag == "local":
        local_avoidance(thymio)
        # if thymio.local_nav_state[0] == 0 and thymio.local_nav_state[1] == 0:
        #     thymio.nav_flag = "global"
        #     thymio.local_nav_dir = "none"

    i += 1
    time.sleep(0.2)  # to try


#     lost = np.sum(np.asarray(thymio.debug_odom), 0);
thymio.set_var("motor.left.target", 0)
thymio.set_var("motor.right.target", 0)
thymio.set_var_array("leds.top", [255, 0, 127])


print("past dl, past dr:", thymio.past_dl, thymio.past_dr)
# with open("output\\d_odom.txt", "w")as f:  # debug
#     for st in thymio.deltas_and_time:
#         print(st, file=f)
pass
