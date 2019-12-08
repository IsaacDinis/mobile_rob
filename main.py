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
import control
from local_avoidance import local_avoidance

map_file = "data\\mapA0.png"
save_dir = "output\\particles_"

# importing sensor calibration data
config_filename = 'data\\config_TP465.json'
with open(config_filename) as infile:
    config = json.load(infile)

# importing the ground map, and converting it to a map in sensor values for each sensor
ground_map = np.flipud(mpimg.imread(map_file).astype(float))[:, :, 0]
vUnitToSensor = np.vectorize(ut.unit_to_sensor, excluded=[1])
ground_map_left = vUnitToSensor(np.transpose(ground_map), config['left'])
ground_map_right = vUnitToSensor(np.transpose(ground_map), config['right'])

# taking a picture to init the position of all the elements
x, y, theta, goal, obsList = Pathplanning.take_picture_to_init(margeObs=10, cam_capture=0)

path = Pathplanning.find_path([x, y], goal, obsList, plotFlag=True)

path = [np.array(tup) for tup in path]  # converting tuples in np array
glob_ctrl = global_controller.GlobalController(path, tubeTol=4, outOfTubeAvancementTarget=3, noTurningDistance=3 )

# connect to the Thymio
thymio = Thymio_custom.Thymio.serial(port="COM10", refreshing_rate=0.1, global_controller=glob_ctrl)


# setting up all the filter parameters
loc = MonteCarlo(ground_map_left, ground_map_right, particles_count=150000, sigma_obs=150., prop_uniform=0,
                 alpha_xy=0.1, alpha_theta=0.1,  state_init=[x, y, theta])

# remove previous output plots
for fl in glob.glob(save_dir+"*"):
    os.remove(fl)

# plot the inital state
loc.plot_state(map_back=ground_map, num_particles=50, path=path)  # base_filename=save_dir+str(0)

Thymio_custom.wait_init(thymio)
Thymio_custom.reset_thymio(thymio)

# %% Main loop
i = 1
d_reck = np.array([x, y, theta])  # dead_reckoning --> debug but doesn't harm
thymio.start_t = time.time()
try:
    while glob_ctrl.state is not "reachedGoal":

        print("----------------------", i, "t{:0.2f}".format(time.time()-thymio.start_t))
        sensor_left, sensor_right, dx, dy, dth = thymio.read_odometry()

        # localization
        start_t_loc = time.time()
        loc.apply_command(dx, dy, dth)
        loc.apply_obs_and_resample(sensor_left, sensor_right)
        est_pos, confidence = loc.estimate_state()
        duration = time.time() - start_t_loc

        # print("Ground values:", sensor_left, sensor_right)
        print("Odometry: {:0.2f} {:0.2f} {:0.2f}".format(dx, dy, dth))
        # print("Dead reckoning: {:0.2f} {:0.2f} {:0.2f}".format(d_reck[0], d_reck[1], d_reck[2])+",  sum dx:", sum_dx)
        print("Estimated state: {:0.2f} {:0.2f} {:0.2f}".format(est_pos[0], est_pos[1], est_pos[2]))
        if confidence < 0.7:
            print("WARNING LOW CONFIDENCE:", confidence)
        else:
            print("Confidence:", confidence)

        # odometry alone --> debug purposes, but doesn't harm anyone if kept
        d_reck[0:2] += (ut.rot_mat2(d_reck[2]) @ np.asarray([dx, dy]).T).T
        d_reck[2] += dth

        # plotting
        plot_time = time.time()  # yes it takes time !
        loc.plot_state(map_back=ground_map, num_particles=50, odom=d_reck,
                       sens=[sensor_left, sensor_right], path=path, base_filename=save_dir+str(i))
        print("Duration algo, plot : {} , {} ms".format(round(1000*duration), round(1000 * (time.time() - plot_time))))

        glob_ctrl.followPath(est_pos[0:2], est_pos[2], thymio, thymio.nav_flag)

        if thymio.nav_flag == "local":
            thymio.set_var_array("leds.top", [255, 255, 0])  # yellow
            local_avoidance(thymio)
        elif glob_ctrl.state == "start":  # everything is going well
            thymio.set_var_array("leds.top", [0, 255, 0])  # green
        elif thymio.nav_flag == "global":  # coming back to the planned path
            thymio.set_var_array("leds.top", [0, 0, 255])  # blue

        i += 1
        time.sleep(0.2)  # slow down the loop

    thymio.set_var_array("leds.top", [255, 0, 127])  # pink when loop is exited normally --> goal reached
except:
    thymio.set_var_array("leds.top", [255, 0, 0])  # red
    pass

control.stop_thymio(thymio)
