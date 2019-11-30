import os
import glob
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from particle_filter import *
import utils as ut
from Thymio_custom import Thymio

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

def read_and_reset_odometry(thymio):
    sensors = thymio["prox.ground.delta"]
    table = thymio["event.args"]
    dx, dy, dtheta = table[0:3]
    if dx > 2**15:
        dx -= 2**16
    if dy > 2**15:
        dy -= 2**16
    if dtheta > 2**15:
        dtheta -= 2**16

    thymio.reset_odom = True
    return sensors[0], sensors[1], dx/1000, dy/1000, dtheta * np.pi/2**15

map_file = 'data\\mapA3.png'
save_dir = "output\\particles_"

# connect to the Thymio
thymio = Thymio.serial(port="COM20", refreshing_rate=0.1)

if True:  # TODO calibration of Thymio still to be done
    # ... generate configuration on the fly
    c_factor = 0.44  # taken from Enki::Thymio2
    s_factor = 9.  # taken from Enki::Thymio2
    m_factor = 884.  # taken from Enki::Thymio2
    a_factor = 60.  # taken from Enki::Thymio2

    calib_table = [response_function(i) for i in np.linspace(0, 1, 17)]
    config = {'left': calib_table, 'right': calib_table}
# else:
#     # ... otherwise load config file
#     config_filename = 'config.json'
#     with open(config_filename) as infile:
#         config = json.load(infile)

ground_map = np.flipud(mpimg.imread(map_file).astype(float))[:,:,0]
# load stuff
vUnitToSensor = np.vectorize(unitToSensor, excluded=[1])
ground_map_left = vUnitToSensor(np.transpose(ground_map), config['left'])  # should be normalized
ground_map_right = vUnitToSensor(np.transpose(ground_map), config['right'])

x = 6.
y = 3.
theta = np.pi/4.

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

# time.sleep(1)
T = 0.5
start_time = 0
i = 1
sum_state = np.array([0.,0.,0.])
while True:

    if time.time() - start_time > T:
        print("----------------------", i)
        sensor_left, sensor_right, dx, dy, dth = read_and_reset_odometry(thymio)
        sum_state += np.asarray([dx, dy, dth])

        # localization
        start_time = time.time()
        loc.apply_command(dx, dy, dth)
        loc.apply_obs_and_resample(sensor_left, sensor_right)
        estimated_particle, confidence = loc.estimate_state()
        duration = time.time() - start_time

        print("Ground values:", sensor_left, sensor_right)
        print("Odometry: {:0.2f} {:0.2f} {:0.2f}".format(dx, dy, dth))
        print("Sum odom: {:0.2f} {:0.2f} {:0.2f}".format(sum_state[0], sum_state[1], sum_state[2]))
        print("Estimated state: ", ["{:0.2f}".format(x) for x in loc.estimated_particle])
        print("Confidence:", confidence)

        # print("Duration algo: {} ms".format()
        plot_time = time.time()

        if True:  # plot or not
            loc.plot_state(base_filename=save_dir+str(i), plot_sens_pos=True, map_back=ground_map, num_particles=50)
        print("Duration algo, plot : {} , {} ms".format(round(1000*duration), round(1000 * (time.time() - plot_time))))

        i += 1

    else:
        pass
        # time.sleep(T-(time.time() - start_time))
    # except:
    #     thymio.set_var("motor.target.left", 0)
    #     thymio.set_var("motor.target.right", 0)