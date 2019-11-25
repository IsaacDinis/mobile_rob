import numpy as np
from numba import jit
import utils as ut

from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

class MonteCarlo:
    _pi = np.pi
    _1sqrt2pi = 1. / np.sqrt(2. * _pi)

    def __init__(self, ground_map_left, ground_map_right, particles_count, sigma_obs, prop_uniform, alpha_xy,
                 alpha_theta, state_init=None):
        # sanity check on parameters
        assert ground_map_left.dtype == np.double
        assert ground_map_right.dtype == np.double
        assert ground_map_left.shape[0] == ground_map_right.shape[0]
        assert ground_map_left.shape[1] == ground_map_right.shape[1]

        # copy parameters
        self.ground_map_left = ground_map_left
        self.ground_map_right = ground_map_right
        self.N_uniform = int(prop_uniform * particles_count)
        self.alpha_xy = alpha_xy
        self.alpha_theta = alpha_theta
        self.sigma_obs = sigma_obs

        # setup limits
        self.conf_theta = np.radians(10)
        self.conf_xy = 3

        # create initial particles
        if state_init is None:
            particles = np.random.uniform(0, 1, size=[particles_count, 3])  # TODO initialize with picture estimated pos.
            particles = particles * [ground_map_left.shape[0], ground_map_left.shape[1], np.pi * 2]
        else:
            particles = np.random.normal(state_init, np.asarray([1, 1, np.pi/10]), size=(particles_count, 3))
        self.particles = particles
        self.weights = np.empty([particles_count])
        self.estimated_particle = np.empty([3], dtype=float)

    # @jit
    def apply_obs_and_resample(self, left_color, right_color):
        """ Apply observation and update probability weights, then resample"""
        particles_count = self.particles.shape[0]
        particles = np.asarray(self.particles)
        weights = np.empty([particles_count])
        nb_ok = 0
        for i in range(particles_count):
            theta = particles[i, 2]

            # compute position of sensors in world coordinates
            rot = ut.rot_mat2(theta)
            left_sensor_pos = rot.dot([7.2, 1.1]) + particles[i, 0:2]
            right_sensor_pos = rot.dot([7.2, -1.1]) + particles[i, 0:2]

            if not self.is_in_bound(left_sensor_pos) or not self.is_in_bound(right_sensor_pos):
                # kill particle if out of map
                weights[i] = 0.
            else:
                # otherwise, compute weight in function of ground color
                # left sensor
                ground_val_left = self.ground_map_left[ut.xyW2C(left_sensor_pos[0]), ut.xyW2C(left_sensor_pos[1])]
                left_weight = ut.norm(left_color, ground_val_left, self.sigma_obs)

                # right sensor
                ground_val_right = self.ground_map_right[ut.xyW2C(right_sensor_pos[0]), ut.xyW2C(right_sensor_pos[1])]
                right_weight = ut.norm(right_color, ground_val_right, self.sigma_obs)

                # compute weight
                weights[i] = left_weight * right_weight

            # update matching particles
            if weights[i] > 0.5:
                nb_ok += 1

            # ratio matching particles
        # print("Proportion of matching particles:", 1. * nb_ok / len(weights))

        # Resample
        resample_count = particles_count - self.N_uniform
        assert weights.sum() > 0.
        weights /= weights.sum()
        self.weights = weights

        resampled = particles[np.random.choice(particles_count, resample_count, p=weights)]
        new_particles = np.random.uniform(0, 1, [self.N_uniform, 3]) * [
            self.ground_map_left.shape[0], self.ground_map_left.shape[1], np.pi * 2]
        particles[:resample_count] = resampled
        particles[resample_count:] = new_particles
        assert particles.shape[0] == particles_count

        # add adaptive noise to fight particle depletion
        one_N3 = 1. / pow(particles_count, 1. / 3.)
        range_x = self.ground_map_left.shape[0] * one_N3
        range_y = self.ground_map_left.shape[1] * one_N3
        range_theta = 2. * np.pi * one_N3

        # for i in range(particles_count):
        #     particles[i, 0] += np.random.uniform(-range_x / 2., range_x / 2.)
        #     particles[i, 1] += np.random.uniform(-range_y / 2., range_y / 2.)
        #     particles[i, 2] += np.random.uniform(-range_theta / 2., range_theta / 2.)
        particles[:, 0] += np.random.uniform(-range_x / 2., range_x / 2., particles_count)
        particles[:, 1] += np.random.uniform(-range_y / 2., range_y / 2., particles_count)
        particles[:, 2] += np.random.uniform(-range_theta / 2., range_theta / 2., particles_count)

        self.particles = particles

    # @jit
    def apply_command(self, d_x, d_y, d_theta):
        """ Apply command to each particle """
        d_xy = np.array([d_x, d_y])
        particles = np.asarray(self.particles)
        particles_count = self.particles.shape[0]

        # error model
        norm_xy = np.sqrt(d_x ** 2 + d_y ** 2)
        e_theta = self.alpha_theta * abs(d_theta) + np.radians(0.25)
        assert e_theta > 0, e_theta
        e_xy = self.alpha_xy * norm_xy + 0.01
        assert e_xy > 0, e_xy

        # apply command and sampled noise to each particle
        for i in range(particles_count):
            theta = particles[i, 2]
            particles[i, 0:2] += ut.rot_mat2(theta).dot(d_xy) + np.random.normal(0, e_xy, [2])
            particles[i, 2] = theta + d_theta + np.random.normal(0, e_theta)

        self.particles = particles

    @staticmethod
    @jit(nopython=True)
    def estimate_state(particles, particles_count, conf_xy, conf_theta):
        # TODO something smarter than the mean, but maybe not as overkill as RANSAC
        # limits for considering participating to the state estimation
        theta_lim = np.radians(5)
        xy_lim = 1.5

        # particles = self.particles
        max_index = particles_count - 1
        iterations_count = 500
        tests_count = 500
        # assert iterations_count <= max_index and tests_count <= max_index

        # no replacement

        tmp = []
        for i in range(max_index):
            tmp.append(i)
        lin = np.array(tmp)
        # np.array(np.linspace(0, max_index, max_index + 1), dtype=np.int)
        iteration_indices = np.random.choice(lin, iterations_count, replace=False)
        test_indices = np.random.choice(lin, tests_count, replace=False)
        best_index = -1
        support, best_support = 0, 0
        # tries a certain number of times
        for i in range(iterations_count):
            index = iteration_indices[i]
            x = particles[index, 0]
            y = particles[index, 1]
            theta = particles[index, 2]
            support = 0
            for j in range(tests_count):
                o_index = test_indices[j]
                o_x = particles[o_index, 0]
                o_y = particles[o_index, 1]
                o_theta = particles[o_index, 2]
                # compute distance
                dist_xy = np.sqrt((x - o_x) * (x - o_x) + (y - o_y) * (y - o_y))
                dist_theta = ut.normalize_angle(theta - o_theta)
                if dist_xy < xy_lim and dist_theta < theta_lim:
                    support += 1
            # if it beats best, replace best
            if support > best_support:
                best_index = index
                best_support = support

        # then do the averaging for best index
        x = particles[best_index, 0]
        y = particles[best_index, 1]
        theta = particles[best_index, 2]

        count, conf_count, xs, ys, ths = 0, 0, 0, 0, 0
        sins, coss = [], []
        for j in range(tests_count):
            o_index = test_indices[j]
            o_x = particles[o_index, 0]
            o_y = particles[o_index, 1]
            o_theta = particles[o_index, 2]
            dist_xy = np.sqrt((x - o_x) * (x - o_x) + (y - o_y) * (y - o_y))
            dist_theta = ut.normalize_angle(theta - o_theta)
            if dist_xy < xy_lim and dist_theta < theta_lim:
                sins.append(np.sin(o_theta))
                coss.append(np.cos(o_theta))
                # ths += ut.normalize_angle(theta)
                xs += o_x
                ys += o_y
                count += 1
            if dist_xy < conf_xy and dist_theta < conf_theta:
                conf_count += 1

        # assert count > 0, count

        x_m = xs / count
        y_m = ys / count
        theta_m = np.arctan2(np.sum(np.asarray(sins)), np.sum(np.asarray(coss)))  # ths / count
        return np.array([x_m, y_m, theta_m]), float(conf_count) / float(tests_count)

        # mean = np.mean(self.particles, axis=0)
        # self.estimated_particle = mean
        # return mean[0], mean[1], mean[2], 42


    def dump_PX(self, base_filename, gt_x=-1, gt_y=-1, gt_theta=-1):
        """ Write particles to an image """
        fig = Figure((3, 3), tight_layout=True)
        canvas = FigureCanvas(fig)
        ax = fig.gca()
        ax.set_xlim([0, self.ground_map_left.shape[0]])
        ax.set_ylim([0, self.ground_map_left.shape[1]])

        for (x, y, theta) in self.particles:
            ax.arrow(x, y, np.cos(theta), np.sin(theta), head_width=0.8, head_length=1, fc='k', ec='k', alpha=0.3)

        ax.arrow(gt_x, gt_y, np.cos(gt_theta) * 2, np.sin(gt_theta) * 2, head_width=1, head_length=1.2, fc='green',
                 ec='green')

        ax.arrow(self.estimated_particle[0], self.estimated_particle[1], np.cos(self.estimated_particle[2]) * 2,
                 np.sin(self.estimated_particle[2]) * 2, head_width=1, head_length=1.2, fc='blue', ec='blue')

        canvas.print_figure(base_filename + '.png', dpi=300)


    def is_in_bound_cell(self, x, y):
        """ Return whether a given position x,y (as int) is within the bounds of a 2D array """
        if 0 <= x < self.ground_map_left.shape[0] and 0 <= y < self.ground_map_left.shape[1]:
            return True
        else:
            return False

    def is_in_bound(self, pos):
        """ Check whether a given position is within the bounds of a 2D array """
        assert pos.shape[0] == 2
        x = ut.xyW2C(pos[0])
        y = ut.xyW2C(pos[1])
        return self.is_in_bound_cell(x, y)

