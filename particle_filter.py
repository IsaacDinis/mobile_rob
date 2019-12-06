import numpy as np
from numba import jit
import utils as ut

from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

class MonteCarlo:
    # _pi = np.pi
    # _1sqrt2pi = 1. / np.sqrt(2. * _pi)

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
            particles = np.random.uniform(0, 1, size=[particles_count, 3])
            particles = particles * [ground_map_left.shape[0], ground_map_left.shape[1], np.pi * 2]
        else:
            particles = np.random.normal(state_init, np.asarray([1, 1, np.pi/20]), size=(particles_count, 3))
        self.particles = particles
        self.weights = np.zeros(particles_count)
        self.estimated_particle = np.zeros_like(particles[0, :], dtype=float)

    def apply_obs_and_resample(self, left_color, right_color):
        mapshape = self.ground_map_left.shape
        self.particles, self.weights = self._apply_obs_and_resample(self.particles, self.particles.shape[0],
                                                                    self.ground_map_left, self.ground_map_right,
                                                                    self.sigma_obs, self.N_uniform, mapshape,left_color,
                                                                    right_color)

    @staticmethod
    @jit(nopython=True)
    def _apply_obs_and_resample(particles, particles_count, ground_map_left, ground_map_right,
                                sigma_obs, N_uniform, mapshape, left_color, right_color):
        """ Apply observation and update probability weights, then resample"""
        ratioA0 = 1.0877  # because the printed map doesn't have the theoretical dimension
        particles_count = particles_count
        # particles = np.asarray(particles)
        weights = np.zeros(particles_count)
        nb_ok = 0
        for i in range(particles_count):
            theta = particles[i, 2]

            # compute position of sensors in world coordinates
            rot = ut.rot_mat2(theta)
            left_sensor_pos = rot.dot(np.array([7.2*ratioA0, 1.1*ratioA0])) + particles[i, 0:2]
            right_sensor_pos = rot.dot(np.array([7.2*ratioA0, -1.1*ratioA0])) + particles[i, 0:2]

            if not ut.is_in_bound(mapshape, left_sensor_pos) or not ut.is_in_bound(mapshape, right_sensor_pos):
                # kill particle if out of map
                weights[i] = 0.
            else:
                # otherwise, compute weight in function of ground color
                # left sensor
                ground_val_left = ground_map_left[ut.xyW2C(left_sensor_pos[0]), ut.xyW2C(left_sensor_pos[1])]
                left_weight = ut.norm(left_color, ground_val_left, sigma_obs)

                # right sensor
                ground_val_right = ground_map_right[ut.xyW2C(right_sensor_pos[0]), ut.xyW2C(right_sensor_pos[1])]
                right_weight = ut.norm(right_color, ground_val_right, sigma_obs)

                # compute weight
                weights[i] = left_weight * right_weight  # + 1e-6

            # update matching particles
            if weights[i] > 0.5:
                nb_ok += 1

            # ratio matching particles
        # print("Proportion of matching particles:", 1. * nb_ok / len(weights))

        # Resample
        resample_count = particles_count - N_uniform
        # assert weights.sum() > 0.
        if not(weights.sum() > 0.):
            print("not(weights.sum() > 0.):", weights.sum())
            print(weights / weights.sum())
            weights += 1e-10
        else:
            weights /= weights.sum()
        # self.weights = weights

        # resampled = particles[np.random.choice(particles_count, resample_count, p=weights)]
        # workaround for numba
        csum = np.cumsum(weights)
        resampled = np.empty((resample_count, 3))
        for k in range(resample_count):
            resampled[k,:] = particles[np.searchsorted(csum, np.random.random(), side="right"), :]
        # resampled = np.array(tmp)
        # if N_uniform:
        #     new_particles = ut.weird(N_uniform, mapshape)
        #     particles[resample_count:] = new_particles
        #     particles[:resample_count] = resampled
        # else:
        particles[:resample_count] = resampled

        # assert particles.shape[0] == particles_count
        if not(particles.shape[0] == particles_count):
            print("Assert failure: not(particles.shape[0] == particles_count)")

        # add adaptive noise to fight particle depletion
        one_N3 = 1. / pow(particles_count, 1. / 3.)
        range_x = np.float(mapshape[0]) * one_N3
        range_y = np.float(mapshape[1]) * one_N3
        range_theta = 2. * np.pi * one_N3

        particles[:, 0] += np.random.uniform(-range_x / 2., range_x / 2., particles_count)
        particles[:, 1] += np.random.uniform(-range_y / 2., range_y / 2., particles_count)
        particles[:, 2] += np.random.uniform(-range_theta / 2., range_theta / 2., particles_count)

        return particles, weights

    def apply_command(self, d_x, d_y, d_theta):
        self.particles = self._apply_command(self.particles, self.particles.shape[0], self.alpha_theta, self.alpha_xy,
                                             np.float(d_x), np.float(d_y), d_theta)

    @staticmethod
    @jit(nopython=True)
    def _apply_command(particles, particles_count, alpha_theta, alpha_xy, d_x, d_y, d_theta):
        """ Apply command to each particle """
        d_xy = np.array([d_x, d_y])
        particles = np.asarray(particles)
        particles_count = particles_count

        # error model
        norm_xy = np.sqrt(d_x ** 2 + d_y ** 2)
        e_theta = alpha_theta * abs(d_theta) + np.radians(0.25)
        # assert e_theta > 0, e_theta
        if not (e_theta > 0):
            print("ERROR, e_theta <= 0")
        e_xy = alpha_xy * norm_xy + 0.01
        # assert e_xy > 0, e_xy
        if not e_xy > 0:
            print("ERROR, e_xy <= 0")

        # apply command and sampled noise to each particle
        for i in range(particles_count):
            theta = particles[i, 2]
            particles[i, 0:2] += ut.rot_mat2(theta).dot(d_xy) + np.random.normal(0., e_xy, 2)
            particles[i, 2] = theta + d_theta + np.random.normal(0., e_theta)

        return particles

    def estimate_state(self):
        """ Run the RANSAC algorithm to find the best approximation of the state
            :return estimated_particle --> np.array(x,y,theta)
                    confidence --> a percentage
            """
        estimated_particle, confidence = self._estimate_state(self.particles, self.particles.shape[0], self.conf_xy,
                                                              self.conf_theta)
        self.estimated_particle = estimated_particle
        return estimated_particle, confidence

    @staticmethod
    @jit(nopython=True)
    def _estimate_state(particles, particles_count, conf_xy, conf_theta):
        # TODO something smarter than the mean, but maybe not as overkill as RANSAC
        # limits for considering participating to the state estimation
        theta_lim = np.radians(5)
        xy_lim = 1.5

        # particles = self.particles
        max_index = particles_count - 1
        iterations_count = round(particles_count/100)  # 500
        tests_count = round(particles_count/100)  # 500
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

    def plot_state(self, base_filename=None, gt=np.array([-1, -1, -1]),
                   map_back=None, num_particles=-1, sens=None, path=None):
        """ Write particles to an image """
        ratioA0 = 1.0877  # because the printed map doesn't have the theoretical dimension
        fig = Figure((3, 3), tight_layout=True)
        canvas = FigureCanvas(fig)
        ax = fig.gca()
        ax.set_xlim([0, self.ground_map_left.shape[0]])
        ax.set_ylim([0, self.ground_map_left.shape[1]])
        x, y, theta = self.estimated_particle

        if map_back is not None:
            ax.imshow(np.uint8(map_back * 255))

        if path is not None:
            p = np.array(path)
            self.p = p
            ax.plot(p[0,0], p[0,1], 'rx', markersize=3)
            ax.plot(p[1:,0], p[1:,1], 'bx', markersize=3)

        if num_particles > 0:
            idx = np.random.choice(self.particles.shape[0], num_particles)
            for i in idx:
                x, y, theta = self.particles[i, :]
                ax.arrow(x, y, np.cos(theta), np.sin(theta), head_width=0.8, head_length=1, fc='k', ec='k', alpha=0.3)

        if gt[0] != -1 and gt[1] != -1:
            ax.arrow(gt[0], gt[1], np.cos(gt[2]) * 2, np.sin(gt[2]) * 2, head_width=1, head_length=1.2, fc='green',
                     ec='green')

        ax.arrow(x, y, np.cos(theta) * 2, np.sin(theta) * 2, head_width=1, head_length=1.2, fc='blue', ec='blue')

        if sens is not None:
            # if gt[0] != -1 and gt[1] != -1:
            #     rot = ut.rot_mat2(gt[2])
            #     left_sensor_pos = rot.dot([7.2*ratioA0, 1.1*ratioA0]) + np.asarray(gt[0:2])
            #     right_sensor_pos = rot.dot([7.2*ratioA0, -1.1*ratioA0]) + np.asarray(gt[0:2])
            # else:
            rot = ut.rot_mat2(theta)
            left_sensor_pos = rot.dot([7.2*ratioA0, 1.1*ratioA0]) + np.asarray([x, y])
            right_sensor_pos = rot.dot([7.2*ratioA0, -1.1*ratioA0]) + np.asarray([x, y])

            if sens[0] > 500:
                col = 'wo'
            else:
                col = 'ko'
            ax.plot(left_sensor_pos[0], left_sensor_pos[1], col, markersize=1)
            if sens[1] > 500:
                col = 'wo'
            else:
                col = 'ko'
            ax.plot(right_sensor_pos[0], right_sensor_pos[1], col, markersize=1)

        if base_filename is not None:
            canvas.print_figure(base_filename + '.png', dpi=300)
        else:
            # ax.imshow()
            fig.show()
