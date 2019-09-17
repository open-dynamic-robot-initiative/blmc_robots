"""Quick & Dirty logger class."""
import os
from collections import namedtuple
import pickle
import matplotlib.pyplot as plt


Observation = namedtuple("Observation", ["torque", "velocity", "angle",
                                         "desired_action", "applied_action"])


class Logger:

    def __init__(self):
        self.name = "unnamed"
        self.clear()

    def set_time(self, t):
        self.t = t

    def start_new_recording(self, name):
        self.name = name
        self.clear()

    def record(self, robot):
        now = robot.get_current_time_index()
        while self.t < now:
            try:
                obs = robot.get_observation(self.t)
                des_action = robot.get_desired_action(self.t)
                app_action = robot.get_applied_action(self.t)

                obs_cpy = Observation(torque=obs.torque,
                                      velocity=obs.velocity,
                                      angle=obs.angle,
                                      desired_action=des_action,
                                      applied_action=app_action)
                self.data.append((self.t, obs_cpy))
            except:
                pass

            self.t += 1

    def check_integrity(self):
        print("record size: %d" % len(self.data))

        if self.data:
            miss = 0
            last = self.data[0][0]

            for d in self.data[1:]:
                if last + 1 != d[0]:
                    miss += 1
                last = d[0]

            print("missing entries: %d" % miss)

    def plot(self):
        ts = tuple(d[0] for d in self.data)
        angles = tuple(d[1].angle[0] for d in self.data)
        velocities = tuple(d[1].velocity[0] for d in self.data)
        torques = tuple(d[1].torque[0] for d in self.data)
        currents = tuple(d[1].torque[0] / (0.02 * 9.0) for d in self.data)
        des_actions = tuple(d[1].desired_action[0] for d in self.data)
        app_actions = tuple(d[1].applied_action[0] for d in self.data)
        app_actions_A = tuple(d[1].applied_action[0] / (0.02 * 9.0) for d in self.data)

        fig, axes = plt.subplots(3, 2)

        axes[0, 0].set_title("Torque [Nm]")
        axes[0, 0].plot(ts, torques)
        axes[0, 1].set_title("Current [A]")
        axes[0, 1].plot(ts, currents)
        axes[1, 0].set_title("Velocity [?]")
        axes[1, 0].plot(ts, velocities)
        axes[1, 1].set_title("Angle [rad]")
        axes[1, 1].plot(ts, angles)
        axes[2, 0].set_title("Commanded Torque [Nm]")
        axes[2, 0].plot(ts, des_actions, label="desired")
        axes[2, 0].plot(ts, app_actions, linestyle=":", label="applied")
        axes[2, 0].legend()
        axes[2, 1].set_title("Applied vs Measured Current [A]")
        axes[2, 1].plot(ts, app_actions_A, label="applied")
        axes[2, 1].plot(ts, currents, linestyle=":", label="measured")
        axes[2, 1].legend()


        plt.show()

    def dump(self, filename=None):
        if not filename:
            filename = self.name + ".p"

        # create directory if not exists
        dirname = os.path.dirname(filename)
        if not os.path.isdir(dirname):
            os.makedirs(dirname)

        with open(filename, "wb") as fh:
            pickle.dump(self.data, fh)
        print("Stored to %s" % filename)

    def load(self, filename):
        with open(filename, "rb") as fh:
            self.data = pickle.load(fh)

    def clear(self):
        self.t = 0
        self.data = []


