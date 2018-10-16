import numpy as np
import matplotlib.pyplot as plt


class Drone:

    def __init__(self, height=0.0, velocity=0.0, acceleration=0.0, mass=10.0, kp=0.0, ki=0.0, kd=0.0, goal=0.0):
        self.integral = 0                   # 高度偏差の積分(ms)
        self.height = height                # 高度(m)
        self.velocity = velocity            # 速度(m/s)
        self.acceleration = acceleration    # 加速度(m/s^2)
        self.mass = mass                    # 重量(kg)
        self.Kp = kp                        # Pゲイン
        self.Ki = ki                        # Iゲイン
        self.Kd = kd                        # Dゲイン
        self.goal = goal                    # 目標高度(m)
        self.difference = self.goal - self.height  # 高度偏差

        self.gravity = Gravity()
        self.rotor = Rotor(self)

        self.forces = [self.gravity, self.rotor]        # ドローンにかかる加速度
        self.acceleration = self.get_acceleration()     # 加速度の初期化

    def get_acceleration(self):
        return sum([force.get() for force in self.forces])

    def __getitem__(self, item):
        return item

    def update(self, interval):     # 時間をinterval分だけ進めて加速度、速度、高度を計算
        self.acceleration = self.get_acceleration()                 # 加速度の合計を取る
        self.velocity += self.acceleration * interval               # 加速度を積分して速度を得る
        self.height += self.velocity * interval                     # 速度を積分して高度を得る
        self.difference = self.goal - self.height                   # 高度偏差の再計算
        self.integral += self.difference * interval                 # 高度偏差の積分


class Simulator:

    def __init__(self, drone, duration=5.0, interval=0.05):
        self.drone = drone
        self.interval = interval
        self.duration = duration

        self.time = []
        self.height = []
        self.goal = []
        self.fig, self.ax = plt.subplots()

    def plot(self, time, drone):
        self.time.append(time)
        self.height.append(drone.height)
        self.goal.append(drone.goal)

    def clear_plot(self):
        self.time = []
        self.height = []
        self.goal = []

    def show_plot(self):
        self.ax.plot(self.time, self.height)
        self.ax.plot(self.time, self.goal)
        plt.show()

    def run(self):
        self.clear_plot()
        for time in np.arange(0, self.duration, self.interval):
            self.plot(time, self.drone)             # プロットする
            self.drone.update(self.interval)        # 時間を進める


class Acceleration:
    def __init__(self, val=0.0):
        self.val = val

    def update(self):
        pass

    def get(self):
        self.update()
        return self.val


class Gravity(Acceleration):

    G = -9.8  # 重力加速度

    def __init__(self):
        super().__init__(self.G)


class Rotor(Acceleration):

    def __init__(self, drone):
        super().__init__()
        self.drone = drone

    def update(self):
        self.val = self.drone.difference * self.drone.Kp                    # P
        self.val += self.drone.integral * self.drone.Ki                     # I
        self.val += -self.drone.velocity * self.drone.Kd                    # D (目標速度=0 - 現在速度)
        self.val /= self.drone.mass                                         # Nを加速度にする
