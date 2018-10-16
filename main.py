from simulator import *

if __name__ == "__main__":

    drone = Drone(mass=2.0, kp=140.0, ki=8.0, kd=60.0, goal=10)
    simulator = Simulator(drone, duration=5.0)
    simulator.run()
    simulator.show_plot()
