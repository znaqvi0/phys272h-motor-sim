import matplotlib.pyplot as plt

from motor import *

n_turns = 10
n_coils = 8
length = 0.1
B = np.array((0, 1, 0)) * 1.0  # 0.5 T magnetic field in the +x direction
inertia_moment = 0.5 * 0.1 * 0.2**2  # moment of 0.1 kg disk of radius 20cm
input_voltage = 12
wire_diameter = 0.5E-3  # approximate diameter of 24 gauge wire

torques = []
times = []
omegas = []

motor = Motor(n_turns=n_turns, n_coils=n_coils, length=length, B=B, inertia_moment=inertia_moment, input_voltage=input_voltage, wire_diameter=wire_diameter)

for i in range(int(2/dt)):
    torque = motor.step()
    torques.append(torque)
    omegas.append(motor.omega)
    times.append(motor.t)

print("final speed: %.2f rad/s" % motor.omega)
plt.plot(times, torques)
plt.xlabel('time (s)')
plt.ylabel('torque (Nm)')
plt.title('torque vs time')
plt.show()