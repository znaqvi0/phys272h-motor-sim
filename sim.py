import matplotlib.pyplot as plt

from motor import *

n_turns = 10
n_coils = 16
length = 0.1
B = np.array((0, 1, 0)) * 1.0  # 0.5 T magnetic field in the +x direction
inertia_moment = 0.5 * 0.1 * 0.2**2  # moment of 0.1 kg disk of radius 20cm
input_voltage = 12
wire_diameter = 0.5E-3  # approximate diameter of 24 gauge wire

torques = []
times = []
omegas = []
powers = []
efficiencies = []
angles = []

motor = Motor(n_turns=n_turns, n_coils=n_coils, length=length, B=B, inertia_moment=inertia_moment, input_voltage=input_voltage, wire_diameter=wire_diameter)

sample_rate = 100
for i in range(int(2/dt)):
    motor.step()
    if i % sample_rate != 0:
        continue
    torques.append(motor.torque*10)
    omegas.append(motor.omega)
    times.append(motor.t)
    powers.append(motor.power)
    efficiencies.append(motor.efficiency)
    angles.append(motor.theta)

print("final speed: %.2f rad/s" % motor.omega)

# torque and power
ax1 = plt.gca()
ax1.plot(omegas, torques, label='Torque (Ndm)', color='blue')
ax1.plot(omegas, powers, label='Power out (W)', color='green')

ax1.set_title(f'Motor Performance Curve ({n_coils} Coils)')
ax1.set_xlabel('Omega (rad/s)')
ax1.set_ylabel('Torque (Ndm) / Power (W)')
ax1.legend(loc='upper left')
ax1.grid(True)

# efficiency
ax2 = ax1.twinx()
ax2.plot(omegas, efficiencies, label='Efficiency', color='purple', linestyle=':')
ax2.set_ylabel('Efficiency')
ax2.set_ylim(0, 1.0)
ax2.legend(loc='upper right')

plt.tight_layout()
plt.show()