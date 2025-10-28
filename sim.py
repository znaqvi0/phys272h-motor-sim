import matplotlib.pyplot as plt

from motor import *

awg_to_diameter = {  # AWG to approximate diameter in meters
    8: 3.2E-3,
    12: 2E-3,
    18: 1E-3,
    24: 0.5E-3
}

n_turns = 10
n_coils = 8
length = 0.1
B = np.array((0, 1, 0)) * 1.0  # 0.5 T magnetic field in the +y direction
inertia_moment = 0.5 * 0.1 * 0.2**2  # moment of 0.1 kg disk of radius 20cm
input_voltage = 12
wire_diameter = awg_to_diameter[18]

wire_cross_section_area = PI * ((wire_diameter / 2) ** 2)
copper_volume = 4 * length * n_turns * n_coils * wire_cross_section_area
copper_mass = copper_volume * copper_density

print("copper mass: %.3f kg" % copper_mass)
print("copper volume: %.3f cm^3" % (copper_volume * 1E6))

torques = []
times = []
omegas = []
powers = []
efficiencies = []
angles = []
currents = []

motor = Motor(n_turns=n_turns, n_coils=n_coils, length=length, B=B, inertia_moment=inertia_moment, input_voltage=input_voltage, wire_diameter=wire_diameter, stall=False)

sample_rate = 1
for i in range(int(2/dt)):
    motor.step()
    if i % sample_rate != 0:
        continue
    torques.append(motor.torque)
    omegas.append(motor.omega)
    times.append(motor.t)
    powers.append(motor.power)
    efficiencies.append(motor.efficiency)
    angles.append(motor.theta)
    currents.append(motor.current)

print("final speed: %.2f rad/s" % motor.omega)
print("max torque: %.2f Nm" % motor.max_torque)
print("max current: %.2f A" % motor.max_current)

# torque and power
ax1 = plt.gca()
ax1.plot(omegas, np.array(torques) * 10, label='Torque (Ndm)', color='blue')
ax1.plot(omegas, powers, label='Power out (W)', color='green')
ax1.plot(omegas, currents, label='Current (A)', color='red')

ax1.set_title(f'Motor Performance Curve ({n_coils} Coils)')
ax1.set_xlabel('Omega (rad/s)')
ax1.set_ylabel('Torque (Ndm) / Power (W) / Current (A)')
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