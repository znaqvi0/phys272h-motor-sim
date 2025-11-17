import matplotlib.pyplot as plt

from motor import *

awg_to_diameter = {  # AWG to approximate diameter in meters
    8: 3.2E-3,
    12: 2E-3,
    18: 1E-3,
    24: 0.5E-3
}

# ------------------------------------------------high-level parameters-------------------------------------------------
n_turns = 12
n_coils = 8
length = 0.1  # meters
B = 1.0  # tesla
inertia_moment = 0.5 * 2 * 0.2**2  # moment of 2 kg disk of radius 20cm
input_voltage = 12  # volts
wire_diameter = awg_to_diameter[24]
# ----------------------------------------------------------------------------------------------------------------------

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

motor = Motor(n_turns=n_turns, n_coils=n_coils, length=length, B=B, inertia_moment=inertia_moment, input_voltage=input_voltage, wire_diameter=wire_diameter, friction_torque=0.1, stall=False)

for i in range(int(4/dt)):  # 4 simulated seconds
    motor.step()

    # record relevant values
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
print("stall torque: %.2f Nm" % motor.stall_torque)
print("stall current: %.2f A" % motor.stall_current)

# torque and power (torque is in Ndm to make it fit on the graph better)
ax1 = plt.gca()
ax1.plot(omegas, np.array(torques) * 10, label='Torque (Ndm)', color='blue')
ax1.plot(omegas, powers, label='Power out (W)', color='green')
ax1.plot(omegas, currents, label='Current (A)', color='red')

ax1.set_title(f'Motor Performance Curves ({n_coils} Coils)')
ax1.set_xlabel('Omega (rad/s)')
ax1.set_ylabel('Torque (Ndm) / Power (W) / Current (A)')
ax1.legend(loc='upper left')
ax1.grid(True)

# efficiency (on a separate y-axis to make it fit better)
ax2 = ax1.twinx()
ax2.plot(omegas, efficiencies, label='Efficiency', color='purple', linestyle=':')
ax2.set_ylabel('Efficiency')
ax2.set_ylim(0, 1.0)
ax2.legend(loc='upper right')

plt.tight_layout()
plt.show()