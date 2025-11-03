import numpy as np

PI = np.pi
copper_resistivity = 1.68E-8
copper_density = 8960  # kg/m^3
dt = 1.0E-5
mu0 = 4E-7 * PI

class Motor:
    def __init__(self, n_turns, n_coils, length, B, inertia_moment, input_voltage, wire_diameter, stall=False):
        self.n_turns = n_turns
        self.n_coils = n_coils
        self.length = length  # length of one side of a square coil
        self.B = B
        self.inertia_moment = inertia_moment
        self.input_voltage = input_voltage
        self.wire_diameter = wire_diameter
        self.stall = stall

        # derived properties
        self.coil_area = length ** 2
        self.angle_between_coils = PI / self.n_coils
        self.coil_wire_length = self.n_turns * 4 * length  # for a square coil
        self.wire_cross_section_area = PI * ((self.wire_diameter / 2) ** 2)
        self.coil_resistance = copper_resistivity * self.coil_wire_length / self.wire_cross_section_area
        self.B_mag = np.linalg.norm(self.B)

        # https://learnemc.com/ext/calculators/inductance/square.html
        self.inductance = (self.n_turns ** 2) * 2 * mu0 * self.length / PI * (np.log(2 * self.length / self.wire_diameter) - 0.774)

        self.radius = self.length / 2
        self.stall_current = self.input_voltage / self.coil_resistance
        self.stall_torque = 2 * self.radius * self.n_turns * self.stall_current * self.length * self.B_mag  # = 2*R*N*ILB

        # state variables
        self.theta = 0
        self.omega = 0
        self.t = 0
        self.current = 0

        # power curve variables
        self.torque = 0
        self.power = 0
        self.efficiency = 0

        # max values
        self.max_torque = 0
        self.max_current = 0

        rl_time_constant = self.inductance / self.coil_resistance
        if dt > rl_time_constant:
            print("dt > L/R time constant; simulation may be unstable")

        print("L/R time constant: %.4f" % rl_time_constant)


    def step(self):
        coil_angles = [self.theta + self.angle_between_coils * i for i in range(self.n_coils)]

        # active back emf is the max of all possible back emfs (for an ideal commutator)
        back_emf_factors = [abs(np.sin(angle)) for angle in coil_angles]
        active_back_emf_factor = max(back_emf_factors)
        back_emf = self.n_turns * self.B_mag * self.coil_area * self.omega * active_back_emf_factor

        # from loop rule: V_in - back_emf - IR - L * dI/dt = 0
        # L * dI/dt = V_in - back_emf - L * dI/dt
        inductor_emf = self.input_voltage - back_emf - (self.current * self.coil_resistance)

        # solve loop rule equation for dI/dt
        current_rate_of_change = inductor_emf / self.inductance

        self.current += current_rate_of_change * dt
        self.current = max(0, self.current)

        force_on_coil = self.n_turns * self.current * self.length * self.B_mag  # F = N * ILB

        # active torque is the max of all possible torques (for an ideal commutator)
        torque_factors = [abs(np.cos(angle)) for angle in coil_angles]
        active_torque_factor = max(torque_factors)

        friction_torque = 0.1 # 1E-4 * self.omega
        self.torque = max(0, 2 * self.radius * force_on_coil * active_torque_factor - friction_torque)

        # update max variables
        self.max_torque = max(self.torque, self.max_torque)
        self.max_current = max(self.current, self.max_current)

        # rotational kinematics
        stall_factor = int(not self.stall)  # 0 acceleration if True; normal acceleration if False
        alpha = self.torque / self.inertia_moment * stall_factor
        self.omega += alpha * dt
        self.theta += self.omega * dt

        self.t += dt

        # power and efficiency calculations
        power_in = self.current * self.input_voltage  # electrical input power
        self.power = self.torque * self.omega  # mechanical output power
        if abs(self.current) < 0.01:  # prevent division by 0
            self.efficiency = 0
        else:
            self.efficiency = self.power / power_in
