import numpy as np

PI = np.pi
copper_resistivity = 1.68E-8
copper_density = 8960  # kg/m^3
dt = 1.0E-5
mu0 = 4E-7 * PI

class Motor:
    def __init__(self, n_turns, n_coils, length, B, inertia_moment, input_voltage, wire_diameter, friction_torque, stall=False):
        self.n_turns = n_turns
        self.n_coils = n_coils
        self.length = length  # length of one side of a square coil
        self.B = B
        self.inertia_moment = inertia_moment
        self.input_voltage = input_voltage
        self.wire_diameter = wire_diameter
        self.friction_torque = friction_torque
        self.stall = stall

        # derived properties
        self.coil_area = length ** 2
        self.angle_between_coils = PI / self.n_coils

        self.coil_wire_length = self.n_turns * 4 * length  # for a square coil
        self.wire_cross_section_area = PI * ((self.wire_diameter / 2) ** 2)
        self.coil_resistance = copper_resistivity * self.coil_wire_length / self.wire_cross_section_area

        self.motor_resistance = n_coils / 4 * self.coil_resistance  # R = N/4 * R_coil in a motor with 2 parallel paths
        self.active_coils = -(-n_coils // 2)  # ceiling division of N/2 for a motor with 2 parallel paths

        self.inductance = 0.005 * self.motor_resistance  # assume time constant of 5 ms and work backwards to inductance

        # state variables
        self.theta = 0
        self.omega = 0
        self.t = 0
        self.current = input_voltage / self.motor_resistance

        # power curve variables
        self.torque = 0
        self.power = 0
        self.efficiency = 0

        # max values
        self.max_torque = 0
        self.max_current = 0

        # stall calculations
        self.stall_current = self.input_voltage / self.motor_resistance
        torque_factors = [abs(np.cos(angle)) for angle in self.active_coil_angles()]
        # tau = mu * Bcos(theta) = NIABcos(theta)
        self.stall_torque = self.n_turns * self.stall_current * self.coil_area * self.B * sum(torque_factors)

        rl_time_constant = self.inductance / self.motor_resistance
        if dt > rl_time_constant:
            print("dt > L/R time constant; simulation may be unstable")

        print("L/R time constant: %.4f" % rl_time_constant)

    def active_coil_angles(self) -> list:
        coil_angles = [self.theta + self.angle_between_coils * i for i in range(self.n_coils)]
        coil_angles = sorted(coil_angles, key=lambda angle: abs(np.cos(angle)), reverse=True)  # sort in descending order by torque factor
        return coil_angles[0:self.active_coils]

    def step(self):
        coil_angles = [self.theta + self.angle_between_coils * i for i in range(self.n_coils)]

        # back emf is the sum of active back emfs
        back_emf_factors = [abs(np.cos(angle)) for angle in coil_angles]
        back_emf_factors = sorted(back_emf_factors, reverse=True)[0:self.active_coils]
        back_emf = self.n_turns * self.B * self.coil_area * self.omega * sum(back_emf_factors)

        # from loop rule: V_in - back_emf - IR - L * dI/dt = 0
        # L * dI/dt = V_in - back_emf - IR
        inductor_emf = self.input_voltage - back_emf - (self.current * self.motor_resistance)

        # solve loop rule equation for dI/dt
        current_rate_of_change = inductor_emf / self.inductance
        self.current += current_rate_of_change * dt

        # torque is the sum of active torques
        torque_factors = [abs(np.cos(angle)) for angle in coil_angles]
        torque_factors = sorted(torque_factors, reverse=True)[0:self.active_coils]
        self.torque = max(0, self.n_turns * self.current * self.coil_area * self.B * sum(torque_factors) - self.friction_torque)

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
