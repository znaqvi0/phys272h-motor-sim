import numpy as np

PI = np.pi
copper_resistivity = 1.68E-8
dt = 1.0E-3
mu0 = 4E-7 * PI

class Motor:
    def __init__(self, n_turns, n_coils, length, B, inertia_moment, input_voltage, wire_diameter):
        self.n_turns = n_turns
        self.n_coils = n_coils
        self.length = length  # length of one side of a square coil
        self.B = B
        self.inertia_moment = inertia_moment
        self.input_voltage = input_voltage
        self.wire_diameter = wire_diameter

        self.theta = 0
        self.coil_area = length ** 2
        self.angle_between_coils = 2 * PI / self.n_coils
        self.phi_max = np.linalg.norm(self.B * self.coil_area)
        self.coil_wire_length = self.n_turns * 4 * length  # for a square coil
        self.wire_cross_section_area = PI * ((self.wire_diameter / 2) ** 2)
        self.coil_resistance = copper_resistivity * self.coil_wire_length / self.wire_cross_section_area

        self.omega = 0
        self.t = 0

    def step(self):
        back_emf = self.back_emf()
        current = (self.input_voltage + back_emf) / self.coil_resistance

        length_vector = self.length * np.array((-1, 0, 0))
        force_on_coil = self.n_turns * current * np.cross(length_vector, self.B)

        current_coil_angle = self.theta % self.angle_between_coils
        r_vector = (self.length / 2) * np.array((0, np.cos(current_coil_angle), np.sin(current_coil_angle)))

        torque_vector = 2 * np.cross(r_vector, force_on_coil)
        torque = np.dot(torque_vector, np.array((-1, 0, 0)))  #np.linalg.norm(torque_vector)

        alpha = torque / self.inertia_moment
        self.omega += alpha * dt
        self.theta += self.omega * dt

        self.t += dt
        return torque

    def back_emf(self):
        dphi_dt = np.linalg.norm(self.B) * self.coil_area * self.omega * np.sin(self.theta)
        return -self.n_turns * dphi_dt

