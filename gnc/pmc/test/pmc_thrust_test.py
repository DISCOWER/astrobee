"""
This script tests the thrust model of the PMC.

We recreate the exact Step function in the fam module,
and will improve the actuation based on this.
"""
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from data import data_size, data_breakpoints, data_cdp, impeller_diameter, air_density, abp_nozzle_intake_height, nozzle_widths, abp_nozzle_flap_length


def Lookup(size, table, breakpoints, x):
    if (x < breakpoints[0]):
        return table[0]
    if (x > breakpoints[size - 1]):
        return table[size - 1]

    start = 0
    end = size
    while (start < end - 1):
        mid = int((start + end) / 2)
        if (breakpoints[mid] <= x):
            start = mid
        else:
            end = mid
    print("start: ", start)
    ratio = (x - breakpoints[start]) / (breakpoints[start + 1] - breakpoints[start])
    return (1.0 - ratio) * table[start] + ratio * table[start + 1]


def ComputePlenumDeltaPressure(impeller_speed, discharge_coeff, nozzle_thrusts):
    total_thrust = np.divide(nozzle_thrusts, discharge_coeff).sum()
    print("TT: ", total_thrust)
    cdp = Lookup(data_size, data_cdp, data_breakpoints,
                 total_thrust / (impeller_speed * impeller_speed))
    print("cdp: ", cdp)
    return impeller_speed * impeller_speed * impeller_diameter * impeller_diameter * air_density * cdp


# Thrusters offsets and orientations
nozzle_offset_1 = np.array([6.00, 4.01, -1.56,
                            -6.00, 4.01, 1.56,
                            2.83, 6.00, 2.83,
                            -2.83, 6.00, -2.83,
                            -2.66, 4.01, 6.00,
                            2.66, 4.01, -6.00]).reshape((6, 3))

nozzle_offset_2 = np.array([-6.00, -4.01, -1.56,
                            6.00, -4.01, 1.56,
                            -2.83, -6.00, 2.83,
                            2.83, -6.00, -2.83,
                            2.66, -4.01, 6.00,
                            -2.66, -4.01, -6.00]).reshape((6, 3))

nozzle_orientation_1 = np.array([1, 0, 0,
                                -1, 0, 0,
                                0, 1, 0,
                                0, 1, 0,
                                0, 0, 1,
                                0, 0, -1]).reshape((6, 3))

nozzle_orientation_2 = np.array([-1, 0, 0,
                                1, 0, 0,
                                0, -1, 0,
                                0, -1, 0,
                                0, 0, 1,
                                0, 0, -1]).reshape((6, 3))

fam_P_nozzle_B_B = np.concatenate((nozzle_offset_1, nozzle_offset_2), axis=0)
fam_nozzle_orientations = np.concatenate((nozzle_orientation_1, nozzle_orientation_2), axis=0)

# Inertial parameters
inertia_com = np.array([0.003713818, -0.000326347, -0.002532192]).reshape((3, 1))
inertia_mass = 9.583788668
inertia_matrix = np.array([0.153427995, 0, 0,
                           0, 0.14271405, 0,
                           0, 0, 0.162302759]).reshape((3, 3))

nozzle_moment_arm = fam_P_nozzle_B_B - inertia_com.transpose().repeat(repeats=12, axis=0)

thrust2torque_ = np.empty((3, 12))
for i in range(12):
    thrust2torque_[:, i] = -np.cross(nozzle_moment_arm[i, :], fam_nozzle_orientations[i, :]).T

thrust2force_ = -fam_nozzle_orientations.T
thrust2forcetorque = np.concatenate((thrust2force_, thrust2torque_), axis=0)
forcetorque2thrust = np.linalg.pinv(thrust2forcetorque)

# Needed matrices are done, now to the rest...
setpoint = np.array([0.0, 0.0, 0.0, 0.00, 0.00, 0.00]).reshape((6, 1))

thrust_per_nozzle = forcetorque2thrust @ setpoint
fam_nozzle_dirs = np.array([1, 2, 7, 8, 3, 4, 9, 10, 5, 6, 11, 12]).reshape((3, 4))

for i in range(3):
    min_thrust = 0.0
    for j in range(4):
        t = thrust_per_nozzle[fam_nozzle_dirs[i, j] - 1]
        if t < min_thrust:
            min_thrust = t
    if min_thrust < 0:
        for j in range(4):
            thrust_per_nozzle[fam_nozzle_dirs[i, j] - 1] -= min_thrust

pmc1_nozzle_thrusts = thrust_per_nozzle[0:6]
pmc2_nozzle_thrusts = thrust_per_nozzle[6:]

# TODO(@Pedro): good up to here
# CalcPM speed command and pwm
speed_gain = int(1)  # can be 0, 1, 2

discharge_coef = np.array([0.914971062, 0.755778254, 0.940762925, 0.792109779, 0.92401881, 0.930319765]).reshape((6, 1))
impeller_speeds_vec = np.array([2000 * 2 * np.pi / 60.0, 2500 * 2 * np.pi / 60.0, 2800 * 2 * np.pi / 60.0])
impeller_speed2pwm = 0.792095

# initialize these
impeller_speed = 0.0
plenum_delta_pressure = 0.0
impeller_speed_cmd = 0.0

# Lets do it for pmc1
if speed_gain != 0:
    impeller_speed = impeller_speeds_vec[speed_gain - 1]
    impeller_speed_cmd = 0.792095 * impeller_speed + 0.5
    plenum_delta_pressure = ComputePlenumDeltaPressure(impeller_speed, discharge_coef, pmc1_nozzle_thrusts)
    print(plenum_delta_pressure)

normalized_pressure_density = 2 * np.multiply(discharge_coef, discharge_coef) * plenum_delta_pressure
command_area_per_nozzle = np.divide(pmc1_nozzle_thrusts, normalized_pressure_density)

nozzle_flap_count = 2

openings = (abp_nozzle_intake_height - (1.0 / nozzle_flap_count) * np.divide(command_area_per_nozzle, nozzle_widths)) / abp_nozzle_flap_length
print(openings)
min_open_angle = 15.68 * np.pi / 180.0
max_open_angle = 79.91 * np.pi / 180.0
min_open_cos = np.cos(max_open_angle)
max_open_cos = np.cos(min_open_angle)

nozzle_theta_cmd = np.empty((6, 1))
for i in range(6):
    if (openings[i] < min_open_cos or np.isnan(openings[i])):
        openings[i] = min_open_cos
    if (openings[i] > max_open_cos):
        openings[i] = max_open_cos

    nozzle_theta_cmd[i] = np.arccos(openings[i]) - min_open_angle

servo_min_pwm = 0
servo_max_pwm = 255
print(nozzle_theta_cmd)
servo_pwm = (servo_max_pwm - servo_min_pwm) / (max_open_angle - min_open_angle) * nozzle_theta_cmd
print(servo_pwm.astype(int))
