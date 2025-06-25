import math

# -------------------- Wheel & Pose Estimation --------------------

def get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t):
    ang_diff_l = 2 * math.pi * (encoderValues[0] - oldEncoderValues[0]) / pulses_per_turn
    ang_diff_r = 2 * math.pi * (encoderValues[1] - oldEncoderValues[1]) / pulses_per_turn
    wl = ang_diff_l / delta_t
    wr = ang_diff_r / delta_t
    return wl, wr

def get_robot_speeds(wl, wr, R, D):
    u = R / 2.0 * (wr + wl)
    w = R / D * (wr - wl)
    return u, w

def get_robot_pose(u, w, x, y, phi, delta_t):
    delta_phi = w * delta_t
    phi += delta_phi
    if phi >= math.pi:
        phi -= 2 * math.pi
    elif phi < -math.pi:
        phi += 2 * math.pi
    delta_x = u * math.cos(phi) * delta_t
    delta_y = u * math.sin(phi) * delta_t
    x += delta_x
    y += delta_y
    return x, y, phi

# -------------------------- PID Control --------------------------

def get_pose_error(xd, yd, x, y, phi):
    x_err = xd - x
    y_err = yd - y
    dist_err = math.sqrt(x_err ** 2 + y_err ** 2)
    phi_d = math.atan2(y_err, x_err)
    phi_err = math.atan2(math.sin(phi_d - phi), math.cos(phi_d - phi))
    return dist_err, phi_err

def pid_controller(e, e_prev, e_acc, delta_t, kp=2.0, kd=0.1, ki=0.00):
    P = kp * e
    I = e_acc + ki * e * delta_t
    D = kd * (e - e_prev) / delta_t
    output = P + I + D
    e_prev = e
    e_acc = I
    return output, e_prev, e_acc

def wheel_speed_commands(u_d, w_d, d, r):
    wr_d = (2 * u_d + d * w_d) / (2 * r)
    wl_d = (2 * u_d - d * w_d) / (2 * r)

    abs_wr = abs(wr_d)
    abs_wl = abs(wl_d)

    if abs_wl > MAX_SPEED or abs_wr > MAX_SPEED:
        speed_ratio = abs_wr / abs_wl if abs_wl != 0 else 1
        if speed_ratio > 1:
            wr_d = math.copysign(MAX_SPEED, wr_d)
            wl_d = math.copysign(MAX_SPEED / speed_ratio, wl_d)
        else:
            wl_d = math.copysign(MAX_SPEED, wl_d)
            wr_d = math.copysign(MAX_SPEED * speed_ratio, wr_d)

    return wl_d, wr_d

# -------------------- Percent Mapping --------------------

def map_pid_to_percent(output, max_pid=6, min_percent=30, max_percent=80):
    output = max(-max_pid, min(max_pid, output))
    percent = ((output + max_pid) / (2 * max_pid)) * (max_percent - min_percent) + min_percent
    return percent

# -------------------- Initialization --------------------

# xd, yd = 0.0765, 0.23
MAX_SPEED = 100
pulses_per_turn = 1939
delta_t = 0.16
encoderValues = [1939, 1939]
oldEncoderValues = [0, 0]

x_old, y_old, phi_old = 0.0, 0.0, math.pi
R = 0.0336
D = 0.097

# -------------------- Run One Step --------------------

wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t)
u, w = get_robot_speeds(wl, wr, R, D)
x, y, phi = get_robot_pose(u, w, x_old, y_old, phi_old, delta_t)

print(x, y, phi)
# position_err, orientation_err = get_pose_error(xd, yd, x, y, phi)
#
# # PID control
# e = orientation_err
# e_prev = 0
# e_acc = 0
# u_d = 0  # pure rotation
#
# w_d, e_prev, e_acc = pid_controller(orientation_err, e_prev, e_acc, delta_t)
# wl_d, wr_d = wheel_speed_commands(u_d, w_d, D, R)
#
# # Map to percent
# left_speed_percent = map_pid_to_percent(wl_d)
# right_speed_percent = map_pid_to_percent(wr_d)
#
# # -------------------- Output --------------------
#
# print(f"Robot Pose: x={x:.3f}, y={y:.3f}, phi={phi:.3f} rad")
# print(f"Orientation Error: {orientation_err:.3f} rad")
# print(f"Wheel Speed %: Left = {left_speed_percent:.1f}%, Right = {right_speed_percent:.1f}%")


def test_segment(from_pose, to_point, phi, label=""):
    x_old, y_old = from_pose
    xd, yd = to_point

    position_err, orientation_err = get_pose_error(xd, yd, x_old, y_old, phi)

    e_prev = 0
    e_acc = 0
    u_d = 0  # still just rotation

    w_d, e_prev, e_acc = pid_controller(orientation_err, e_prev, e_acc, delta_t)
    wl_d, wr_d = wheel_speed_commands(u_d, w_d, D, R)

    left_speed_percent = map_pid_to_percent(wl_d)
    right_speed_percent = map_pid_to_percent(wr_d)

    print(f"== {label} ==")
    print(f"From: ({x_old:.3f}, {y_old:.3f}), To: ({xd:.3f}, {yd:.3f})")
    print(f"Orientation Error: {math.degrees(orientation_err):.2f}°")
    print(f"Left Speed %: {left_speed_percent:.1f}, Right Speed %: {right_speed_percent:.1f}")
    print("")

# Case A: Needs to turn ~90° (upward motion from horizontal start)
# test_segment((-0.0, 0.0), (-0.0, 0.115), 0, "Turn ~90° Up")
#
# # Case B: Needs to turn ~0° (straight horizontal)
# test_segment((-0.0, 0.115), (-0.0, 0.23), math.pi / 2, "Go Straight Up")
#
# # Case C: Needs to turn ~90° Right
# test_segment((-0.0, 0.23), (-0.0765, 0.23), math.pi / 2, "Turn ~90° Left")
#
# # Case D: Needs to turn ~180°
# test_segment((-0.0765, 0.23), (-0.0, 0.23), math.pi, "Turn ~180° Back")
#
# # Case E: Needs to turn ~30°
# test_segment((-0.0, 0.0), (-0.1, 0.0577), math.pi, "Turn ~30° Right")
#
# # From (0.0, 0.23) to (0.0765, 0.23) — facing right (0 radians)
# test_segment((-0.0, 0.23), (-0.0765, 0.23), math.pi, "Go Straight → Right in X")
