import numpy as np

def pioneer_controller(robot_pose, desired, gains, limits=[0.5, 0.5], a=0.15):

    robot_x, robot_y, robot_yaw = robot_pose
    desired_x, desired_y, desired_linear_x, desired_linear_y = desired
    kp_x, kp_y = gains

    Kp = np.array([[kp_x, 0],
                    [0, kp_y]])

    K = np.array([[np.cos(robot_yaw), -a*np.sin(robot_yaw)],
                    [np.sin(robot_yaw), a*np.cos(robot_yaw)]])

    Xtil = np.array([desired_x - robot_x, desired_y - robot_y]) 

    desired_velocity = np.array([desired_linear_x, desired_linear_y])

    reference_velocity = np.dot(np.linalg.inv(K), (desired_velocity.T + np.dot(Kp, Xtil.T)))

    reference_linear_velocity = reference_velocity[0]
    reference_angular_velocity = reference_velocity[1]

    linear_velocity_limit, angular_velocity_limit = limits

    if np.abs(reference_linear_velocity) > linear_velocity_limit:
        reference_linear_velocity = np.sign(reference_linear_velocity)*linear_velocity_limit

    if np.abs(reference_angular_velocity) > angular_velocity_limit:
        reference_angular_velocity = np.sign(reference_angular_velocity)*angular_velocity_limit

    return reference_linear_velocity, reference_angular_velocity

def solver_bot_controller(robot_pose, desired, gains, limits=[12, 12], a=0.15):

    robot_x, robot_y, robot_yaw = robot_pose
    desired_x, desired_y, desired_linear_x, desired_linear_y = desired
    kp_x, kp_y = gains

    Kp = np.array([[kp_x, 0],
                    [0, kp_y]])

    K = np.array([[np.cos(robot_yaw), -a*np.sin(robot_yaw)],
                    [np.sin(robot_yaw), a*np.cos(robot_yaw)]])

    Xtil = np.array([desired_x - robot_x, desired_y - robot_y]) 

    desired_velocity = np.array([desired_linear_x, desired_linear_y])

    reference_velocity = np.dot(np.linalg.inv(K), (desired_velocity.T + np.dot(Kp, Xtil.T)))

    inv_solver = np.array([[13.1579, -2.5921], [13.1579, 2.5921]])

    wheel_ref = np.dot(inv_solver, reference_velocity)

    right_wheel = wheel_ref[0]
    left_wheel = wheel_ref[1]

    right_wheel_limit, left_wheel_limit = limits

    if np.abs(right_wheel) > right_wheel_limit:
        right_wheel = np.sign(right_wheel)*right_wheel_limit

    if np.abs(left_wheel) > left_wheel_limit:
        left_wheel = np.sign(left_wheel)*left_wheel_limit

    return right_wheel, left_wheel

# %% Envio de mensagens para o robô
# pub = rospublisher('/md49_driverCommand','sensor_msgs/ChannelFloat32');
# msg = rosmessage(pub);

# msg.Values = P.pSC.wheel_ref;                            
# send(pub,msg);      

# %% Bloco para baixo nível SOLVERBOT
# % raio das rodas r = 0.076;
# % distância dos eixos d = 0.394;
# % [w_direita; w_esquerda] = inv([r/2 r/2; -r/d r/d])*[u; w]
# P.pSC.wheel_ref = [13.1579 -2.5921; 13.1579 2.5921]*P.pSC.U_real;
# wheelVmax = 24;
# P.pSC.wheel_ref = min(max(P.pSC.wheel_ref,-wheelVmax),wheelVmax);
# % Fim do bloco baixo nível SOLVERBOT