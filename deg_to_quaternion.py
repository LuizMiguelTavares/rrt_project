import math
import transforms3d.euler as euler
import transforms3d.quaternions as quaternions

def degrees_to_quaternion(deg_angle):
    # Convert degrees to radians
    rad_angle = math.radians(deg_angle)
    
    # Convert the angle to quaternion (assuming rotation around the Z-axis)
    quaternion = euler.euler2quat(0, 0, rad_angle)  # Roll, Pitch, Yaw (in radians)
    
    return quaternion

def quaternion_to_degrees(quaternion):
    # Extract the euler angles (roll, pitch, yaw) from quaternion
    euler_angles = euler.quat2euler(quaternion)
    
    # Yaw is the Z-axis angle, convert it from radians to degrees
    yaw_angle_deg = math.degrees(euler_angles[2])
    
    return yaw_angle_deg

if __name__ == "__main__":
    # Example usage:
    # Convert 45 degrees to quaternion
    angle_in_deg = - 2.85
    quaternion = degrees_to_quaternion(angle_in_deg)
    print(f"Quaternion from {angle_in_deg} degrees: {quaternion[1]} {quaternion[2]} {quaternion[3]} {quaternion[0]}")
    
    # Convert back from quaternion to degrees
    angle_in_deg_converted = quaternion_to_degrees(quaternion)
    print(f"Angle from quaternion: {angle_in_deg_converted} degrees")