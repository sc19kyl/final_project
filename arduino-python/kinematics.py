import math

def calculate_wheel_velocities(vx, vy, omega, wheelbase):
    # Calculate linear velocity component of each wheel
    v1 = vy - vx + (omega * wheelbase)
    v2 = vy + vx - (omega * wheelbase)
    v3 = vy - vx - (omega * wheelbase)
    v4 = vy + vx + (omega * wheelbase)
    
    return v1, v2, v3, v4

# Example usage:
# Assume a robot with a linear velocity of 0.2 m/s in the x direction,
# 0.1 m/s in the y direction, and an angular velocity of 0.5 rad/s (for rotation).
# Also, let's assume the wheelbase (W) between wheels 1 and 3 (or 2 and 4) is 0.3 meters.

vx = 50  # Linear velocity in x direction (m/s)
vy = 10  # Linear velocity in y direction (m/s)
omega = 10  # Angular velocity (rad/s)
wheelbase = 0.3  # Wheelbase (meters)

# Calculate linear velocity of each wheel
v1, v2, v3, v4 = calculate_wheel_velocities(vx, vy, omega, wheelbase)

print("Linear velocity of wheel 1:", v1, "m/s")
print("Linear velocity of wheel 2:", v2, "m/s")
print("Linear velocity of wheel 3:", v3, "m/s")
print("Linear velocity of wheel 4:", v4, "m/s")