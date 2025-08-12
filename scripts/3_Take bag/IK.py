import math

def inverse_kinematics(x, y):
    L = 105.0  # Length of each link
    total_length = 2 * L  # Maximum reach (210 units)
    R = math.sqrt(x*x + y*y)  # Distance from base to end-effector
    
    # Check if point is reachable
    if R > total_length + 30:
        return []  # Unreachable position
    
    # Calculate cos(θ₂) using law of cosines
    cos_theta2 = (x*x + y*y) / (2 * L*L) - 1
    
    # Handle numerical edge cases
    cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
    
    # Calculate possible sin(θ₂) values (elbow up/down)
    sin_theta2 = math.sqrt(1.0 - cos_theta2**2)
    sin_options = [sin_theta2, -sin_theta2] if abs(sin_theta2) > 1e-10 else [0.0]
    
    solutions = []
    for sin_val in sin_options:
        # Calculate θ₂ in radians
        theta2 = math.atan2(sin_val, cos_theta2)
        
        # Calculate direction to end-effector
        base_angle = math.atan2(y, x) if (abs(x) > 1e-10 or abs(y) > 1e-10) else 0.0
        
        # Calculate θ₁
        theta1 = base_angle - theta2/2
        
        # NEW CONSTRAINT: θ₁ + θ₂ + θ₃ = 0
        theta3 = -theta1 - theta2  # Since sum must be 0
        
        # Convert to degrees
        t1_deg = math.degrees(theta1)
        t2_deg = math.degrees(theta2)
        t3_deg = math.degrees(theta3)
        
        solutions.append((t1_deg, t2_deg, t3_deg))
    
    # Filter solutions based on realistic joint constraints
    valid_solutions = []
    for t1, t2, t3 in solutions:
        # Joint limit constraints (customize for your hardware)
        if (-180 <= t1 <= 180 and 
            -150 <= t2 <= 150 and 
            -150 <= t3 <= 150 and 
            abs(t1 + t2 + t3) < 1e-5):  # Verify sum ≈ 0
            
            # Additional mechanical feasibility check
            # Avoid configurations where joints are folded beyond physical limits
            if not (abs(t2) > 170 or abs(t3) > 170):
                valid_solutions.append((t1, t2, t3))
    
    # Return all physically feasible solutions
    return valid_solutions

# sol = inverse_kinematics(100,150)
# print(sol)