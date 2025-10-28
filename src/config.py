# Aircraft Configuration Parameters

class AircraftConfig:
    # Environment Constants
    RHO = 1.225         # Air density at sea level (kg/m^3)
    GRAVITY = 9.81      # Standard gravity (m/s^2)

    # Aircraft Physical Parameters (Cessna 172-like)
    MASS = 1000.0       # Aircraft mass (kg)
    WING_AREA = 16.2    # Wing reference area (m^2)
    REF_LENGTH = 5.0    # Reference chord length for moments (m)

    # Aerodynamic Coefficients
    CL_MAX = 1.6        # Maximum lift coefficient
    CD_MIN = 0.03       # Minimum drag coefficient (parasitic drag)
    STALL_ANGLE_DEG = 15.0 # Angle of Attack (AoA) at which stall occurs (degrees)

    # Engine/Thrust Parameters
    ENGINE_POWER = 10000.0 # Maximum Thrust Force (Newtons)

    # Control Surface Effectiveness (Placeholder for simplified model)
    ELEVATOR_EFFECTIVENESS = 0.1 # Pitch moment coefficient per unit elevator input
    AILERON_EFFECTIVENESS = 0.05 # Roll moment coefficient per unit aileron input
    RUDDER_EFFECTIVENESS = 0.03  # Yaw moment coefficient per unit rudder input
