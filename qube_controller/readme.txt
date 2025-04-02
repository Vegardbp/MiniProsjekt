qube_controller folderen har en python fil med en node, som regulerer en qube. Parameterene er:
self.declare_parameter('kp', 1.0)  # Proportional gain
self.declare_parameter('ki', 0.0)  # Integral gain
self.declare_parameter('kd', 0.0)  # Derivative gain
self.declare_parameter('setpoint', 0.0)  # Desired position
self.declare_parameter('joint_name', 'motor_joint')  # Joint to control
som kan endres i kommando linjen.
Denne noden blir startet av launch filen i qube_bringup