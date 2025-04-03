import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from rcl_interfaces.msg import SetParametersResult

class QubeController(Node):
    def __init__(self):
        super().__init__('qube_controller')
        
        # deklarere pid parametere
        self.declare_parameter('kp', 1.0)  # Proportional gain
        self.declare_parameter('ki', 0.0)  # Integral gain
        self.declare_parameter('kd', 0.0)  # Derivative gain
        self.declare_parameter('setpoint', 0.0)  # Desired position
        self.declare_parameter('joint_name', 'motor_joint')  # Joint to control
        
        # Skaffe parameter verdier
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.setpoint = self.get_parameter('setpoint').value
        self.joint_name = self.get_parameter('joint_name').value
        
        # initialisere pid
        self.integral = 0.0  # Accumulated error for integral term
        self.previous_error = 0.0  # Error from the previous step
        self.previous_time = self.get_clock().now()  # Last update time
        
        # abonere til joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # publisere til /velocity_controller/commands
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10)
        
        # Sette opp et parameter callback for å endre på parametere
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        # Oppdatere parameterene
        for param in params:
            if param.name == 'kp':
                self.kp = param.value
            elif param.name == 'ki':
                self.ki = param.value
            elif param.name == 'kd':
                self.kd = param.value
            elif param.name == 'setpoint':
                self.setpoint = param.value
        return SetParametersResult(successful=True)
    
    def joint_state_callback(self, msg):
        # fin motor_joint indexen
        try:
            index = msg.name.index(self.joint_name)
        except ValueError:
            self.get_logger().error(f"Joint {self.joint_name} not found in /joint_states")
            return
        
        # Få posisjon
        position = msg.position[index]
        
        # kalkuler delta tid
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9  # i sekkund
        dt = max(0.01,dt) # slik at det aldri blir nulldivisjon
        
        # finn avvik
        error = self.setpoint - position
        
        # oppdater i
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        
        # regn ut output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, 100.0), -100.0) #begrens output til grensen til modellen
        
        # publiser pådrag
        command_msg = Float64MultiArray()
        command_msg.data = [output]
        self.publisher.publish(command_msg)
        
        # lagre error og tid for neste iterasjon
        self.previous_error = error
        self.previous_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = QubeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()