import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.declare_parameter("my_parameter")
        # self.declare_parameter("LED_scale")
        
        # self.declare_parameter("channel_order.duckiebot")
        # self.declare_parameter("channel_order.traffic_light")

        # self.declare_parameter("LED_protocol.colors.switchedoff")
        # self.declare_parameter("LED_protocol.colors.white")
        # self.declare_parameter("LED_protocol.colors.green")
        # self.declare_parameter("LED_protocol.colors.red")
        # self.declare_parameter("LED_protocol.colors.blue")
        # self.declare_parameter("LED_protocol.colors.yellow")
        # self.declare_parameter("LED_protocol.colors.purple")
        # self.declare_parameter("LED_protocol.colors.cyan")
        # self.declare_parameter("LED_protocol.colors.pink")

        # self.declare_parameter("LED_protocol.frequencies.f1")
        # self.declare_parameter("LED_protocol.frequencies.f2")
        # self.declare_parameter("LED_protocol.frequencies.f3")
        # self.declare_parameter("LED_protocol.frequencies.f4")
        # self.declare_parameter("LED_protocol.frequencies.f5")

        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_PRIORITY.color_mask")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_PRIORITY.color_list")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_PRIORITY.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_PRIORITY.frequency")

        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_SACRIFICE_FOR_PRIORITY.color_mask")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_SACRIFICE_FOR_PRIORITY.color_list")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_SACRIFICE_FOR_PRIORITY.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_SACRIFICE_FOR_PRIORITY.frequency")

        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_A.color_mask")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_A.color_list")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_A.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_A.frequency")

        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_GREEN.color_mask")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_GREEN.color_list")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_GREEN.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.CAR_SIGNAL_GREEN.frequency")

        # self.declare_parameter("LED_protocol.signals.CAR_DRIVING.color_mask")
        # self.declare_parameter("LED_protocol.signals.CAR_DRIVING.color_list")
        # self.declare_parameter("LED_protocol.signals.CAR_DRIVING.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.CAR_DRIVING.frequency")

        # self.declare_parameter("LED_protocol.signals.WHITE.color_mask")
        # self.declare_parameter("LED_protocol.signals.WHITE.color_list")
        # self.declare_parameter("LED_protocol.signals.WHITE.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.WHITE.frequency")

        # self.declare_parameter("LED_protocol.signals.RED.color_mask")
        # self.declare_parameter("LED_protocol.signals.RED.color_list")
        # self.declare_parameter("LED_protocol.signals.RED.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.RED.frequency")

        # self.declare_parameter("LED_protocol.signals.GREEN.color_mask")
        # self.declare_parameter("LED_protocol.signals.GREEN.color_list")
        # self.declare_parameter("LED_protocol.signals.GREEN.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.GREEN.frequency")

        # self.declare_parameter("LED_protocol.signals.BLUE.color_mask")
        # self.declare_parameter("LED_protocol.signals.BLUE.color_list")
        # self.declare_parameter("LED_protocol.signals.BLUE.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.BLUE.frequency")

        # self.declare_parameter("LED_protocol.signals.LIGHT_OFF.color_mask")
        # self.declare_parameter("LED_protocol.signals.LIGHT_OFF.color_list")
        # self.declare_parameter("LED_protocol.signals.LIGHT_OFF.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.LIGHT_OFF.frequency")

        # self.declare_parameter("LED_protocol.signals.light_off.color_mask")
        # self.declare_parameter("LED_protocol.signals.light_off.color_list")
        # self.declare_parameter("LED_protocol.signals.light_off.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.light_off.frequency")

        # self.declare_parameter("LED_protocol.signals.OBSTACLE_ALERT.color_mask")
        # self.declare_parameter("LED_protocol.signals.OBSTACLE_ALERT.color_list")
        # self.declare_parameter("LED_protocol.signals.OBSTACLE_ALERT.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.OBSTACLE_ALERT.frequency")

        # self.declare_parameter("LED_protocol.signals.OBSTACLE_STOPPED.color_mask")
        # self.declare_parameter("LED_protocol.signals.OBSTACLE_STOPPED.color_list")
        # self.declare_parameter("LED_protocol.signals.OBSTACLE_STOPPED.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.OBSTACLE_STOPPED.frequency")

        # self.declare_parameter("LED_protocol.signals.POPO.color_mask")
        # self.declare_parameter("LED_protocol.signals.POPO.color_list")
        # self.declare_parameter("LED_protocol.signals.POPO.frequency_mask")
        # self.declare_parameter("LED_protocol.signals.POPO.frequency")


    def timer_callback(self):
        # First get the value parameter "my_parameter" and get its string value
        my_param = self.get_parameter("my_parameter").get_parameter_value().string_value

        # Send back a hello with the name
        self.get_logger().info('Hello %s!' % my_param)

        # Then set the parameter "my_parameter" back to string value "world"
        my_new_param = rclpy.parameter.Parameter(
            "my_parameter",
            rclpy.Parameter.Type.STRING,
            "world"
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
