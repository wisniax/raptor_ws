#!/usr/bin/env python3

import sys
import rclpy
import time
from rex_interfaces.msg import SamplerControl
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


TOPIC = "/MQTT/SamplerControl"


def print_usage():
    print("""
Sampler CLI

Mission commands:
    start
    stop
    abort
    restart
    restart_deep
    restart_surface
    calibrate

Manual commands:
    platform <value>
    drill <value>
    container <value>

Rotor commands:
    drill_rotor <value>
    vacuum <value>
    brush <value>

Vacuum:
    open_vacuum <0|1>

Examples:
    ./sampler_cli.py start
    ./sampler_cli.py stop
    ./sampler_cli.py platform 0.5
    ./sampler_cli.py drill -0.2
    ./sampler_cli.py container 30
    ./sampler_cli.py drill_rotor 3.0
    ./sampler_cli.py vacuum 5.0
    ./sampler_cli.py brush 2.0
    ./sampler_cli.py open_vacuum 1
""")


def main():

    if len(sys.argv) < 2:
        print_usage()
        return 1

    command = sys.argv[1].lower()

    # ---------------------------------------------------------
    # Mission command definitions
    # ---------------------------------------------------------

    mission_commands = {
        "start": SamplerControl.START,
        "stop": SamplerControl.STOP,
        "abort": SamplerControl.ABORT,
        "restart": SamplerControl.RESTART,
        "calibrate": SamplerControl.CALIBRATE,
    }

    # ---------------------------------------------------------
    # Initialize ROS
    # ---------------------------------------------------------

    rclpy.init()

    node = rclpy.create_node("sampler_cli")
    qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    )

    publisher = node.create_publisher(
        SamplerControl,
        TOPIC,
        qos
    )

    msg = SamplerControl()

    # ---------------------------------------------------------
    # Mission command
    # ---------------------------------------------------------

    if command in mission_commands:

        if len(sys.argv) != 2:
            print("Mission commands cannot be combined with other commands.")
            rclpy.shutdown()
            return 1

        msg.mission_cmd = mission_commands[command]

    # ---------------------------------------------------------
    # Manual commands
    # ---------------------------------------------------------

    else:

        args = sys.argv[1:]

        if len(args) % 2 != 0:
            print("Commands must be provided as <command> <value> pairs.")
            print_usage()
            rclpy.shutdown()
            return 1

        for i in range(0, len(args), 2):

            command = args[i].lower()
            value = args[i + 1]

            try:
                value = float(value)
            except ValueError:
                print(f"Invalid value: {value}")
                rclpy.shutdown()
                return 1

            if command == "platform":
                msg.platform_movement = value

            elif command == "drill":
                msg.drill_movement = value

            elif command == "container":
                msg.container_degrees = value

            elif command == "drill_rotor":
                msg.drill_action = value

            elif command == "vacuum":
                msg.vacuum_suction = value

            elif command == "brush":
                msg.brush_rotation = value

            elif command == "open_vacuum":

                if value not in (0.0, 1.0):
                    print("open_vacuum must be 0 or 1")
                    rclpy.shutdown()
                    return 1

                msg.open_vacuum = bool(value)

            else:
                print(f"Unknown command: {command}")
                print_usage()
                rclpy.shutdown()
                return 1

    # # ---------------------------------------------------------
    # # Drill rotor
    # # ---------------------------------------------------------

    # elif command == "drill_rotor":

    #     if len(sys.argv) != 3:
    #         print("Usage: drill_rotor <value>")
    #         rclpy.shutdown()
    #         return 1

    #     msg.drill_action = float(sys.argv[2])

    # # ---------------------------------------------------------
    # # Vacuum
    # # ---------------------------------------------------------

    # elif command == "vacuum":

    #     if len(sys.argv) != 3:
    #         print("Usage: vacuum <value>")
    #         rclpy.shutdown()
    #         return 1

    #     msg.vacuum_suction = float(sys.argv[2])

    # # ---------------------------------------------------------
    # # Brush
    # # ---------------------------------------------------------

    # elif command == "brush":

    #     if len(sys.argv) != 3:
    #         print("Usage: brush <value>")
    #         rclpy.shutdown()
    #         return 1

    #     msg.brush_rotation = float(sys.argv[2])

    # # ---------------------------------------------------------
    # # Vacuum valve
    # # ---------------------------------------------------------

    # elif command == "open_vacuum":

    #     if len(sys.argv) != 3:
    #         print("Usage: open_vacuum <0|1>")
    #         rclpy.shutdown()
    #         return 1

    #     value = int(sys.argv[2])

    #     if value not in (0, 1):
    #         print("open_vacuum must be 0 or 1")
    #         rclpy.shutdown()
    #         return 1

    #     msg.open_vacuum = bool(value)

    # ---------------------------------------------------------
    # Unknown command
    # ---------------------------------------------------------

    # else:

    #     print(f"Unknown command: {command}")
    #     print_usage()

    #     rclpy.shutdown()
    #     return 1

    # ---------------------------------------------------------
    # Publish
    # ---------------------------------------------------------

    # Give ROS a moment to establish the publisher connection.
    for _ in range(20):
        if publisher.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Subscribers: {publisher.get_subscription_count()}")
    # for _ in range():
    publisher.publish(msg)
    
    rclpy.spin_once(node, timeout_sec=0.1)


    print("Command sent:")
    print(msg)

    # Give DDS time to transmit
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())