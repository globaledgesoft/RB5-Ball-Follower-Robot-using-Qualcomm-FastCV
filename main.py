import rclpy

from src.ball_follower import BallFollow


def main(args=None):
    rclpy.init(args=args)
    avoider = BallFollow()
    rclpy.spin(avoider)

    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
