from .rmv import RmvApp
from rclpy.executors import MultiThreadedExecutor
import rclpy
import asyncio
import threading


def spin_executor(executor):
    executor.spin()


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("rmv_app")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        loop = asyncio.get_event_loop()
        loop.create_task(RmvApp(node).async_run())
        loop.run_forever()

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
