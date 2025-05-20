import rclpy.executors
from .rmv import RmvApp
from rclpy.executors import MultiThreadedExecutor
import rclpy
import asyncio
import threading


def spin_executor(node):
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        node.destroy_node()


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("rmv_app")
    app = RmvApp(node)
    spin_thread = None
    try:
        spin_thread = threading.Thread(target=spin_executor, args=(node,), daemon=True)
        loop = asyncio.get_event_loop()
        loop.create_task(app.async_run())
        loop.run_forever()

        spin_thread.start()

    except KeyboardInterrupt:
        print("Shutting down...")
        pass
        if spin_thread:
            spin_thread.join()
    except Exception as e:
        print(f"An error occurred: {e}")
        pass
        # if spin_thread:
        #     spin_thread.join()
    finally:
        app.stop()
