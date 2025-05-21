import rclpy
import asyncio
import threading
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException
from .rmv import RmvApp


def spin_executor(node):
    print("Spin thread starting...")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Spin interrupted.")
    finally:
        if rclpy.ok():
            print("Destroying node from spin thread...")
            node.destroy_node()
    print("Spin thread exiting.")


async def main_async(node):
    app = RmvApp(node)
    try:
        await app.async_run()
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught in main_async")
    finally:
        print("Stopping app...")
        app.stop()


def main():
    rclpy.init()
    node = rclpy.create_node("rmv_app")

    spin_thread = threading.Thread(target=spin_executor, args=(node,), daemon=True)
    spin_thread.start()

    try:
        asyncio.run(main_async(node))
    except KeyboardInterrupt:
        print("Main loop interrupted.")
    finally:
        if rclpy.ok():
            print("Calling rclpy.shutdown()...")
            rclpy.shutdown()
        if spin_thread.is_alive():
            spin_thread.join()


if __name__ == "__main__":
    main()
