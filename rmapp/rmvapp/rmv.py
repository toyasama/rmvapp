from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from .uix.main_window import MainWindow
from .rmv_manager import RmvManager
from kivy.core.window import Window
from kivy.clock import Clock
import rclpy

Window.minimum_width, Window.minimum_height = 1200, 800


class RmvApp(App):
    def __init__(self, node, **kwargs):
        super().__init__(**kwargs)
        self.rmv_manager = RmvManager(node)
        self.main_window = None

    def build(self):
        self.main_window = MainWindow()
        Clock.schedule_interval(self.updateUi, 1 / 10)
        return self.main_window

    def updateUi(self, dt):
        if self.main_window:
            image = self.rmv_manager.get_image()
            if image is not None:
                # print("Updating image in UI", image)
                self.main_window.updateImage(image)


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("rmv_app")
    RmvApp(node).run()
