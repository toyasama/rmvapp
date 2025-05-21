from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from .uix.main_window import MainWindow
from .rmv_manager import RmvManager
from kivy.core.window import Window
from kivy.clock import Clock
import rclpy
from matplotlib.image import imread
import time
from pathlib import Path
from kivy.core.image import Image as CoreImage

Window.minimum_width, Window.minimum_height = 1200, 800


class RmvApp(App):

    __MAX_DELAY_FOR_LOST_IMAGE = 3
    __IMAGE_REFRESH_PERIOD = 1 / 10

    def __init__(self, node, **kwargs):
        super().__init__(**kwargs)
        self.rmv_manager = RmvManager(node)
        self.main_window = None
        dir_path = Path(__file__).resolve().parent
        path = dir_path / "images" / "no_image.png"
        self._no_image_texture = CoreImage(str(path)).texture
        self.last_received_image_time = None
        self.setUixParams()

    def setUixParams(self):
        self.title_label_hint_x_size = 0.7
        self.other_hint_x_size = 0.3

    def build(self):
        self.main_window = MainWindow()
        Clock.schedule_interval(self.updateUi, RmvApp.__IMAGE_REFRESH_PERIOD)
        return self.main_window

    def updateUi(self, dt):
        if self.main_window:
            image = self.rmv_manager.get_image()
            if image is not None:
                self.main_window.updateImage(image)
                self.last_received_image_time = time.time()
                return
            if (
                self.last_received_image_time is not None
                and time.time() - self.last_received_image_time
                < RmvApp.__MAX_DELAY_FOR_LOST_IMAGE
            ):
                return
            self.main_window.setNoImage(self._no_image_texture)

    def on_stop(self):
        print("Kivy is closing, cleaning up...")
