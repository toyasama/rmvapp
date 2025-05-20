from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
import numpy as np
from .main_content import MainContent
from .lateral_bar import LateralBar
from .params_dock import ParamsDock

from kivy.lang import Builder
import os
from pathlib import Path

dir_path = Path(__file__).resolve().parent
Builder.load_file(os.path.join(dir_path, "buttons.kv"))
Builder.load_file(os.path.join(dir_path, "main_window.kv"))


class MainWindow(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.params_visible = False
        self.ids.params_dock.hide(immediate=True)

    def toggle_params(self):
        panel = self.ids.params_dock
        if self.params_visible:
            panel.hide()
        else:
            panel.show()
        self.params_visible = not self.params_visible

    def updateImage(self, image: np.ndarray):
        self.ids.main_content.updateImage(image)

    def setNoImage(self, texture):
        self.ids.main_content.setNoImage(texture)
