from kivy.uix.boxlayout import BoxLayout
from kivy.animation import Animation
import numpy as np
from .main_content import MainContent


from kivy.lang import Builder
import os
from pathlib import Path

dir_path = Path(__file__).resolve().parent
Builder.load_file(os.path.join(dir_path, "buttons.kv"))
Builder.load_file(os.path.join(dir_path, "panel.kv"))
Builder.load_file(os.path.join(dir_path, "main_window.kv"))


class MainWindow(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.params_visible = False

    def toggle_params(self):
        panel = self.ids.params_panel
        if self.params_visible:
            anim = Animation(size_hint_x=0, opacity=0, duration=0.1)
            panel.disabled = True
            anim.bind(on_complete=lambda *args: setattr(panel, "disabled", True))
        else:
            panel.disabled = False
            anim = Animation(size_hint_x=0.3, opacity=1, duration=0.1)
        anim.start(panel)
        self.params_visible = not self.params_visible

    def updateImage(self, image: np.ndarray):
        self.ids.main_content.updateImage(image)
