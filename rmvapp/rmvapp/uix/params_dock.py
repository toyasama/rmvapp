from kivy.uix.boxlayout import BoxLayout
from kivy.animation import Animation
from kivy.lang import Builder
from .custom.input import IntegerInput, FloatInput, LetterInput
from .custom.label import CustomLabel, GroupParameter
from .custom.switch import CustomSwitch
import os
from pathlib import Path
from kivy.app import App

dir_path = Path(__file__).resolve().parent
Builder.load_file(os.path.join(dir_path, "params_dock.kv"))


class ParamsDock(BoxLayout):
    def show(self):
        self.disabled = False
        self.width = self.parent.width * 0.3 if self.parent else 300  # fallback
        Animation(size_hint_x=0.3, opacity=1, duration=0.2).start(self)
        self.display_params()

    def hide(self, immediate=False):
        if immediate:
            self.size_hint_x = 0
            self.width = 0
            self.opacity = 0
            self.disabled = True
        else:
            anim = Animation(size_hint_x=0, opacity=0, duration=0.2)
            anim.bind(on_complete=self._finalize_hide)
            anim.start(self)

    def _finalize_hide(self, *args):
        self.width = 0
        self.disabled = True

    def display_params(self):
        params = App.get_running_app().get_params()

        self.ids.main_frame_input.text = params["frames"]["main_frame"]
        self.ids.display_axes_switch.active = params["frames"]["show_axes"]
        self.ids.display_connections_switch.active = params["frames"][
            "show_connections"
        ]
        self.ids.display_names_switch.active = params["frames"]["show_frame_names"]

        self.ids.camera_position_x.text = str(
            params["visualizations"]["camera"]["position"]["x"]
        )
        self.ids.camera_position_y.text = str(
            params["visualizations"]["camera"]["position"]["y"]
        )
        self.ids.camera_position_z.text = str(
            params["visualizations"]["camera"]["position"]["z"]
        )
        self.ids.camera_rotation_angle.text = str(
            params["visualizations"]["camera"]["position"]["theta"]
        )
        self.ids.camera_fov.text = str(params["visualizations"]["camera"]["fov_deg"])
