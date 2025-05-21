from kivy.uix.boxlayout import BoxLayout
from kivy.animation import Animation
from kivy.lang import Builder
from .custom.input import IntegerInput, FloatInput, LetterInput
from .custom.label import CustomLabel, GroupParameter
from .custom.switch import CustomSwitch
import os
from pathlib import Path

dir_path = Path(__file__).resolve().parent
Builder.load_file(os.path.join(dir_path, "params_dock.kv"))


class ParamsDock(BoxLayout):
    def show(self):
        self.disabled = False
        self.width = self.parent.width * 0.3 if self.parent else 300  # fallback
        Animation(size_hint_x=0.3, opacity=1, duration=0.2).start(self)

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
