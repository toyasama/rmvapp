from kivy.uix.boxlayout import BoxLayout

from kivy.lang import Builder
import os
from pathlib import Path

dir_path = Path(__file__).resolve().parent
Builder.load_file(os.path.join(dir_path, "lateral_bar.kv"))


class LateralBar(BoxLayout):
    pass
