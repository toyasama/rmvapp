from kivy.uix.label import Label
from pathlib import Path
import re
from kivy.lang import Builder
from os import path

dir_path = Path(__file__).resolve().parent
Builder.load_file(path.join(dir_path, "label.kv"))


class CustomLabel(Label):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)


class GroupParameter(Label):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
