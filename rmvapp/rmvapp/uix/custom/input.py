from kivy.uix.textinput import TextInput
from pathlib import Path
import re
from kivy.lang import Builder
from os import path

dir_path = Path(__file__).resolve().parent
Builder.load_file(path.join(dir_path, "input.kv"))


class CustomInput(TextInput):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)


class FloatInput(CustomInput):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def insert_text(self, substring, from_undo=False):
        pattern = r"^-?\d*\.?\d*$"
        new_text = (
            self.text[: self.cursor_index()]
            + substring
            + self.text[self.cursor_index() :]
        )

        if re.match(pattern, new_text):
            super().insert_text(substring, from_undo=from_undo)


class IntegerInput(CustomInput):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def insert_text(self, substring, from_undo=False):
        if substring.isdigit():
            super().insert_text(substring, from_undo=from_undo)


class LetterInput(CustomInput):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def insert_text(self, substring, from_undo=False):
        if substring.isalpha():
            super().insert_text(substring, from_undo=from_undo)
