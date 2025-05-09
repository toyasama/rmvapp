from kivy.uix.boxlayout import BoxLayout
from kivy.graphics.texture import Texture
import cv2
from pathlib import Path
import numpy as np


from kivy.lang import Builder
import os

dir_path = Path(__file__).resolve().parent
Builder.load_file(os.path.join(dir_path, "main_content.kv"))


class MainContent(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def updateImage(self, image: np.ndarray):
        if image is None:
            return
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        frame = np.flip(frame, 0)
        height, width, _ = frame.shape
        texture = Texture.create(size=(width, height))
        texture.blit_buffer(frame.tobytes(), colorfmt="rgb", bufferfmt="ubyte")
        self.ids.video_feed.texture = texture
