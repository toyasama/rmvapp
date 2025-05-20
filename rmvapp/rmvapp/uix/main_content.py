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
        if image.ndim == 3 and image.shape[2] == 3:
            tex = Texture.create(size=(image.shape[1], image.shape[0]), colorfmt="rgb")
            tex.blit_buffer(image.tobytes(), colorfmt="rgb", bufferfmt="ubyte")
            tex.flip_vertical()
            self.ids.video_feed.texture = tex

    def setNoImage(self, texture):
        self.ids.video_feed.texture = texture
