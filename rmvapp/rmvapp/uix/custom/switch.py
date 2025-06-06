from kivy.uix.widget import Widget
from kivy.properties import BooleanProperty, NumericProperty
from kivy.animation import Animation
from kivy.graphics import Color, RoundedRectangle, Ellipse
from pathlib import Path
from kivy.lang import Builder
from os import path

dir_path = Path(__file__).resolve().parent
Builder.load_file(path.join(dir_path, "switch.kv"))


class CustomSwitch(Widget):
    active = BooleanProperty(False)
    knob_pos = NumericProperty(0)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.bind(
            pos=self.update_canvas,
            size=self.update_canvas,
            active=self.animate_knob,
            knob_pos=self.update_canvas,
        )
        self.update_canvas()

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            self.active = not self.active
            return True
        return super().on_touch_down(touch)

    def animate_knob(self, *args):
        end_pos = self.width - self.height if self.active else 0
        anim = Animation(knob_pos=end_pos, duration=0.2, t="out_back")
        anim.start(self)

    def update_canvas(self, *args):
        self.canvas.clear()
        with self.canvas:
            Color(*(0.3, 0.7, 1, 1) if self.active else (0.6, 0.6, 0.6, 1))
            RoundedRectangle(pos=self.pos, size=self.size, radius=[self.height / 2])

            Color(1, 1, 1, 1)
            Ellipse(
                size=(self.height - 6, self.height - 6),
                pos=(self.x + self.knob_pos + 3, self.y + 3),
            )
