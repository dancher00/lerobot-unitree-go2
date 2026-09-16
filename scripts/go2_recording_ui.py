"""Native LeRobot n/r/q terminal controls plus a release-aware live video window."""
import logging
import os
import threading

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame
from lerobot.teleoperators import Teleoperator
from lerobot.utils.keyboard_input import TerminalKeyListener, apply_recording_control

from lerobot_robot_unitree_go2 import UnitreeGo2KeyboardTeleopConfig


class RecordingUI(Teleoperator):
    config_class = UnitreeGo2KeyboardTeleopConfig
    name = "go2_network_keyboard"

    def __init__(self, cfg, readonly=False):
        super().__init__(UnitreeGo2KeyboardTeleopConfig())
        self.speeds = [cfg["safety"]["max_" + axis] for axis in ("vx", "vy", "wz")]
        self.readonly = readonly
        self.events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
        self.ready = threading.Event()
        self.pressed = set()
        self.keyboard_focus = False
        self.locked = False
        self.phase = "PREVIEW | Enter to start; q to quit"
        self.screen = None
        self.listener = TerminalKeyListener(self.key)
        self.phase_stop = False
        self._last_input_status = None
        self.last_action = dict.fromkeys(self.action_features, 0.)

    @property
    def action_features(self):
        return dict.fromkeys(("base.vx", "base.vy", "base.wz"), float)

    @property
    def feedback_features(self):
        return {}

    @property
    def is_connected(self):
        return self.screen is not None

    @property
    def is_calibrated(self):
        return True

    def configure(self):
        pass

    def calibrate(self):
        pass

    def connect(self, calibrate=True):
        if self.screen is None:
            pygame.init()
            self.screen = pygame.display.set_mode((640, 576))
            pygame.display.set_caption("Go2 | RealSense front | n next, r retry, q finish")
            self.font = pygame.font.Font(None, 22)
            self.keyboard_focus = bool(pygame.key.get_focused())
            self.listener.start()

    def key(self, name):
        key = name.lower()
        if key in ("enter", "return", "keypad enter"):
            self.ready.set()
        mapping = {"n": "right", "right": "right", "r": "left", "left": "left", "q": "esc", "esc": "esc"}
        if key in mapping:
            self.phase_stop = True
            apply_recording_control(mapping[key], self.events)

    def pump(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.key("q")
            elif event.type == pygame.WINDOWFOCUSGAINED:
                self.keyboard_focus = True
            elif event.type == pygame.WINDOWFOCUSLOST:
                self.keyboard_focus = False
                self.pressed.clear()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # A click delivered to this SDL window proves it has input focus.
                # pygame.key.get_focused() is unreliable under some Wayland compositors.
                self.keyboard_focus = True
            elif event.type == pygame.KEYDOWN:
                # Receiving a key event is stronger evidence than SDL's focus query.
                self.keyboard_focus = True
                self.key(pygame.key.name(event.key))
                if not getattr(event, "repeat", False):
                    self.pressed.add(event.key)
                if event.key in (pygame.K_SPACE, pygame.K_k, pygame.K_x):
                    self.pressed.clear()
                    self.pressed.add(event.key)
                if event.key == pygame.K_x:
                    self.locked = True
                if event.key == pygame.K_v:
                    self.locked = False
            elif event.type == pygame.KEYUP:
                self.pressed.discard(event.key)

    def display(self, observation):
        self.pump()
        frame = observation["front"]
        surface = pygame.image.frombuffer(frame.tobytes(), (640, 480), "RGB")
        self.screen.blit(surface, (0, 0))
        self.screen.fill((20, 20, 20), (0, 480, 640, 96))
        self.screen.blit(self.font.render(self.phase, True, (255, 255, 255)), (8, 486))
        help_text = "i/, move | j/l turn | Shift+j/l side | n=next r=retry q=quit"
        self.screen.blit(self.font.render(help_text, True, (210, 210, 210)), (8, 512))
        status = self.block_reason()
        label = {
            "no_focus": "TELEOP OFF: click video, then press movement key",
            "no_keys": "TELEOP READY: release and press i/, j/l to move",
            "locked": "TELEOP LOCKED: press v to unlock",
            "phase_stop": "TELEOP STOPPED: changing recording phase",
            "stop_key": "TELEOP STOPPED: stop key held",
            "readonly": "READ ONLY: no motion commands",
        }.get(status, "TELEOP ACTIVE: " + str(tuple(self.last_action.values())))
        self.screen.blit(self.font.render(label, True, (255, 200, 80) if status else (100, 255, 140)), (8, 544))
        pygame.display.flip()

    def reset_controls(self):
        self.pressed.clear()
        self.phase_stop = False

    def get_action(self):
        self.pump()
        zero = dict.fromkeys(self.action_features, 0.)
        reason = self.block_reason()
        keys = self.pressed
        shift = bool(keys.intersection((pygame.K_LSHIFT, pygame.K_RSHIFT)))
        vx = float(pygame.K_i in keys) - float(pygame.K_COMMA in keys)
        turn = float(pygame.K_j in keys) - float(pygame.K_l in keys)
        action = zero if reason else dict(zip(self.action_features, (vx * self.speeds[0], turn * self.speeds[1] if shift else 0.,
                                               0. if shift else turn * self.speeds[2]), strict=True))
        status = (self.phase, reason, tuple(action.values()))
        if status != self._last_input_status:
            logging.debug("Go2 teleop: phase=%s, input=%s, command=%s", self.phase, reason or "active", tuple(action.values()))
            self._last_input_status = status
        self.last_action = action
        return action

    def block_reason(self):
        if self.readonly:
            return "readonly"
        if self.locked:
            return "locked"
        if self.phase_stop:
            return "phase_stop"
        if self.pressed.intersection((pygame.K_SPACE, pygame.K_k)):
            return "stop_key"
        if not self.keyboard_focus:
            return "no_focus"
        if not self.pressed.intersection((pygame.K_i, pygame.K_COMMA, pygame.K_j, pygame.K_l)):
            return "no_keys"
        return None

    def send_feedback(self, feedback):
        pass

    def disconnect(self):
        self.pressed.clear()
        self.listener.stop()
        if self.screen:
            pygame.quit()
            self.screen = None
