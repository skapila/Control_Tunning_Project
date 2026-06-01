#!/usr/bin/env python3
"""
SM600Input — RC transmitter driver + built-in Keyboard Override GUI
====================================================================
Reads Roll / Pitch / Yaw / Throttle from the SM600 joystick axes.

Features NOT wired on SM600 hardware (mode switching, D-pad, takeoff,
land) are controlled from an embedded pygame GUI window that opens
automatically in a background thread — no separate script needed.

Keyboard map (GUI window must have focus):
  1-4   → Flight mode  (STABILIZE / ALT_HOLD / GPS_HOLD / GUIDED)
  5     → Takeoff  (pulse)
  6     → Land     (pulse)
  ↑↓←→  → D-Pad    (pulse)
  Q/Esc → Quit
"""

import os
import sys
import threading
import time

import pygame

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

from utils.logger import Logger

# ─────────────────────────────────────────────────────────────────────────────
#  SM600 CHANNEL MAP
# ─────────────────────────────────────────────────────────────────────────────
CHANNEL_MAP = {
    "roll":     3,
    "pitch":    0,
    "throttle": 1,
    "yaw":      4,
    "flip_btn": 0,
}

INVERT = {
    "throttle": False,
    "pitch":    True,
    "roll":     False,
    "yaw":      False,
}

# ─────────────────────────────────────────────────────────────────────────────
#  GUI COLOURS
# ─────────────────────────────────────────────────────────────────────────────
BG       = ( 18,  18,  24)
PANEL    = ( 30,  30,  40)
BORDER   = ( 60,  60,  80)
ACCENT   = ( 80, 140, 255)
GREEN    = ( 60, 200, 100)
RED      = (220,  70,  70)
YELLOW   = (240, 200,  50)
WHITE    = (230, 230, 240)
GREY     = (100, 100, 120)
BAR_FILL = ( 70, 130, 220)
BAR_EMPTY= ( 40,  40,  55)
HIGHLIGHT= ( 50,  80, 160)

# ─────────────────────────────────────────────────────────────────────────────
#  GUI LAYOUT
# ─────────────────────────────────────────────────────────────────────────────
WIN_W, WIN_H = 780, 500
FPS          = 20
MENU_X, MENU_Y, MENU_W, MENU_ROW_H   = 30, 70, 300, 38
STATE_X, STATE_Y, STATE_W, STATE_ROW_H = 370, 70, 380, 36
LOG_Y        = WIN_H - 48

MENU_ITEMS = [
    ("1", "STABILIZE",   "mode"),
    ("2", "ALT_HOLD",    "mode"),
    ("3", "GPS_HOLD",    "mode"),
    ("4", "GUIDED",      "mode"),
    ("5", "TAKEOFF",     "cmd"),
    ("6", "LAND",        "cmd"),
    ("↑", "D-Pad FRONT", "dpad"),
    ("↓", "D-Pad BACK",  "dpad"),
    ("←", "D-Pad LEFT",  "dpad"),
    ("→", "D-Pad RIGHT", "dpad"),
]


# ─────────────────────────────────────────────────────────────────────────────
#  SM600Input
# ─────────────────────────────────────────────────────────────────────────────

class SM600Input:
    """
    Unified SM600 RC driver + Keyboard Override GUI.

    The autopilot consumes this object exactly as it would pilot_input.py —
    the same public attributes and getters are present.  The GUI window opens
    automatically; no separate script or import is needed.
    """

    def __init__(self):
        # ── PWM state (SM600 axes) ────────────────────────────────────────────
        self.roll_pwm     = 1500
        self.pitch_pwm    = 1500
        self.yaw_pwm      = 1500
        self.throttle_pwm = 1500

        # ── Scaling constants ─────────────────────────────────────────────────
        self.max_angle    = 45
        self.max_yaw_rate = 100
        self.min_pwm      = 1000
        self.max_pwm      = 2000

        # ── Flight modes ──────────────────────────────────────────────────────
        self.gps_hold    = "GPS_HOLD"
        self.alt_hold    = "ALT_HOLD"
        self.stabilize   = "STABILIZE"
        self.guided      = "GUIDED"
        self.mode_switch = self.stabilize   # keyboard GUI writes here

        # ── Features NOT on SM600 — written by keyboard GUI ───────────────────
        self.dpad_up    = False
        self.dpad_down  = False
        self.dpad_left  = False
        self.dpad_right = False

        self.takeoff_pressed = False
        self.land_pressed    = False

        # ── SM600 flip button ─────────────────────────────────────────────────
        self.flip_button = False

        # ── Shared stop flag ──────────────────────────────────────────────────
        self.running = True

        # ── Start RC listener thread ──────────────────────────────────────────
        self._rc_thread = threading.Thread(
            target=self._listen_to_rc, daemon=True, name="SM600-RC")
        self._rc_thread.start()

        # ── Start GUI thread ──────────────────────────────────────────────────
        self._gui_thread = threading.Thread(
            target=self._run_gui, daemon=True, name="SM600-GUI")
        self._gui_thread.start()

    # ─────────────────────────────────────────────────────────────────────────
    #  Helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _normalize(self, pwm):
        return (pwm - 1500) / 500.0

    def _normalize_throttle(self, pwm):
        return (pwm - self.min_pwm) / (self.max_pwm - self.min_pwm)

    def _scale_joystick(self, value, min_out=1000, max_out=2000):
        return int((value + 1) / 2 * (max_out - min_out) + min_out)

    def _pulse(self, attr, duration=0.5):
        setattr(self, attr, True)
        time.sleep(duration)
        setattr(self, attr, False)

    # ─────────────────────────────────────────────────────────────────────────
    #  Public getters
    # ─────────────────────────────────────────────────────────────────────────

    def get_desired_roll_angle(self):
        return self._normalize(self.roll_pwm) * self.max_angle

    def get_desired_pitch_angle(self):
        return self._normalize(self.pitch_pwm) * self.max_angle

    def get_desired_yaw_rate(self):
        return self._normalize(self.yaw_pwm) * self.max_yaw_rate

    def get_throttle_pwm(self):
        return self.throttle_pwm

    def get_roll_pwm(self):
        return self.roll_pwm

    def get_pitch_pwm(self):
        return self.pitch_pwm

    def get_yaw_pwm(self):
        return self.yaw_pwm

    # ─────────────────────────────────────────────────────────────────────────
    #  RC listener thread
    # ─────────────────────────────────────────────────────────────────────────

    def _listen_to_rc(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            Logger.info("SM600: No joystick detected — axes will stay at 1500.")
            return

        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        Logger.info(f"SM600Input: Connected to → {joystick.get_name()}")

        while self.running:
            pygame.event.pump()

            roll_raw     = joystick.get_axis(CHANNEL_MAP["roll"])
            pitch_raw    = joystick.get_axis(CHANNEL_MAP["pitch"])
            throttle_raw = joystick.get_axis(CHANNEL_MAP["throttle"])
            yaw_raw      = joystick.get_axis(CHANNEL_MAP["yaw"])

            self.roll_pwm     = self._scale_joystick(roll_raw     * (-1 if INVERT["roll"]     else 1))
            self.pitch_pwm    = self._scale_joystick(pitch_raw    * (-1 if INVERT["pitch"]    else 1))
            self.throttle_pwm = self._scale_joystick(throttle_raw * (-1 if INVERT["throttle"] else 1))
            self.yaw_pwm      = self._scale_joystick(yaw_raw      * (-1 if INVERT["yaw"]      else 1))

            self.flip_button  = bool(joystick.get_button(CHANNEL_MAP["flip_btn"]))

            Logger.debug(
                f"SM600 PWM :- Roll:{self.roll_pwm}  Pitch:{self.pitch_pwm}  "
                f"Throttle:{self.throttle_pwm}  Yaw:{self.yaw_pwm}"
            )
            Logger.debug(
                f"SM600 Flip:{self.flip_button} | Mode:{self.mode_switch} | "
                f"Takeoff:{self.takeoff_pressed}  Land:{self.land_pressed}"
            )
            Logger.debug(
                f"D-Pad: UP={self.dpad_up}  DOWN={self.dpad_down}  "
                f"LEFT={self.dpad_left}  RIGHT={self.dpad_right}"
            )

            time.sleep(0.1)

    # ─────────────────────────────────────────────────────────────────────────
    #  GUI helpers
    # ─────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _draw_panel(surf, rect, radius=8):
        pygame.draw.rect(surf, PANEL,  rect, border_radius=radius)
        pygame.draw.rect(surf, BORDER, rect, width=1, border_radius=radius)

    @staticmethod
    def _draw_bar(surf, x, y, w, h, frac, col=BAR_FILL):
        pygame.draw.rect(surf, BAR_EMPTY, (x, y, w, h), border_radius=4)
        fill_w = max(4, int(frac * w))
        pygame.draw.rect(surf, col, (x, y, fill_w, h), border_radius=4)

    @staticmethod
    def _draw_led(surf, cx, cy, r, on, col_on=GREEN, col_off=GREY):
        col = col_on if on else col_off
        pygame.draw.circle(surf, col, (cx, cy), r)
        pygame.draw.circle(surf, BORDER, (cx, cy), r, width=1)

    @staticmethod
    def _label(surf, font, text, x, y, col=GREY, anchor="left"):
        s = font.render(text, True, col)
        if anchor == "right":
            x -= s.get_width()
        elif anchor == "center":
            x -= s.get_width() // 2
        surf.blit(s, (x, y))

    @staticmethod
    def _pwm_frac(pwm):
        return max(0.0, min(1.0, (pwm - 1000) / 1000.0))

    # ─────────────────────────────────────────────────────────────────────────
    #  GUI thread
    # ─────────────────────────────────────────────────────────────────────────

    def _run_gui(self):
        # Wait for pygame to be initialised by the RC thread
        time.sleep(0.6)

        screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("SM600 Keyboard Override")
        clock = pygame.time.Clock()

        font_title = pygame.font.SysFont("monospace", 18, bold=True)
        font_label = pygame.font.SysFont("monospace", 14)
        font_value = pygame.font.SysFont("monospace", 14, bold=True)
        font_key   = pygame.font.SysFont("monospace", 15, bold=True)
        font_log   = pygame.font.SysFont("monospace", 13)

        last_log    = "Ready — press a key"
        flash_key   = None
        flash_until = 0.0

        while self.running:
            now = time.time()

            # ── Events ────────────────────────────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

                elif event.type == pygame.KEYDOWN:
                    k = event.key

                    if k in (pygame.K_q, pygame.K_ESCAPE):
                        self.running = False

                    elif k == pygame.K_1:
                        self.mode_switch = self.stabilize
                        last_log = "Mode → STABILIZE";  flash_key = "1"
                    elif k == pygame.K_2:
                        self.mode_switch = self.alt_hold
                        last_log = "Mode → ALT_HOLD";   flash_key = "2"
                    elif k == pygame.K_3:
                        self.mode_switch = self.gps_hold
                        last_log = "Mode → GPS_HOLD";   flash_key = "3"
                    elif k == pygame.K_4:
                        self.mode_switch = self.guided
                        last_log = "Mode → GUIDED";     flash_key = "4"

                    elif k == pygame.K_5:
                        threading.Thread(target=self._pulse,
                            args=("takeoff_pressed", 0.5), daemon=True).start()
                        last_log = "TAKEOFF triggered";  flash_key = "5"
                    elif k == pygame.K_6:
                        threading.Thread(target=self._pulse,
                            args=("land_pressed", 0.5), daemon=True).start()
                        last_log = "LAND triggered";     flash_key = "6"

                    elif k == pygame.K_UP:
                        threading.Thread(target=self._pulse,
                            args=("dpad_up", 0.3), daemon=True).start()
                        last_log = "↑  D-Pad FRONT";    flash_key = "↑"
                    elif k == pygame.K_DOWN:
                        threading.Thread(target=self._pulse,
                            args=("dpad_down", 0.3), daemon=True).start()
                        last_log = "↓  D-Pad BACK";     flash_key = "↓"
                    elif k == pygame.K_LEFT:
                        threading.Thread(target=self._pulse,
                            args=("dpad_left", 0.3), daemon=True).start()
                        last_log = "←  D-Pad LEFT";     flash_key = "←"
                    elif k == pygame.K_RIGHT:
                        threading.Thread(target=self._pulse,
                            args=("dpad_right", 0.3), daemon=True).start()
                        last_log = "→  D-Pad RIGHT";    flash_key = "→"

                    if flash_key:
                        flash_until = now + 0.25

            if now > flash_until:
                flash_key = None

            # ── Background ────────────────────────────────────────────────────
            screen.fill(BG)

            # Title bar
            pygame.draw.rect(screen, PANEL, (0, 0, WIN_W, 52))
            pygame.draw.line(screen, BORDER, (0, 52), (WIN_W, 52), 1)
            self._label(screen, font_title, "SM600  Keyboard Override",
                        WIN_W // 2, 16, WHITE, anchor="center")
            self._label(screen, font_log, "Q / Esc to quit",
                        WIN_W - 16, 20, GREY, anchor="right")

            # ── Left panel — menu ─────────────────────────────────────────────
            self._draw_panel(screen, (MENU_X - 10, MENU_Y - 10,
                                      MENU_W + 20, len(MENU_ITEMS) * MENU_ROW_H + 20))
            self._label(screen, font_label, "KEYBOARD CONTROLS",
                        MENU_X + 4, MENU_Y - 6, ACCENT)

            for idx, (key_char, disp, section) in enumerate(MENU_ITEMS):
                row_y    = MENU_Y + 10 + idx * MENU_ROW_H
                is_flash = (key_char == flash_key)

                if is_flash:
                    pygame.draw.rect(screen, HIGHLIGHT,
                        (MENU_X - 8, row_y - 2, MENU_W + 16, MENU_ROW_H - 4),
                        border_radius=4)

                badge_col = ACCENT if is_flash else BORDER
                pygame.draw.rect(screen, badge_col,
                    (MENU_X, row_y + 2, 28, 24), border_radius=4)
                self._label(screen, font_key, key_char,
                            MENU_X + 14, row_y + 5, WHITE, anchor="center")

                if section == "mode":
                    txt_col = WHITE
                elif section == "cmd":
                    txt_col = YELLOW if key_char == "5" else RED
                else:
                    txt_col = GREEN if is_flash else WHITE

                self._label(screen, font_label, disp, MENU_X + 38, row_y + 7, txt_col)

            # ── Right panel — live state ───────────────────────────────────────
            rp_h = 14 * STATE_ROW_H + 20
            self._draw_panel(screen, (STATE_X - 10, STATE_Y - 10,
                                      STATE_W + 20, rp_h))
            self._label(screen, font_label, "LIVE  STATE",
                        STATE_X + 4, STATE_Y - 6, ACCENT)

            sy = STATE_Y + 10

            def state_row(title, value_str, bar_frac=None,
                          led=None, val_col=WHITE):
                nonlocal sy
                self._label(screen, font_label, title, STATE_X, sy, GREY)
                if bar_frac is not None:
                    self._draw_bar(screen, STATE_X + 120, sy + 4,
                                   STATE_W - 200, 14, bar_frac)
                    self._label(screen, font_value, value_str,
                                STATE_X + STATE_W - 68, sy, val_col)
                elif led is not None:
                    self._draw_led(screen, STATE_X + 128, sy + 9, 7, led,
                                   col_on=val_col)
                    self._label(screen, font_value,
                                "ON" if led else "off",
                                STATE_X + 145, sy,
                                val_col if led else GREY)
                else:
                    self._label(screen, font_value, value_str,
                                STATE_X + 120, sy, val_col)
                sy += STATE_ROW_H

            state_row("Flight Mode", self.mode_switch, val_col=ACCENT)

            pygame.draw.line(screen, BORDER,
                (STATE_X, sy), (STATE_X + STATE_W - 10, sy), 1)
            sy += 10

            state_row("Roll",     f"{self.roll_pwm}",
                      bar_frac=self._pwm_frac(self.roll_pwm))
            state_row("Pitch",    f"{self.pitch_pwm}",
                      bar_frac=self._pwm_frac(self.pitch_pwm))
            state_row("Yaw",      f"{self.yaw_pwm}",
                      bar_frac=self._pwm_frac(self.yaw_pwm))
            state_row("Throttle", f"{self.throttle_pwm}",
                      bar_frac=self._pwm_frac(self.throttle_pwm))

            pygame.draw.line(screen, BORDER,
                (STATE_X, sy), (STATE_X + STATE_W - 10, sy), 1)
            sy += 10

            state_row("Takeoff",  "", led=self.takeoff_pressed, val_col=YELLOW)
            state_row("Land",     "", led=self.land_pressed,    val_col=RED)
            state_row("Flip Btn", "", led=self.flip_button,     val_col=GREEN)

            pygame.draw.line(screen, BORDER,
                (STATE_X, sy), (STATE_X + STATE_W - 10, sy), 1)
            sy += 10

            # D-Pad row
            self._label(screen, font_label, "D-Pad", STATE_X, sy, GREY)
            arrows = [
                ("↑", self.dpad_up),
                ("↓", self.dpad_down),
                ("←", self.dpad_left),
                ("→", self.dpad_right),
            ]
            ax = STATE_X + 110
            for sym, state in arrows:
                col = GREEN if state else GREY
                pygame.draw.circle(screen, col, (ax + 10, sy + 9), 9)
                pygame.draw.circle(screen, BORDER, (ax + 10, sy + 9), 9, 1)
                self._label(screen, font_key, sym,
                            ax + 10, sy + 2, BG, anchor="center")
                ax += 60

            # Log bar
            pygame.draw.rect(screen, PANEL, (0, LOG_Y - 4, WIN_W, 44))
            pygame.draw.line(screen, BORDER, (0, LOG_Y - 4), (WIN_W, LOG_Y - 4), 1)
            self._label(screen, font_log,   "Last action :", 20,  LOG_Y + 8, GREY)
            self._label(screen, font_value, last_log,        140, LOG_Y + 7, WHITE)

            pygame.display.flip()
            clock.tick(FPS)

        # ── Cleanup ───────────────────────────────────────────────────────────
        pygame.quit()
        sys.exit(0)

    # ─────────────────────────────────────────────────────────────────────────

    def stop(self):
        self.running = False
        self._rc_thread.join()
        self._gui_thread.join()
