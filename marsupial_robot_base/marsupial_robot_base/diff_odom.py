import math
import time
from dataclasses import dataclass
from typing import Optional

@dataclass
class DiffOdom:
    L: float = 0.31
    CIRC: float = 0.515

    sign_l: float = -1.0
    sign_r: float = +1.0

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    v_l: float = 0.0
    v_r: float = 0.0
    v: float = 0.0
    w: float = 0.0

    _last_l_turns: Optional[float] = None
    _last_r_turns: Optional[float] = None
    _t_last: Optional[float] = None

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta
        self.v_l = self.v_r = self.v = self.w = 0.0
        self._last_l_turns = self._last_r_turns = None
        self._t_last = None

    def _norm_angle(self, a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def update_from_turns(self, pos_l_raw_turns: float, pos_r_raw_turns: float):
        t = time.time()

        pos_l = self.sign_l * pos_l_raw_turns
        pos_r = self.sign_r * pos_r_raw_turns

        if self._last_l_turns is None:
            self._last_l_turns, self._last_r_turns, self._t_last = pos_l, pos_r, t
            return self.x, self.y, self.theta

        dt = max(1e-6, t - self._t_last)

        dl = (pos_l - self._last_l_turns) * self.CIRC
        dr = (pos_r - self._last_r_turns) * self.CIRC

        self._last_l_turns, self._last_r_turns, self._t_last = pos_l, pos_r, t

        ds = 0.5 * (dr + dl)
        dth = (dr - dl) / self.L
        th_m = self.theta + 0.5 * dth

        self.x += ds * math.cos(th_m)
        self.y += ds * math.sin(th_m)
        self.theta = self._norm_angle(self.theta + dth)

        self.v_l = dl / dt
        self.v_r = dr / dt
        self.v = 0.5 * (self.v_r + self.v_l)
        self.w = (self.v_r - self.v_l) / self.L

        return self.x, self.y, self.theta

    def update_from_vel(self, vel_l_raw_tps: float, vel_r_raw_tps: float, dt: float):
        v_l = self.sign_l * vel_l_raw_tps * self.CIRC
        v_r = self.sign_r * vel_r_raw_tps * self.CIRC

        self.v_l, self.v_r = v_l, v_r
        self.v = 0.5 * (v_r + v_l)
        self.w = (v_r - v_l) / self.L

        th_m = self.theta + 0.5 * self.w * dt
        self.x += self.v * math.cos(th_m) * dt
        self.y += self.v * math.sin(th_m) * dt
        self.theta = self._norm_angle(self.theta + self.w * dt)

        return self.x, self.y, self.theta