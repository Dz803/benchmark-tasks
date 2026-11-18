"""Telemetry-only score prototype. No simulator imports or hidden backlash input.

The adapter must supply measured instrument readings and independently observed
integrity/contact flags, never demo waypoint completion. Not a physical test.
"""
from dataclasses import dataclass
from collections import deque
import math


@dataclass(frozen=True)
class Reading:
    t: float
    left_n: float
    right_n: float
    encoder_count: int  # unwrapped measured output encoder, 4096 counts/rev
    left_tangent_error_deg: float = 0.0
    right_tangent_error_deg: float = 0.0
    direct_contact: bool = False
    fixture_intact: bool = True
    links_intact: bool = True
    setting_unchanged: bool = True
    gauges_released: bool = False


class AcquisitionScore:
    DWELL = 2.0
    MAX_GAP = 0.16  # intended acquisition cadence 10 Hz

    def __init__(self):
        self.last_t = None
        self.window = deque()
        self.phase = 'baseline'
        self.active = None
        self.pending_loaded = None
        self.records = {}
        self.invalid_reason = None
        self.overload_since = None
        self.final_released = False
        self.events = []

    def _restart(self, reason):
        # A touched output/telemetry discontinuity can alter the shared reference.
        self.window.clear()
        self.records.clear()
        self.phase = 'baseline'
        self.active = None
        self.pending_loaded = None
        self.final_released = False
        self.events.append(reason)

    @staticmethod
    def _mode(s):
        if s.left_n <= .10 and s.right_n <= .10:
            return 'zero'
        if 1.2 <= s.left_n <= 1.8 and s.right_n <= .10 and abs(s.left_tangent_error_deg) <= 10:
            return 'left'
        if 1.2 <= s.right_n <= 1.8 and s.left_n <= .10 and abs(s.right_tangent_error_deg) <= 10:
            return 'right'
        return 'transition'

    def _stable(self, mode):
        w = list(self.window)
        if not w or w[-1].t-w[0].t < self.DWELL-1e-8:
            return False
        if any(self._mode(s) != mode for s in w):
            return False
        if max(s.encoder_count for s in w)-min(s.encoder_count for s in w) > 2:
            return False
        if mode in ('left','right'):
            fs = [getattr(s, mode+'_n') for s in w]
            if max(fs)-min(fs) > .20+1e-8:
                return False
        return True

    def _record_window(self):
        return [dict(t=s.t, left_n=s.left_n, right_n=s.right_n,
                     encoder_count=s.encoder_count) for s in self.window]

    def update(self, s):
        numeric = (s.t,s.left_n,s.right_n,s.encoder_count,
                   s.left_tangent_error_deg,s.right_tangent_error_deg)
        if any(not math.isfinite(x) for x in numeric) or min(s.left_n,s.right_n) < 0:
            self._restart('invalid telemetry')
            return self.score
        if self.last_t is not None and s.t <= self.last_t:
            self._restart('stale/nonmonotonic telemetry')
            return self.score
        if self.last_t is not None and s.t-self.last_t > self.MAX_GAP:
            self._restart('telemetry gap')
        self.last_t = s.t
        self.final_released = False
        if not (s.fixture_intact and s.links_intact and s.setting_unchanged):
            self.invalid_reason = 'fixture/link/servo-setting changed'
        if max(s.left_n,s.right_n) >= 2.0:
            if self.overload_since is None:
                self.overload_since = s.t
            elif s.t-self.overload_since >= .2-1e-8:
                self.invalid_reason = 'overload protocol violation'
        else:
            self.overload_since = None
        if self.invalid_reason:
            return 0.0
        if s.direct_contact:
            self._restart('direct contact invalidates shared measurement reference')
            return self.score

        self.window.append(s)
        while len(self.window)>1 and s.t-self.window[1].t >= self.DWELL-1e-8:
            self.window.popleft()
        mode = self._mode(s)
        if self.phase == 'baseline':
            if self._stable('zero'):
                self.phase = 'load'
                self.window.clear()
        elif self.phase == 'load':
            if mode in ('left','right') and mode not in self.records and self._stable(mode):
                self.active = mode
                self.pending_loaded = self._record_window()
                self.phase = 'unload'
                self.window.clear()
        elif self.phase == 'unload':
            # Never accept opposite load before finishing the pending unloading.
            if (s.right_n if self.active == 'left' else s.left_n) > .10:
                self.pending_loaded = None
                self.active = None
                self.phase = 'baseline'
                self.window.clear()
            elif self._stable('zero'):
                self.records[self.active] = dict(loaded=self.pending_loaded,
                                                unloaded=self._record_window())
                self.active = None
                self.pending_loaded = None
                self.phase = 'load'
                self.window.clear()
        if len(self.records)==2 and self._stable('zero'):
            self.final_released = all(x.gauges_released for x in self.window)
        return self.score

    @property
    def score(self):
        if self.invalid_reason or not self.records:
            return 0.0
        return 1.0 if len(self.records)==2 and self.final_released else .5

    def report(self):
        means={d:{k:sum(x['encoder_count'] for x in v[k])/len(v[k])
                  for k in ('loaded','unloaded')} for d,v in self.records.items()}
        reversal={}
        if len(means)==2:
            reversal={k:abs(means['left'][k]-means['right'][k])*360/4096
                      for k in ('loaded','unloaded')}
        return dict(score=self.score, phase=self.phase, invalid_reason=self.invalid_reason,
                    records=self.records, measured_reversal_deg=reversal,
                    final_released=self.final_released, events=self.events)
