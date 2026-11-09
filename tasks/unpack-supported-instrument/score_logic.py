"""Pure numeric support/drop history and staged scoring, SI units."""
from dataclasses import dataclass,asdict
@dataclass
class History:
 invalid: bool=False
 max_unsupported_drop_m: float=0.
 unsupported_peak_z: float|None=None
 last_z: float|None=None
 stand_stable_s: float=0.
 complete_stable_s: float=0.
 score: float=0.
 reason: str=''
 def update(self,*,z,tilt_deg,surface_support,grasp_support,stand_ok,packing_ok,released,dt):
  if tilt_deg>25:
   self.invalid=True;self.reason='instrument_tilt_over_25deg'
  controlled=surface_support or grasp_support
  # Check accumulated descent before ending an unsupported interval on landing.
  if self.unsupported_peak_z is not None:
   self.unsupported_peak_z=max(self.unsupported_peak_z,z)
   drop=max(0.,self.unsupported_peak_z-z)
   self.max_unsupported_drop_m=max(self.max_unsupported_drop_m,drop)
   if drop>.03:self.invalid=True;self.reason='unsupported_drop_over_30mm'
  if controlled:self.unsupported_peak_z=None
  elif self.unsupported_peak_z is None:self.unsupported_peak_z=max(z,self.last_z if self.last_z is not None else z)
  self.last_z=z
  self.stand_stable_s=self.stand_stable_s+dt if stand_ok and released else 0.
  self.complete_stable_s=self.complete_stable_s+dt if stand_ok and packing_ok and released else 0.
  self.score=0. if self.invalid or self.stand_stable_s+1e-8<1. else (1. if self.complete_stable_s+1e-8>=1. else .5)
  return asdict(self)
