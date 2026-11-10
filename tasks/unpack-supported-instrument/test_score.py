from score_logic import History

def tick(h,**kw):
 args=dict(z=.8,tilt_deg=0.,surface_support=True,grasp_support=False,stand_ok=False,packing_ok=False,released=True,dt=.05);args.update(kw);return h.update(**args)
# Idle cannot earn credit.
h=History()
for _ in range(40):tick(h)
assert h.score==0 and not h.invalid
# Released stable stand earns partial; both fully contained foam pieces need stability.
for _ in range(20):tick(h,stand_ok=True)
assert h.score==.5
for _ in range(19):tick(h,stand_ok=True,packing_ok=True)
assert h.score==.5
tick(h,stand_ok=True,packing_ok=True);assert h.score==1
# Descending 10 cm while grasp-supported is valid.
h=History()
for i in range(30):tick(h,z=1.-i*.004,surface_support=False,grasp_support=True)
assert not h.invalid
# A 4 cm unsupported fall is caught even if it lands on the final update.
h=History();tick(h,z=1.)
for z in (.995,.985,.975):tick(h,z=z,surface_support=False)
tick(h,z=.955,surface_support=True);assert h.invalid
# Failure is irreversible, and a tip is independently fatal.
for _ in range(50):tick(h,stand_ok=True,packing_ok=True)
assert h.score==0
h=History();tick(h,tilt_deg=25.1);assert h.invalid
print('7 scorer boundary/history controls passed (synthetic; not physical evidence)')
