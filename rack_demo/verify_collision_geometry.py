"""Regression checks: collision coverage, hand sweep, and deliberate collisions."""
import json
import math
import fcl
from collision_guard import CollisionGuard,transform
from demo_scene import joints,HERE

g=CollisionGuard()
original=['crane_plus_base']+[f'crane_plus_link{i}' for i in range(1,5)]+['crane_plus_link_hand']
for name in original:
    assert g.robot.find(f"link[@name='{name}']/collision") is not None,name
tray=g.robot.find("link[@name='rack_support_tray']")
assert len(tray.findall('collision'))==len(tray.findall('visual'))==6
q=joints(.1,0,.16,lift=.1)+[.1]
for i in range(81): g.check(q,-.4+.8*i/80)
clearance=g.minimum
hand=[p for p in g.parts if p[0]=='crane_plus_link_hand']
g.parts=hand+[('rack_support_tray','deliberate_obstacle',fcl.Box(.08,.08,.08),
              transform((.02,-.02,.08)))]
try:
    g.check(q)
except ValueError as exc:
    assert 'crane_plus_link_hand' in str(exc) and 'rack_support_tray' in str(exc),str(exc)
    fixture_rejected=str(exc)
else: raise AssertionError('Hand/fixture collision was missed')
g.parts=hand
frames=g.fk(q,.4)
g.environment=[('rack_stock_test/body',fcl.Box(.08,.08,.08),
                frames['rack_support_tray']@transform((.02,-.02,.08)))]
try: g.check(q)
except ValueError as exc:
    assert 'crane_plus_link_hand' in str(exc) and 'rack_stock_test' in str(exc),str(exc)
    equipment_rejected=str(exc)
else: raise AssertionError('Hand/equipment collision was missed')
report={'original_links_with_collisions':original,'fixture_collision_shapes':6,
        'hand_sweep_samples':81,'hand_sweep_minimum_clearance_m':clearance,
        'deliberate_fixture_collision_rejected':fixture_rejected,
        'deliberate_equipment_collision_rejected':equipment_rejected}
(HERE/'collision_geometry_verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
