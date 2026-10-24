"""Audit the documented 1U-lower / blocked 2U-lower / 2U-upper scenario."""
import json,math
from demo_scene import HERE,BASE_Z,SIZES,LEVELS,server_offset
from session_scene import STATE,snapshot

report=json.loads((HERE/'session_verification.json').read_text())
trace=json.loads((HERE/'session_log.json').read_text())
state=json.loads(STATE.read_text())
assert [r['installed'] for r in report]==[True,False,True],report
assert report[1]['motion_batches']==0,report
assert len(state['installed'])==2,state
models=snapshot()
requests={e['size']:e for e in trace if e['state']=='request_accepted'}
paths=[e for e in trace if e['state']=='path_checked']
assert paths and min(e['minimum_clearance_m'] for e in paths)>=.002
assert not any(e['state'] in ('path_collision_blocked','motion_failed') for e in trace)
contacts=json.loads((HERE/'contact_audit.json').read_text())
assert contacts['subscriptions']==11 and contacts['pairs'],contacts
unexpected=[p for p in contacts['pairs'] if not (
    'rack_support_tray_fixed_joint_lump__plate_collision_collision' in p['pair']
    and 'rack_stock_' in p['pair'] and p['maximum_depth_m']<=.001)]
assert not unexpected,unexpected
physics=json.loads((HERE/'collision_physics_verification.json').read_text())
assert {p['mode'] for p in physics if p['passed']}=={'fixture','equipment'}
offline=json.loads((HERE/'collision_paths_verification.json').read_text())
assert len(offline)==8 and all(p['passed'] for p in offline)
result={'sequence':report,'placements':[],'collision_rejected_before_motion':True,
        'checked_motion_batches':len(paths),
        'minimum_planned_clearance_m':min(e['minimum_clearance_m'] for e in paths),
        'unexpected_physical_contacts':unexpected,'physical_contacts':contacts,
        'isolated_physics_checks':physics,'offline_path_checks':offline}
for item in state['installed']:
    size,level=item['size'],item['level']
    model=models[item['model']]
    assert model['id']==requests[size]['entity_id']
    p=model['position']
    error=math.dist([p.get(k,0) for k in ('x','y','z')],
        [.23,0,BASE_Z+LEVELS[level]+server_offset(size)])
    assert error<.012,(item,error)
    checks=[e for e in trace if e['state']=='equipment_check' and e['size']==size]
    assert all(e['entity_id']==model['id'] for e in checks)
    last=checks[-1]['position']
    drift=math.dist([p.get(k,0) for k in ('x','y','z')],
                    [last.get(k,0) for k in ('x','y','z')])
    assert drift<.001,(size,drift)
    result['placements'].append({'size':size,'level':level,'entity_id':model['id'],
        'position':p,'error_m':error,'drift_after_other_operations_m':drift})
(HERE/'session_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps(result,ensure_ascii=False,indent=2))
