"""Independent evidence audit of recorded acquisition telemetry (not a new sim)."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from scorer import AcquisitionScore, Reading


def verify(path):
    raw = path.read_bytes()
    rows = json.loads(raw)
    if isinstance(rows,dict):
        rows = rows['telemetry']
    required = set(Reading.__dataclass_fields__)
    if not rows:
        raise ValueError('empty telemetry')
    scorer = AcquisitionScore()
    for n,r in enumerate(rows):
        missing = required-set(r)
        if missing:
            raise ValueError(f'row {n}: missing actual instrumentation fields {missing}')
        scorer.update(Reading(**{k:r[k] for k in required}))
    report=scorer.report()
    # Independently verify the acquired records against raw timestamps.
    by_time={r['t']:r for r in rows}
    audits=[]
    for direction,record in report['records'].items():
        active=direction+'_n'
        inactive=('right' if direction=='left' else 'left')+'_n'
        for stage,window in record.items():
            assert len(window)>1
            assert window[-1]['t']-window[0]['t'] >= 2.-1e-7
            full=[by_time[r['t']] for r in window]
            assert max(r['encoder_count'] for r in full)-min(r['encoder_count'] for r in full)<=2
            assert all(not r['direct_contact'] and r['fixture_intact'] and r['links_intact'] and r['setting_unchanged'] for r in full)
            if stage=='loaded':
                assert all(1.2<=r[active]<=1.8 and r[inactive]<=.1 and abs(r[direction+'_tangent_error_deg'])<=10 for r in full)
                assert max(r[active] for r in full)-min(r[active] for r in full)<=.20000001
            else:
                assert all(max(r['left_n'],r['right_n'])<=.1 for r in full)
            audits.append({'direction':direction,'stage':stage,'start':window[0]['t'],
                           'end':window[-1]['t'],'samples':len(window)})
        assert record['unloaded'][0]['t']>record['loaded'][-1]['t']
    if report['score']==1:
        final=[r for r in rows if r['t']>=rows[-1]['t']-2.-1e-7]
        assert final[-1]['t']-final[0]['t']>=2.-1e-7
        assert all(r['gauges_released'] and max(r['left_n'],r['right_n'])<=.1 and not r['direct_contact'] for r in final)
    return {'evidence_type':'offline audit of recorded telemetry; physical provenance still requires code/video review',
            'telemetry_sha256':hashlib.sha256(raw).hexdigest(),'samples':len(rows),
            'score':report['score'],'invalid_reason':report['invalid_reason'],
            'measured_reversal_deg':report['measured_reversal_deg'],'windows':audits}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('telemetry',type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args(); result=verify(a.telemetry)
    text=json.dumps(result,indent=2)+'\n'
    if a.output:a.output.write_text(text)
    print(text)
