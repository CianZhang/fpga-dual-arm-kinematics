#!/usr/bin/env python3
"""Verify curated file hashes, project input paths, links and Python sample.

Does not compile RTL, open CAD assemblies, or validate physical motion.
"""
from pathlib import Path
from urllib.parse import unquote
import hashlib
import importlib.util
import json
import math
import re
import sys

ROOT = Path(__file__).resolve().parents[1]

def check():
    errors = []
    entries = json.loads((ROOT/'docs/source-manifest.json').read_text())
    for item in entries:
        p = ROOT/item['destination']
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != item['sha256']:
            errors.append('Missing or changed source artifact: '+item['destination'])
    pds = ROOT/'fpga/pango/top_delta_ctrl.pds'
    generated = []
    inputs = re.findall(r'\(_file "([^"]+\.(?:v|fdc))"',pds.read_text())
    for name in inputs:
        if name.startswith('generated/'):
            generated.append(name)
        elif not (pds.parent/name).is_file():
            errors.append('Missing PDS input: '+name)
    link_count = 0
    for doc in ROOT.rglob('*.md'):
        for link in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)',doc.read_text()):
            if '://' in link or link.startswith('#'): continue
            link_count += 1
            target = unquote(link.split('#')[0].strip('<>'))
            if not (doc.parent/target).exists():
                errors.append('Broken link in '+str(doc.relative_to(ROOT))+': '+link)
    spec=importlib.util.spec_from_file_location('delta_reference', ROOT/'reference/delta_inverse.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    results=module.solve(0,0,-237.5)
    expected=6680.766846777271/256
    if not all(math.isclose(v,expected,rel_tol=0,abs_tol=1e-10) for v in results):
        errors.append('Reference sample differs from original Python output')
    result={'status':'passed' if not errors else 'failed',
            'source_artifacts_checked':len(entries),'pds_inputs_checked':len(inputs),
            'markdown_local_links_checked':link_count,
            'required_local_generated_ip':generated,
            'reference_sample_mm':[0,0,-237.5], 'reference_motor_commands_deg':results,
            'not_performed':['PDS synthesis and place-and-route','HDL simulation','CAD assembly rebuild','hardware test'],
            'errors':errors}
    return result

if __name__=='__main__':
    result=check();print(json.dumps(result,ensure_ascii=False,indent=2))
    sys.exit(0 if result['status']=='passed' else 1)
