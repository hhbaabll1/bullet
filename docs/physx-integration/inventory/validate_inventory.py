#!/usr/bin/env python3
"""Validate step-2 links and pinned Git sources. No engine tests or worktree edits."""
import argparse
import collections
import fnmatch
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent

def read(name):
    return json.loads((ROOT/name).read_text(encoding='utf-8'))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--physx', type=Path)
    parser.add_argument('--bullet', type=Path)
    parser.add_argument('--report', type=Path)
    args=parser.parse_args()
    inv=read('inventory.json'); entries=inv['items']; source=read('source-index.json')
    profiles=read('parameter-profiles.json'); objects=read('object-profiles.json'); tests=read('tests.json')
    errors=[]
    def check(ok,msg):
        if not ok: errors.append(msg)
    ids=[x['id'] for x in entries]; tids=[x['id'] for x in tests]
    check(len(ids)==len(set(ids)),'duplicate coverage ID')
    check(len(tids)==len(set(tids)),'duplicate test ID')
    fields=('id','family','name','layer','physx_sources','parameter_profiles','object_profile','bullet_action','required_support','test_ids','known_difference','inventory_status','implementation_status','validation_status','work_package','responsible_task')
    for x in entries:
        for f in fields: check(bool(x.get(f)),x['id']+': missing '+f)
        for p in x['parameter_profiles']: check(p in profiles,x['id']+': unknown profile '+p)
        check(x['object_profile'] in objects,x['id']+': unknown object profile')
        for t in x['test_ids']: check(t in tids,x['id']+': unknown test '+t)
        for s in x['physx_sources']+x['bullet_sources']: check(s in source,x['id']+': unknown evidence')
        check(x['validation_status']=='planned_not_run',x['id']+': inventory must not imply passed engine tests')
    for t in tests:
        for x in t['coverage_ids']: check(x in ids,t['id']+': unknown coverage '+x)
        check(bool(t['acceptance']),t['id']+': missing acceptance')
    counts=collections.Counter(x['family'] for x in entries)
    for k,v in {'ordinary':8,'d6_motion':18,'d6_drives':8,'articulation_joint':5,'attachment':11,'attachment_rejected':6,'row_semantics':7,'row_hints':6,'backend_dispatch':14}.items():
        check(counts[k]==v,f'{k}: expected {v}, got {counts[k]}')
    for entry in read('enum-audit.json'):
        for pattern in entry['coverage_patterns']:
            check(any(fnmatch.fnmatch(x,pattern) for x in ids),'empty enum coverage pattern '+pattern)
        check(entry['source'] in source,'unknown enum source')
    for entry in read('api-contracts.json')+read('implementation-contracts.json'):
        check(entry['source'] in source,'unknown contract source')
    paths={'physx':args.physx,'bullet':args.bullet}
    verified_files=0
    for repo,root in paths.items():
        if root is None: continue
        refs={s['path']:s for s in source.values() if s['repository']==repo}
        queries=[s['commit']+':'+p for p,s in refs.items()]
        result=subprocess.run(['git','-C',str(root),'cat-file','--batch'],input=('\n'.join(queries)+'\n').encode(),capture_output=True,check=True)
        data=result.stdout; pos=0; blobs={}
        for path,s in refs.items():
            end=data.find(b'\n',pos); header=data[pos:end].decode(); pos=end+1
            check(not header.endswith(' missing'),repo+': missing '+path)
            if header.endswith(' missing'): continue
            sha,kind,size=header.split(); size=int(size)
            content=data[pos:pos+size]; pos+=size+1
            check(kind=='blob',repo+': not blob '+path)
            check(sha==s['git_blob_sha1'],repo+': hash mismatch '+path)
            check(hashlib.sha1(b'blob '+str(size).encode()+b'\0'+content).hexdigest()==sha,repo+': bytes hash mismatch '+path)
            blobs[path]=content.decode('utf-8').splitlines(); verified_files+=1
        for key,s in source.items():
            if s['repository']!=repo or s['path'] not in blobs: continue
            lines=blobs[s['path']]
            check(0<s['line']<=len(lines),key+': line out of range')
            if 0<s['line']<=len(lines): check(s['symbol'] in lines[s['line']-1],key+': symbol mismatch')
    report={'result':'PASS' if not errors else 'FAIL','coverage_items':len(entries),'test_designs':len(tests),'source_references':len(source),'git_source_files_verified':verified_files,
            'verification_scope':'inventory structure + specified pinned source Git blobs; engine tests not run','errors':errors}
    if args.report: args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 1 if errors else 0

if __name__=='__main__':
    raise SystemExit(main())
