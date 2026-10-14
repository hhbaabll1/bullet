import json,pathlib,subprocess,time
root=pathlib.Path('/cnic/work/wangky/bullet3_baseline/2026-10-13')
source=root/'source-bullet-a3ebe727'
results=[]
for precision in ('float','double'):
 build=root/('build-original-'+precision)
 cmds=[['cmake','-S',str(source),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release','-DBUILD_SHARED_LIBS=OFF','-DBUILD_BULLET2_DEMOS=OFF','-DBUILD_OPENGL3_DEMOS=OFF','-DBUILD_CPU_DEMOS=OFF','-DBUILD_EXTRAS=OFF','-DBUILD_UNIT_TESTS=ON','-DBUILD_BULLET3=OFF','-DINSTALL_LIBS=ON','-DUSE_DOUBLE_PRECISION='+('ON' if precision=='double' else 'OFF')],['cmake','--build',str(build),'-j','8'],['ctest','--test-dir',str(build),'--output-on-failure']]
 entry={'precision':precision,'source_commit':'a3ebe727e0bc8d4e780e9c8fb90f512769ca4299','phases':[]}
 for phase,cmd in zip(('configure','build','ctest'),cmds):
  log=root/('original-'+precision+'-'+phase+'.log')
  start=time.time()
  with log.open('w') as f:
   f.write('COMMAND '+json.dumps(cmd)+'\n');f.flush()
   code=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT).returncode
  entry['phases'].append({'phase':phase,'exit_code':code,'seconds':round(time.time()-start,2),'log':str(log)})
  (root/'original-checks-progress.json').write_text(json.dumps(results+[entry],indent=2))
  if code: break
 results.append(entry)
(root/'original-checks.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
