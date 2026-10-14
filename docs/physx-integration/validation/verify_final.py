import json,pathlib,subprocess,os,time
r=pathlib.Path('/cnic/work/wangky/bullet3');out=pathlib.Path('/cnic/work/wangky/bullet3_baseline/2026-10-14-interfaces')
subprocess.run(['python3',str(r/'.git/codex-transfer/run_interface_checks.py')],check=True)
core=json.loads((out/'result.json').read_text())
assert all(p['exit_code']==0 for e in core for p in e['phases']), 'core build/tests failed'
results=[]
def run(name,cmd,env=None):
 log=out/(name+'.log');start=time.time()
 with log.open('w') as f:
  f.write('COMMAND '+json.dumps(cmd)+'\n');f.flush();code=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,env=env).returncode
 results.append({'phase':name,'exit_code':code,'seconds':round(time.time()-start,2),'log':str(log)})
 (out/'extra-checks.json').write_text(json.dumps(results,indent=2))
 assert code==0,name+' failed'
consumer=out/'installed_consumer.cpp'
consumer.write_text('#include <BulletDynamics/ConstraintSolver/btConstraintExtension.h>\n#include <BulletDynamics/ConstraintSolver/btRowDescriptorConstraint.h>\n#include <BulletDynamics/Featherstone/btMultiBodyRowDescriptorConstraint.h>\n#include <BulletSoftBody/btConstraintElementOperator.h>\nint main(){btConstraintDescriptionGraph g;btConstraintElementPoint p;return btConstraintDescriptionImporter::validate(g).isOk() && p.validate().isOk()?0:1;}\n')
for precision in ('float','double'):
 build=out/('build-verified-'+precision);prefix=out/('install-'+precision);exe=out/('consumer-'+precision)
 run(precision+'-install',['cmake','--install',str(build),'--prefix',str(prefix)])
 cmd=['c++','-std=c++11','-I'+str(prefix/'include/bullet'),str(consumer),'-L'+str(prefix/'lib'),'-lBulletDynamics','-lBulletCollision','-lLinearMath','-o',str(exe)]
 if precision=='double':cmd.insert(1,'-DBT_USE_DOUBLE_PRECISION')
 run(precision+'-consumer-compile',cmd);run(precision+'-consumer-run',[str(exe)])
build=out/'build-sanitize'
run('sanitize-configure',['cmake','-G','Ninja','-S',str(r),'-B',str(build),'-DCMAKE_BUILD_TYPE=Debug','-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer','-DBUILD_SHARED_LIBS=OFF','-DBUILD_BULLET2_DEMOS=OFF','-DBUILD_OPENGL3_DEMOS=OFF','-DBUILD_CPU_DEMOS=OFF','-DBUILD_EXTRAS=OFF','-DBUILD_UNIT_TESTS=ON','-DBUILD_BULLET3=OFF'])
run('sanitize-build',['cmake','--build',str(build),'--clean-first','--target','Test_ConstraintInterface','-j','8'])
env=os.environ.copy();env['ASAN_OPTIONS']='detect_leaks=1:halt_on_error=1';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
run('sanitize-ctest',['ctest','--test-dir',str(build),'-R','ConstraintInterface','--output-on-failure'],env)
print('ALL_CHECKS_PASSED')
