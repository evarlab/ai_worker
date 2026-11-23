"""Run the real shell start sequence with ALL robot/ROS operations replaced by stubs."""
import os
from pathlib import Path
import subprocess
import pytest


@pytest.mark.parametrize('recovery_fails',[False,True])
def test_manual_start_recovers_before_motion_and_aborts_cleanly(tmp_path,recovery_fails):
    root=tmp_path/'repo';root.mkdir();bin_dir=tmp_path/'bin';bin_dir.mkdir();events=tmp_path/'events'
    source=Path(__file__).resolve().parents[2]/'resume_teleop.sh'
    script=root/'resume_teleop.sh'
    script.write_text(source.read_text().replace('/usr/bin/python3','stubpy'))
    package=root/'ffw_teleop';(package/'scripts').mkdir(parents=True);(package/'ffw_teleop').mkdir()
    (package/'scripts/session_log.sh').write_text('teleop_session_log() { :; }\n')
    for name in ['vr_processes.py','hand_recovery.py']:(package/'ffw_teleop'/name).write_text('# inert input for SSH stub\n')
    stop=root/'stop_teleop.sh';stop.write_text('echo STOP >> "$TEST_EVENTS"\n')
    stubs={
      'ssh':'''#!/usr/bin/python3
import os,sys
args=' '.join(sys.argv[1:]);sys.stdin.read()
event='RECOVER' if 'ai-worker-hand-recovery --recover' in args else 'CHECK_GATE'
with open(os.environ['TEST_EVENTS'],'a') as f:f.write(event+'\\n')
raise SystemExit(7 if event=='RECOVER' and os.environ['TEST_RECOVERY_FAIL']=='1' else 0)
''',
      'systemctl':'''#!/usr/bin/python3
import os,sys
with open(os.environ['TEST_EVENTS'],'a') as f:f.write('SYSTEMCTL '+ ' '.join(sys.argv[1:])+'\\n')
''',
      'stubpy':'''#!/usr/bin/python3
import os,sys
if '-m' in sys.argv:event='INIT'
else:
 text=sys.stdin.read()
 if "create_node('resume_teleop_once')" in text:event='DISABLE_ARM'
 elif "create_node('start_vr_after_initial_pose')" in text:event='ENABLE_ARM'
 elif "create_node('resume_fingers_once')" in text:event='ENABLE_FINGERS'
 else:raise RuntimeError('Unexpected inline operation')
with open(os.environ['TEST_EVENTS'],'a') as f:f.write(event+'\\n')
'''}
    for name,text in stubs.items():
        p=bin_dir/name;p.write_text(text);p.chmod(0o755)
    env={**os.environ,'PATH':str(bin_dir)+':'+os.environ['PATH'],
         'XDG_CACHE_HOME':str(tmp_path/'cache'),'TEST_EVENTS':str(events),
         'TEST_RECOVERY_FAIL':str(int(recovery_fails))}
    result=subprocess.run(['bash',str(script)],env=env,capture_output=True,text=True,timeout=15)
    log=events.read_text().splitlines()
    if recovery_fails:
        assert result.returncode==7,(result.stdout,result.stderr)
        assert 'RECOVER' in log and 'STOP' in log
        assert not any(x in log for x in ['INIT','ENABLE_ARM','ENABLE_FINGERS'])
    else:
        assert result.returncode==0,(result.stdout,result.stderr)
        stages=[x for x in log if x in ['CHECK_GATE','DISABLE_ARM','RECOVER','INIT','ENABLE_ARM','ENABLE_FINGERS']]
        assert stages==['CHECK_GATE','DISABLE_ARM','RECOVER','INIT','ENABLE_ARM','ENABLE_FINGERS']
        assert 'STOP' not in log
        assert log.index('SYSTEMCTL --user start contactglove-sharpa.service') < log.index('RECOVER')
