import json
from pathlib import Path
import pytest
from ffw_teleop.vr_start_pose import load_start_pose


def test_saved_waiting_pose_has_exactly_arms_and_never_commands_lift():
    path=Path(__file__).resolve().parents[1]/'config/teleop_start_pose.json'
    positions,lift,tolerance=load_start_pose(path)
    assert len(positions)==14 and 'lift_joint' not in positions
    assert lift==pytest.approx(-.20076) and tolerance==.01
    assert positions['arm_l_joint2']>1 and positions['arm_r_joint2']< -1
    assert positions['arm_l_joint4']==pytest.approx(positions['arm_r_joint4'],abs=.001)


def test_profile_rejects_missing_joints_nonfinite_and_outside_limits(tmp_path):
    original=json.loads((Path(__file__).resolve().parents[1]/'config/teleop_start_pose.json').read_text())
    path=tmp_path/'pose.json'
    for kind in ('missing','nan','limits','lift'):
        profile=json.loads(json.dumps(original))
        if kind=='missing':del profile['arm_positions']['arm_l_joint1']
        if kind=='nan':profile['arm_positions']['arm_l_joint1']=float('nan')
        if kind=='limits':profile['arm_positions']['arm_l_joint1']=100.
        if kind=='lift':profile['reference_lift_m']=-1.
        path.write_text(json.dumps(profile))
        with pytest.raises(ValueError):load_start_pose(path)
