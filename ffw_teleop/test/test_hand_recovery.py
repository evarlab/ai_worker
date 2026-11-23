import copy
import pytest
from ffw_teleop.hand_recovery import recovery_action,recover_once


def status(message='enabled',level=0,code='0x0000'):
    return dict(message=message,level=level,values=dict(ros2_control_active='true',motors_enabled='true',device_status=code,device_feedback_hz='500'))


def test_healthy_hand_is_not_restarted():
    assert recovery_action(status(),.1,.01)=='ready'


@pytest.mark.parametrize('reason',['ERROR_TORQUE_LIMIT_EXCEEDED (0x0503) on right_index_MCP_FE','ERROR_JOINT_LIMIT_EXCEEDED (0x0501)','joint feedback lost'])
def test_only_cleared_latched_fault_is_recovered(reason):
    s=status('holding: '+reason,1)
    assert recovery_action(s,.1,.01)=='recover'
    s['values']['device_status']='0x0503'
    with pytest.raises(ValueError,match='現在も'):recovery_action(s,.1,.01)


@pytest.mark.parametrize('change', ['stale_diag','stale_joint','inactive_controller','inactive_hardware','disabled_motors','actual_fault','unknown_hold','manual_disable','no_feedback'])
def test_unhealthy_or_intentionally_disabled_hands_never_auto_recover(change):
    s=status();age=.1;joint=.01;active=True
    if change=='stale_diag':age=3.
    if change=='stale_joint':joint=1.
    if change=='inactive_controller':active=False
    if change=='inactive_hardware':s['values']['ros2_control_active']='false'
    if change=='disabled_motors':s['values']['motors_enabled']='false'
    if change=='actual_fault':s.update(level=2,message='ERROR_TORQUE_LIMIT_EXCEEDED');s['values']['device_status']='0x0503'
    if change=='unknown_hold':s.update(level=1,message='holding: unknown condition')
    if change=='manual_disable':s.update(level=1,message='holding: motors were disabled outside ros2_control')
    if change=='no_feedback':s['values']['device_feedback_hz']='0'
    with pytest.raises(ValueError):recovery_action(s,age,joint,active)


def test_single_recovery_stops_controller_before_cycling_hardware():
    commands=[];recover_once('right',commands.append)
    assert commands==[
        ['ros2','control','switch_controllers','--deactivate','hand_r_controller'],
        ['ros2','control','set_hardware_component_state','sharpa_right_hand','inactive'],
        ['ros2','control','set_hardware_component_state','sharpa_right_hand','active'],
        ['ros2','control','switch_controllers','--activate','hand_r_controller']]


def test_failed_reactivation_is_not_retried_or_followed_by_controller_activation():
    commands=[]
    def command(args):
        commands.append(args)
        if len(commands)==3:raise RuntimeError('device fault returned')
    with pytest.raises(RuntimeError):recover_once('left',command)
    assert len(commands)==3
    assert not any('--activate' in args for args in commands)
