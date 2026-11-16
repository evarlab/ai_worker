from ffw_teleop.vr_processes import process_ids


def test_live_gate_detection_does_not_confuse_preview_with_real_robot(tmp_path):
    for pid, domain in ((12301, b'98'), (12302, b'0'), (12303, None)):
        process = tmp_path/str(pid)
        process.mkdir()
        (process/'cmdline').write_bytes(b'/workspace/install/ffw_teleop/lib/ffw_teleop/sharpa_vr_bridge\0--ros-args\0')
        (process/'environ').write_bytes(b'ROS_DOMAIN_ID='+domain+b'\0' if domain else b'PATH=/usr/bin\0')
    assert process_ids('/lib/ffw_teleop/sharpa_vr_bridge', 0, tmp_path) == [12302, 12303]
    assert process_ids('/lib/ffw_teleop/sharpa_vr_bridge', 98, tmp_path) == [12301]
