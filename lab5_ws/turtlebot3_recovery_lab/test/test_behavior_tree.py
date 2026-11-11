"""Check the recovery contract across XML and the service interface."""
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]


def test_completion_follows_successful_recovery_motion_and_preserves_original_goal():
    tree = ET.parse(PACKAGE / 'behavior_trees' / 'navigate_w_recovery_divert.xml')
    recovery = tree.find('.//RecoveryNode')
    assert recovery.attrib['number_of_retries'] == '3'
    normal, diversion = list(recovery)
    assert normal.find('.//ComputePathToPose').attrib['goal'] == '{goal}'
    stages = list(diversion)
    assert [node.tag for node in stages] == [
        'ClearEntireCostmap', 'BeepAndDivertBridge', 'ComputePathToPose', 'FollowPath', 'RecoveryComplete']
    assert stages[2].attrib['goal'] == '{recovery_pose}'
    assert stages[2].attrib['path'] == stages[3].attrib['path'] == '{recovery_path}'
    assert stages[1].attrib['service_name'] == '/divert_robot'
    assert stages[4].attrib['service_name'] == '/recovery_complete'
    assert all(int(stages[i].attrib['server_timeout']) >= 1000 for i in [1, 4])
