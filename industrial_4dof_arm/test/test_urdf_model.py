"""Structural checks for the 4-DOF industrial arm URDF.

Expands the Xacro and verifies links, joints, axes, limits and the
parent/child chain against the project specification.
"""

import math
import os
import xml.etree.ElementTree as ET

import pytest
import xacro

XACRO_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'urdf', 'industrial_arm.urdf.xacro')

# name: (parent, child, axis, lower_deg, upper_deg)
EXPECTED_JOINTS = {
    'joint_1': ('base_link', 'link_1', (0.0, 0.0, 1.0), -180.0, 180.0),
    'joint_2': ('link_1', 'link_2', (0.0, 1.0, 0.0), -90.0, 90.0),
    'joint_3': ('link_2', 'link_3', (0.0, 1.0, 0.0), -135.0, 135.0),
    'joint_4': ('link_3', 'link_4', (0.0, 1.0, 0.0), -180.0, 180.0),
}

EXPECTED_LINKS = {'world', 'base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'tool0'}
PHYSICAL_LINKS = EXPECTED_LINKS - {'world', 'tool0'}


def load_robot(mappings=None):
    doc = xacro.process_file(XACRO_FILE, mappings=mappings or {})
    return ET.fromstring(doc.toxml())


@pytest.fixture(scope='module')
def robot():
    return load_robot()


def floats(text):
    return tuple(float(v) for v in text.split())


def test_robot_name(robot):
    assert robot.get('name') == 'industrial_4dof_arm'


def test_links_present(robot):
    assert {link.get('name') for link in robot.findall('link')} == EXPECTED_LINKS


def test_physical_links_have_visual_collision_inertial(robot):
    for link in robot.findall('link'):
        if link.get('name') not in PHYSICAL_LINKS:
            continue
        assert link.find('visual') is not None, link.get('name')
        assert link.find('collision') is not None, link.get('name')
        inertial = link.find('inertial')
        assert inertial is not None, link.get('name')
        assert float(inertial.find('mass').get('value')) > 0.0
        inertia = inertial.find('inertia')
        ixx, iyy, izz = (float(inertia.get(k)) for k in ('ixx', 'iyy', 'izz'))
        assert min(ixx, iyy, izz) > 0.0
        # Triangle inequality must hold for a physically valid inertia tensor.
        assert ixx + iyy >= izz and iyy + izz >= ixx and ixx + izz >= iyy


def test_revolute_joints(robot):
    joints = {j.get('name'): j for j in robot.findall('joint')}
    for name, (parent, child, axis, lower_deg, upper_deg) in EXPECTED_JOINTS.items():
        joint = joints[name]
        assert joint.get('type') == 'revolute'
        assert joint.find('parent').get('link') == parent
        assert joint.find('child').get('link') == child
        assert floats(joint.find('axis').get('xyz')) == axis
        limit = joint.find('limit')
        assert math.isclose(float(limit.get('lower')), math.radians(lower_deg))
        assert math.isclose(float(limit.get('upper')), math.radians(upper_deg))
        assert float(limit.get('effort')) > 0.0
        assert float(limit.get('velocity')) > 0.0
        assert joint.find('dynamics') is not None


def test_fixed_joints(robot):
    joints = {j.get('name'): j for j in robot.findall('joint')}
    assert joints['world_to_base'].get('type') == 'fixed'
    assert joints['world_to_base'].find('parent').get('link') == 'world'
    assert joints['link_4_to_tool0'].get('type') == 'fixed'
    assert joints['link_4_to_tool0'].find('child').get('link') == 'tool0'


def test_single_rooted_tree(robot):
    children = [j.find('child').get('link') for j in robot.findall('joint')]
    assert len(children) == len(set(children)), 'a link has more than one parent'
    roots = EXPECTED_LINKS - set(children)
    assert roots == {'world'}


def test_wrist_roll_variant():
    robot = load_robot({'wrist_axis': 'roll'})
    joint_4 = [j for j in robot.findall('joint') if j.get('name') == 'joint_4'][0]
    assert floats(joint_4.find('axis').get('xyz')) == (1.0, 0.0, 0.0)
