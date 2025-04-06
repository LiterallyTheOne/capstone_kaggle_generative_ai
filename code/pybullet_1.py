"""
Pybullet test 1
"""

import time

import pybullet as p
import pybullet_data

import numpy as np


def load_box(position, size=None, mass=1.0):
    """load the box object

    :param position: _description_
    :type position: _type_
    :param size: _description_, defaults to None
    :type size: _type_, optional
    :param mass: _description_, defaults to 1.0
    :type mass: float, optional
    :return: _description_
    :rtype: _type_
    """
    if size is None:
        size = [0.05, 0.05, 0.05]
    visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=size)
    collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    return p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=position,
    )


def main():
    """main function"""
    # Connect to PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load plane and set gravity
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.81)

    # Load a table
    table_id = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, 0])

    # Load KUKA IIWA (6 DoF robot arm)
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[-0.5, 0, 0])

    # Place a few boxes randomly on the table
    object_ids = []
    for i in range(3):
        x = 0.5 + (i * 0.1)
        y = -0.1 + i * 0.1
        z = 0.65  # height of table is ~0.62, so a bit above
        obj_id = load_box([x, y, z])
        object_ids.append(obj_id)

    num_joints = p.getNumJoints(robot_id)

    target_position = [0.2, 0.0, 0.7]
    end_effector_index = num_joints - 1

    for step in range(10000):
        # Calculate IK for the target
        joint_positions = p.calculateInverseKinematics(
            robot_id, end_effector_index, target_position
        )

        # Set joint positions
        for joint_index in range(num_joints):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_positions[joint_index],
            )

        p.stepSimulation()
        time.sleep(1.0 / 240)


if __name__ == "__main__":
    main()
