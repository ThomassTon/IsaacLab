# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the TIAGO robots.

The following configurations are available:

* :obj:`FRANKA_PANDA_CFG`: Franka Emika Panda robot with Panda hand
* :obj:`FRANKA_PANDA_HIGH_PD_CFG`: Franka Emika Panda robot with Panda hand with stiffer PD control

Reference: https://github.com/pal-robotics/tiago_robot
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg, DCMotorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

TIAGO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="usd/tiago_dual_omni_pal_screen/tiago_dual_omni_pal_screen.usd",                                      #"tiago_dual_omni_pal_screen/tiago_dual_omni_pal_screen.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=10.0,
            # enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=1
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            # 'base_x': 0.0, 
            # 'base_y': 0.0, 
            # 'base_r': 0.0, 
            'torso_lift_joint':0.35, 
            'arm_left_1_joint':0.0, 
            'arm_right_1_joint':0.0, 
            'head_1_joint':0.3, 
            'arm_left_2_joint':0.0, 
            'arm_right_2_joint':0.0, 
            'head_2_joint':0.2, 
            'arm_left_3_joint':0, 
            'arm_right_3_joint':0, 
            'arm_left_4_joint':0.5, 
            'arm_right_4_joint':0.3, 
            'arm_left_5_joint':0.2, 
            'arm_right_5_joint':0.7, 
            'arm_left_6_joint':0.2, 
            'arm_right_6_joint':0.5, 
            'arm_left_7_joint':-0.3, 
            'arm_right_7_joint':0.2, 
            'gripper_left_left_finger_joint':0.04, 
            'gripper_left_right_finger_joint':0.04, 
            'gripper_right_left_finger_joint':0.04, 
            'gripper_right_right_finger_joint':0.04
        },
        joint_vel={
            'base_x': 0.0, 
            'base_y': 0.0, 
            'base_r': 0.0,   

        },
        
    ),
    actuators={
        "torso_lift_joint": ImplicitActuatorCfg(
            joint_names_expr=["torso_lift_joint"],
            # effort_limit=2000.0,
            velocity_limit=2.175,
            stiffness=2e10,
            damping=None,
        ),
        "base": ImplicitActuatorCfg(
            joint_names_expr=["base.*"],
            effort_limit=500.0,
            velocity_limit=None,
            stiffness=None,
            damping=10,
            # saturation_effort = 500
        ),

        "tiago_arm": ImplicitActuatorCfg(
            joint_names_expr=["arm.*"],
            # effort_limit=50,
            # velocity_limit=2.61,
            stiffness=600.0,
            damping=None,
        ),
      
        "tiago_hand": ImplicitActuatorCfg(
            joint_names_expr=["gripper.*"],
            # effort_limit=10.0,
            # velocity_limit=0.2,
            stiffness=2e3,
            damping=1e2,
        ),
        
        # "tiago_hand_left": ImplicitActuatorCfg(
        #     joint_names_expr=["gripper.*"],
        #     # effort_limit=10.0,
        #     # velocity_limit=0.2,
        #     stiffness=2e3,
        #     damping=1e2,
        # ),
        
        "tiago_head": ImplicitActuatorCfg(
            joint_names_expr=["head.*"],
            # effort_limit=10.0,
            # velocity_limit=0.2,
            stiffness=2e3,
            damping=1e2,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Franka Emika Panda robot."""


# FRANKA_PANDA_HIGH_PD_CFG = FRANKA_PANDA_CFG.copy()
# FRANKA_PANDA_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_shoulder"].stiffness = 400.0
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_shoulder"].damping = 80.0
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_forearm"].stiffness = 400.0
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_forearm"].damping = 80.0

"""Configuration of Franka Emika Panda robot with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""

