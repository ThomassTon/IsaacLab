# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates different single-arm manipulators.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p source/standalone/demos/tiago.py

"""

"""Launch Isaac Sim Simulator first."""

import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different single-arm manipulators.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import numpy as np
import torch

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

##
# Pre-defined configs
##
# isort: off
from omni.isaac.lab_assets.tiago import (
    TIAGO_CFG,
)

# isort: on


def define_origins(num_origins: int, spacing: float) -> list[list[float]]:
    """Defines the origins of the the scene."""
    # create tensor based on number of environments
    env_origins = torch.zeros(num_origins, 3)
    # create a grid of origins
    num_rows = np.floor(np.sqrt(num_origins))
    num_cols = np.ceil(num_origins / num_rows)
    xx, yy = torch.meshgrid(torch.arange(num_rows), torch.arange(num_cols), indexing="xy")
    env_origins[:, 0] = spacing * xx.flatten()[:num_origins] - spacing * (num_rows - 1) / 2
    env_origins[:, 1] = spacing * yy.flatten()[:num_origins] - spacing * (num_cols - 1) / 2
    env_origins[:, 2] = 0.0
    # return the origins
    return env_origins.tolist()


def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Create separate groups called "Origin1", "Origin2", "Origin3"
    # Each group will have a mount and a robot on top of it
    origins = define_origins(num_origins=6, spacing=2.0)

    # Origin 1 with Franka Panda
    prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])
    # -- Table
    # cfg = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd")
    # cfg.func("/World/Origin1/Table", cfg, translation=(0.55, 0.0, 1.05))
    # -- Robot
    tiago_cfg = TIAGO_CFG.replace(prim_path="/World/Origin1/Robot")
    tiago = Articulation(cfg=tiago_cfg)

    # return the scene information
    scene_entities = {
        "franka_panda": tiago,
    }
    return scene_entities, origins


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    # Simulate physics
    while simulation_app.is_running():
        # reset
        # if count % 200 == 0:
        #     # reset counters
        #     sim_time = 0.0
        #     count = 0
        #     # reset the scene entities
        #     for index, robot in enumerate(entities.values()):
        #         # root state
        #         # root_state = robot.data.default_root_state.clone()
        #         # print(root_state)
        #         # root_state[:, :3] += origins[index]
        #         # robot.write_root_state_to_sim(root_state)
        #         # # set joint positions
        #         # joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
        #         # robot.write_joint_state_to_sim(joint_pos, joint_vel)
        #         # # clear internal buffers
        #         # robot.reset()
        #     # print("[INFO]: Resetting robots state...")
        # # apply random actions to the robots
        for robot in entities.values():
            # generate random joint positions
            joint_pos_target = robot.data.default_joint_pos[:,:] #+ torch.randn_like(robot.data.joint_pos[:,4:]) * 0.1
            joint_pos_target = joint_pos_target.clamp_(
                robot.data.soft_joint_pos_limits[:,:, 0], robot.data.soft_joint_pos_limits[:,:, 1]
            )
            # joint_pos_target[:,0:4] = torch.tensor([0.0, 0.0, 0.0, 0.35], device=joint_pos_target.device)
            # joint_pos_target = robot.data.default_joint_pos
            # print(joint_pos_target[:,10:12])
            
            if robot.data.joint_pos[:,-1:] > 0.039:
                joint_pos_target[:,-4:]=0.0
            elif robot.data.joint_pos[:,-1:] < 0.0001:
                joint_pos_target[:,-4:]=0.04
            
            joint_pos_target[:,10:13] += torch.tensor([0.0001, 0.0001, 0.0001], device=joint_pos_target.device)
            joint_vel_target = robot.data.default_joint_vel
            # print("dd: ",joint_pos_target)
            print(robot.find_joints("arm_left_5_joint"))
            joint_pos_target = joint_pos_target.clamp_(
                robot.data.soft_joint_pos_limits[:,:, 0], robot.data.soft_joint_pos_limits[:,:, 1]
            )
            robot.data.joint_pos
            # apply action to the robot
            # robot.set_joint_position_target(joint_pos_target,joint_ids=slice(0,24))
            robot.set_joint_position_target(joint_pos_target)
            robot.set_joint_velocity_target(joint_vel_target)
            # robot.write_joint_state_to_sim(joint_pos_target, joint_vel_target)
            # write data to sim
            robot.write_data_to_sim()
            
            # print(joint_vel_target)
        # perform step
        
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        for robot in entities.values():
            robot.update(sim_dt)


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg()
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
