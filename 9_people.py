#!/usr/bin/env python
"""
| File: 9_people.py
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API to run a simulation
| where people move around in the world.
"""

# Imports to start Isaac Sim from this script
import carb

from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
    # Disable unnecessary windows
    "hide_ui": False,  # We need some UI for viewports
    "active_gpu": 0,
    # Minimal UI configuration
    "window/dockPreference": {
        "omni.kit.window.property": "left",
        "omni.kit.window.console": "bottom",
        "omni.kit.viewport.window": "main",
    }
})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World
from isaacsim.core.utils.extensions import enable_extension


# Enable/disable ROS bridge extensions to keep only ROS2 Bridge
enable_extension("isaacsim.ros2.bridge")
# enable_extension("foxglove.tools.ws_bridge")
# enable_extension("omni.kit.livestream.webrtc")

# Update the simulation app with the new extensions
simulation_app.update()

# -------------------------------------------------------------------------------------------------
# These lines are needed to restart the USD stage and make sure that the people extension is loaded
# -------------------------------------------------------------------------------------------------
import omni.usd
omni.usd.get_context().new_stage()

import numpy as np

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.people.person import Person
from pegasus.simulator.logic.people.person_controller import PersonController
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Example controller class that make a person move in a circle around the origin of the world
# Note: You could create a different controller with a different behaviour. For instance, you could:
# 1. read the keyboard input to move the person around the world.
# 2. read the target position from a ros topic,
# 3. read the target position from a file,
# 4. etc.
class CirclePersonController(PersonController):

    def __init__(self):
        super().__init__()

        self._radius = 5.0
        self.gamma = 0.0
        self.gamma_dot = 0.3
        
    def update(self, dt: float):

        # Update the reference position for the person to track
        self.gamma += self.gamma_dot * dt
        
        # Set the target position for the person to track
        self._person.update_target_position([self._radius * np.cos(self.gamma), self._radius * np.sin(self.gamma), 0.0])
        

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

# -------------------------------------------------------------------------------------------------
# Define the PegasusApp class where the simulation will be run
# -------------------------------------------------------------------------------------------------
class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics,
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        #self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        self.pg.load_asset(SIMULATION_ENVIRONMENTS["Curved Gridroom"], "/World/layout")

        # Check the available assets for people
        people_assets_list = Person.get_character_asset_list()
        for person in people_assets_list:
            print(person)

        # Create the controller to make on person walk around in circles
        person_controller = CirclePersonController()
        p1 = Person("person1", "original_male_adult_construction_05", init_pos=[3.0, 0.0, 0.0], init_yaw=1.0, controller=person_controller)
        
        # Create a person without setting up a controller, and just setting a manual target position for it to track
        p2 = Person("person2", "original_female_adult_business_02", init_pos=[2.0, 0.0, 0.0])
        p2.update_target_position([10.0, 0.0, 0.0], 1.0)

        config_multirotor = MultirotorConfig()
        # Create the multirotor configuration
        import os
        # Use host path when running outside Docker
        px4_path = os.path.expanduser("~/PX4-Autopilot") if os.path.exists(os.path.expanduser("~/PX4-Autopilot")) else "/workspace/PX4-Autopilot"
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": px4_path
        })

        config_multirotor.backends = [
            PX4MavlinkBackend(mavlink_config),
            ROS2Backend(vehicle_id=1,
                config={
                    "namespace": 'drone',
                    "pub_sensors": True,
                    "pub_graphical_sensors": True,
                    "pub_state": True,
                    "pub_tf": True,
                    "sub_control": False,})]
        
        config_multirotor.graphical_sensors = [MonocularCamera("camera", config={"update_rate": 60.0})]
        
        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # After the Multirotor is created, add TF publisher
        from pegasus.simulator.logic.backends import Backend

        class TFPublisher(Backend):
            def __init__(self):
                super().__init__()
                
            def initialize(self, vehicle):
                # Publish static TF frame
                import subprocess
                subprocess.Popen([
                    "bash", "-c",
                    "source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map drone1/base_link"
                ])

        # Set the camera of the viewport to a nice position
        self.pg.set_viewport_camera([5.0, 9.0, 6.5], [0.0, 0.0, 0.0])

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Setup dual viewport layout (main view + drone camera)
        self._setup_viewports()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def _setup_viewports(self):
        """Setup minimal UI with 2 viewports: main camera and drone camera feed"""
        import omni.ui as ui
        from omni.kit.viewport.utility import get_active_viewport_window, create_viewport_window

        # Close unnecessary windows
        windows_to_close = [
            "Stage",
            "Layer",
            "Content",
            "Render Settings",
            "Property",
            "Semantics Schema Editor",
            "Extensions"
        ]

        for window_name in windows_to_close:
            try:
                window = ui.Workspace.get_window(window_name)
                if window:
                    window.visible = False
            except:
                pass

        # Create drone camera viewport
        try:
            # Create second viewport for drone camera
            drone_viewport = create_viewport_window("Drone Camera")

            # Set the drone camera as active camera for this viewport
            # The camera path is: /World/quadrotor/camera
            import omni.kit.viewport.utility as vp_util
            drone_viewport.viewport_api.set_active_camera("/World/quadrotor/camera")

            # Position viewports side by side using QuickLayout
            from omni.kit.quicklayout import QuickLayout
            QuickLayout.set_layout({
                "Viewport": {"dock_id": "left", "width": 0.5},
                "Drone Camera": {"dock_id": "right", "width": 0.5},
                "Console": {"dock_id": "bottom", "height": 0.2}
            })

            print("✅ Dual viewport setup complete: Main view + Drone camera")
        except Exception as e:
            print(f"⚠️  Could not setup dual viewport layout: {e}")
            print("   Using default single viewport")

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
