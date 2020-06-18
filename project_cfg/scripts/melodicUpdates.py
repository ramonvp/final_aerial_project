# importing the modules
import os, sys
from pathlib import Path
import shutil

if __name__ == '__main__':

    current_path = Path(sys.path[0]).absolute()
    catkin_src_path = current_path.parent.parent.parent

    # Husky files
    file = "localization.yaml"
    originPath = catkin_src_path / "aerial_project/project_cfg/Husky_updateFiles"
    destPath = catkin_src_path / "husky/husky_control/config"
    shutil.copyfile(str((originPath / file).absolute()), str((destPath / file).absolute()))

    file = "twist_mux.yaml"
    originPath = catkin_src_path / "aerial_project/project_cfg/Husky_updateFiles"
    destPath = catkin_src_path / "husky/husky_control/config"
    shutil.copyfile(str((originPath / file).absolute()), str((destPath / file).absolute()))

    file = "spawn_husky.launch"
    originPath = catkin_src_path / "aerial_project/project_cfg/Husky_updateFiles"
    destPath = catkin_src_path / "husky/husky_gazebo/launch"
    shutil.copyfile(str((originPath / file).absolute()), str((destPath / file).absolute()))

    file = "control.launch"
    originPath = catkin_src_path / "aerial_project/project_cfg/Husky_updateFiles"
    destPath = catkin_src_path / "husky/husky_control/launch"
    shutil.copyfile(str((originPath / file).absolute()), str((destPath / file).absolute()))

    # RotorS files
    file = "component_snippets.xacro"
    originPath = catkin_src_path / "aerial_project/project_cfg/rotorS_updateFiles"
    destPath = catkin_src_path / "rotors_simulator/rotors_description/urdf"
    shutil.copyfile(str((originPath / file).absolute()), str((destPath / file).absolute()))

    file = "CMakeLists.txt"
    originPath = catkin_src_path / "aerial_project/project_cfg/rotorS_updateFiles"
    destPath = catkin_src_path / "rotors_simulator/rotors_gazebo_plugins"
    shutil.copyfile(str((originPath / file).absolute()), str((destPath / file).absolute()))

    #place CATKIN_IGNORE on minkndr_python
    file = "CATKIN_IGNORE"
    originPath = catkin_src_path / "aerial_project/project_cfg"
    destPath = catkin_src_path / "minkindr/minkindr_python"
    shutil.copyfile(str((originPath / file).absolute()), str((destPath / file).absolute()))







