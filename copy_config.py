import os
import shutil

try:
    from SCons.Script import Import  # type: ignore
    Import("env")
    project_dir = env["PROJECT_DIR"]
except Exception:
    env = None
    project_dir = os.path.dirname(os.path.abspath(__file__))

config_example = os.path.join(project_dir, 'SRC', 'ShineWiFi-ModBus', 'Config.h.example')
config_target = os.path.join(project_dir, 'SRC', 'ShineWiFi-ModBus', 'Config.h')


def copy_config() -> None:
    if os.path.isfile(config_example) and not os.path.isfile(config_target):
        shutil.copy(config_example, config_target)
        print('copy_config.py: copied Config.h.example to Config.h')


def main() -> None:
    copy_config()


main()
