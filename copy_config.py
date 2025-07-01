import os
import shutil

try:
    from SCons.Script import Import  # type: ignore
    Import("env")
    project_dir = env["PROJECT_DIR"]
except Exception:
    project_dir = os.path.dirname(os.path.abspath(__file__))

config_example = os.path.join(project_dir, 'SRC', 'ShineWiFi-ModBus', 'Config.h.example')
config_target = os.path.join(project_dir, 'SRC', 'ShineWiFi-ModBus', 'Config.h')


def running_in_codex() -> bool:
    return any(key.startswith("CODEX_") for key in os.environ)


def main() -> None:
    if not running_in_codex():
        return

    if not os.path.isfile(config_target) and os.path.isfile(config_example):
        shutil.copy(config_example, config_target)
        print('copy_config.py: copied Config.h.example to Config.h')


main()

