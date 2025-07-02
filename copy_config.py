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


def running_in_codex() -> bool:
    return any(key.startswith("CODEX_") for key in os.environ)


def copy_config() -> None:
    if os.path.isfile(config_example) and not os.path.isfile(config_target):
        shutil.copy(config_example, config_target)
        print('copy_config.py: copied Config.h.example to Config.h')


def remove_config(*_args: object, **_kwargs: object) -> None:
    if os.path.isfile(config_target):
        os.remove(config_target)
        print('copy_config.py: removed Config.h')


def main() -> None:
    if not running_in_codex():
        return

    copy_config()

    if env is not None:
        env.AddPostAction("buildprog", remove_config)


main()

