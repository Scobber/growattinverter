import os
import shutil

try:
    from SCons.Script import Import  # type: ignore
    Import("env")
    project_dir = env["PROJECT_DIR"]
except Exception:
    env = None
    project_dir = os.path.dirname(os.path.abspath(__file__))

src_dir = os.path.join(project_dir, 'SRC', 'ShineWiFi-ModBus')
config_example = os.path.join(src_dir, 'Config.h.example')
config_target = os.path.join(src_dir, 'Config.h')
version_target = os.path.join(src_dir, 'version.h')


def copy_config() -> None:
    if os.path.isfile(config_example) and not os.path.isfile(config_target):
        shutil.copy(config_example, config_target)
        print('copy_config.py: copied Config.h.example to Config.h')


def write_version() -> None:
    version = os.environ.get('FW_VERSION', 'dev')
    content = f'#pragma once\n#define FW_VERSION "{version}"\n'
    with open(version_target, 'w') as f:
        f.write(content)
    print(f'copy_config.py: wrote version.h with FW_VERSION="{version}"')


def main() -> None:
    copy_config()
    write_version()


main()
