#!/usr/bin/env python3
"""
CLI entry point for the Linorobot2 AI Robot Configuration Engine.
Usage:
    python3 generate_config.py spec.json --out-dir ./output/
    python3 generate_config.py spec.json --merge --commit
"""

import os
import sys
import json
import math
import subprocess
import argparse
from validator import validate_robot_spec
from generator import generate_cpp_header, generate_platformio_env, generate_urdf_xacro


def merge_configuration(spec, repo_root):
    robot_name = spec["robot_name"]
    robot_name_upper = robot_name.upper()
    header_code = generate_cpp_header(spec)
    pio_code = generate_platformio_env(spec)
    urdf_code = generate_urdf_xacro(spec)

    print(f"\nMerging configuration for '{robot_name}' into repository root: {repo_root}")

    # 1. Custom Header
    custom_dir = os.path.join(repo_root, "config", "custom")
    os.makedirs(custom_dir, exist_ok=True)
    header_path = os.path.join(custom_dir, f"{robot_name}_config.h")
    with open(header_path, "w") as f:
        f.write(header_code)
    print(f" ✅ Written header: {header_path}")

    # 2. Register in config/config.h
    config_h_path = os.path.join(repo_root, "config", "config.h")
    if os.path.exists(config_h_path):
        with open(config_h_path, "r") as f:
            config_h_content = f.read()

        macro_name = f"USE_{robot_name_upper}_CONFIG"
        if macro_name not in config_h_content:
            inclusion_block = f"#ifdef {macro_name}\n    #include \"custom/{robot_name}_config.h\"\n#endif\n\n"
            barrier = "// add user configurations above this line"
            if barrier in config_h_content:
                config_h_content = config_h_content.replace(barrier, inclusion_block + barrier)
            else:
                config_h_content += "\n" + inclusion_block

            with open(config_h_path, "w") as f:
                f.write(config_h_content)
            print(f" ✅ Registered '{macro_name}' in config/config.h")
        else:
            print(f" ℹ️  '{macro_name}' already registered in config/config.h")

    # 3. PlatformIO environment injection
    pio_targets = [
        os.path.join(repo_root, "firmware", "platformio.ini"),
        os.path.join(repo_root, "calibration", "platformio.ini"),
        os.path.join(repo_root, "test_sensors", "platformio.ini")
    ]
    env_header = f"[env:{robot_name}]"
    for pio_path in pio_targets:
        if os.path.exists(pio_path):
            with open(pio_path, "r") as f:
                pio_content = f.read()
            if env_header not in pio_content:
                pio_content += f"\n{pio_code}\n"
                with open(pio_path, "w") as f:
                    f.write(pio_content)
                print(f" ✅ Appended '{env_header}' to {os.path.relpath(pio_path, repo_root)}")
            else:
                print(f" ℹ️  '{env_header}' already exists in {os.path.relpath(pio_path, repo_root)}")

    # 4. URDF Generation
    urdf_dir = os.path.join(repo_root, "urdf")
    os.makedirs(urdf_dir, exist_ok=True)
    urdf_path = os.path.join(urdf_dir, f"{robot_name}_properties.urdf.xacro")
    with open(urdf_path, "w") as f:
        f.write(urdf_code)
    print(f" ✅ Written URDF: {urdf_path}")


def commit_configuration(spec, repo_root, branch_name=None, commit_msg=None):
    robot_name = spec["robot_name"]
    branch = branch_name or f"config/{robot_name}"
    msg = commit_msg or f"feat(config): add configuration for {robot_name}"

    try:
        # Create or checkout branch
        subprocess.run(["git", "checkout", "-b", branch], cwd=repo_root, capture_output=True, check=False)
        subprocess.run(["git", "checkout", branch], cwd=repo_root, capture_output=True, check=False)

        # Stage files
        subprocess.run(["git", "add", "config/", "firmware/platformio.ini", "calibration/platformio.ini", "test_sensors/platformio.ini", "urdf/"], cwd=repo_root, check=True)

        # Commit
        res = subprocess.run(["git", "commit", "-m", msg], cwd=repo_root, capture_output=True, text=True)
        if res.returncode == 0:
            print(f"\n✅ Successfully committed configuration to Git branch '{branch}':\n   {msg}")
        else:
            print(f"\nℹ️  Git status: {res.stdout.strip() or res.stderr.strip()}")
    except Exception as e:
        print(f"\n⚠️  Git commit failed: {e}")


def main():
    parser = argparse.ArgumentParser(description="Linorobot2 Hardware Rule Validator & Code Generator")
    parser.add_argument("spec_file", help="Path to input JSON specification file")
    parser.add_argument("--out-dir", help="Output directory for generated files", default=None)
    parser.add_argument("--merge", action="store_true", help="Merge generated header, platformio.ini, and URDF directly into codebase")
    parser.add_argument("--commit", action="store_true", help="Automatically create Git commit with merged configuration")
    parser.add_argument("--repo-root", help="Repository root path (defaults to auto-detected root)", default=None)
    args = parser.parse_args()

    if not os.path.exists(args.spec_file):
        print(f"Error: Specification file '{args.spec_file}' not found.")
        sys.exit(1)

    with open(args.spec_file, "r") as f:
        try:
            spec = json.load(f)
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON syntax in '{args.spec_file}': {e}")
            sys.exit(1)

    print(f"\n==========================================")
    print(f" Validating: {spec.get('robot_name', 'Unknown Robot')}")
    print(f"==========================================")

    valid, errors, stats = validate_robot_spec(spec)

    for err in errors:
        prefix = "❌" if err.level == "ERROR" else "⚠️ "
        print(f"{prefix} [{err.level}] {err.field}: {err.message}")

    if not valid:
        print("\n❌ Hardware validation FAILED. Fix errors before generating code.")
        sys.exit(1)

    print("\n✅ Hardware rule validation PASSED!")
    print("\nKinematics & Performance Summary:")
    print(f" - Wheel Circumference: {stats.get('wheel_circumference_m')} m")
    print(f" - Max Linear Velocity (85% headroom): {stats.get('max_linear_speed_m_s')} m/s ({round(stats.get('max_linear_speed_m_s', 0)*3.6, 2)} km/h)")
    print(f" - Max Angular Velocity: {stats.get('max_angular_speed_rad_s')} rad/s ({round(math.degrees(stats.get('max_angular_speed_rad_s', 0)), 1)} deg/s)" if "max_angular_speed_rad_s" in stats else "")
    print(f" - Ticks Per Meter: {stats.get('ticks_per_meter')} ticks/m")

    header_code = generate_cpp_header(spec)
    pio_code = generate_platformio_env(spec)
    urdf_code = generate_urdf_xacro(spec)

    repo_root = args.repo_root or os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

    if args.merge:
        merge_configuration(spec, repo_root)

    if args.commit:
        if not args.merge:
            merge_configuration(spec, repo_root)
        commit_configuration(spec, repo_root)

    if args.out_dir:
        os.makedirs(args.out_dir, exist_ok=True)
        robot_name = spec["robot_name"]

        header_path = os.path.join(args.out_dir, f"{robot_name}_config.h")
        pio_path = os.path.join(args.out_dir, "platformio_section.ini")
        urdf_path = os.path.join(args.out_dir, f"{robot_name}_properties.urdf.xacro")

        with open(header_path, "w") as f:
            f.write(header_code)
        with open(pio_path, "w") as f:
            f.write(pio_code)
        with open(urdf_path, "w") as f:
            f.write(urdf_code)

        print(f"\nGenerated Artifacts in '{args.out_dir}':")
        print(f" 1. C++ Header: {header_path}")
        print(f" 2. PlatformIO Env: {pio_path}")
        print(f" 3. URDF Description: {urdf_path}")
    elif not args.merge and not args.commit:
        print("\n--- C++ Header Preview ---")
        print(header_code)
        print("\n--- PlatformIO Environment Preview ---")
        print(pio_code)


if __name__ == "__main__":
    main()
