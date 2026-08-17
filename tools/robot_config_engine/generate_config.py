#!/usr/bin/env python3
"""
CLI entry point for the Linorobot2 AI Robot Configuration Engine.
Usage:
    python3 generate_config.py spec.json --out-dir ./output/
"""

import os
import sys
import json
import math
import argparse
from validator import validate_robot_spec
from generator import generate_cpp_header, generate_platformio_env, generate_urdf_xacro


def main():
    parser = argparse.ArgumentParser(description="Linorobot2 Hardware Rule Validator & Code Generator")
    parser.add_argument("spec_file", help="Path to input JSON specification file")
    parser.add_argument("--out-dir", help="Output directory for generated files", default=None)
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
    else:
        print("\n--- C++ Header Preview ---")
        print(header_code)
        print("\n--- PlatformIO Environment Preview ---")
        print(pio_code)


if __name__ == "__main__":
    main()
