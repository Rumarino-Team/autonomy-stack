#!/usr/bin/env python3
import argparse
import yaml


def fmt_row(values):
    return ", ".join(f"{float(v):.9g}" for v in values)


def main():
    parser = argparse.ArgumentParser(description="Extract T_cam_imu from Kalibr camchain and print ORB-SLAM matrix block.")
    parser.add_argument("camchain", help="Path to camchain-imucam.yaml produced by kalibr_calibrate_imu_camera")
    parser.add_argument("--camera", default="cam0", help="Camera key in camchain file (default: cam0)")
    args = parser.parse_args()

    with open(args.camchain, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if args.camera not in data:
        raise KeyError(f"Camera key '{args.camera}' not found. Available keys: {list(data.keys())}")

    t_cam_imu = data[args.camera].get("T_cam_imu")
    if t_cam_imu is None:
        raise KeyError(f"T_cam_imu not found under '{args.camera}'")

    if len(t_cam_imu) != 4 or any(len(r) != 4 for r in t_cam_imu):
        raise ValueError("T_cam_imu must be a 4x4 matrix")

    print("IMU.T_b_c1: !!opencv-matrix")
    print("   rows: 4")
    print("   cols: 4")
    print("   dt: f")
    print("   data: [" + fmt_row(t_cam_imu[0]) + ",")
    print("          " + fmt_row(t_cam_imu[1]) + ",")
    print("          " + fmt_row(t_cam_imu[2]) + ",")
    print("          " + fmt_row(t_cam_imu[3]) + "]")
    print()
    print("# For compatibility with your ORB-SLAM config, copy the same matrix into Tbc as well.")


if __name__ == "__main__":
    main()
