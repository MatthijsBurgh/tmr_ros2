#! /usr/bin/env python3
import argparse
import sys
from collections.abc import Iterable
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from tm_description._modify_urdf import urdf_DH_from_tm_DH, xyzrpys_from_urdf_DH
from tm_msgs.srv import AskItem


class GenKinematicsYaml(Node):
    def __init__(self) -> None:
        super().__init__("gen_kinematics_yaml")

        self.ask_item = self.create_client(AskItem, "ask_item")
        if not self.ask_item.wait_for_service(3.0):
            raise RuntimeError("No AskItem service")

    def call_ask_item(self, req_item: str) -> str:
        req = AskItem.Request()
        req.wait_time = 1.0
        req.id = req_item.replace("_", "")
        req.item = req_item
        future = self.ask_item.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res: AskItem.Response = future.result()
        if not res.ok:
            raise RuntimeError(f"AskItem service call failed for id: {req.id}, item: {req.item}")

        return res.value

    def run(self, tm_type: str, output_file: str = "") -> None:
        """Generate a kinematics YAML file from the robot's calibrated DH parameters.

        The output YAML can be passed to tm.urdf.xacro via the kinematics_params argument,
        overriding the default nominal kinematics for the selected robot type.
        """
        res_dh = self.call_ask_item("DHTable")
        if not res_dh.startswith("DHTable={") or not res_dh.endswith("}"):
            msg = f"Invalid DHTable response, expected DHTable={{...}}, got: {res_dh}"
            raise ValueError(msg)

        res_dd = self.call_ask_item("DeltaDH")
        if not res_dd.startswith("DeltaDH={") or not res_dd.endswith("}"):
            msg = f"Invalid DeltaDH response, expected DeltaDH={{...}}, got: {res_dd}"
            raise ValueError(msg)

        self.get_logger().info("Received kinematics parameters from TM Robot")

        dh_strs = res_dh[9:-1].split(",")
        dd_strs = res_dd[9:-1].split(",")

        if len(dh_strs) != 42:
            msg = (
                f"Invalid DHTable length: expected 42 values "
                f"(7 parameters for each of the 6 joints), got {len(dh_strs)}"
            )
            raise ValueError(msg)

        if len(dd_strs) != 30:
            msg = (
                f"Invalid DeltaDH length: expected 30 values "
                f"(5 parameters for each of the 6 joints), got {len(dd_strs)}"
            )
            raise ValueError(msg)

        dh = list(map(float, dh_strs))
        dd = list(map(float, dd_strs))

        udh = urdf_DH_from_tm_DH(dh, dd)
        xyzs, rpys = xyzrpys_from_urdf_DH(udh)

        if not output_file:
            res = self.call_ask_item("ControlBox_SN")
            if not res.startswith('ControlBox_SN="') or not res.endswith('"'):
                msg = f'Invalid ControlBox_SN response, expected ControlBox_SN="...", got: {res}'
                raise ValueError(msg)
            sn = res[15:-1].lower()
            output_file = f"{sn}_kinematics.yaml"

        file_out = Path.cwd() / output_file

        lines = [f"# Calibrated kinematics parameters for {tm_type}", ""]
        lines.append("kinematics:")

        for i, name in enumerate(["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]):
            lines.append(f"  {name}:")
            lines.append(f"    x: {np.round(xyzs[i, 0], 8)}")
            lines.append(f"    y: {np.round(xyzs[i, 1], 8)}")
            lines.append(f"    z: {np.round(xyzs[i, 2], 8)}")
            lines.append(f"    roll: {np.round(rpys[i, 0], 8)}")
            lines.append(f"    pitch: {np.round(rpys[i, 1], 8)}")
            lines.append(f"    yaw: {np.round(rpys[i, 2], 8)}")

        lines.append("  base_fixed_joint:")
        lines.append("    x: 0.0")
        lines.append("    y: 0.0")
        lines.append("    z: 0.0")
        lines.append("    roll: 0.0")
        lines.append("    pitch: 0.0")
        lines.append("    yaw: 0.0")

        lines.append("  flange_fixed_joint:")
        lines.append(f"    x: {np.round(xyzs[6, 0], 8)}")
        lines.append(f"    y: {np.round(xyzs[6, 1], 8)}")
        lines.append(f"    z: {np.round(xyzs[6, 2], 8)}")
        lines.append(f"    roll: {np.round(rpys[6, 0], 8)}")
        lines.append(f"    pitch: {np.round(rpys[6, 1], 8)}")
        lines.append(f"    yaw: {np.round(rpys[6, 2], 8)}")

        file_out.write_text("\n".join(lines) + "\n")

        self.get_logger().info(f"Kinematics YAML saved to: {file_out}")
        self.get_logger().info(
            f"Usage: xacro tm.urdf.xacro tm_type:={tm_type} kinematics_params:={file_out}"
        )


def main(args: Iterable | None = None) -> int:
    from rclpy.executors import ExternalShutdownException

    parser = argparse.ArgumentParser(
        description=(
            "Generate a calibrated kinematics YAML file from the connected TM Robot. "
            "The output YAML can be used with tm.urdf.xacro via the kinematics_params argument."
        )
    )
    parser.add_argument(
        "tm_type",
        type=str,
        help="Robot model type (e.g. tm5s, tm12s, tm25s)",
    )
    parser.add_argument(
        "--output",
        "-o",
        dest="output_file",
        type=str,
        default="",
        help="Output YAML file path. Defaults to <serial_number>_kinematics.yaml in the current directory.",
    )
    parsed_args = parser.parse_args(remove_ros_args(args)[1:])

    rclpy.init(args=args)
    try:
        node = GenKinematicsYaml()
        node.run(**vars(parsed_args))
    except (RuntimeError, ValueError) as e:
        node.get_logger().error(f"Failed to generate kinematics YAML:\n{e}")
        return 1
    except KeyboardInterrupt:
        return 0
    except ExternalShutdownException:
        return 1
    finally:
        rclpy.try_shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
