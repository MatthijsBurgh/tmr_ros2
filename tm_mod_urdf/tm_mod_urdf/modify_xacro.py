#! /usr/bin/env python3
import argparse
import shutil
import sys
import xml.etree.cElementTree as ET  # noqa: N812, N817
from collections.abc import Iterable
from math import radians
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from tm_mod_urdf._modify_urdf import modify_urdf, pretty_xml, urdf_DH_from_tm_DH, xyzrpys_from_urdf_DH
from tm_msgs.srv import AskItem


class ModifyXacro(Node):
    def __init__(self) -> None:
        super().__init__("modify_xacro")

        self.ask_item = self.create_client(AskItem, "ask_item")
        if not self.ask_item.wait_for_service(3.0):
            raise RuntimeError("No AskItem service")

    def find_tm_description_xacro_path(self) -> Path:
        """Find tm_description/xacro path in the workspace."""
        curr_path = Path(__file__).absolute().parent
        dirs = ["src", "install", "build"]
        idx = -1
        for d in dirs:
            try:
                idx = curr_path.parts.index(d)
            except ValueError:
                continue
        if idx == -1:
            msg = f"Workspace directory not found in path: {curr_path}"
            self.get_logger().error(msg)
            raise RuntimeError(msg)
        src_path = Path(*curr_path.parts[:idx], "src")
        xacro_path = None
        for pkg_dir in src_path.iterdir():
            if pkg_dir.name.endswith("tm_description"):
                xacro_path = pkg_dir / "xacro"
                break

        if xacro_path is None:
            msg = "tm_description package not found in src directory: {}".format(src_path)
            self.get_logger().error(msg)
            raise RuntimeError(msg)

        return xacro_path

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

    def gen_xacro(self, original_model: str, new_model: str, specific_w: str = "") -> None:
        ###############################################################################################
        # example: generate a new_model file (macro.xxxooo.urdf.xacro), base on tm5-900-norminal model
        # syntax : python3 modify_xacro.py original_model new_model
        # [key-in] original_model: tm5-900  , [key-in] new_model: xxxooo
        # [key-in] shell cmd $ python3 modify_xacro.py tm5-900 xxxooo
        ###############################################################################################

        if not new_model:
            res = self.call_ask_item("ControlBox_SN")
            if not res.startswith('ControlBox_SN="') or not res.endswith('"'):
                msg = f'Invalid parameters ControlBox_SN, expected format: ControlBox_SN="...", got: {res}'
                raise ValueError(msg)
            new_model = res[15:-1].lower()
            self.get_logger().info(
                f"No new model name specified, using the serial number as the new model name: {new_model}"
            )

        # specific keyword default
        overwrite = False
        restore: str | None = None
        tm_model = "reference"
        ###############################################################################################
        # You can restore some nominal kinematic parameters by using specific keyword settings
        if new_model == "tm5s-nominal" or specific_w == "-K5S":
            tm_model = restore = "tm5s-nominal"
        elif new_model == "tm5sx-nominal" or specific_w == "-K5SX":
            tm_model = restore = "tm5sx-nominal"
        elif new_model == "tm6s-nominal" or specific_w == "-K6S":
            tm_model = restore = "tm6s-nominal"
        elif new_model == "tm7s-nominal" or specific_w == "-K7S":
            tm_model = restore = "tm7s-nominal"
        elif new_model == "tm7sx-nominal" or specific_w == "-K7SX":
            tm_model = restore = "tm7sx-nominal"
        elif new_model == "tm12s-nominal" or specific_w == "-K12S":
            tm_model = restore = "tm12s-nominal"
        elif new_model == "tm12sx-nominal" or specific_w == "-K12SX":
            tm_model = restore = "tm12sx-nominal"
        elif new_model == "tm14s-nominal" or specific_w == "-K14S":
            tm_model = restore = "tm14s-nominal"
        elif new_model == "tm14sx-nominal" or specific_w == "-K14SX":
            tm_model = restore = "tm14sx-nominal"
        elif new_model == "tm25s-nominal" or specific_w == "-K25S":
            tm_model = restore = "tm25s-nominal"
        elif new_model == "tm25sx-nominal" or specific_w == "-K25SX":
            tm_model = restore = "tm25sx-nominal"
        elif new_model == "tm30s-nominal" or specific_w == "-K30S":
            tm_model = restore = "tm30s-nominal"
        elif new_model == "tm30sx-nominal" or specific_w == "-K30SX":
            tm_model = restore = "tm30sx-nominal"

        if restore:
            self.get_logger().info(f"Notice! You have chosen to restore a {tm_model} xacro model file!")
        if specific_w == "-OW":
            overwrite = True
            self.get_logger().info(f"Notice! You have chosen to overwrite the original {tm_model} file!")
        ###############################################################################################

        res_dh = self.call_ask_item("DHTable")
        if not res_dh.startswith("DHTable={") or not res_dh.endswith("}"):
            msg = f"Invalid parameters DHTable, expected format: DHTable={{...}}, got: {res_dh}"
            raise ValueError(msg)

        res_dd = self.call_ask_item("DeltaDH")
        if not res_dd.startswith("DeltaDH={") or not res_dd.endswith("}"):
            msg = f"Invalid parameters DeltaDH, expected format: DeltaDH={{...}}, got: {res_dd}"
            raise ValueError(msg)

        if not restore or overwrite:
            self.get_logger().info("Loading the correction kinematics parameters from your TM Robot")
            if specific_w == "-VAL":
                self.get_logger().info(res_dh)
                self.get_logger().info(res_dd)

        ###############################################################################################
        # You can restore some nominal kinematic parameters by using specific keyword settings
        if restore == "tm5s-nominal" or restore == "tm5sx-nominal":
            self.get_logger().info("Restore with TM5S(X) nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,148.4,0,-360,360,-90,0,429,0,0,-360,360,0,0,386,0,0,-158,158,90,90,0,-147.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif restore == "tm6s-nominal":
            self.get_logger().info("Restore with TM6S nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,165.2,0,-360,360,-90,0,436.1,0,0,-360,360,0,0,782.5,0,0,-166,166,90,90,0,-181.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif restore == "tm7s-nominal" or restore == "tm7sx-nominal":
            self.get_logger().info("Restore with TM7S(X) nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,148.4,0,-360,360,-90,0,329,0,0,-360,360,0,0,298,0,0,-152,152,90,90,0,-147.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif restore == "tm12s-nominal" or restore == "tm12sx-nominal":
            self.get_logger().info("Restore with TM12S(X) nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,165.2,0,-360,360,-90,0,636.1,0,0,-360,360,0,0,532.4,0,0,-162,162,90,90,0,-181.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif restore == "tm14s-nominal" or restore == "tm14sx-nominal":
            self.get_logger().info("Restore with TM14S(X) nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,165.2,0,-360,360,-90,0,536.1,0,0,-360,360,0,0,432.4,0,0,-159,159,90,90,0,-181.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif restore == "tm25s-nominal" or restore == "tm25sx-nominal":
            self.get_logger().info("Restore with TM25S(X) nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,235.0,0,-360,360,-90,0,950.0,0,0,-360,360,90,90,0,-70.0,0,-166,166,0,-90,0,660.0,0,-360,360,0,90,0,170.2,0,-360,360,0,0,0,152.95,0,-360,360}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif restore == "tm30s-nominal" or restore == "tm30sx-nominal":
            self.get_logger().info("Restore with TM30S(X) nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,235.0,0,-360,360,-90,0,890.0,0,0,-360,360,90,90,0,-70.0,0,-170,170,0,-90,0,800.0,0,-360,360,0,90,0,170.2,0,-360,360,0,0,0,152.95,0,-360,360}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif restore is not None:
            msg = f"Invalid value for nominal restore: {restore}"
            raise ValueError(msg)

        self.get_logger().info(res_dh)
        self.get_logger().info(res_dd)
        dh_strs = res_dh[9:-1].split(",")
        dd_strs = res_dd[9:-1].split(",")
        ###############################################################################################

        if len(dh_strs) != 42:
            msg = (
                f"Invalid length of DH parameters, "
                f"expected 42 values (7 parameters for each of the 6 joints), got {len(dh_strs)}"
            )
            raise ValueError(msg)

        if len(dd_strs) != 30:
            msg = (
                f"Invalid length of DeltaDH parameters, "
                f"expected 30 values (5 parameters for each of the 6 joints), got {len(dd_strs)}"
            )
            raise ValueError(msg)

        dh = list(map(float, dh_strs))
        dd = list(map(float, dd_strs))

        xacro_name = f"macro.{original_model}-nominal.urdf.xacro"
        new_xacro_name = f"{new_model}.urdf.xacro"
        if specific_w == "+M":
            new_xacro_name = f"macro.{new_model}.urdf.xacro"

        file_in = self.find_tm_description_xacro_path() / xacro_name
        file_out = Path.cwd() / new_xacro_name

        link_tag = "<!--LinkDescription-->"
        link_start = '<data xmlns:xacro="https://www.ros.org/wiki/xacro">'
        link_end = "</data>"

        self.get_logger().info(f"[reference file path:] {file_in}")

        with file_in.open("r") as fr:
            data_in = fr.read()

        datas = data_in.split(link_tag)
        if len(datas) < 3:
            raise RuntimeError("Incorrect reference xacro file")

        link_data = link_start + datas[1] + link_end
        link_root = ET.fromstring(link_data)

        udh = urdf_DH_from_tm_DH(dh, dd)
        xyzs, rpys = xyzrpys_from_urdf_DH(udh)
        modify_urdf(link_root, xyzs, rpys, udh, "${prefix}")

        if "x" not in original_model:
            # Need to get the camera position
            res = self.call_ask_item("HandCamera_Value")
            if not res.startswith("HandCamera_Value={") or not res.endswith("}"):
                msg = f"Invalid parameters HandCamera_Value, expected format: HandCamera_Value={{...}}, got: {res}"
                raise ValueError(msg)

            cam_strs = res[18:-1].split(",")
            if len(cam_strs) != 6:
                msg = (
                    f"Invalid length of HandCamera_Value parameters, "
                    f"expected 6 values (x, y, z, R, P, Y), got {len(cam_strs)}"
                )
                raise ValueError(msg)

            cam_values = list(map(float, cam_strs))
            cam_xyz = list(map(lambda x: x / 1000.0, cam_values[:3]))  # convert mm to m
            cam_rpy = list(map(radians, cam_values[3:]))

            # Add the hand camera link and joint to the xacro
            cam_link = ET.Element("link", name="${prefix}hand_camera")
            cam_joint = ET.Element("joint", name="${prefix}hand_camera_fixed_joint", type="fixed")
            ET.SubElement(cam_joint, "parent", link="${prefix}flange")
            ET.SubElement(cam_joint, "child", link="${prefix}hand_camera")
            ET.SubElement(cam_joint, "origin", rpy="{} {} {}".format(*cam_rpy), xyz="{} {} {}".format(*cam_xyz))
            link_root.append(cam_link)
            link_root.append(cam_joint)

        pretty_xml(link_root, "  ", "\n")
        link_data = ET.tostring(link_root, encoding="UTF-8", xml_declaration=False).decode("UTF-8")
        link_data = link_data.replace("ns0", "xacro", -1)
        link_data = link_data.replace(link_start, link_tag, 1)
        link_data = link_data.replace(link_end, link_tag, 1)

        # Join link_data with the rest of the xacro file
        data_out = datas[0] + link_data + datas[2]

        # Inject robot name in the xacro file
        robot_tag = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
        robot_tag_named = f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{new_model}">'
        data_out = data_out.replace(robot_tag, robot_tag_named, 1)

        if overwrite:
            file_save = file_in
            shutil.copyfile(file_in, file_out)
        else:
            file_save = file_out

        with file_save.open("w") as fw:
            fw.write(data_out)

        if overwrite:
            self.get_logger().info("File saved with new kinematic values")
            self.get_logger().info(f"[overwrite reference file path:] {file_in.as_posix()}")
            self.get_logger().info(f"[new save file path:] {file_save.as_posix()}")
        elif restore:
            self.get_logger().info("File restored with the nominal kinematic values")
            self.get_logger().info(f"[new save file path:] {file_save.as_posix()}")
        else:
            self.get_logger().info("File saved with new kinematic values")
            self.get_logger().info(f"[new save file path:] {file_save.as_posix()}")


def main(args: Iterable | None = None) -> int:
    from rclpy.executors import ExternalShutdownException

    parser = argparse.ArgumentParser()
    parser.add_argument("original_model", type=str, help="The original model name (e.g., tm5-900)")
    parser.add_argument(
        "new_model",
        type=str,
        nargs="?",
        default="",
        help="The new model name (e.g., xxxooo), "
        "if not specified, the serial number will be used as the new model name",
    )
    parser.add_argument(
        "--specific",
        "-s",
        dest="specific_w",
        type=str.upper,
        default="",
        help="Specific keyword for nominal restore or overwrite (e.g., -K59, -OW)",
    )
    parsed_args = parser.parse_args(remove_ros_args(args)[1:])

    rclpy.init(args=args)
    try:
        node = ModifyXacro()
        node.gen_xacro(**vars(parsed_args))
    except (RuntimeError, ValueError) as e:
        node.get_logger().error(f"Failed to generate a new URDF model:\n{e}")
    except KeyboardInterrupt:
        return 0
    except ExternalShutdownException:
        return 1
    finally:
        rclpy.try_shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
