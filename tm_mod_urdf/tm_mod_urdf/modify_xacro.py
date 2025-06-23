#! /usr/bin/env python3

import os
import shutil
import sys
import xml.etree.cElementTree as ET  # noqa: N812, N817
from pathlib import Path

import rclpy

from tm_mod_urdf._modify_urdf import modify_urdf, urdf_DH_from_tm_DH, xyzrpys_from_urdf_DH
from tm_msgs.srv import AskItem


def _gen_xacro() -> None:
    rclpy.init()
    node = rclpy.create_node("modify_xacro")

    ###############################################################################################
    # example: generate a new_model file (macro.xxxooo.urdf.xacro), base on tm5-900-norminal model
    # syntax : python3 modify_xacro.py original_model new_model
    # [key-in] original_model: tm5-900  , [key-in] new_model: xxxooo
    # [key-in] shell cmd $ python3 modify_xacro.py tm5-900 xxxooo
    ###############################################################################################

    if len(sys.argv) < 3:
        print("Incorrect syntax! at least 2 parameters are required")
        print("You can try: python3 modify_xacro.py tm5-900 test")
        return

    original_model = sys.argv[1]
    new_model = sys.argv[2]
    specific_w = ""
    # specific keyword default
    overwrite = False
    # nominal_model_restore = False
    tm5_900_nominal_restore = False
    tm5_700_nominal_restore = False
    tm12_nominal_restore = False
    tm14_nominal_restore = False
    tm5s_nominal_restore = False
    tm7s_nominal_restore = False
    tm12s_nominal_restore = False
    tm14s_nominal_restore = False
    tm25s_nominal_restore = False
    tm30s_nominal_restore = False
    tm_model = "reference"
    ###############################################################################################
    # You can restore some nominal kinematic parameters by using specific keyword settings
    if len(sys.argv) == 4:
        specific_w = sys.argv[3].upper()
    if new_model == "tm5-900-nominal" or specific_w == "-K59":
        tm_model = "tm5-900-nominal"
        nominal_model_restore = True
        tm5_900_nominal_restore = True
    elif new_model == "tm5-700-nominal" or specific_w == "-K57":
        tm_model = "tm5-700-nominal"
        nominal_model_restore = True
        tm5_700_nominal_restore = True
    elif new_model == "tm12-nominal" or specific_w == "-K12":
        tm_model = "tm12-nominal"
        nominal_model_restore = True
        tm12_nominal_restore = True
    elif new_model == "tm14-nominal" or specific_w == "-K14":
        tm_model = "tm14-nominal"
        nominal_model_restore = True
        tm14_nominal_restore = True
    elif new_model == "tm5s-nominal" or specific_w == "-K5S":
        tm_model = "tm5s-nominal"
        nominal_model_restore = True
        tm5s_nominal_restore = True
    elif new_model == "tm7s-nominal" or specific_w == "-K7S":
        tm_model = "tm7s-nominal"
        nominal_model_restore = True
        tm7s_nominal_restore = True
    elif new_model == "tm12s-nominal" or specific_w == "-K12S":
        tm_model = "tm12s-nominal"
        nominal_model_restore = True
        tm12s_nominal_restore = True
    elif new_model == "tm14s-nominal" or specific_w == "-K14S":
        tm_model = "tm14s-nominal"
        nominal_model_restore = True
        tm14s_nominal_restore = True
    elif new_model == "tm25s-nominal" or specific_w == "-K25S":
        tm_model = "tm25s-nominal"
        nominal_model_restore = True
        tm25s_nominal_restore = True
    elif new_model == "tm30s-nominal" or specific_w == "-K30S":
        tm_model = "tm30s-nominal"
        nominal_model_restore = True
        tm30s_nominal_restore = True
    else:
        nominal_model_restore = False
    if nominal_model_restore:
        message_s0 = "Notice! You have chosen to restore a " + tm_model + " xacro model file"
        node.get_logger().info("%s!" % message_s0)
    if specific_w == "-OW":
        overwrite = True
        message_s1 = "Notice!!! You have chosen to overwrite the original " + tm_model + " file"
        node.get_logger().info("%s!" % message_s1)
    ###############################################################################################

    ask_item = node.create_client(AskItem, "ask_item")
    if not ask_item.wait_for_service(3.0):
        node.get_logger().error("stop service, No AskItem service")
        return

    # Notice !!!
    # You must have finished running the driver to connect to your TM Robot before.
    # [svr] (ask_item) -> id:dh (DHTable),id:dd (DeltaDH)
    req = AskItem.Request()
    req.wait_time = 1.0

    req.id = "dh"
    req.item = "DHTable"
    future = ask_item.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res_dh = future.result()

    req.id = "dd"
    req.item = "DeltaDH"
    future = ask_item.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res_dd = future.result()
    if not res_dh.value.startswith("DHTable={") or not res_dh.value.endswith("}"):
        node.get_logger().error("stop service, invalid parameters dh")
        return
    if not res_dd.value.startswith("DeltaDH={") or not res_dd.value.endswith("}"):
        node.get_logger().error("stop service, invalid parameters delta_dh")
        return

    if not nominal_model_restore or overwrite:
        node.get_logger().info("loading the correction kinematics parameters from your TM Robot")
        if specific_w == "-VAL":
            node.get_logger().info(res_dh.value)
            node.get_logger().info(res_dd.value)

    dh_strs = res_dh.value[9:-1].split(",")
    dd_strs = res_dd.value[9:-1].split(",")

    ###############################################################################################
    # You can restore some nominal kinematic parameters by using specific keyword settings
    if nominal_model_restore:
        if tm5_900_nominal_restore:
            node.get_logger().info("Restore with TM5-900 nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,145.2,0,-270,270,-90,0,429,0,0,-180,180,0,0,411.5,0,0,-155,155,90,90,0,-122.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif tm5_700_nominal_restore:
            node.get_logger().info("Restore with TM5-700 nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,145.2,0,-270,270,-90,0,329,0,0,-180,180,0,0,311.5,0,0,-155,155,90,90,0,-122.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif tm12_nominal_restore:
            node.get_logger().info("Restore with TM12 nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,165.2,0,-270,270,-90,0,636.1,0,0,-180,180,0,0,557.9,0,0,-166,166,90,90,0,-156.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif tm14_nominal_restore:
            node.get_logger().info("Restore with TM14 nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,165.2,0,-270,270,-90,0,536.1,0,0,-180,180,0,0,457.9,0,0,-166,166,90,90,0,-156.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        if tm5s_nominal_restore:
            node.get_logger().info("Restore with TM5S nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,148.4,0,-360,360,-90,0,429,0,0,-360,360,0,0,386,0,0,-158,158,90,90,0,-147.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}"
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif tm7s_nominal_restore:
            node.get_logger().info("Restore with TM7S nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,148.4,0,-360,360,-90,0,329,0,0,-360,360,0,0,298,0,0,-152,152,90,90,0,-147.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}"
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif tm12s_nominal_restore:
            node.get_logger().info("Restore with TM12S nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,165.2,0,-360,360,-90,0,636.1,0,0,-360,360,0,0,532.4,0,0,-162,162,90,90,0,-181.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}"
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif tm14s_nominal_restore:
            node.get_logger().info("Restore with TM14S nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,165.2,0,-360,360,-90,0,536.1,0,0,-360,360,0,0,432.4,0,0,-159,159,90,90,0,-181.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}"
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif tm25s_nominal_restore:
            node.get_logger().info("Restore with TM25S nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,235.0,0,-360,360,-90,0,890.0,0,0,-360,360,90,90,0,-70.0,0,-166,166,0,-90,0,660.0,0,-360,360,0,90,0,170.2,0,-360,360,0,0,0,152.95,0,-360,360}"
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif tm30s_nominal_restore:
            node.get_logger().info("Restore with TM30S nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,235.0,0,-360,360,-90,0,890.0,0,0,-360,360,90,90,0,-70.0,0,-166,166,0,-90,0,660.0,0,-360,360,0,90,0,170.2,0,-360,360,0,0,0,152.95,0,-360,360}"
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        else:
            # Example: TM5S nominal kinematics parameters
            node.get_logger().info("Restore with TM5S nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,148.4,0,-360,360,-90,0,429,0,0,-360,360,0,0,386,0,0,-158,158,90,90,0,-147.8,0,-360,360,0,90,0,131.5,0,-360,360,0,0,0,134.95,0,-360,360}"
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        node.get_logger().info(res_dh)
        node.get_logger().info(res_dd)
        dh_strs = res_dh[9:-1].split(",")
        dd_strs = res_dd[9:-1].split(",")
    ###############################################################################################

    if len(dh_strs) != 42:
        node.get_logger().error("stop service, invalid dh parameters")
        return
    if len(dd_strs) != 30:
        node.get_logger().error("stop service, invalid delta_dh parameters")
        return

    dh = list(map(float, dh_strs))
    dd = list(map(float, dd_strs))

    # find xacro path
    curr_path = Path(__file__).absolute().parent
    dirs = ["src", "install", "build"]
    idx = -1
    for d in dirs:
        try:
            idx = curr_path.parts.index(d)
        except ValueError:
            continue
    if idx == -1:
        node.get_logger().error("workspace directory not find")
        return
    src_path = Path(*curr_path.parts[:idx], "src")
    xacro_path = None
    for pkg_dir in src_path.iterdir():
        if pkg_dir.name.endswith("tm_description"):
            xacro_path = pkg_dir / "xacro"
            break

    if xacro_path is None:
        node.get_logger().error("xacro directory not found")
        return

    xacro_name = f"macro.{original_model}-nominal.urdf.xacro"
    new_xacro_name = f"{new_model}.urdf.xacro"
    if specific_w == "+M":
        new_xacro_name = f"macro.{new_model}.urdf.xacro"

    file_in = xacro_path / xacro_name
    file_out = xacro_path / new_xacro_name

    link_tag = "<!--LinkDescription-->"
    link_head = "<?xml version='1.0' encoding='UTF-8'?>\n"
    link_start = '<data xmlns:xacro="https://www.ros.org/wiki/xacro">'
    link_end = "</data>"

    node.get_logger().info("[reference file path:] %s" % file_in)

    with file_in.open("r") as fr:
        data_in = fr.read()

    datas = data_in.split(link_tag)

    if len(datas) < 3:
        node.get_logger().error("stop service, incorrect reference xacro file")
        return

    link_data = link_start + datas[1] + link_end
    root = ET.fromstring(link_data)

    udh = urdf_DH_from_tm_DH(dh, dd)
    xyzs, rpys = xyzrpys_from_urdf_DH(udh)
    modify_urdf(root, xyzs, rpys, udh, "${prefix}")

    link_data = ET.tostring(root, encoding="UTF-8", xml_declaration=True).decode("UTF-8")
    link_data = link_data.replace("ns0", "xacro")
    link_data = link_data.replace(link_head, "", 1)
    link_data = link_data.replace(link_start, link_tag, 1)
    link_data = link_data.replace(link_end, link_tag, 1)

    data_out = datas[0] + link_data + datas[2]

    if overwrite:
        file_save = file_in
        shutil.copyfile(file_in, file_out)
    else:
        file_save = file_out

    with file_save.open("w") as fw:
        fw.write(data_out)

    if overwrite:
        node.get_logger().info("File saved with new kinematic values")
        node.get_logger().info("[overwrite reference file path:] " + str(file_in))
        node.get_logger().info("[new save file path:] " + str(file_out))
    elif nominal_model_restore:
        node.get_logger().info("File restored with the nominal kinematic values")
        node.get_logger().info("[new save file path:] " + str(file_save))
    else:
        node.get_logger().info("File saved with new kinematic values")
        node.get_logger().info("[new save file path:] " + str(file_save))


def main() -> None:
    from rclpy.exceptions import ROSInterruptException

    try:
        _gen_xacro()
    except ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
