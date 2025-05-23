import os
import shutil
import sys
import xml.etree.cElementTree as ET  # noqa: N812, N817

import rclpy

from tm_mod_urdf._modify_urdf import modify_urdf, urdf_DH_from_tm_DH, xyzrpys_from_urdf_DH
from tm_msgs.srv import AskItem


def _gen_urdf() -> None:
    rclpy.init()
    node = rclpy.create_node("modify_urdf")

    ###############################################################################################
    # example: generate a new_model file (xxxooo.urdf), base on tm5-900-norminal model
    # syntax : python3 modify_urdf.py original_model new_model
    # [key-in] original_model: tm5-900  , [key-in] new_model: xxxooo
    # [key-in] shell cmd $ python3 modify_urdf.py tm5-900 xxxooo
    ###############################################################################################

    if len(sys.argv) < 3:
        print("Incorrect syntax! at least 2 parameters are required")
        print("You can try: python3 modify_urdf.py tm5-900 test")
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
    else:
        nominal_model_restore = False
    if nominal_model_restore is True:
        message_s0 = "Notice! You have chosen to restore a " + tm_model + " urdf model file"
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
    if nominal_model_restore is True:
        if tm5_900_nominal_restore is True:
            node.get_logger().info("Restore with TM5-900 nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,145.2,0,-270,270,-90,0,429,0,0,-180,180,0,0,411.5,0,0,-155,155,90,90,0,-122.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif tm5_700_nominal_restore is True:
            node.get_logger().info("Restore with TM5-700 nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,145.2,0,-270,270,-90,0,329,0,0,-180,180,0,0,311.5,0,0,-155,155,90,90,0,-122.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif tm12_nominal_restore is True:
            node.get_logger().info("Restore with TM12 nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,165.2,0,-270,270,-90,0,636.1,0,0,-180,180,0,0,557.9,0,0,-166,166,90,90,0,-156.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        elif tm14_nominal_restore is True:
            node.get_logger().info("Restore with TM14 nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,165.2,0,-270,270,-90,0,536.1,0,0,-180,180,0,0,457.9,0,0,-166,166,90,90,0,-156.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}"  # noqa: E501
            res_dd = "DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
        else:
            # Example: TM5-900 nominal kinematics parameters
            node.get_logger().info("Restore with TM5-900 nominal kinematics parameters")
            res_dh = "DHTable={0,-90,0,145.2,0,-270,270,-90,0,429,0,0,-180,180,0,0,411.5,0,0,-155,155,90,90,0,-122.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}"  # noqa: E501
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

    # find urdf path
    curr_path = os.path.dirname(os.path.abspath(__file__))
    dirs = ["src", "install"]
    ind = -1
    for d in dirs:
        ind = curr_path.find(d)
        if ind != -1:
            break
    if ind == -1:
        node.get_logger().error("workspace directory not find")
        return
    src_path = curr_path[:ind] + "src"
    urdf_path = ""
    for dir_path, _dir_names, _file_names in os.walk(src_path, followlinks=True):
        if dir_path.endswith("tm_description"):
            urdf_path = os.path.join(dir_path, "urdf")
            break
    if urdf_path == "":
        node.get_logger().error("urdf directory not found")
        return

    urdf_name = f"{original_model}-nominal.urdf"
    new_urdf_name = f"{new_model}.urdf"
    if specific_w == "+M":
        new_urdf_name = f"macro.{new_model}.urdf"

    file_in = os.path.join(urdf_path, urdf_name)
    file_out = os.path.join(urdf_path, new_urdf_name)

    node.get_logger().info(f"[reference file path:] {file_in}")

    with open(file_in, "r") as fr:
        link_data = fr.read()

    root = ET.fromstring(link_data)

    udh = urdf_DH_from_tm_DH(dh, dd)
    xyzs, rpys = xyzrpys_from_urdf_DH(udh)
    modify_urdf(root, xyzs, rpys, udh)

    link_data = ET.tostring(root, encoding="UTF-8", xml_declaration=True).decode("UTF-8")

    if overwrite:
        file_save = file_in
        shutil.copyfile(file_in, file_out)
    else:
        file_save = file_out

    with open(file_save, "w") as fw:
        fw.write(link_data)

    if overwrite:
        node.get_logger().info("File saved with new kinematic values")
        node.get_logger().info(f"[overwrite reference file path:] {file_in}")
        node.get_logger().info(f"[new save file path:] {file_out}")
    elif nominal_model_restore:
        node.get_logger().info("File restored with the nominal kinematic values")
        node.get_logger().info(f"[new save file path:] {file_save}")
    else:
        node.get_logger().info("File saved with new kinematic values")
        node.get_logger().info(f"[new save file path:] {file_save}")


def main() -> None:
    from rclpy.exceptions import ROSInterruptException

    try:
        _gen_urdf()
    except ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
