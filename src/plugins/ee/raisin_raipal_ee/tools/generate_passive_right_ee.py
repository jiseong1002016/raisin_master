#!/usr/bin/env python3
"""Stage fixed EE plant resources and complete articulated display models."""

import argparse
from collections import Counter
import copy
import hashlib
import json
import math
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PLUGINS = Path(__file__).resolve().parents[3]
ROOT = PLUGINS.parent / "raisin_raipal"
EE = PLUGINS / "ee/raisin_raipal_ee/resource"
URDF = ROOT / "resource/urdf_raipal"
MODULE_EXPORTER = ROOT / "resource/module-exporter.py"
URDF_WRITER = URDF / "scripts/urdf-writer.py"
MOUNTS = EE / "ee_mounts.yaml"
EXPECTED_MOUNTS = {
    "R": {"parent_link": "R_ee", "xyz_m": ["0", "0.027", "0"],
          "rpy_rad": ["-1.5707963267949", "0", "-1.5707963267949"]},
    "L": {"parent_link": "L_ee", "xyz_m": ["0", "-0.027", "0"],
          "rpy_rad": ["-1.0471975511965976", "0", "1.5707963267949"]},
}
LOCAL_HASHES = {
    "rsc/end-effector/urdf/end-effector.urdf": "fb5bca6a2f69c303d6a366e444a0c70fec0e20905e4569b6ae39d33b230aeeb0",
    "rsc/RAIPAL-URDF/modules/end-effector_R": "cfd06528c22ffb50aa36f3caf24902f0e9124ee03630dcbe07133e8c2677ec79",
}
def profile(side, single_arm=False):
    if side not in ("R", "L"):
        raise ValueError("side must be R or L")
    if single_arm and side != "L":
        raise ValueError("single-arm staging requires side L")
    mount_name = "right" if side == "R" else "left"
    mount_text = MOUNTS.read_text()
    m = re.search(rf"(?ms)^  {mount_name}:\s*\n    parent_link:\s*([^\s]+)\s*\n    xyz_m:\s*\[([^]]+)\]\s*\n    rpy_rad:\s*\[([^]]+)\]", mount_text)
    mount = {"parent_link": m.group(1), "xyz_m": [v.strip() for v in m.group(2).split(",")], "rpy_rad": [v.strip() for v in m.group(3).split(",")]} if m else None
    if mount != EXPECTED_MOUNTS[side]:
        raise ValueError(f"reviewed source {mount_name} mount changed: {mount}")
    modules = (["_fixed-base", "raipal_R", f"raipal-ee-visual_{side}", "raipal_L", "tool-empty_L"]
               if side == "R" else
               ["_fixed-base", "raipal_R", "stub-0_R", "raipal_L", f"raipal-ee-visual_{side}"])
    if single_arm:
        return mount_name, mount, "raipal-ee-visual_L", "raipal_single_L_ee", "flat_terrain_single_L_ee.xml", ["_fixed-base", "raipal_L", "raipal-ee-visual_L"]
    return mount_name, mount, f"raipal-ee-visual_{side}", f"raipal_dual_ee_visual_{side}", f"flat_terrain_ee_visual_{side}.xml", modules
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def make_module(side, single_arm=False):
    mount_name, MOUNT, MODULE, MODEL, CONFIG_NAME, MODULES = profile(side, single_arm)
    manifest = json.loads((EE / "ee_visual_manifest.json").read_text())
    listed = {item["path"]: item["sha256"] for item in manifest["source_paths"]}
    if sha(EE / "ee_visual.urdf") != manifest["generated_urdf_sha256"] or any(listed.get(k) != v for k, v in LOCAL_HASHES.items()):
        raise ValueError("master EE visual or local grasp_iteration source hash mismatch")
    mesh_hashes = {}
    for name in manifest["mesh_files"]:
        path = EE / "meshes" / name
        digest = sha(path)
        if listed.get(f"rsc/end-effector/meshes/{name}") != digest:
            raise ValueError(f"CAD mesh source hash mismatch: {name}")
        mesh_hashes[name] = digest

    source = ET.parse(EE / "ee_visual.urdf").getroot()
    links, joints = source.findall("link"), source.findall("joint")
    if len(links) != 9 or len(joints) != 8 or any(j.get("type") != "revolute" for j in joints):
        raise ValueError("expected 9 source links and 8 revolute EE joints")
    names = {link.get("name"): f"{side}_ee_visual_" + link.get("name") for link in links}
    module = ET.Element("robot")
    for original in links:
        link = ET.Element("link", {"name": names[original.get("name")]})
        visual = copy.deepcopy(original.find("visual"))
        mesh = visual.find("geometry/mesh") if visual is not None else None
        if mesh is None or Path(mesh.get("filename", "")).name not in mesh_hashes:
            raise ValueError(f"unexpected EE visual on {original.get('name')}")
        mesh.set("filename", f"../meshes/raipal_ee_visual/{Path(mesh.get('filename')).name}")
        link.append(visual)
        module.append(link)

    mount = ET.SubElement(module, "joint", {"name": f"{side}E_end_effector_fixed", "type": "fixed"})
    if side == "L":
        mount.append(ET.Comment(
            " Original mount RPY [-pi/2, 0, pi/2], roll -90 deg. "
            "Earlier +180 deg mount correction removed: it was a not-yet-homed L7 pose. "
            "EE versus straight stub mounting phase is exactly -30 deg about L7 +Y. "
            "Stub L7=-0.015 rad and EE L7=-0.493 rad are heavily rounded reference readings only. "
            "Compensate +30 deg about L7 +Y (right-hand rule), equivalently EE +X: roll -60 deg. "
            "L7 encoder zero/homing unchanged. Restore roll -90 deg only after fixing the mechanical EE mounting phase. "))
    ET.SubElement(mount, "origin", {"xyz": " ".join(MOUNT["xyz_m"]), "rpy": " ".join(MOUNT["rpy_rad"])})
    ET.SubElement(mount, "parent", {"link": MOUNT["parent_link"]})
    ET.SubElement(mount, "child", {"link": names["ee_base"]})
    for original in joints:
        parent, child = original.find("parent"), original.find("child")
        joint = ET.SubElement(module, "joint", {"name": f"{side}_ee_visual_" + original.get("name") + "_fixed", "type": "fixed"})
        if original.find("origin") is not None:
            joint.append(copy.deepcopy(original.find("origin")))
        ET.SubElement(joint, "parent", {"link": names[parent.get("link")]})
        ET.SubElement(joint, "child", {"link": names[child.get("link")]})
    # RobotConfig requests the selected arm's tip alias, not a calibrated TCP.
    module.append(ET.Element("link", {"name": f"{side}_ee_visual_tip_frame"}))
    tip = ET.SubElement(module, "joint", {"name": f"{side}E_tip_fixed", "type": "fixed"})
    ET.SubElement(tip, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(tip, "parent", {"link": names["ee_base"]})
    ET.SubElement(tip, "child", {"link": f"{side}_ee_visual_tip_frame"})

    if side == "L":
        camera = json.loads((URDF / "data/grasp/d405_mount_nominal.json").read_text())["camera_to_ee_base"]
        rpy = [math.atan2(camera[2][1], camera[2][2]), math.asin(-camera[2][0]),
               math.atan2(camera[1][0], camera[0][0])]
        link = ET.SubElement(module, "link", {"name": "d405_depth_optical_frame"})
        visual = ET.SubElement(link, "visual")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", {"filename": "../meshes/raipal_d405/d405_optical.obj"})
        joint = ET.SubElement(module, "joint", {"name": "L_d405_fixed", "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": names["ee_base"]})
        ET.SubElement(joint, "child", {"link": "d405_depth_optical_frame"})
        ET.SubElement(joint, "origin", {"xyz": " ".join(str(row[3]) for row in camera[:3]),
                                      "rpy": " ".join(map(str, rpy))})

    fragments = []
    for child in module:
        ET.indent(child, space="  ")
        child.tail = None
        fragments.append(ET.tostring(child, encoding="unicode"))
    text = f"<!-- Fixed CAD preview; {mount_name} mount unverified; {side}E_tip_fixed is an EE-root alias, not a TCP. -->\n" + "\n".join(fragments) + "\n"
    return text, mesh_hashes, (MODULE, MODEL, CONFIG_NAME, MODULES)

def validate_visual_instances(robot, ee_meshes, camera):
    for tag in ("link", "joint"):
        names = [element.get("name") for element in robot.findall(tag)]
        if len(names) != len(set(names)):
            raise ValueError(f"duplicate {tag} names in EE assembly")
    meshes = [Path(mesh.get("filename")).name for mesh in robot.findall(".//visual/geometry/mesh")]
    if Counter(name for name in meshes if name in ee_meshes) != Counter(list(ee_meshes)):
        raise ValueError("EE assembly must contain exactly one visual of each EE part")
    if meshes.count("d405_optical.obj") != int(camera):
        raise ValueError("EE assembly camera visual count does not match the selected profile")


def make_articulated_display(stage, urdf, terrain, config_name, side, arm_joints):
    display = ET.parse(urdf, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    robot = display.getroot()
    robot.set("name", "robot")
    source = ET.parse(EE / "ee_visual.urdf").getroot()
    prefix = f"{side}_ee_visual_"
    followers = {
        "ee_13_left": ("ee_01_left", "0.4"),
        "ee_34_left": ("ee_01_left", "-0.4"),
        "ee_02_left": ("ee_01_left", "1"),
        "ee_01_right": ("ee_02_right", "1"),
        "ee_13_right": ("ee_02_right", "0.4"),
        "ee_34_right": ("ee_02_right", "-0.4"),
    }
    for original in source.findall("link"):
        link = robot.find(f"./link[@name='{prefix}{original.get('name')}']")
        link.insert(0, copy.deepcopy(original.find("inertial")))
    ee_order = []
    for original in source.findall("joint"):
        name = original.get("name")
        fixed = robot.find(f"./joint[@name='{prefix}{name}_fixed']")
        joint = copy.deepcopy(original)
        joint.set("name", prefix + name)
        for tag in ("parent", "child"):
            joint.find(tag).set("link", prefix + original.find(tag).get("link"))
        if name in followers:
            leader, multiplier = followers[name]
            ET.SubElement(joint, "mimic", {"joint": prefix + leader,
                                         "multiplier": multiplier, "offset": "0"})
        robot.insert(list(robot).index(fixed), joint)
        robot.remove(fixed)
        ee_order.append(prefix + name)
    display_urdf = urdf.with_stem(urdf.stem + "_display")
    display.write(display_urdf, encoding="utf-8", xml_declaration=True)

    display_config = copy.deepcopy(terrain)
    system = display_config.getroot().find("./objects/articulatedSystem")
    system.set("name", "robot")
    system.set("urdfPath", f"[THIS_DIR]/../urdf_raipal/urdf/{display_urdf.name}")
    joint_order = [joint.get("name") for joint in arm_joints] + ee_order
    system.set("jointOrder", " ".join(joint_order))
    state = system.find("state")
    for attribute in ("qpos", "qvel"):
        values = state.get(attribute).split(",")
        if len(values) != len(arm_joints):
            raise ValueError(f"plant {attribute} does not match arm joint count")
        state.set(attribute, ",".join(values + ["0"] * 8))
    config_name = Path(config_name)
    display_xml = stage / "resource/raisim_config" / (config_name.stem + "_display.xml")
    display_config.write(display_xml, encoding="utf-8", xml_declaration=True)
    return {
        "urdf": str(display_urdf.relative_to(stage)),
        "xml": str(display_xml.relative_to(stage)),
        "sha256": {"urdf": sha(display_urdf), "xml": sha(display_xml)},
        "arm_dof": len(arm_joints), "ee_dof": len(ee_order), "mimic_count": len(followers),
        "joint_order": joint_order,
    }


def generate(stage, side="R", single_arm=False):
    stage = Path(stage).resolve()
    install_resource = ROOT.parents[1] / "install/resource/raisin_raipal/resource"
    if stage == ROOT or ROOT in stage.parents or stage == PLUGINS or PLUGINS in stage.parents or stage == install_resource or stage in install_resource.parents or install_resource in stage.parents:
        raise ValueError(f"refusing protected output path: {stage}")
    module_text, mesh_hashes, (MODULE, MODEL, CONFIG_NAME, MODULES) = make_module(side, single_arm)
    resource, export = stage / "resource", stage / "_module_export_source/resource"
    if resource.exists() and not (stage / "passive_ee_manifest.json").is_file():
        raise ValueError(f"refusing to replace unmarked resource tree: {resource}")
    for path in (resource, stage / "_module_export_source"):
        if path.exists():
            shutil.rmtree(path)
    package = export / "raipal_ee_visual"
    (package / "modules").mkdir(parents=True)
    (package / "meshes/raipal_ee_visual").mkdir(parents=True)
    (package / "modules" / MODULE).write_text(module_text)
    for name in mesh_hashes:
        shutil.copy2(EE / "meshes" / name, package / "meshes/raipal_ee_visual" / name)
    exporter = export / "module-exporter.py"
    shutil.copy2(MODULE_EXPORTER, exporter)
    subprocess.run([sys.executable, str(exporter)], check=True, capture_output=True, text=True)
    resource.mkdir(parents=True)
    (resource / "raisim_config").mkdir()
    terrain = ET.parse(ROOT / "resource/raisim_config" / (
        "flat_terrain_single_L_ee.xml" if single_arm else "flat_terrain.xml"))
    robot = terrain.getroot().find("./objects/articulatedSystem")
    if robot is None:
        raise ValueError("base terrain config has no robot articulatedSystem")
    robot.set("urdfPath", f"[THIS_DIR]/../urdf_raipal/urdf/{MODEL}.urdf")
    robot.set("resDir", "[THIS_DIR]/../urdf_raipal/urdf/")
    robot.attrib.pop("modules", None)
    terrain.write(resource / "raisim_config" / CONFIG_NAME, encoding="utf-8", xml_declaration=True)
    shutil.copytree(export / "modules", resource / "modules")
    shutil.copytree(export / "meshes", resource / "meshes")
    shutil.rmtree(stage / "_module_export_source")

    full = resource / "urdf_raipal"
    (full / "modules").mkdir(parents=True)
    (full / "scripts").mkdir()
    for name in ("_fixed-base", "raipal_R", "stub-0_R", "raipal_L", "tool-empty_L"):
        shutil.copy2(URDF / "modules" / name, full / "modules" / name)
    (full / "modules" / MODULE).write_text(module_text)
    shutil.copytree(URDF / "meshes", full / "meshes")
    shutil.copytree(EE / "meshes", full / "meshes/raipal_ee_visual", dirs_exist_ok=True)
    if side == "L":
        for meshes in (resource / "meshes/raipal_d405", full / "meshes/raipal_d405"):
            meshes.mkdir(parents=True, exist_ok=True)
            shutil.copy2(PLUGINS / "raisin_d405_plugin/resource/meshes/d405_optical.obj", meshes)
    writer = full / "scripts/urdf-writer.py"
    shutil.copy2(URDF_WRITER, writer)
    shutil.copy2(URDF / "scripts/_template", full / "scripts/_template")
    run_writer = "import importlib.util,sys;s=importlib.util.spec_from_file_location('w',sys.argv[1]);w=importlib.util.module_from_spec(s);s.loader.exec_module(w);w.write_urdf(" + repr(MODULES) + "," + repr(MODEL) + ")"
    subprocess.run([sys.executable, "-c", run_writer, str(writer)], check=True)
    urdf = full / "urdf" / f"{MODEL}.urdf"

    if side == "L":
        # The operator GUI animates a separate floating EE preview.
        preview = ET.parse(EE / "ee_visual.urdf")
        fragment = ET.fromstring("<robot>" + module_text + "</robot>")
        camera_link = fragment.find("./link[@name='d405_depth_optical_frame']")
        camera_link.find("visual/geometry/mesh").set("filename", "meshes/d405_optical.obj")
        camera_joint = fragment.find("./joint[@name='L_d405_fixed']")
        camera_joint.find("parent").set("link", "ee_base")
        preview.getroot().extend([camera_link, camera_joint])
        preview_dir = stage / "ee_resource"
        shutil.copytree(EE / "meshes", preview_dir / "meshes", dirs_exist_ok=True)
        shutil.copy2(PLUGINS / "raisin_d405_plugin/resource/meshes/d405_optical.obj", preview_dir / "meshes")
        validate_visual_instances(preview.getroot(), mesh_hashes, camera=True)
        preview.write(preview_dir / "ee_visual.urdf", encoding="utf-8", xml_declaration=True)

    module = resource / "modules" / MODULE
    robot = ET.parse(urdf).getroot()
    validate_visual_instances(robot, mesh_hashes, camera=side == "L")
    links, joints = robot.findall("link"), robot.findall("joint")
    movable = [j for j in joints if j.get("type") in ("revolute", "continuous", "prismatic")]
    expected_dof = 7 if single_arm else 14
    if len(movable) != expected_dof:
        raise ValueError(f"expected {expected_dof} movable arm joints, got {len(movable)}")
    fixed_ee = [j for j in joints if j.get("name", "").startswith(f"{side}_ee_visual_ee_") or j.get("name") in (f"{side}E_end_effector_fixed", f"{side}E_tip_fixed")]

    result = {
        "preview": f"fixed source CAD; physical mount and encoder pose unverified; {side}E_tip_fixed is not a TCP",
        "mount": profile(side)[1],
        "right_mount": profile("R")[1],
        "local_reference_hashes": LOCAL_HASHES,
        "master_ee_visual_sha256": sha(EE / "ee_visual.urdf"),
        "master_mount_sha256": sha(EE / "ee_mounts.yaml"),
        "mesh_sha256": mesh_hashes,
        "module": f"resource/modules/{MODULE}", "meshes": "resource/meshes/raipal_ee_visual/",
        "full_urdf": f"resource/urdf_raipal/urdf/{MODEL}.urdf", "modules": MODULES,
        "checks": {"links": len(links), "joints": len(joints), "movable_joints": len(movable), "ee_fixed_joints": len(fixed_ee), "mesh_references": len(robot.findall('.//mesh'))},
        "output_sha256": {"module": sha(module), "full_urdf": sha(urdf)},
        "articulated_display": make_articulated_display(stage, urdf, terrain, CONFIG_NAME, side, movable),
    }
    (stage / "passive_ee_manifest.json").write_text(json.dumps(result, indent=2) + "\n")
    return result

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/raisin_tactile_live_20261003/urdf_stage"))
    parser.add_argument("--side", choices=("R", "L"), default="R")
    parser.add_argument("--single-arm", action="store_true")
    args = parser.parse_args()
    print(json.dumps(generate(args.output_dir, args.side, args.single_arm), indent=2))
