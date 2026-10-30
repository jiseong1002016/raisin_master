#!/usr/bin/env python3
import hashlib
import copy
import json
import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PLUGINS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PLUGINS))
from ee.raisin_raipal_ee.tools.generate_passive_right_ee import ROOT, generate, make_module, validate_visual_instances


class PassiveVisualGeneratorTest(unittest.TestCase):
    def test_camera_mount_matches_canonical_source(self):
        canonical = json.loads((ROOT / "resource/urdf_raipal/data/grasp/d405_mount_nominal.json").read_text())
        plugin = json.loads((PLUGINS / "raisin_d405_plugin/resource/mount.nominal.json").read_text())
        self.assertEqual(plugin, canonical)
        source = ET.fromstring("<robot>" + (ROOT / "resource/urdf_raipal/modules/grasp/d405_nominal_L").read_text() + "</robot>")
        module = ET.fromstring("<robot>" + make_module("L")[0] + "</robot>")
        for attribute in ("xyz", "rpy"):
            expected = map(float, source.find("./joint[@name='L_d405_fixed']/origin").get(attribute).split())
            actual = map(float, module.find("./joint[@name='L_d405_fixed']/origin").get(attribute).split())
            for generated, target in zip(actual, expected):
                self.assertAlmostEqual(generated, target)

    def test_real_output_limits_match_urdf_joint_mapping(self):
        robot = ET.parse(PLUGINS / "ee/raisin_raipal_ee/resource/ee_visual.urdf").getroot()
        limits = {
            joint.get("name"): tuple(float(joint.find("limit").get(bound))
                                     for bound in ("lower", "upper"))
            for joint in robot.findall("joint")
        }
        joint8 = (max(limits[name][0] for name in ("ee_01_left", "ee_02_left")),
                  min(limits[name][1] for name in ("ee_01_left", "ee_02_left")))
        joint9 = limits["ee_02_right"]
        self.assertEqual(joint8, (-0.96, 0.0))
        self.assertEqual(joint9, (0.0, 0.96))
        profiles = yaml.safe_load((ROOT / "config/L_ee/hardware_profiles.yaml").read_text())["profiles"]
        for name in ("single_L_ee", "dual_arm_left_ee"):
            axes = profiles[name]["parameters"]["ee"]["axes"]
            for axis, expected, direction in (("left", joint9, 1), ("right", joint8, -1)):
                with self.subTest(profile=name, axis=axis):
                    config = axes[axis]
                    self.assertEqual((config["lower_limit_rad"], config["upper_limit_rad"]), expected)
                    self.assertEqual(config["gear_ratio"], 68.0)
                    self.assertEqual(config["direction"], direction)

    def test_articulated_displays_preserve_plant_and_source_geometry(self):
        source = ET.parse(PLUGINS / "ee/raisin_raipal_ee/resource/ee_visual.urdf").getroot()
        for side, single_arm in (("R", False), ("L", False), ("L", True)):
            with self.subTest(side=side, single_arm=single_arm), tempfile.TemporaryDirectory() as temporary:
                stage = Path(temporary)
                result = generate(stage, side, single_arm)
                entry = result["articulated_display"]
                arm_dof = 7 if single_arm else 14
                self.assertEqual((entry["arm_dof"], entry["ee_dof"], entry["mimic_count"]),
                                 (arm_dof, 8, 6))
                model = Path(result["full_urdf"])
                self.assertEqual(entry["urdf"], str(model.with_stem(model.stem + "_display")))
                config_stem = "flat_terrain_single_L_ee" if single_arm else f"flat_terrain_ee_visual_{side}"
                self.assertEqual(entry["xml"], f"resource/raisim_config/{config_stem}_display.xml")
                plant = ET.parse(stage / model).getroot()
                display = ET.parse(stage / entry["urdf"]).getroot()
                self.assertEqual(display.get("name"), "robot")
                self.assertIsNotNone(display.find("./link[@name='world']"))
                self.assertEqual(len(display.findall("link")), len(plant.findall("link")))
                self.assertEqual(len(display.findall("joint")), len(plant.findall("joint")))
                prefix = f"{side}_ee_visual_"
                ee_joints = {joint.get("name")[len(prefix):]: joint for joint in display.findall("joint")
                             if joint.get("name", "").startswith(prefix)}
                self.assertEqual(len(ee_joints), 8)
                self.assertTrue(all(joint.get("type") == "revolute" for joint in ee_joints.values()))
                self.assertEqual(len(display.findall("./joint/mimic")), 6)
                mapping = {
                    "ee_13_left": ("ee_01_left", .4), "ee_34_left": ("ee_01_left", -.4),
                    "ee_02_left": ("ee_01_left", 1.), "ee_01_right": ("ee_02_right", 1.),
                    "ee_13_right": ("ee_02_right", .4), "ee_34_right": ("ee_02_right", -.4),
                }
                for original in source.findall("joint"):
                    name = original.get("name")
                    joint = ee_joints[name]
                    for tag in ("origin", "axis", "limit"):
                        self.assertEqual(joint.find(tag).attrib, original.find(tag).attrib)
                    for tag in ("parent", "child"):
                        self.assertEqual(joint.find(tag).get("link"), prefix + original.find(tag).get("link"))
                    mimic = joint.find("mimic")
                    if name in mapping:
                        leader, multiplier = mapping[name]
                        self.assertEqual(mimic.get("joint"), prefix + leader)
                        self.assertEqual(float(mimic.get("multiplier")), multiplier)
                        self.assertEqual(float(mimic.get("offset")), 0.)
                    else:
                        self.assertIsNone(mimic)
                for original in source.findall("link"):
                    link = display.find(f"./link[@name='{prefix}{original.get('name')}']")
                    self.assertEqual(ET.tostring(link.find("inertial")), ET.tostring(original.find("inertial")))
                    self.assertIsNone(link.find("collision"))
                for link in plant.findall("link"):
                    display_link = display.find(f"./link[@name='{link.get('name')}']")
                    if link.find("visual") is not None:
                        self.assertEqual(ET.tostring(display_link.find("visual")),
                                         ET.tostring(link.find("visual")))
                for joint in plant.findall("joint"):
                    if not joint.get("name").startswith(prefix):
                        self.assertEqual(ET.tostring(display.find(f"./joint[@name='{joint.get('name')}']")),
                                         ET.tostring(joint))
                validate_visual_instances(display, result["mesh_sha256"], camera=side == "L")
                self.assertTrue(all((stage / entry["urdf"]).parent.joinpath(mesh.get("filename")).is_file()
                                    for mesh in display.findall(".//mesh")))
                plant_system = ET.parse(stage / f"resource/raisim_config/{config_stem}.xml").getroot().find("./objects/articulatedSystem")
                system = ET.parse(stage / entry["xml"]).getroot().find("./objects/articulatedSystem")
                self.assertEqual(system.get("name"), "robot")
                self.assertEqual(Path(system.get("urdfPath")).name, Path(entry["urdf"]).name)
                arm_order = [joint.get("name") for joint in plant.findall("joint") if joint.get("type") == "revolute"]
                expected_order = arm_order + [prefix + joint.get("name") for joint in source.findall("joint")]
                self.assertEqual(system.get("jointOrder").split(), expected_order)
                self.assertEqual(entry["joint_order"], expected_order)
                for attribute in ("qpos", "qvel"):
                    plant_values = plant_system.find("state").get(attribute).split(",")
                    values = system.find("state").get(attribute).split(",")
                    self.assertEqual(len(plant_values), arm_dof)
                    self.assertEqual(values, plant_values + ["0"] * 8)
                first_bytes = {key: (stage / entry[key]).read_bytes() for key in ("urdf", "xml")}
                self.assertEqual(generate(stage, side, single_arm)["articulated_display"], entry)
                for key, content in first_bytes.items():
                    self.assertEqual((stage / entry[key]).read_bytes(), content)
                    self.assertEqual(hashlib.sha256(content).hexdigest(), entry["sha256"][key])
                self.assert_independent_finger_fk(ee_joints, prefix)

    def assert_independent_finger_fk(self, joints, prefix):
        def fingertip(finger, leader_a, leader_b):
            values = {"ee_01_left": leader_a, "ee_02_right": leader_b}
            position_x, position_y, angle = 0., 0., 0.
            for segment in ("01", "13", "34"):
                name = f"ee_{segment}_{finger}"
                joint = joints[name]
                mimic = joint.find("mimic")
                value = (values[mimic.get("joint")[len(prefix):]] * float(mimic.get("multiplier"))
                         + float(mimic.get("offset"))) if mimic is not None else values[name]
                origin = joint.find("origin")
                offset_x, offset_y, offset_z = map(float, origin.get("xyz").split())
                roll, pitch, yaw = map(float, origin.get("rpy").split())
                self.assertEqual((roll, pitch), (0., 0.))
                axis_x, axis_y, axis_z = map(float, joint.find("axis").get("xyz").split())
                self.assertEqual((axis_x, axis_y), (0., 0.))
                position_x += math.cos(angle) * offset_x - math.sin(angle) * offset_y
                position_y += math.sin(angle) * offset_x + math.cos(angle) * offset_y
                angle += yaw + axis_z * value
            return (position_x - .03 * math.sin(angle), position_y + .03 * math.cos(angle))

        for finger in ("left", "right"):
            neutral = fingertip(finger, 0., 0.)
            changed_a = fingertip(finger, -.5, 0.)
            changed_b = fingertip(finger, 0., .6)
            self.assertEqual(neutral, changed_b if finger == "left" else changed_a)
            self.assertGreater(math.dist(neutral, changed_a if finger == "left" else changed_b), .01)
            self.assertNotEqual(fingertip(finger, -.5, .6), neutral)

    def test_refuses_foreign_and_source_output_trees(self):
        with tempfile.TemporaryDirectory() as temporary:
            foreign = Path(temporary)
            sentinel = foreign / "resource/keep.txt"
            sentinel.parent.mkdir()
            sentinel.write_text("keep")
            with self.assertRaises(ValueError):
                generate(foreign)
            self.assertEqual(sentinel.read_text(), "keep")
        with self.assertRaises(ValueError):
            generate(ROOT)
        with self.assertRaises(ValueError):
            generate(PLUGINS)
        with self.assertRaises(ValueError):
            generate(PLUGINS / "ee")
        self.assertTrue((ROOT / "resource/raisim_config/flat_terrain.xml").is_file())

    def test_deterministic_14_dof_model_and_source_hashes(self):
        with tempfile.TemporaryDirectory() as first_dir, tempfile.TemporaryDirectory() as second_dir:
            first, second = Path(first_dir), Path(second_dir)
            a, b = generate(first), generate(second)
            self.assertEqual(a["checks"]["movable_joints"], 14)
            config = ET.parse(first / "resource/raisim_config/flat_terrain_ee_visual_R.xml").getroot()
            system = config.find("./objects/articulatedSystem")
            self.assertIsNone(system.get("modules"))
            self.assertIn("raipal_dual_ee_visual_R.urdf", system.get("urdfPath"))
            self.assertEqual(system.get("resDir"), "[THIS_DIR]/../urdf_raipal/urdf/")
            self.assertEqual(len(system.find("state").get("qpos").split(",")), 14)
            self.assertEqual(len(system.find("state").get("qvel").split(",")), 14)
            self.assertEqual(a["right_mount"]["xyz_m"], ["0", "0.027", "0"])
            model = first / "resource/urdf_raipal/urdf/raipal_dual_ee_visual_R.urdf"
            robot = ET.parse(model).getroot()
            model_resource = (first / "resource/raisim_config/../urdf_raipal/urdf").resolve()
            self.assertEqual(model_resource, model.parent.resolve())
            self.assertTrue(all((model_resource / mesh.get("filename")).is_file()
                                for mesh in robot.findall(".//mesh")))
            ee_joints = [j for j in robot.findall("joint") if j.get("name", "").startswith("R_ee_visual_ee_")]
            self.assertEqual(len(ee_joints), 8)
            self.assertEqual(a["checks"]["ee_fixed_joints"], 10)
            self.assertTrue(all(j.get("type") == "fixed" for j in ee_joints))
            mount = next(j for j in robot.findall("joint") if j.get("name") == "RE_end_effector_fixed")
            tip = next(j for j in robot.findall("joint") if j.get("name") == "RE_tip_fixed")
            self.assertEqual(mount.get("type"), "fixed")
            self.assertEqual(tip.get("type"), "fixed")
            ee_links = [link for link in robot.findall("link") if link.get("name", "").startswith("R_ee_visual_")]
            self.assertEqual(len(ee_links), 10)
            self.assertTrue(all(link.find("inertial") is None and link.find("collision") is None for link in ee_links))
            links = {link.get("name") for link in robot.findall("link")}
            parents = {}
            children = {}
            for joint in robot.findall("joint"):
                parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
                self.assertIn(parent, links)
                self.assertIn(child, links)
                self.assertNotIn(child, parents)
                parents[child] = parent
                children.setdefault(parent, []).append(child)
            self.assertEqual(links - set(parents), {"world"})
            reached, pending = set(), ["world"]
            while pending:
                current = pending.pop()
                self.assertNotIn(current, reached)
                reached.add(current)
                pending.extend(children.get(current, []))
            self.assertEqual(reached, links)
            self.assertTrue(all((model.parent / mesh.get("filename")).is_file() for mesh in robot.findall(".//mesh")))
            module = first / "resource/modules/raipal-ee-visual_R"
            module_xml = ET.fromstring("<robot>" + module.read_text() + "</robot>")
            self.assertTrue(all((module.parent / mesh.get("filename")).is_file() for mesh in module_xml.findall(".//mesh")))
            self.assertIn("RE_tip_fixed is not a TCP", a["preview"])
            self.assertEqual(a["local_reference_hashes"]["rsc/end-effector/urdf/end-effector.urdf"],
                             "fb5bca6a2f69c303d6a366e444a0c70fec0e20905e4569b6ae39d33b230aeeb0")
            self.assertEqual(a["local_reference_hashes"]["rsc/RAIPAL-URDF/modules/end-effector_R"],
                             "cfd06528c22ffb50aa36f3caf24902f0e9124ee03630dcbe07133e8c2677ec79")
            for path in ("resource/modules/raipal-ee-visual_R",
                         "resource/raisim_config/flat_terrain_ee_visual_R.xml",
                         "resource/urdf_raipal/urdf/raipal_dual_ee_visual_R.urdf"):
                self.assertEqual(hashlib.sha256((first / path).read_bytes()).hexdigest(),
                                 hashlib.sha256((second / path).read_bytes()).hexdigest())

    def test_left_profile_uses_canonical_mount_and_composed_urdf(self):
        with tempfile.TemporaryDirectory() as temporary:
            stage = Path(temporary)
            result = generate(stage, "L")
            self.assertEqual(result["mount"], {
                "parent_link": "L_ee", "xyz_m": ["0", "-0.027", "0"],
                "rpy_rad": ["-1.0471975511965976", "0", "1.5707963267949"]})
            self.assertEqual(result["checks"]["movable_joints"], 14)
            config = ET.parse(stage / "resource/raisim_config/flat_terrain_ee_visual_L.xml").getroot()
            system = config.find("./objects/articulatedSystem")
            self.assertIn("raipal_dual_ee_visual_L.urdf", system.get("urdfPath"))
            self.assertEqual(system.get("resDir"), "[THIS_DIR]/../urdf_raipal/urdf/")
            model = stage / "resource/urdf_raipal/urdf/raipal_dual_ee_visual_L.urdf"
            robot = ET.parse(model).getroot()
            model_resource = (stage / "resource/raisim_config/../urdf_raipal/urdf").resolve()
            self.assertEqual(model_resource, model.parent.resolve())
            self.assertTrue(all((model_resource / mesh.get("filename")).is_file()
                                for mesh in robot.findall(".//mesh")))
            self.assertIn("L_ee", {link.get("name") for link in robot.findall("link")})
            self.assertTrue(any(j.get("name") == "LE_end_effector_fixed"
                                for j in robot.findall("joint")))
            mount_origin = robot.find("./joint[@name='LE_end_effector_fixed']/origin")
            roll, pitch, yaw = map(float, mount_origin.get("rpy").split())
            self.assertAlmostEqual(roll + math.pi / 2, math.pi / 6, places=13)
            self.assertAlmostEqual(pitch, 0.)
            self.assertAlmostEqual(yaw, math.pi / 2)
            columns = (
                (math.cos(yaw), math.sin(yaw), 0.),
                (-math.sin(yaw) * math.cos(roll), math.cos(yaw) * math.cos(roll), math.sin(roll)),
                (math.sin(yaw) * math.sin(roll), -math.cos(yaw) * math.sin(roll), math.cos(roll)),
            )
            for (axis_x, axis_y, axis_z), expected in zip(columns, ((0., 1., 0.), (0., 0., -1.), (-1., 0., 0.))):
                compensated = (
                    math.cos(-math.pi / 6) * axis_x + math.sin(-math.pi / 6) * axis_z,
                    axis_y,
                    -math.sin(-math.pi / 6) * axis_x + math.cos(-math.pi / 6) * axis_z,
                )
                for actual, target in zip(compensated, expected):
                    self.assertAlmostEqual(actual, target, places=13)
            canonical = ET.fromstring("<robot>" + (ROOT / "resource/urdf_raipal/modules/grasp/end-effector_L").read_text() + "</robot>")
            self.assertEqual(canonical.find("./joint[@name='ee_base']/origin").attrib, mount_origin.attrib)
            for model_path in (model, stage / result["articulated_display"]["urdf"]):
                self.assertIn("exactly -30 deg", model_path.read_text())
                self.assertIn("only after fixing the mechanical EE mounting phase", model_path.read_text())
            self.assertEqual(len(robot.findall("./link[@name='d405_depth_optical_frame']")), 1)
            preview = stage / "ee_resource/ee_visual.urdf"
            preview_robot = ET.parse(preview).getroot()
            camera_mesh = preview_robot.find("./link[@name='d405_depth_optical_frame']/visual/geometry/mesh")
            self.assertIsNotNone(camera_mesh)
            self.assertEqual(camera_mesh.get("filename"), "meshes/d405_optical.obj")
            self.assertTrue((preview.parent / camera_mesh.get("filename")).is_file())
            self.assertEqual(preview_robot.find("./joint[@name='L_d405_fixed']/parent").get("link"), "ee_base")
            ee_meshes = result["mesh_sha256"]
            for mesh in ("ee_base.obj", "d405_optical.obj"):
                duplicate = copy.deepcopy(robot)
                link = ET.SubElement(duplicate, "link", {"name": "unwanted_second_assembly"})
                visual = ET.SubElement(link, "visual")
                geometry = ET.SubElement(visual, "geometry")
                ET.SubElement(geometry, "mesh", {"filename": mesh})
                with self.assertRaises(ValueError):
                    validate_visual_instances(duplicate, ee_meshes, camera=True)
            camera = robot.find("./joint[@name='L_d405_fixed']")
            self.assertEqual(camera.find("parent").get("link"), "L_ee_visual_ee_base")
            self.assertEqual(list(map(float, camera.find("origin").get("xyz").split())), [0., 0., -.1])
            roll, pitch, yaw = map(float, camera.find("origin").get("rpy").split())
            optical_forward = (
                math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll),
                math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll),
                math.cos(pitch) * math.cos(roll),
            )
            for actual, expected in zip(optical_forward, (0., .25 / math.hypot(.25, .1), .1 / math.hypot(.25, .1))):
                self.assertAlmostEqual(actual, expected)
            self.assertAlmostEqual(math.cos(yaw) * math.cos(pitch), -1.)
            for variant in (preview_robot, ET.parse(stage / result["articulated_display"]["urdf"]).getroot()):
                self.assertEqual(variant.find("./joint[@name='L_d405_fixed']/origin").attrib,
                                 camera.find("origin").attrib)



if __name__ == "__main__":
    unittest.main()
