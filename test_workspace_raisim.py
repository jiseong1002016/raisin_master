"""The workspace RaiSim SDK follows the delivered binaries; ~/raisim_ws/raisim2Lib is only the machine default."""

import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).parent))

from commands import workspace_raisim  # noqa: E402


def make_sdk(root, version):
    for relative in ("raisim/lib/cmake/raisim/raisimConfig.cmake", f"raisim/lib/libraisim.so.{version}",
                     f"raisim/lib/libraisim_contact.so.{version}", "rayrai/lib/cmake/rayrai/rayraiConfig.cmake",
                     "rayrai/lib/librayrai.so"):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("")


class WorkspaceRaisimTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.home = Path(self._tmp.name)
        self.ws = self.home / "raisim_ws"
        for version in ("2.10.2", "2.17.0"):
            make_sdk(self.ws / f"raisim{version}", version)
        (self.ws / "raisim2Lib").symlink_to("raisim2.17.0")

    def delivered(self, *needed):
        rows = [(Path(f"/install/lib/lib{i}.so"), list(names)) for i, names in enumerate(needed)]
        return mock.patch.object(workspace_raisim, "_binaries", lambda prefixes: iter(rows))

    def test_delivered_binaries_choose_their_own_version_over_the_machine_default(self):
        with self.delivered(["libraisim.so.2.10.2", "libraisim_contact.so.2.10.2"], []):
            sdk, version = workspace_raisim.select_sdk(self.home, [Path("/install")])
        self.assertEqual((sdk, version), (self.ws / "raisim2.10.2", "2.10.2"))

    def test_without_delivered_raisim_the_machine_default_is_used(self):
        with self.delivered([]):
            sdk, version = workspace_raisim.select_sdk(self.home, [Path("/install")])
        self.assertEqual((sdk, version), (self.ws / "raisim2Lib", "2.17.0"))

    def test_mixed_delivered_versions_are_refused(self):
        with self.delivered(["libraisim.so.2.10.2"], ["libraisim.so.2.17.0"]):
            with self.assertRaisesRegex(ValueError, "different RaiSim versions"):
                workspace_raisim.select_sdk(self.home, [Path("/install")])

    def test_a_missing_versioned_sdk_is_named(self):
        with self.delivered(["libraisim.so.2.9.4"]):
            with self.assertRaisesRegex(ValueError, "raisim2.9.4"):
                workspace_raisim.select_sdk(self.home, [Path("/install")])

    def test_an_incomplete_versioned_sdk_is_refused(self):
        (self.ws / "raisim2.10.2/rayrai/lib/librayrai.so").unlink()
        with self.delivered(["libraisim.so.2.10.2"]):
            with self.assertRaisesRegex(ValueError, "librayrai"):
                workspace_raisim.select_sdk(self.home, [Path("/install")])

    def test_an_unversioned_raisim_dependency_is_refused(self):
        with self.delivered(["libraisim.so"]):
            with self.assertRaisesRegex(ValueError, "unversioned"):
                workspace_raisim.select_sdk(self.home, [Path("/install")])

    def test_the_default_must_stay_a_versioned_symlink(self):
        (self.ws / "raisim2Lib").unlink()
        (self.ws / "raisim2Lib").mkdir()
        with self.delivered([]):
            with self.assertRaisesRegex(ValueError, "must be a symlink"):
                workspace_raisim.select_sdk(self.home, [Path("/install")])

    def test_abi_check_rejects_a_binary_for_another_version(self):
        with self.delivered(["libraisim.so.2.17.0"]):
            with self.assertRaisesRegex(ValueError, "needs libraisim.so.2.17.0"):
                workspace_raisim.verify_binary_abi([Path("/install")], "2.10.2")


if __name__ == "__main__":
    unittest.main()
