from __future__ import annotations

import tempfile
import unittest
import sys
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools import hako


class HakoTest(unittest.TestCase):
    def test_manifest_defaults_resolve_portable_components(self) -> None:
        cfg = hako.resolve_config({"version": 1})
        self.assertTrue(cfg["components"]["hakotime"])
        self.assertTrue(cfg["components"]["hakopdu_ev3"])
        self.assertTrue(cfg["validation"]["tests"])

    def test_manifest_rejects_build_with_no_component(self) -> None:
        with self.assertRaisesRegex(hako.ConfigError, "at least one"):
            hako.resolve_config({
                "version": 1,
                "components": {"hakotime": False, "hakopdu_ev3": False},
            })

    def test_windows_configure_uses_mapped_workspace_paths(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary)
            root = workspace / "athrill-device"
            manifest = root / "hakoniwa-build.yaml"
            root.mkdir()
            manifest.write_text("version: 1\n", encoding="utf-8")
            ctx = hako.create_context(manifest, root)
            ctx.platform_name = "windows"
            command = hako.configure_command(ctx)
            self.assertEqual(command[:2], ["cmake", "--fresh"])
            self.assertTrue(any("!CD!" in argument for argument in command))
            self.assertIn("-A", command)
            self.assertNotIn(str(workspace), " ".join(command))

    def test_host_configure_uses_core_and_athrill_paths(self) -> None:
        root = hako.repo_root()
        manifest = root / "hakoniwa-build.yaml"
        with patch.object(hako, "_host_platform", return_value=("linux", "x64")):
            ctx = hako.create_context(manifest, root)
        command = hako.configure_command(ctx)
        self.assertIn(f"-DCMAKE_PREFIX_PATH={ctx.core_root}", command)
        self.assertIn(f"-DATHRILL_SOURCE_DIR={ctx.athrill_root}", command)

    def test_disabled_profile_removes_only_owned_stale_artifacts(self) -> None:
        root = hako.repo_root()
        ctx = hako.create_context(root / "hakoniwa-build.yaml", root)
        ctx.cfg["components"]["hakotime"] = False
        ctx.cfg["components"]["hakopdu_ev3"] = True
        with tempfile.TemporaryDirectory() as temporary:
            install = Path(temporary)
            stale = install / "lib" / "libhakotime.so"
            keep = install / "lib" / "libhakotime.a"
            unrelated = install / "lib" / "libshakoc.so"
            stale.parent.mkdir(parents=True)
            for path in (stale, keep, unrelated):
                path.write_text("test", encoding="utf-8")

            hako._remove_disabled_artifacts(ctx, install)

            self.assertFalse(stale.exists())
            self.assertTrue(keep.exists())
            self.assertTrue(unrelated.exists())


if __name__ == "__main__":
    unittest.main()
