#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

COMPONENT_ID = "athrill-device"
COMPONENT_VERSION = "0.1.0"
VALID_BUILD_TYPES = {"Debug", "Release", "RelWithDebInfo", "MinSizeRel"}
DEFAULT_CONFIG: dict[str, Any] = {
    "version": 1,
    "build": {"type": "Release", "dir": ".hako/build", "parallel": 0},
    "components": {"hakotime": True, "hakopdu_ev3": True},
    "validation": {"tests": True},
    "paths": {
        "athrill_root": "../athrill",
        "hakoniwa_core_root": "../hakoniwa-business-pack/work/foundation/install",
    },
}


class ConfigError(RuntimeError):
    pass


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _strip_comment(text: str) -> str:
    quote: str | None = None
    escaped = False
    out: list[str] = []
    for char in text:
        if escaped:
            out.append(char)
            escaped = False
        elif char == "\\" and quote:
            out.append(char)
            escaped = True
        elif char in {"'", '"'}:
            quote = None if quote == char else char if quote is None else quote
            out.append(char)
        elif char == "#" and quote is None:
            break
        else:
            out.append(char)
    return "".join(out).rstrip()


def _parse_scalar(text: str) -> Any:
    value = text.strip()
    if not value:
        return {}
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered in {"null", "~"}:
        return None
    if value.startswith(("'", '"')):
        if len(value) < 2 or value[-1] != value[0]:
            raise ConfigError(f"unterminated quoted scalar: {value}")
        if value[0] == '"':
            try:
                return json.loads(value)
            except json.JSONDecodeError as exc:
                raise ConfigError(f"invalid quoted scalar: {value}") from exc
        return value[1:-1].replace("''", "'")
    try:
        return int(value)
    except ValueError:
        return value


def load_simple_yaml(path: Path) -> dict[str, Any]:
    root: dict[str, Any] = {}
    stack: list[tuple[int, dict[str, Any]]] = [(-1, root)]
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ConfigError(f"cannot read build manifest: {path}: {exc}") from exc
    for lineno, raw in enumerate(lines, 1):
        if "\t" in raw:
            raise ConfigError(f"{path}:{lineno}: tabs are not allowed")
        line = _strip_comment(raw)
        if not line.strip():
            continue
        stripped = line.lstrip(" ")
        indent = len(line) - len(stripped)
        if stripped.startswith("-") or ":" not in stripped:
            raise ConfigError(f"{path}:{lineno}: expected a mapping entry")
        key, raw_value = stripped.split(":", 1)
        key = key.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        if not key or not stack:
            raise ConfigError(f"{path}:{lineno}: invalid mapping")
        parent = stack[-1][1]
        if key in parent:
            raise ConfigError(f"{path}:{lineno}: duplicate key: {key}")
        value = _parse_scalar(raw_value)
        parent[key] = value
        if isinstance(value, dict):
            stack.append((indent, value))
    return root


def _merge_known(
    defaults: Mapping[str, Any], overrides: Mapping[str, Any], prefix: str = ""
) -> dict[str, Any]:
    unknown = sorted(set(overrides) - set(defaults))
    if unknown:
        raise ConfigError(
            f"unknown key(s) under {prefix or 'root'}: {', '.join(unknown)}"
        )
    result: dict[str, Any] = {}
    for key, default in defaults.items():
        value = overrides.get(key, default)
        path = f"{prefix}.{key}" if prefix else key
        if isinstance(default, Mapping):
            if not isinstance(value, Mapping):
                raise ConfigError(f"{path} must be a mapping")
            result[key] = _merge_known(default, value, path)
        else:
            result[key] = value
    return result


def resolve_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    cfg = _merge_known(DEFAULT_CONFIG, raw)
    if cfg["version"] != 1:
        raise ConfigError("version must be 1")
    if cfg["build"]["type"] not in VALID_BUILD_TYPES:
        raise ConfigError("build.type is not supported")
    if not isinstance(cfg["build"]["dir"], str) or not cfg["build"]["dir"]:
        raise ConfigError("build.dir must be a non-empty string")
    parallel = cfg["build"]["parallel"]
    if not isinstance(parallel, int) or isinstance(parallel, bool) or parallel < 0:
        raise ConfigError("build.parallel must be a non-negative integer")
    for key in ("hakotime", "hakopdu_ev3"):
        if not isinstance(cfg["components"][key], bool):
            raise ConfigError(f"components.{key} must be true or false")
    if not any(cfg["components"].values()):
        raise ConfigError("at least one EXDEV component must be enabled")
    if not isinstance(cfg["validation"]["tests"], bool):
        raise ConfigError("validation.tests must be true or false")
    for key in ("athrill_root", "hakoniwa_core_root"):
        if not isinstance(cfg["paths"][key], str) or not cfg["paths"][key]:
            raise ConfigError(f"paths.{key} must be a non-empty string")
    return cfg


def _resolve_path(value: str, root: Path) -> Path:
    path = Path(value).expanduser()
    return (root / path).resolve() if not path.is_absolute() else path.resolve()


def _host_platform() -> tuple[str, str]:
    os_name = (
        "windows" if sys.platform == "win32"
        else "macos" if sys.platform == "darwin"
        else "linux"
    )
    machine = platform.machine().lower()
    arch = {
        "amd64": "x64", "x86_64": "x64",
        "arm64": "arm64", "aarch64": "arm64",
    }.get(machine, machine or "unknown")
    return os_name, arch


@dataclass
class BuildContext:
    repo_root: Path
    manifest: Path
    cfg: dict[str, Any]
    source_dir: Path
    build_dir: Path
    athrill_root: Path
    core_root: Path
    platform_name: str
    arch: str
    dry_run: bool = False


def create_context(manifest: Path, root: Path, dry_run: bool = False) -> BuildContext:
    cfg = resolve_config(load_simple_yaml(manifest))
    platform_name, arch = _host_platform()
    return BuildContext(
        repo_root=root,
        manifest=manifest,
        cfg=cfg,
        source_dir=root / "device" / "hako-exdev",
        build_dir=_resolve_path(cfg["build"]["dir"], root),
        athrill_root=_resolve_path(cfg["paths"]["athrill_root"], root),
        core_root=_resolve_path(cfg["paths"]["hakoniwa_core_root"], root),
        platform_name=platform_name,
        arch=arch,
        dry_run=dry_run,
    )


def _relative_to_workspace(ctx: BuildContext, path: Path) -> str:
    workspace = ctx.repo_root.parent.resolve()
    try:
        relative = path.resolve().relative_to(workspace)
    except ValueError as exc:
        raise ConfigError(
            f"Windows paths must stay under workspace {workspace}: {path}"
        ) from exc
    return str(relative).replace("/", "\\")


def _cmake_path(ctx: BuildContext, path: Path) -> str:
    if ctx.platform_name == "windows":
        return f"!CD!\\{_relative_to_workspace(ctx, path)}"
    return str(path)


def configure_command(ctx: BuildContext) -> list[str]:
    command = [
        "cmake", "--fresh",
        "-S", _cmake_path(ctx, ctx.source_dir),
        "-B", _cmake_path(ctx, ctx.build_dir),
        f"-DCMAKE_PREFIX_PATH={_cmake_path(ctx, ctx.core_root)}",
        f"-DATHRILL_SOURCE_DIR={_cmake_path(ctx, ctx.athrill_root)}",
        f"-DHAKO_EXDEV_BUILD_HAKOTIME={'ON' if ctx.cfg['components']['hakotime'] else 'OFF'}",
        f"-DHAKO_EXDEV_BUILD_HAKOPDU_EV3={'ON' if ctx.cfg['components']['hakopdu_ev3'] else 'OFF'}",
        f"-DBUILD_TESTING={'ON' if ctx.cfg['validation']['tests'] else 'OFF'}",
    ]
    if ctx.platform_name == "windows":
        command.extend(["-A", "x64"])
    else:
        command.append(f"-DCMAKE_BUILD_TYPE={ctx.cfg['build']['type']}")
    return command


def _run(ctx: BuildContext, command: list[str]) -> None:
    if ctx.platform_name == "windows":
        native = subprocess.list2cmdline(command)
        script = f'pushd "{ctx.repo_root.parent}" && {native}'
        print("+", subprocess.list2cmdline(["cmd.exe", "/v:on", "/d", "/c", script]))
        if not ctx.dry_run:
            subprocess.run(
                "cmd.exe /v:on /d /c " + script,
                cwd=os.environ.get("SystemDrive", "C:") + "\\",
                check=True,
            )
        return
    print("+", " ".join(command))
    if not ctx.dry_run:
        subprocess.run(command, cwd=ctx.repo_root, check=True)


def _yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, int):
        return str(value)
    return json.dumps(str(value), ensure_ascii=False)


def _dump_yaml(data: Mapping[str, Any], indent: int = 0) -> str:
    lines: list[str] = []
    prefix = " " * indent
    for key, value in data.items():
        if isinstance(value, Mapping):
            lines.append(f"{prefix}{key}:")
            lines.append(_dump_yaml(value, indent + 2).rstrip())
        else:
            lines.append(f"{prefix}{key}: {_yaml_scalar(value)}")
    return "\n".join(lines) + "\n"


def _resolved_record(ctx: BuildContext) -> dict[str, Any]:
    return {
        "version": 1,
        "component": COMPONENT_ID,
        "source_manifest": str(ctx.manifest),
        "platform": {"os": ctx.platform_name, "architecture": ctx.arch},
        "build": {**ctx.cfg["build"], "dir": str(ctx.build_dir)},
        "components": ctx.cfg["components"],
        "validation": ctx.cfg["validation"],
        "paths": {
            "athrill_root": str(ctx.athrill_root),
            "hakoniwa_core_root": str(ctx.core_root),
        },
    }


def write_resolved(ctx: BuildContext) -> Path:
    path = ctx.repo_root / ".hako" / "resolved-build.yaml"
    if not ctx.dry_run:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(_dump_yaml(_resolved_record(ctx)), encoding="utf-8")
    return path


def configure(ctx: BuildContext) -> None:
    ctx.build_dir.parent.mkdir(parents=True, exist_ok=True)
    _run(ctx, configure_command(ctx))
    write_resolved(ctx)


def build(ctx: BuildContext) -> None:
    configure(ctx)
    command = [
        "cmake", "--build", _cmake_path(ctx, ctx.build_dir),
        "--config", ctx.cfg["build"]["type"],
    ]
    if ctx.cfg["build"]["parallel"]:
        command.extend(["--parallel", str(ctx.cfg["build"]["parallel"])])
    _run(ctx, command)


def test(ctx: BuildContext) -> None:
    if not ctx.cfg["validation"]["tests"]:
        print("SKIP validation.tests=false")
        return
    if not (ctx.build_dir / "CMakeCache.txt").is_file():
        raise ConfigError("build directory is missing; run build first")
    _run(ctx, [
        "ctest", "--test-dir", _cmake_path(ctx, ctx.build_dir),
        "--output-on-failure", "-C", ctx.cfg["build"]["type"],
    ])


def smoke(ctx: BuildContext) -> None:
    test(ctx)


def _git_revision(root: Path) -> str:
    result = subprocess.run(
        ["git", "-c", f"safe.directory={root}", "-C", str(root), "rev-parse", "HEAD"],
        capture_output=True, text=True, check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def _cmake_cache_value(build_dir: Path, key: str) -> str:
    cache = build_dir / "CMakeCache.txt"
    if not cache.is_file():
        return "unknown"
    prefix = f"{key}:"
    for line in cache.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith(prefix) and "=" in line:
            return line.split("=", 1)[1] or "unknown"
    return "unknown"


def _read_dependency_receipt(prefix: Path, component_id: str) -> dict[str, Any]:
    path = prefix / "share" / "hakoniwa" / "receipts" / f"{component_id}.yaml"
    if not path.is_file():
        raise ConfigError(f"dependency receipt not found: {path}")
    result: dict[str, Any] = {"build_limits": {}}
    section = ""
    for raw in path.read_text(encoding="utf-8").splitlines():
        if raw and not raw.startswith(" ") and raw.endswith(":"):
            section = raw[:-1]
            continue
        if not raw.startswith("  ") or raw.startswith("    ") or ":" not in raw:
            continue
        key, value = raw.strip().split(":", 1)
        parsed = _parse_scalar(value)
        if section == "component" and key in {"version", "source_revision"}:
            result[key] = parsed
        elif section == "build_limits":
            result["build_limits"][key] = parsed
    if not result.get("version") or not result.get("source_revision"):
        raise ConfigError(f"incomplete dependency receipt: {path}")
    return result


def _installed_artifacts(install_dir: Path) -> list[tuple[Path, str]]:
    candidates = [
        (Path("include/hako_exdev/hakotime.h"), "header"),
        (Path("include/hako_exdev/runtime.h"), "header"),
        (Path("include/hako_exdev/hakopdu_ev3.h"), "header"),
    ]
    for directory in ("bin", "lib"):
        root = install_dir / directory
        if root.is_dir():
            for path in root.iterdir():
                if path.is_file() and (
                    "hakotime" in path.name or "hakopdu_ev3" in path.name
                ):
                    candidates.append((path.relative_to(install_dir), "native-library"))
    return sorted(
        {(path, kind) for path, kind in candidates if (install_dir / path).exists()},
        key=lambda item: item[0].as_posix(),
    )


def write_receipt(ctx: BuildContext, install_dir: Path) -> Path:
    artifacts = _installed_artifacts(install_dir)
    if not artifacts:
        raise ConfigError(f"no installed EXDEV artifacts found under: {install_dir}")
    dependency = _read_dependency_receipt(ctx.core_root, "hakoniwa-core-pro")
    receipt_root = install_dir / "share" / "hakoniwa" / "receipts"
    resolved_relative = (
        Path("share/hakoniwa/receipts/resolved") / f"{COMPONENT_ID}.yaml"
    )
    stored = install_dir / resolved_relative
    if not ctx.dry_run:
        stored.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(write_resolved(ctx), stored)
    capabilities = {
        "hakotime_static": True,
        "hakotime_shared": ctx.cfg["components"]["hakotime"],
        "hakopdu_ev3": ctx.cfg["components"]["hakopdu_ev3"],
        "core_pro_polling": True,
        "exdev_load_test": ctx.cfg["validation"]["tests"],
    }
    lines = [
        "schema_version: 1",
        "component:",
        f"  id: {COMPONENT_ID}",
        f"  version: {COMPONENT_VERSION}",
        f"  source_revision: {_yaml_scalar(_git_revision(ctx.repo_root))}",
        "platform:",
        f"  os: {ctx.platform_name}",
        f"  architecture: {ctx.arch}",
        f"  toolchain: {_yaml_scalar(_cmake_cache_value(ctx.build_dir, 'CMAKE_C_COMPILER'))}",
        "install:",
        f"  prefix: {_yaml_scalar(install_dir)}",
        "capabilities:",
    ]
    for key, value in capabilities.items():
        lines.append(f"  {key}: {_yaml_scalar(value)}")
    lines.extend([
        "build_limits: {}",
        "dependencies:",
        "  hakoniwa-core-pro:",
        f"    version: {_yaml_scalar(dependency['version'])}",
        f"    source_revision: {_yaml_scalar(dependency['source_revision'])}",
        "    build_limits:",
    ])
    for key, value in dependency["build_limits"].items():
        lines.append(f"      {key}: {_yaml_scalar(value)}")
    lines.append("artifacts:")
    for path, kind in artifacts:
        lines.extend([
            f"  - path: {_yaml_scalar(path.as_posix())}",
            f"    kind: {kind}",
        ])
    lines.append(f"resolved_manifest: {_yaml_scalar(resolved_relative.as_posix())}")
    receipt = receipt_root / f"{COMPONENT_ID}.yaml"
    if not ctx.dry_run:
        receipt_root.mkdir(parents=True, exist_ok=True)
        receipt.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return receipt


def _remove_disabled_artifacts(ctx: BuildContext, install_dir: Path) -> None:
    disabled: list[Path] = []
    if not ctx.cfg["components"]["hakotime"]:
        disabled.extend([
            Path("bin/hakotime.dll"),
            Path("lib/hakotime_import.lib"),
            Path("lib/libhakotime.so"),
            Path("lib/libhakotime.dylib"),
        ])
    if not ctx.cfg["components"]["hakopdu_ev3"]:
        disabled.extend([
            Path("bin/hakopdu_ev3.dll"),
            Path("lib/hakopdu_ev3_import.lib"),
            Path("lib/libhakopdu_ev3.so"),
            Path("lib/libhakopdu_ev3.dylib"),
            Path("include/hako_exdev/hakopdu_ev3.h"),
        ])
    for relative in disabled:
        artifact = install_dir / relative
        if artifact.is_file():
            artifact.unlink()


def install(ctx: BuildContext, install_dir: Path) -> None:
    if not (ctx.build_dir / "CMakeCache.txt").is_file():
        raise ConfigError("build directory is missing; run build first")
    _remove_disabled_artifacts(ctx, install_dir)
    _run(ctx, [
        "cmake", "--install", _cmake_path(ctx, ctx.build_dir),
        "--prefix", _cmake_path(ctx, install_dir),
        "--config", ctx.cfg["build"]["type"],
    ])
    print(f"Component Receipt: {write_receipt(ctx, install_dir)}")


def doctor(ctx: BuildContext) -> bool:
    checks = {
        "cmake": shutil.which("cmake") is not None,
        "Athrill EXDEV ABI": (
            ctx.athrill_root / "src" / "inc" / "athrill_exdev.h"
        ).is_file(),
        "Hakoniwa Core CMake package": (
            ctx.core_root / "lib" / "cmake" / "hakoniwa-core"
        ).is_dir(),
        "Hakoniwa Core Receipt": (
            ctx.core_root / "share" / "hakoniwa" / "receipts"
            / "hakoniwa-core-pro.yaml"
        ).is_file(),
    }
    for name, ok in checks.items():
        print(f"{'OK' if ok else 'NG'} {name}")
    print(f"INFO platform={ctx.platform_name}/{ctx.arch}")
    print(f"INFO build_dir={ctx.build_dir}")
    print(f"INFO athrill_root={ctx.athrill_root}")
    print(f"INFO hakoniwa_core_root={ctx.core_root}")
    if ctx.platform_name == "windows" and str(ctx.repo_root).startswith("\\\\"):
        print("INFO UNC workspace detected; hako.py maps it with cmd pushd")
    return all(checks.values())


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Business Pack build entry point for Athrill Hakoniwa EXDEVs"
    )
    parser.add_argument(
        "--config", help="build manifest (default: repository root/hakoniwa-build.yaml)"
    )
    parser.add_argument("--install-dir", help="install prefix (required for install)")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "operation",
        choices=("doctor", "configure", "build", "test", "install", "smoke"),
    )
    args = parser.parse_args(argv)
    root = repo_root()
    manifest = Path(args.config).resolve() if args.config else root / "hakoniwa-build.yaml"
    try:
        ctx = create_context(manifest, root, args.dry_run)
        if args.operation == "doctor":
            return 0 if doctor(ctx) else 1
        if args.operation == "configure":
            configure(ctx)
        elif args.operation == "build":
            build(ctx)
        elif args.operation in {"test", "smoke"}:
            smoke(ctx)
        elif args.operation == "install":
            if not args.install_dir:
                raise ConfigError("install requires --install-dir")
            install(ctx, Path(args.install_dir).resolve())
        return 0
    except (ConfigError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
