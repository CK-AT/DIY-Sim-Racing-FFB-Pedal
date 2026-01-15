import json
import os
import zipfile
from datetime import datetime
from pathlib import Path

Import("env")


def _compute_md5(path: Path) -> str:
    import hashlib

    md5 = hashlib.md5()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            md5.update(chunk)
    return md5.hexdigest()


def _write_ffbota(source, target, env):
    if env.subst("$PIOENV") == "native":
        return

    project_dir = Path(env.subst("$PROJECT_DIR"))
    firmware_path = Path(env.subst("$PROG_PATH"))
    if not firmware_path.exists():
        print(f"[ffbota] firmware not found: {firmware_path}")
        return

    version_path = project_dir / "version"
    version = version_path.read_text(encoding="utf-8").strip() if version_path.exists() else "0.0.0"
    build_env = env.subst("$PIOENV")
    board_id = env.subst("$BOARD")

    ota_dir = project_dir.parent / "OTA"
    ota_dir.mkdir(parents=True, exist_ok=True)
    output_path = ota_dir / f"firmware_{build_env}.ffbota"

    md5_hex = _compute_md5(firmware_path)
    manifest = {
        "version": version,
        "board": board_id,
        "build_env": build_env,
        "build_timestamp": datetime.now().isoformat(),
        "md5": md5_hex,
        "size": firmware_path.stat().st_size,
        "firmware_name": firmware_path.name,
    }

    with zipfile.ZipFile(output_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("manifest.json", json.dumps(manifest, indent=2, sort_keys=True))
        archive.write(firmware_path, "firmware.bin")

    print(f"[ffbota] wrote {output_path}")


env.AddPostAction("buildprog", _write_ffbota)
