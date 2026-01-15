import datetime
import json
import os

Import("env")


def _load_state(state_path):
    if not os.path.exists(state_path):
        return {}
    try:
        with open(state_path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError):
        return {}


def _write_state(state_path, state):
    with open(state_path, "w", encoding="utf-8") as handle:
        json.dump(state, handle, indent=2, sort_keys=True)


def _parse_version(version_str):
    parts = version_str.strip().split(".")
    if len(parts) != 3:
        raise ValueError("Version must be in MAJOR.MINOR.PATCH format")
    major, minor, patch = parts
    return int(major), int(minor), int(patch)


def _format_version(major, minor, patch):
    return f"{major}.{minor}.{patch}"


def _write_version_header(header_path, version_str):
    header = f"""
    // AUTO GENERATED FILE, DO NOT EDIT
    #ifndef VERSION
        #define VERSION "{version_str}"
    #endif
    #ifndef BUILD_TIMESTAMP
        #define BUILD_TIMESTAMP "{datetime.datetime.now()}"
    #endif
    """
    with open(header_path, "w", encoding="utf-8") as handle:
        handle.write(header)


def _ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)


def main():
    if env.subst("$PIOENV") == "native":
        return

    project_dir = env.subst("$PROJECT_DIR")
    version_path = os.path.join(project_dir, "version")
    state_path = os.path.join(project_dir, ".version_state.json")

    state = _load_state(state_path)
    last_success_version = state.get("last_success_version")
    last_success_mtime = state.get("last_success_mtime")
    last_build_success = bool(state.get("last_build_success", False))

    if not os.path.exists(version_path):
        _ensure_dir(project_dir)
        with open(version_path, "w", encoding="utf-8") as handle:
            handle.write("0.1.0")

    with open(version_path, "r", encoding="utf-8") as handle:
        version_str = handle.read().strip()

    current_mtime = os.path.getmtime(version_path)
    version_changed_since_success = (
        last_success_version != version_str or last_success_mtime != current_mtime
    )

    auto_incremented = False
    if last_build_success and not version_changed_since_success:
        major, minor, patch = _parse_version(version_str)
        patch += 1
        version_str = _format_version(major, minor, patch)
        with open(version_path, "w", encoding="utf-8") as handle:
            handle.write(version_str)
        current_mtime = os.path.getmtime(version_path)
        version_changed_since_success = True
        auto_incremented = True

    include_dir = os.environ.get("PLATFORMIO_INCLUDE_DIR")
    if include_dir is None:
        include_dir = os.path.join(project_dir, "include")
    _ensure_dir(include_dir)
    header_path = os.path.join(include_dir, "Version.h")
    if version_changed_since_success or not os.path.exists(header_path):
        _write_version_header(header_path, version_str)

    state["last_build_success"] = False
    state["last_seen_version"] = version_str
    state["last_seen_mtime"] = current_mtime
    state["last_header_written"] = datetime.datetime.now().isoformat()
    _write_state(state_path, state)

    if auto_incremented:
        print(f"Build number: {version_str} (auto-incremented)")
    else:
        print(f"Build number: {version_str}")


main()
