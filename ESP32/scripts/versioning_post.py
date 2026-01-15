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


def _mark_success(source, target, env):
    if env.subst("$PIOENV") == "native":
        return

    project_dir = env.subst("$PROJECT_DIR")
    version_path = os.path.join(project_dir, "version")
    state_path = os.path.join(project_dir, ".version_state.json")

    state = _load_state(state_path)
    state["last_build_success"] = True
    state["last_success_time"] = datetime.datetime.now().isoformat()

    if os.path.exists(version_path):
        with open(version_path, "r", encoding="utf-8") as handle:
            version_str = handle.read().strip()
        state["last_success_version"] = version_str
        state["last_success_mtime"] = os.path.getmtime(version_path)
        state["last_seen_version"] = version_str
        state["last_seen_mtime"] = state["last_success_mtime"]

    _write_state(state_path, state)


env.AddPostAction("buildprog", _mark_success)
