import json
import os
from pathlib import Path

Import("env")


def _load_board_versions(json_path: Path) -> list[dict]:
    try:
        data = json.loads(json_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []
    boards = data.get("boards")
    if not isinstance(boards, list):
        return []
    return [item for item in boards if isinstance(item, dict)]


def _render_header(boards: list[dict]) -> str:
    lines = ["// AUTO GENERATED FILE, DO NOT EDIT", ""]
    for board in boards:
        pcb_version = board.get("pcb_version")
        name = board.get("name")
        if not isinstance(pcb_version, int) or not isinstance(name, str):
            continue
        lines.append(f"#if PCB_VERSION == {pcb_version}")
        lines.append(f'    #define CONTROL_BOARD "{name}"')
        lines.append("#endif")
        lines.append("")
    lines.append("")
    return "\n".join(lines)


def _write_version_board(project_dir: Path) -> None:
    json_path = project_dir / "board_versions.json"
    if not json_path.exists():
        return
    header_path = project_dir / "include" / "Version_Board.h"
    header_path.parent.mkdir(parents=True, exist_ok=True)
    boards = _load_board_versions(json_path)
    header_path.write_text(_render_header(boards), encoding="utf-8")


def main():
    if env.subst("$PIOENV") == "native":
        return
    project_dir = Path(env.subst("$PROJECT_DIR"))
    _write_version_board(project_dir)


main()
