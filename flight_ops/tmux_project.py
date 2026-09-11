#!/usr/bin/env python3
"""tmux_project.py -- start a tmuxinator project without tmuxinator.

WHY THIS EXISTS

session_bringup.sh drives two tmuxinator project files
(lab_config/o134_project/tmuxinator/{drone,ground_station}.yaml). tmuxinator
is a Ruby gem; a Jetson has no Ruby, no sudo for the operator, and no route
to rubygems from the lab. Found on 2026-09-11: the launcher could not run on
the aircraft at all, and the drone role had to be typed in by hand -- which is
exactly the drift the launcher exists to prevent.

This runner takes THE SAME project files and THE SAME command line as
`tmuxinator start`, renders the ERB the way tmuxinator would, and drives tmux
directly. The launcher uses tmuxinator when it is installed and this when it
is not, so both machines run the same project files.

    tmux_project.py start  -n SESSION -p PROJECT.yaml [key=value ...]
    tmux_project.py plan   -n SESSION -p PROJECT.yaml [key=value ...]
    tmux_project.py render -p PROJECT.yaml [key=value ...]
    tmux_project.py --self-test

`plan` prints the tmux commands and runs nothing. `render` prints the YAML
after ERB. Neither needs tmux.

THE ERB SUBSET, ON PURPOSE

Only what the two project files use is implemented, and anything else is a
hard error rather than a guess:

    <% name = @settings["key"]            %>   prelude assignments
    <% name = (other == "literal")        %>
    <% name = @settings["key"] == "true"  %>
    <%= name %>                                substitution
    <%- if name -%> ... <%- else -%> ... <%- end -%>
    <%- if !name -%>
    <%- name.split(',').each do |v| -%> ... <%- end -%>

A project file that starts using some other Ruby will fail here with a line
number, not render something subtly different from what tmuxinator renders.
Rendering parity with tmuxinator 3.3.7 was checked pane by pane on the ground
station (2026-09-11).

WHAT tmux SEES

The same sequence tmuxinator emits: new-session -d with the first window;
new-window for the rest; split-window + `select-layout tiled` per extra pane
(so a small detached window never runs out of space); `pre_window` then the
pane command sent with send-keys to every pane; the window's own layout last;
startup_window selected. Panes and windows are addressed by their tmux ids
(%N / @N), so base-index and pane-base-index settings do not matter.

Exit codes: 0 done; 1 the self-test failed; 2 bad arguments, bad project
file, tmux missing, or the session already exists.
"""

from __future__ import annotations

import argparse
import re
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple, Union

try:
    import yaml
except ImportError:  # pragma: no cover -- both lab machines have it
    yaml = None

HERE = Path(__file__).resolve().parent
PROJECTS_DIR = HERE / "lab_config" / "o134_project" / "tmuxinator"

# A detached session has no client to size it, and tmux's 80x24 default is
# too small for five stacked panes plus borders. Same as deploy/mocap_bringup.sh.
DETACHED_SIZE = ("-x", "220", "-y", "50")

Value = Union[str, bool]


class ProjectError(Exception):
    """Anything wrong with the project file or the settings."""


# --------------------------------------------------------------------------- #
# ERB -- the subset
# --------------------------------------------------------------------------- #

_PRELUDE_RE = re.compile(r"\A\s*<%(?![=-])(.*?)%>", re.S)
_ASSIGN_RE = re.compile(r"^\s*([A-Za-z_]\w*)\s*=\s*(.+?)\s*$")
_SETTING_RE = re.compile(r'^@settings\["([^"]+)"\]$')
_LITERAL_RE = re.compile(r'^"([^"]*)"$')
_IDENT_RE = re.compile(r"^[A-Za-z_]\w*$")
_TAG_LINE_RE = re.compile(r"^\s*<%-?\s*(.*?)\s*-?%>\s*$")
_SUBST_RE = re.compile(r"<%=\s*([A-Za-z_]\w*)\s*%>")
_ANY_TAG_RE = re.compile(r"<%")
_EACH_RE = re.compile(r"^([A-Za-z_]\w*)\.split\('([^']*)'\)\.each\s+do\s+\|([A-Za-z_]\w*)\|$")
_IF_RE = re.compile(r"^if\s+(!?)([A-Za-z_]\w*)$")


def parse_settings(pairs: Sequence[str]) -> Dict[str, str]:
    """key=value arguments exactly as tmuxinator takes them: split on the
    FIRST '=', so a value may itself contain '=' (middleware_env does)."""
    out: Dict[str, str] = {}
    for item in pairs:
        if "=" not in item:
            raise ProjectError(f"setting '{item}' is not key=value")
        key, _, value = item.partition("=")
        if not _IDENT_RE.match(key):
            raise ProjectError(f"setting name '{key}' is not an identifier")
        out[key] = value
    return out


def _eval_term(term: str, settings: Dict[str, str],
               variables: Dict[str, Value]) -> Value:
    m = _SETTING_RE.match(term)
    if m:
        key = m.group(1)
        if key not in settings:
            raise ProjectError(f"project needs setting '{key}' and it was not given")
        return settings[key]
    m = _LITERAL_RE.match(term)
    if m:
        return m.group(1)
    if _IDENT_RE.match(term):
        if term not in variables:
            raise ProjectError(f"prelude uses '{term}' before assigning it")
        return variables[term]
    raise ProjectError(f"cannot evaluate '{term}' (not a setting, literal or variable)")


def _eval_expr(expr: str, settings: Dict[str, str],
               variables: Dict[str, Value]) -> Value:
    expr = expr.strip()
    if expr.startswith("(") and expr.endswith(")"):
        expr = expr[1:-1].strip()
    if "==" in expr:
        left, _, right = expr.partition("==")
        return _eval_term(left.strip(), settings, variables) \
            == _eval_term(right.strip(), settings, variables)
    return _eval_term(expr, settings, variables)


def _parse_prelude(text: str, settings: Dict[str, str]
                   ) -> Tuple[Dict[str, Value], str]:
    """The leading <% ... %> block: assignments only. Returns the variables
    and the body that follows."""
    variables: Dict[str, Value] = {}
    m = _PRELUDE_RE.match(text)
    if not m:
        return variables, text
    for lineno, raw in enumerate(m.group(1).splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        a = _ASSIGN_RE.match(line)
        if not a:
            raise ProjectError(f"prelude line {lineno}: not an assignment: {line!r}")
        variables[a.group(1)] = _eval_expr(a.group(2), settings, variables)
    body = text[m.end():]
    if body.startswith("\n"):
        body = body[1:]
    return variables, body


def _to_text(value: Value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _substitute(line: str, variables: Dict[str, Value], lineno: int) -> str:
    def repl(m: "re.Match[str]") -> str:
        name = m.group(1)
        if name not in variables:
            raise ProjectError(f"line {lineno}: <%= {name} %> is not defined")
        return _to_text(variables[name])
    out = _SUBST_RE.sub(repl, line)
    if _ANY_TAG_RE.search(out):
        raise ProjectError(f"line {lineno}: unsupported ERB in {line.strip()!r}")
    return out


def _render_block(lines: List[Tuple[int, str]], pos: int,
                  variables: Dict[str, Value], out: List[str],
                  stop_at: Sequence[str]) -> Tuple[int, Optional[str]]:
    """Render from `pos` until a control tag in `stop_at` ('else'/'end') or
    the end of input. Returns (next position, the tag that stopped us)."""
    while pos < len(lines):
        lineno, raw = lines[pos]
        tag = _TAG_LINE_RE.match(raw)
        if tag and not raw.lstrip().startswith("<%="):
            body = tag.group(1)
            if body in stop_at:
                return pos + 1, body
            if body in ("else", "end"):
                raise ProjectError(f"line {lineno}: unexpected '{body}'")
            m = _IF_RE.match(body)
            if m:
                negate, name = m.group(1) == "!", m.group(2)
                if name not in variables:
                    raise ProjectError(f"line {lineno}: if {name}: not defined")
                cond = bool(variables[name]) != negate
                then_out: List[str] = []
                else_out: List[str] = []
                pos, stopped = _render_block(lines, pos + 1, variables, then_out,
                                             ("else", "end"))
                if stopped == "else":
                    pos, stopped = _render_block(lines, pos, variables, else_out,
                                                 ("end",))
                if stopped != "end":
                    raise ProjectError(f"line {lineno}: 'if' without 'end'")
                out.extend(then_out if cond else else_out)
                continue
            m = _EACH_RE.match(body)
            if m:
                name, sep, var = m.group(1), m.group(2), m.group(3)
                if name not in variables:
                    raise ProjectError(f"line {lineno}: each over {name}: not defined")
                items = [s for s in _to_text(variables[name]).split(sep)]
                # Find the loop body once, then render it per item.
                body_lines: List[str] = []
                scan = pos + 1
                depth = 0
                while scan < len(lines):
                    t = _TAG_LINE_RE.match(lines[scan][1])
                    if t and not lines[scan][1].lstrip().startswith("<%="):
                        b = t.group(1)
                        if _IF_RE.match(b) or _EACH_RE.match(b):
                            depth += 1
                        elif b == "end":
                            if depth == 0:
                                break
                            depth -= 1
                    scan += 1
                else:
                    raise ProjectError(f"line {lineno}: 'each' without 'end'")
                body_pairs = lines[pos + 1:scan]
                for item in items:
                    inner = dict(variables)
                    inner[var] = item
                    _render_block(body_pairs, 0, inner, out, ())
                pos = scan + 1
                continue
            raise ProjectError(f"line {lineno}: unsupported ERB tag {raw.strip()!r}")
        out.append(_substitute(raw, variables, lineno))
        pos += 1
    return pos, None


def render(text: str, settings: Dict[str, str]) -> str:
    """ERB subset -> YAML text."""
    variables, body = _parse_prelude(text, settings)
    lines = list(enumerate(body.splitlines(), start=1))
    out: List[str] = []
    _, stopped = _render_block(lines, 0, variables, out, ())
    if stopped is not None:
        raise ProjectError(f"stray '{stopped}'")
    return "\n".join(out) + "\n"


# --------------------------------------------------------------------------- #
# project -> tmux commands
# --------------------------------------------------------------------------- #

class Window:
    def __init__(self, name: str, layout: Optional[str], panes: List[str]) -> None:
        self.name = name
        self.layout = layout
        self.panes = panes


class Project:
    def __init__(self, name: str, pre_window: str, startup_window: Optional[str],
                 windows: List[Window]) -> None:
        self.name = name
        self.pre_window = pre_window
        self.startup_window = startup_window
        self.windows = windows


_KNOWN_KEYS = {"name", "attach", "startup_window", "pre_window", "windows", "root"}


def load_project(rendered: str) -> Project:
    if yaml is None:
        raise ProjectError("python3-yaml is not installed")
    try:
        doc = yaml.safe_load(rendered)
    except yaml.YAMLError as exc:
        raise ProjectError(f"rendered project is not valid YAML: {exc}") from exc
    if not isinstance(doc, dict):
        raise ProjectError("rendered project is not a mapping")
    unknown = sorted(set(doc) - _KNOWN_KEYS)
    if unknown:
        raise ProjectError(f"unsupported project keys: {', '.join(unknown)}")
    name = str(doc.get("name") or "").strip()
    if not name:
        raise ProjectError("project has no name")
    pre_window = str(doc.get("pre_window") or "").strip()
    startup = doc.get("startup_window")
    windows_raw = doc.get("windows")
    if not isinstance(windows_raw, list) or not windows_raw:
        raise ProjectError("project has no windows")
    windows: List[Window] = []
    for entry in windows_raw:
        if not isinstance(entry, dict) or len(entry) != 1:
            raise ProjectError(f"window entry is not a single-key mapping: {entry!r}")
        (wname, spec), = entry.items()
        wname = str(wname)
        if isinstance(spec, str):
            layout, panes_raw = None, [spec]
        elif isinstance(spec, dict):
            extra = sorted(set(spec) - {"layout", "panes"})
            if extra:
                raise ProjectError(f"window {wname}: unsupported keys {', '.join(extra)}")
            layout = spec.get("layout")
            panes_raw = spec.get("panes")
        else:
            raise ProjectError(f"window {wname}: unsupported spec {spec!r}")
        if not isinstance(panes_raw, list) or not panes_raw:
            raise ProjectError(f"window {wname}: needs a non-empty panes list")
        panes: List[str] = []
        for pane in panes_raw:
            if not isinstance(pane, str) or not pane.strip():
                raise ProjectError(f"window {wname}: every pane must be a command string, got {pane!r}")
            panes.append(" ".join(pane.split()))
        windows.append(Window(wname, None if layout is None else str(layout), panes))
    names = [w.name for w in windows]
    if len(set(names)) != len(names):
        raise ProjectError("window names must be unique")
    if startup is not None and str(startup) not in names:
        raise ProjectError(f"startup_window '{startup}' is not a window")
    return Project(name, pre_window, None if startup is None else str(startup), windows)


def plan(project: Project, session: str) -> List[List[str]]:
    """The tmux commands, with symbolic targets (SESSION:window.pane). `start`
    replaces those with the ids tmux hands back."""
    cmds: List[List[str]] = []
    first = project.windows[0]
    cmds.append(["tmux", "new-session", "-d", "-s", session, "-n", first.name, *DETACHED_SIZE])
    for i, win in enumerate(project.windows):
        target = f"{session}:{win.name}"
        if i > 0:
            cmds.append(["tmux", "new-window", "-t", session, "-n", win.name])
        for j, command in enumerate(win.panes):
            if j > 0:
                cmds.append(["tmux", "split-window", "-t", target])
                cmds.append(["tmux", "select-layout", "-t", target, "tiled"])
            pane = f"{target}.{j}"
            if project.pre_window:
                cmds.append(["tmux", "send-keys", "-t", pane, "-l", project.pre_window])
                cmds.append(["tmux", "send-keys", "-t", pane, "Enter"])
            cmds.append(["tmux", "send-keys", "-t", pane, "-l", command])
            cmds.append(["tmux", "send-keys", "-t", pane, "Enter"])
        if win.layout:
            cmds.append(["tmux", "select-layout", "-t", target, win.layout])
    startup = project.startup_window or first.name
    cmds.append(["tmux", "select-window", "-t", f"{session}:{startup}"])
    return cmds


def _tmux(args: Sequence[str], capture: bool = False) -> str:
    proc = subprocess.run(["tmux", *args], capture_output=True, text=True)
    if proc.returncode != 0:
        raise ProjectError(f"tmux {' '.join(args)} failed: {proc.stderr.strip() or proc.stdout.strip()}")
    return proc.stdout.strip() if capture else ""


def start(project: Project, session: str) -> None:
    """Run the plan against a live tmux, resolving windows and panes to ids."""
    if subprocess.run(["tmux", "-V"], capture_output=True).returncode != 0:
        raise ProjectError("tmux is not installed")
    if subprocess.run(["tmux", "has-session", "-t", f"={session}"],
                      capture_output=True).returncode == 0:
        raise ProjectError(f"tmux session '{session}' already exists")
    first = project.windows[0]
    _tmux(["new-session", "-d", "-s", session, "-n", first.name, *DETACHED_SIZE])
    window_ids: Dict[str, str] = {}
    for i, win in enumerate(project.windows):
        if i == 0:
            wid = _tmux(["display-message", "-p", "-t", f"={session}:", "#{window_id}"], capture=True)
        else:
            wid = _tmux(["new-window", "-t", f"={session}:", "-n", win.name, "-P", "-F", "#{window_id}"],
                        capture=True)
        window_ids[win.name] = wid
        pane_ids = [_tmux(["display-message", "-p", "-t", wid, "#{pane_id}"], capture=True)]
        for j in range(1, len(win.panes)):
            pane_ids.append(_tmux(["split-window", "-t", wid, "-P", "-F", "#{pane_id}"], capture=True))
            _tmux(["select-layout", "-t", wid, "tiled"])
        for pid, command in zip(pane_ids, win.panes):
            if project.pre_window:
                _tmux(["send-keys", "-t", pid, "-l", project.pre_window])
                _tmux(["send-keys", "-t", pid, "Enter"])
            _tmux(["send-keys", "-t", pid, "-l", command])
            _tmux(["send-keys", "-t", pid, "Enter"])
        if win.layout:
            _tmux(["select-layout", "-t", wid, win.layout])
        _tmux(["select-pane", "-t", pane_ids[0]])
    startup = project.startup_window or first.name
    _tmux(["select-window", "-t", window_ids[startup]])


# --------------------------------------------------------------------------- #
# self-test -- no tmux, no ROS
# --------------------------------------------------------------------------- #

def _base_settings(**over: str) -> Dict[str, str]:
    s = {
        "drone_namespace": "drone0", "mode": "hardware",
        "project_dir": "/p/o134_project", "repo_root": "/p", "ws_setup": "/w/setup.bash",
        "ros_domain_id": "0",
        "middleware_env": "export RMW_IMPLEMENTATION='rmw_cyclonedds_cpp' && export CYCLONEDDS_URI='file:///tmp/x.xml'",
        "fmu_prefix": "/uav_0", "target_system_id": "1", "rigid_body_name": "drone0",
        "mocap_topic": "/drone0/mocap/rigid_bodies", "log_level": "info",
        "drones_comma": "drone0,drone1", "guard_args": "--drones drone0,drone1 --dry-run",
        "guard_ros_args": "-r /drone0/fmu/in/vehicle_command:=/uav_0/fmu/in/vehicle_command",
        "mocap_bodies": "drone0 drone1", "rviz": "false", "viewer": "true",
    }
    s.update(over)
    return s


def self_test() -> int:  # noqa: C901 -- a flat list of cases reads better
    failures: List[str] = []

    def check(label: str, condition: bool) -> None:
        print(f"  [{'PASS' if condition else 'FAIL'}] {label}")
        if not condition:
            failures.append(label)

    def raises(label: str, fn, *args) -> None:
        try:
            fn(*args)
            check(label, False)
        except ProjectError:
            check(label, True)

    print("settings")
    check("key=value splits on the first '='",
          parse_settings(["a=b=c", "x="]) == {"a": "b=c", "x": ""})
    raises("a bare word is refused", parse_settings, ["nope"])

    print("ERB subset")
    tpl = ('<%\n# comment\nmode = @settings["mode"]\nsim = (mode == "sim")\n'
           'on = @settings["flag"] == "true"\n%>\n'
           'name: x_<%= mode %>\n'
           '<%- if sim -%>\n  a: sim\n<%- else -%>\n  a: hw\n<%- end -%>\n'
           '<%- if !on -%>\n  off: yes\n<%- end -%>\n'
           '<%- list.split(\',\').each do |d| -%>\n  - item_<%= d %>\n<%- end -%>\n')
    tpl_list = tpl.replace("<%\n", '<%\nlist = @settings["list"]\n', 1)
    out = render(tpl_list, {"mode": "sim", "flag": "false", "list": "p,q"})
    check("prelude, if/else, negated if and each all render",
          out == "name: x_sim\n  a: sim\n  off: yes\n  - item_p\n  - item_q\n")
    out = render(tpl_list, {"mode": "hardware", "flag": "true", "list": "z"})
    check("the other branches render", out == "name: x_hardware\n  a: hw\n  - item_z\n")
    raises("a missing setting is an error, not nil",
           render, tpl_list, {"mode": "sim", "flag": "false"})
    raises("an unknown tag is an error",
           render, '<% a = @settings["a"] %>\n<%- a.upcase -%>\n', {"a": "x"})
    raises("an undefined substitution is an error", render, "v: <%= nope %>\n", {})
    raises("Ruby in the prelude is an error",
           render, '<% a = @settings["a"].to_i + 1 %>\nv: 1\n', {"a": "1"})
    raises("if without end is an error",
           render, '<% a = @settings["a"] == "1" %>\n<%- if a -%>\nx: 1\n', {"a": "1"})
    check("booleans substitute as tmuxinator prints them",
          render('<% a = @settings["a"] == "1" %>\nv: <%= a %>\n', {"a": "1"}) == "v: true\n")

    print("the real project files")
    drone_tpl = (PROJECTS_DIR / "drone.yaml").read_text(encoding="utf-8")
    ground_tpl = (PROJECTS_DIR / "ground_station.yaml").read_text(encoding="utf-8")

    hw = load_project(render(drone_tpl, _base_settings()))
    check("drone.yaml (hardware): 3 windows, 5 panes",
          [w.name for w in hw.windows] == ["platform", "basics", "behaviors"]
          and sum(len(w.panes) for w in hw.windows) == 5)
    check("drone.yaml (hardware): the platform pane is the pixhawk launch",
          "as2_platform_pixhawk pixhawk_launch.py" in hw.windows[0].panes[0]
          and "fmu_prefix:=/uav_0" in hw.windows[0].panes[0]
          and "target_system_id:=1" in hw.windows[0].panes[0])
    check("drone.yaml: every pane carries the namespace",
          all("namespace:=drone0" in p for w in hw.windows for p in w.panes))
    check("drone.yaml: pre_window runs the launcher's middleware fragment",
          "export CYCLONEDDS_URI='file:///tmp/x.xml'" in hw.pre_window
          and "export ROS_DOMAIN_ID=0" in hw.pre_window)
    check("drone.yaml: the session is named after the namespace", hw.name == "o134_drone0")
    sim = load_project(render(drone_tpl, _base_settings(mode="sim")))
    check("drone.yaml (sim): the platform pane is the simulator",
          "as2_platform_multirotor_simulator" in sim.windows[0].panes[0]
          and "pixhawk" not in sim.windows[0].panes[0])
    check("drone.yaml: mocap_topic reaches the estimator launch",
          any("mocap_topic:=/drone0/mocap/rigid_bodies" in p for p in hw.windows[1].panes))

    gs = load_project(render(ground_tpl, _base_settings()))
    check("ground_station.yaml (hardware, 2 drones, no rviz): safety + monitoring only",
          [w.name for w in gs.windows] == ["safety", "monitoring"])
    check("ground_station.yaml (hardware): one safety pane -- the guard, no fake_mocap",
          len(gs.windows[0].panes) == 1 and "volume_guard.py" in gs.windows[0].panes[0]
          and "--dry-run" in gs.windows[0].panes[0])
    check("ground_station.yaml: guard_ros_args land inside --ros-args",
          "--ros-args --params-file" in gs.windows[0].panes[0]
          and gs.windows[0].panes[0].endswith("-r /drone0/fmu/in/vehicle_command:=/uav_0/fmu/in/vehicle_command"))
    check("ground_station.yaml: one viewer pane per drone",
          [p.split("__ns:=")[1] for p in gs.windows[1].panes] == ["/drone0", "/drone1"])
    gs_sim = load_project(render(ground_tpl, _base_settings(mode="sim", rviz="true", viewer="false")))
    check("ground_station.yaml (sim, rviz, no viewer): fake_mocap + guard, viz window",
          [w.name for w in gs_sim.windows] == ["safety", "viz"]
          and len(gs_sim.windows[0].panes) == 2
          and "fake_mocap.py" in gs_sim.windows[0].panes[0]
          and "--bodies drone0 drone1" in gs_sim.windows[0].panes[0]
          and "swarm_viz.launch.py" in gs_sim.windows[1].panes[0])
    check("ground_station.yaml: startup window is safety", gs.startup_window == "safety")

    print("the tmux plan")
    cmds = plan(hw, "o134_drone0")
    joined = [" ".join(c) for c in cmds]
    check("starts one detached session named as asked",
          joined[0].startswith("tmux new-session -d -s o134_drone0 -n platform"))
    check("two more windows", sum(1 for c in cmds if c[1] == "new-window") == 2)
    check("panes - windows splits, each followed by a tiled relayout",
          sum(1 for c in cmds if c[1] == "split-window") == 2
          and all(cmds[i + 1][1:] == ["select-layout", "-t", cmds[i][3], "tiled"]
                  for i, c in enumerate(cmds) if c[1] == "split-window"))
    sends = [c for c in cmds if c[1] == "send-keys" and "-l" in c]
    check("pre_window then the command is typed into every pane",
          len(sends) == 10 and all(sends[k][-1] == hw.pre_window for k in range(0, 10, 2)))
    check("every typed line is followed by Enter",
          all(cmds[i + 1][1:] == ["send-keys", "-t", c[3], "Enter"]
              for i, c in enumerate(cmds) if c[1] == "send-keys" and "-l" in c))
    check("each window gets its own layout last",
          [c[-1] for c in cmds if c[1] == "select-layout" and c[-1] != "tiled"]
          == ["even-vertical"] * 3)
    check("finishes on the startup window",
          cmds[-1] == ["tmux", "select-window", "-t", "o134_drone0:basics"])

    print("project validation")
    raises("a window with no panes is refused", load_project, "name: x\nwindows:\n  - w:\n      panes: []\n")
    raises("a non-string pane is refused",
           load_project, "name: x\nwindows:\n  - w:\n      panes:\n        - {a: 1}\n")
    raises("duplicate window names are refused",
           load_project, "name: x\nwindows:\n  - w: ls\n  - w: ls\n")
    raises("an unknown startup window is refused",
           load_project, "name: x\nstartup_window: q\nwindows:\n  - w: ls\n")
    raises("unknown top-level keys are refused (no silent tmuxinator features)",
           load_project, "name: x\non_project_start: rm -rf /\nwindows:\n  - w: ls\n")

    print()
    if failures:
        print(f"{len(failures)} FAILURE(S): " + "; ".join(failures))
        return 1
    print("all self-tests passed")
    return 0


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #

def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="tmux_project.py",
        description="start a tmuxinator project file without tmuxinator "
                    "(same project files, same command line)")
    ap.add_argument("--self-test", action="store_true",
                    help="run the offline test suite and exit (no tmux, no ROS)")
    sub = ap.add_subparsers(dest="command")
    for name, needs_session in (("start", True), ("plan", True), ("render", False)):
        p = sub.add_parser(name)
        if needs_session:
            p.add_argument("-n", "--name", required=True, help="tmux session name")
        p.add_argument("-p", "--project", required=True, help="project file (ERB + YAML)")
        p.add_argument("settings", nargs="*", help="key=value, as for tmuxinator")
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()
    if not args.command:
        ap.print_usage(file=sys.stderr)
        return 2

    try:
        settings = parse_settings(args.settings)
        path = Path(args.project)
        if not path.is_file():
            raise ProjectError(f"no such project file: {path}")
        rendered = render(path.read_text(encoding="utf-8"), settings)
        if args.command == "render":
            sys.stdout.write(rendered)
            return 0
        project = load_project(rendered)
        if args.command == "plan":
            for cmd in plan(project, args.name):
                print(" ".join(shlex.quote(c) for c in cmd))
            return 0
        start(project, args.name)
        return 0
    except ProjectError as exc:
        print(f"tmux_project: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
