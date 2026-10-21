#!/usr/bin/env python3
"""Open the real rqt_graph window, set it to show every topic, and capture it.

Companion to rqt_export.py. That one produces rqt_graph's DOT without a GUI;
this one proves the GUI itself opens on this machine (WSLg, in our case) and
records what a person would see, with the settings that matter already set:

    Nodes/Topics (all)            graph_type_combo_box = 2
    Hide: Dead sinks    UNTICKED  dead_sinks_check_box
    Hide: Leaf topics   UNTICKED  leaf_topics_check_box
    Hide: Debug         ticked    quiet_check_box
    Hide: Params        ticked    hide_dynamic_reconfigure_check_box
    Group: Namespaces   2         namespace_cluster_spin_box
    Group: tf           ticked    group_tf_check_box

It is `ros2 run rqt_graph rqt_graph` -- literally rqt_gui.main.Main with the
same standalone argument -- plus a QTimer that, once the window is up:

  1. seeds ~/.config/ros.org/rqt_gui.ini with the settings above (done before
     launch, so rqt restores them), and ALSO sets them on the widget directly,
     because the picture must be right even if rqt's settings restore has
     other ideas;
  2. waits for DDS discovery to settle, then presses Refresh -- rqt_graph
     fetches the graph once at construction, before discovery is complete,
     and never again unless asked;
  3. presses Fit, grabs the whole window to <out>_grab.png, and renders the
     graph scene to <out>_scene.svg / <out>_scene.png exactly as the GUI's own
     "Save as SVG" / "Save as image" buttons do (same QSvgGenerator / QImage
     code, minus the file dialog);
  4. writes <out>_ready (containing the window title) so a supervisor can
     take its own screenshot from outside, then waits for <out>_close (or
     RQT_SHOT_HOLD_S seconds) and closes the window cleanly.

Configuration is by environment, because qt_gui parses argv itself:

    RQT_SHOT_OUT        output base path, no extension        (required)
    RQT_SHOT_DELAY_MS   ms to wait before the first Refresh   (default 8000)
    RQT_SHOT_HOLD_S     seconds to keep the window open        (default 120)
    RQT_SHOT_NS_FILTER  node filter text                       (default '/')

Usage (with ROS sourced, the stack running, and a display):
    RQT_SHOT_OUT=/tmp/rqt_sim python3 flight_ops/rqt_gui_shot.py
"""

from __future__ import annotations

import os
import shutil
import sys
import time
from pathlib import Path

OUT = Path(os.environ.get("RQT_SHOT_OUT", "")).expanduser()
DELAY_MS = int(os.environ.get("RQT_SHOT_DELAY_MS", "8000"))
HOLD_S = float(os.environ.get("RQT_SHOT_HOLD_S", "120"))
NS_FILTER = os.environ.get("RQT_SHOT_NS_FILTER", "/")

# rqt_graph's own settings keys (rqt_graph/ros_graph.py, save_settings), with
# the values that make it show every topic.
SETTINGS = {
    "graph_type_combo_box_index": 2,           # Nodes/Topics (all)
    "filter_line_edit_text": NS_FILTER,
    "topic_filter_line_edit_text": "/",
    "namespace_cluster_spin_box_value": 2,
    "actionlib_check_box_state": True,
    "dead_sinks_check_box_state": False,       # SHOW dead sinks
    "leaf_topics_check_box_state": False,      # SHOW leaf topics
    "quiet_check_box_state": True,
    "unreachable_check_box_state": True,
    "auto_fit_graph_check_box_state": True,
    "highlight_connections_check_box_state": True,
    "group_tf_check_box_state": True,
    "hide_tf_nodes_check_box_state": False,
    "group_image_check_box_state": True,
    "hide_dynamic_reconfigure_check_box_state": True,
}
# Where the standalone rqt_graph keeps its instance settings (qt_gui
# perspective_manager: '@' + plugin id with '/' -> '__', then plugin_manager
# and the direct plugin handler add their groups).
SETTINGS_GROUP = ("perspective/@rqt_graph__RosGraph/pluginmanager/"
                  "plugin__rqt_graph__RosGraph__0/plugin")


def seed_ini() -> None:
    from python_qt_binding.QtCore import QSettings
    settings = QSettings(QSettings.IniFormat, QSettings.UserScope, "ros.org", "rqt_gui")
    ini = Path(settings.fileName())
    if ini.exists():
        backup = ini.with_suffix(".ini.rqt_gui_shot.bak")
        if not backup.exists():
            shutil.copy2(ini, backup)
            print(f"rqt_gui_shot: backed up {ini} -> {backup}")
    settings.beginGroup(SETTINGS_GROUP)
    for key, value in SETTINGS.items():
        settings.setValue(key, value)
    settings.endGroup()
    settings.sync()
    print(f"rqt_gui_shot: seeded {ini}")


def apply_to_widget(ui) -> None:
    """Set the same state straight on the widgets, whatever was restored."""
    ui.graph_type_combo_box.setCurrentIndex(SETTINGS["graph_type_combo_box_index"])
    ui.filter_line_edit.setText(SETTINGS["filter_line_edit_text"])
    ui.topic_filter_line_edit.setText(SETTINGS["topic_filter_line_edit_text"])
    ui.namespace_cluster_spin_box.setValue(SETTINGS["namespace_cluster_spin_box_value"])
    for box in ("actionlib", "dead_sinks", "leaf_topics", "quiet", "unreachable",
                "auto_fit_graph", "highlight_connections", "group_tf", "hide_tf_nodes",
                "group_image", "hide_dynamic_reconfigure"):
        getattr(ui, f"{box}_check_box").setChecked(SETTINGS[f"{box}_check_box_state"])


def main() -> int:
    if not os.environ.get("RQT_SHOT_OUT"):
        print("rqt_gui_shot: set RQT_SHOT_OUT to the output base path", file=sys.stderr)
        return 2
    OUT.parent.mkdir(parents=True, exist_ok=True)
    for suffix in ("_ready", "_close"):
        Path(f"{OUT}{suffix}").unlink(missing_ok=True)

    seed_ini()

    from rqt_gui.main import Main

    class ShotMain(Main):
        def create_application(self, argv):
            app = super().create_application(argv)
            from python_qt_binding.QtCore import QTimer
            self._stage = 0
            self._t0 = 0.0
            self._timer = QTimer()
            self._timer.setSingleShot(True)
            self._timer.timeout.connect(self._step)
            self._timer.start(DELAY_MS)
            return app

        def _find(self):
            from python_qt_binding.QtWidgets import QApplication, QMainWindow
            ui = next((w for w in QApplication.allWidgets()
                       if w.objectName() == "RosGraphUi"), None)
            win = next((w for w in QApplication.topLevelWidgets()
                        if isinstance(w, QMainWindow)), None)
            return ui, win

        def _step(self):
            from python_qt_binding.QtWidgets import QApplication
            ui, win = self._find()
            if ui is None or win is None:
                self._timer.start(2000)
                return

            if self._stage == 0:
                win.resize(1900, 1050)
                apply_to_widget(ui)
                QApplication.processEvents()
                # pressed -> _update_rosgraph -> Graph.update -> dot layout, synchronous
                ui.refresh_graph_push_button.pressed.emit()
                QApplication.processEvents()
                self._stage = 1
                self._timer.start(3000)
                return

            if self._stage == 1:
                from python_qt_binding.QtCore import QRectF
                from python_qt_binding.QtGui import QImage, QPainter
                from python_qt_binding.QtSvg import QSvgGenerator
                ui.fit_in_view_push_button.pressed.emit()
                QApplication.processEvents()

                win.grab().save(f"{OUT}_grab.png")

                # Mirror of RosGraph._save_svg / _save_image.
                scene = ui.graphics_view.scene()
                size = (scene.sceneRect().size() * 2.0).toSize()
                gen = QSvgGenerator()
                gen.setFileName(f"{OUT}_scene.svg")
                gen.setSize(size)
                gen.setViewBox(QRectF(0, 0, size.width(), size.height()))
                painter = QPainter(gen)
                painter.setRenderHint(QPainter.Antialiasing)
                scene.render(painter)
                painter.end()

                img = QImage(size, QImage.Format_ARGB32_Premultiplied)
                img.fill(0xFFFFFFFF)
                painter = QPainter(img)
                painter.setRenderHint(QPainter.Antialiasing)
                scene.render(painter)
                painter.end()
                img.save(f"{OUT}_scene.png")

                Path(f"{OUT}_ready").write_text(win.windowTitle() + "\n", encoding="utf-8")
                print(f"rqt_gui_shot: captured; window title '{win.windowTitle()}'; "
                      f"holding for up to {HOLD_S:.0f}s or until {OUT}_close exists")
                self._t0 = time.monotonic()
                self._stage = 2
                self._timer.start(500)
                return

            if Path(f"{OUT}_close").exists() or time.monotonic() - self._t0 > HOLD_S:
                print("rqt_gui_shot: closing")
                win.close()      # closeEvent -> settings saved -> app quits
                return
            self._timer.start(500)

    # Identical to `ros2 run rqt_graph rqt_graph` (rqt_graph/main.py).
    return ShotMain().main([sys.argv[0]], standalone="rqt_graph.ros_graph.RosGraph")


if __name__ == "__main__":
    sys.exit(main())
