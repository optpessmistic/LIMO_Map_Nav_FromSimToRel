#!/usr/bin/env python3
"""Convert a # . S T maze to a geometry-only SDF world plus spawn metadata.

This exports simulator ground truth, never a planner map. No robot, camera
detector, ROS bridge or controller is included. Requires Python 3.6+.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def parse_maze(text):
    rows = [line.rstrip() for line in text.splitlines() if line.strip()]
    if len(rows) < 3 or len(rows[0]) < 3 or len(set(map(len, rows))) != 1:
        raise ValueError("Maze must be rectangular and at least 3x3.")
    if set("".join(rows)) - set("#.ST "):
        raise ValueError("Only # . S T and spaces are allowed.")
    starts = [(x, y) for y, row in enumerate(rows) for x, c in enumerate(row) if c == "S"]
    targets = [(x, y) for y, row in enumerate(rows) for x, c in enumerate(row) if c == "T"]
    if len(starts) != 1:
        raise ValueError("Exactly one S is required; any number of T is allowed.")
    if any(c != "#" for c in rows[0] + rows[-1]) or any(row[0] != "#" or row[-1] != "#" for row in rows):
        raise ValueError("The boundary must be closed by # walls.")
    return rows, starts[0], targets


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def box(world, name, xyz, size, color):
    model = element(world, "model", name=name)
    element(model, "static", "true")
    element(model, "pose", "{} {} {} 0 0 0".format(*xyz))
    link = element(model, "link", name="body")
    for kind in ("collision", "visual"):
        part = element(link, kind, name=kind)
        geometry = element(part, "geometry")
        element(element(geometry, "box"), "size", "{} {} {}".format(*size))
        if kind == "visual":
            material = element(part, "material")
            element(material, "ambient", color)
            element(material, "diffuse", color)


def build(text, cell=0.6, wall_height=0.6):
    if not math.isfinite(cell) or not math.isfinite(wall_height) or cell <= 0 or wall_height <= 0:
        raise ValueError("Cell size and wall height must be finite and positive.")
    rows, start, targets = parse_maze(text)
    root = ET.Element("sdf", version="1.9")
    world = element(root, "world", name="limo_maze")
    for library, system in [("physics", "Physics"), ("user-commands", "UserCommands"),
                            ("scene-broadcaster", "SceneBroadcaster"), ("sensors", "Sensors"),
                            ("imu", "Imu")]:
        plugin = element(world, "plugin", filename="gz-sim-"+library+"-system", name="gz::sim::systems::"+system)
        if system == "Sensors":
            element(plugin, "render_engine", "ogre2")
    element(world, "gravity", "0 0 -9.81")
    scene = element(world, "scene")
    element(scene, "ambient", "0.6 0.6 0.6 1")
    light = element(world, "light", name="sun", type="directional")
    element(light, "pose", "0 0 10 0 0 0")
    element(light, "diffuse", "0.8 0.8 0.8 1")
    element(light, "direction", "-0.5 0.2 -1")
    width, height = len(rows[0]), len(rows)
    # Start center is world x=y=0; row index grows downward, Gazebo y upward.
    xy = lambda x, y: ((x-start[0])*cell, (start[1]-y)*cell)
    center = xy((width-1)/2.0, (height-1)/2.0)
    box(world, "floor", (center[0], center[1], -0.05), (width*cell, height*cell, 0.1), "0.72 0.72 0.72 1")
    for y, row in enumerate(rows):
        for x, c in enumerate(row):
            if c == "#":
                px, py = xy(x, y)
                box(world, "wall_{}_{}".format(x, y), (px, py, wall_height/2),
                    (cell, cell, wall_height), "0.26 0.32 0.39 1")
    # Place a visual surrogate beside its cell center to keep the navigation
    # centerline usable. It is not an AprilTag or a validated target detector.
    target_meta = []
    for index, target in enumerate(targets):
        tx, ty = xy(*target)
        side = min(0.06, cell/10.0)
        edge = cell/2.0 - side
        box(world, "target_surrogate_{}".format(index), (tx+edge, ty, 0.10), (side, side, 0.20), "0.9 0.05 0.05 1")
        target_meta.append({"x": tx+edge, "y": ty, "z": 0.10, "appearance": "red_box_surrogate"})
    metadata = {"role": "simulator_and_offline_evaluator_only", "cell_m": cell,
                "wall_height_m": wall_height, "robot_spawn": {"x": 0, "y": 0, "yaw": 0},
                "spawn_z": "use robot model launch default and verify ground clearance",
                "targets_ground_truth": target_meta,
                "note": "World geometry only; no robot or ROS integration. Target is an unvalidated visual surrogate."}
    return root, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("maze", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cell", type=float, default=0.6)
    parser.add_argument("--wall-height", type=float, default=0.6)
    args = parser.parse_args()
    meta_path = args.output.with_suffix(".metadata.json")
    if args.output.exists() or meta_path.exists():
        parser.error("Output exists; choose a new filename.")
    try:
        root, meta = build(args.maze.read_text(encoding="utf-8"), args.cell, args.wall_height)
    except ValueError as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(str(args.output), encoding="utf-8", xml_declaration=True)
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(str(args.output.resolve()))


if __name__ == "__main__":
    main()
