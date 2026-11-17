#!/usr/bin/env python3
"""Measure a shelf product's width and depth at every height from the wrist Orbbec cloud.

For checking whether the wrist cloud can size a 2F-85 front grasp from the product itself
instead of from its exemplars.yaml catalog size (which, for the mustard, is its narrowest part).
See design_docs/wrist_cloud_product_profile.md.

Two steps:

  snap     Grab the wrist colour image, camera_info and a few clouds, each with the world <- camera
           transform at its own stamp, and save them to a directory. Needs ROS: run it in the
           runtime container with the stack up, the arm still, the product in view.

  measure  Offline (numpy, cv2, PyYAML; no ROS). Keep the cloud points that project into a pixel
           box drawn around the product in the saved colour image, drop the board and the
           background, cut the rest into horizontal slices and report, per slice:
             - width: the trimmed extent along world X (along the rack), the same trimmed extent
               ComputeProductKeepoutBox takes;
             - visible depth: the trimmed extent along world +Y (into the shelf), front face to the
               outline. For a round or oval section this is about half the body depth;
             - fitted depth: 2b from fitting the front profile y = cy - b*sqrt(1 - ((x - cx)/a)^2)
               with a = width/2 (an oval section seen square on); uses the curvature of the front,
               not the noisy outline.
           At the pipeline's front grasp height it also prints where Compute 2F-85 Shelf Grasps puts
           grasp_link from the catalog size, and where it would put it from the measured one.

Usage (runtime container; `snap` cannot run with python3 -I, which drops the ROS PYTHONPATH):

  docker exec -it -u $USER moveit_pro-runtime-1 bash -c 'source /opt/ros/jazzy/setup.bash; \\
    export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml; cd ~/user_ws; \\
    python3 scripts/measure_product_profile.py snap --out profiles/mustard_1'
  # open profiles/mustard_1/color.png, read off a pixel box around the product, then:
  docker exec -u $USER moveit_pro-runtime-1 bash -c 'cd ~/user_ws; python3 -I \\
    scripts/measure_product_profile.py measure profiles/mustard_1 --roi U0 V0 U1 V1 --product mustard'

World axes are the pipeline's: the rack is square to world, X along it, +Y into the shelf, Z up.
"""

import argparse
import json
import re
import sys
import time
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parent.parent
EXEMPLARS = WS / "src/shelf_stocking/src/shelf_stocking_behaviors/config/exemplars.yaml"
GRASP_XML = (
    WS / "src/shelf_stocking/src/shelf_stocking_objectives/adapters/robotiq_2f85/compute_2f85_shelf_grasps.xml"
)
GRASP_PORTS = (
    "front_pad_offset",
    "lateral_trim_fraction",
    "gripper_opening",
    "fingertip_reach",
    "fingertip_reach_per_closing",
    "grip_squeeze",
    "back_face_margin",
    "grasp_height_fraction",
)

CLOUD_TOPIC = "/camera/wrist_orbbec/depth_registered/points"
IMAGE_TOPIC = "/camera/wrist_orbbec/color/image_raw"
INFO_TOPIC = "/camera/wrist_orbbec/color/camera_info"


# --------------------------------------------------------------------------------------------- snap


def quat_matrix(t, q):
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def snap(args):
    import cv2
    import rclpy
    import tf2_ros
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from sensor_msgs_py import point_cloud2

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("measure_product_profile_snap")
    tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30))
    tf2_ros.TransformListener(tf_buffer, node)
    got = {"clouds": [], "image": None, "info": None}
    # Let the TF buffer fill first, so the first cloud has a transform at its own stamp.
    settle = time.time() + 1.5
    while time.time() < settle:
        rclpy.spin_once(node, timeout_sec=0.1)

    def on_cloud(m):
        if len(got["clouds"]) < args.frames:
            got["clouds"].append(m)

    node.create_subscription(PointCloud2, CLOUD_TOPIC, on_cloud, qos_profile_sensor_data)
    node.create_subscription(Image, IMAGE_TOPIC, lambda m: got.__setitem__("image", m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, INFO_TOPIC, lambda m: got.__setitem__("info", m), qos_profile_sensor_data)

    deadline = time.time() + args.timeout
    while time.time() < deadline and (
        len(got["clouds"]) < args.frames or got["image"] is None or got["info"] is None
    ):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not got["clouds"] or got["image"] is None or got["info"] is None:
        sys.exit(
            f"Timed out: {len(got['clouds'])} clouds, image {'yes' if got['image'] else 'no'}, "
            f"camera_info {'yes' if got['info'] else 'no'}. Is the wrist Orbbec publishing "
            f"(and CYCLONEDDS_URI exported)?"
        )

    points, transforms, stamps = [], [], []
    for cloud in got["clouds"]:
        frame = cloud.header.frame_id
        stamp = Time.from_msg(cloud.header.stamp)
        try:
            tr = tf_buffer.lookup_transform("world", frame, stamp, Duration(seconds=2.0))
            note = "at the cloud stamp"
        except tf2_ros.TransformException:
            tr = tf_buffer.lookup_transform("world", frame, Time(), Duration(seconds=2.0))
            note = "LATEST (no transform at the cloud stamp; keep the arm still)"
        m = quat_matrix(tr.transform.translation, tr.transform.rotation)
        xyz = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True).astype(np.float64)
        points.append(xyz)
        transforms.append(m)
        stamps.append(cloud.header.stamp.sec + 1e-9 * cloud.header.stamp.nanosec)
        print(f"cloud {len(points)}: {len(xyz)} points in {frame}, world transform {note}")

    img = got["image"]
    if img.encoding not in ("rgb8", "bgr8"):
        sys.exit(f"Unsupported image encoding {img.encoding}")
    rgb = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.step)[:, : img.width * 3]
    rgb = rgb.reshape(img.height, img.width, 3)
    bgr = rgb[:, :, ::-1] if img.encoding == "rgb8" else rgb
    cv2.imwrite(str(out / "color.png"), np.ascontiguousarray(bgr))

    info = got["info"]
    try:
        tr = tf_buffer.lookup_transform("world", "grasp_link", Time(), Duration(seconds=1.0))
        grasp_link = quat_matrix(tr.transform.translation, tr.transform.rotation).tolist()
    except tf2_ros.TransformException:
        grasp_link = None
    meta = {
        "cloud_frame": got["clouds"][0].header.frame_id,
        "image_frame": img.header.frame_id,
        "image_size": [img.width, img.height],
        "K": list(info.k),
        "D": list(info.d),
        "distortion_model": info.distortion_model,
        "cloud_stamps": stamps,
        "world_from_grasp_link": grasp_link,
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2))
    np.savez_compressed(
        out / "clouds.npz", **{f"points_{i}": p for i, p in enumerate(points)}, world_from_cam=np.array(transforms)
    )
    if meta["cloud_frame"] != meta["image_frame"]:
        print(
            f"WARNING: the cloud is in {meta['cloud_frame']} but the image in {meta['image_frame']}; "
            f"the pixel box selection assumes a colour-registered cloud."
        )
    print(f"Saved {out}/color.png, clouds.npz, meta.json. Draw a pixel box around the product in color.png.")
    node.destroy_node()
    rclpy.shutdown()


# ------------------------------------------------------------------------------------------ measure


def grasp_port_defaults():
    text = GRASP_XML.read_text()
    found = {}
    for name in GRASP_PORTS:
        m = re.search(rf'<input_port\s+name="{name}"\s+default="([^"]+)"', text)
        if not m:
            sys.exit(f"Could not read the {name} default from {GRASP_XML}")
        found[name] = float(m.group(1))
    return found


def catalog_entry(product):
    import yaml

    entry = yaml.safe_load(EXEMPLARS.read_text())["exemplars"].get(product)
    if entry is None:
        sys.exit(f"No exemplars.yaml entry for {product!r}")
    tall, wide, deep = (float(v) for v in str(entry["dimensions"]).split(";"))
    return entry, tall, wide, deep


def trimmed(values, fraction):
    v = np.sort(values)
    n = len(v)
    k = int(np.floor(fraction * n))
    return v[k], v[n - 1 - k]


def grasp_link_from_front(deep, wide, p):
    """grasp_link's distance into the shelf from the front face, as Compute 2F-85 Shelf Grasps sets it
    for a box: centre half the depth behind the front face, then min(front_pad_offset, cap) past it."""
    closing = p["gripper_opening"] - wide + p["grip_squeeze"]
    reach = p["fingertip_reach"] + p["fingertip_reach_per_closing"] * closing
    cap = deep / 2.0 - reach - p["back_face_margin"]
    fo = min(p["front_pad_offset"], cap)
    return deep / 2.0 + fo, reach, cap < p["front_pad_offset"]


def fit_front_profile(x, y, cx, a):
    """Least squares for y = cy - b*s, s = sqrt(1 - ((x - cx)/a)^2), on the middle of the face."""
    u = (x - cx) / a
    keep = np.abs(u) < 0.9
    if keep.sum() < 30:
        return None
    s = np.sqrt(1.0 - u[keep] ** 2)
    A = np.stack([np.ones_like(s), -s], axis=1)
    (cy, b), *_ = np.linalg.lstsq(A, y[keep], rcond=None)
    rms = float(np.sqrt(np.mean((A @ np.array([cy, b]) - y[keep]) ** 2)))
    return float(b), rms


def measure(args):
    import cv2

    d = Path(args.snapshot)
    meta = json.loads((d / "meta.json").read_text())
    data = np.load(d / "clouds.npz")
    K = np.array(meta["K"]).reshape(3, 3)
    D = np.array(meta["D"], dtype=np.float64)
    u0, v0, u1, v1 = args.roi
    u0, u1 = sorted((u0, u1))
    v0, v1 = sorted((v0, v1))

    world_pts = []
    covered = np.zeros((int(v1 - v0) + 1, int(u1 - u0) + 1), bool)
    ranges = []
    for i, m in enumerate(data["world_from_cam"]):
        pc = data[f"points_{i}"] * args.depth_scale  # a pure range scale about the optical centre
        pc = pc[pc[:, 2] > args.min_range]  # the gripper's own fingers are in view, nearer than this
        uv, _ = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
        uv = uv.reshape(-1, 2)
        inside = (uv[:, 0] >= u0) & (uv[:, 0] <= u1) & (uv[:, 1] >= v0) & (uv[:, 1] <= v1)
        cu = uv[inside].astype(int)
        covered[cu[:, 1] - int(v0), cu[:, 0] - int(u0)] = True
        ranges.append(pc[inside, 2])
        pw = (m[:3, :3] @ pc[inside].T).T + m[:3, 3]
        world_pts.append(pw)
    pts = np.concatenate(world_pts)
    if len(pts) < 100:
        sys.exit(f"Only {len(pts)} points in the pixel box; check --roi against color.png.")
    ranges = np.concatenate(ranges)
    all_ranges = np.concatenate([data[f"points_{i}"][:, 2] for i in range(len(data["world_from_cam"]))])
    print(f"Pixel box: {covered.mean()*100:.0f}% of its pixels have a cloud point; range in the box "
          f"{np.percentile(ranges, 5):.3f} to {np.percentile(ranges, 95):.3f} m (5th to 95th percentile); "
          f"nearest point in the whole cloud {np.percentile(all_ranges, 0.5):.3f} m.")
    if covered.mean() < 0.4:
        print("WARNING: most of the box has no depth. The product may be nearer than the camera's minimum "
              "range (the whole cloud stops at the nearest-point figure above), or too glossy to match. "
              "Snap from further back, from the pick's start view.")

    # Board: the densest 2 mm band among the lowest 30% of the points in the box (the board in front of
    # the product's base), unless given. Drop everything within board_clear above it, so the board in
    # front of the product (nearer the camera than the product's front face) cannot set the front.
    if args.board_z is not None:
        board_z = args.board_z
    else:
        low = np.sort(pts[:, 2])[: max(10, int(0.3 * len(pts)))]
        counts, edges = np.histogram(low, bins=np.arange(low[0], low[-1] + 0.002, 0.002))
        board_z = float(edges[np.argmax(counts)] + 0.001)
    pts = pts[pts[:, 2] > board_z + args.board_clear]
    # Background: keep points within depth_gate behind the front face.
    front_y = float(np.percentile(pts[:, 1], 2.0))
    pts = pts[pts[:, 1] < front_y + args.depth_gate]
    if len(pts) < 100:
        sys.exit("Too few points left after dropping the board and the background.")

    p = grasp_port_defaults()
    trim = args.trim if args.trim is not None else p["lateral_trim_fraction"]
    rows = []
    top = float(np.max(pts[:, 2]))
    z = board_z + args.board_clear
    while z < top:
        s = pts[(pts[:, 2] >= z) & (pts[:, 2] < z + args.slice)]
        if len(s) >= args.min_points:
            lo_x, hi_x = trimmed(s[:, 0], trim)
            lo_y, hi_y = trimmed(s[:, 1], trim)
            width = hi_x - lo_x
            cx = 0.5 * (lo_x + hi_x)
            fit = fit_front_profile(s[:, 0], s[:, 1], cx, width / 2.0) if width > 0.005 else None
            rows.append(
                {
                    "height": z + args.slice / 2.0 - board_z,
                    "n": len(s),
                    "width": width,
                    "centre_x": cx,
                    "front_y": lo_y,
                    "visible_depth": hi_y - lo_y,
                    "fitted_depth": 2.0 * fit[0] if fit else float("nan"),
                    "fit_rms": fit[1] if fit else float("nan"),
                }
            )
        z += args.slice
    if not rows:
        sys.exit("No slice had enough points.")

    entry, tall, wide, deep = (None, None, None, None)
    if args.product:
        entry, tall, wide, deep = catalog_entry(args.product)

    print(f"{len(pts)} points kept. Board top z = {board_z:.4f} m "
          f"({'given' if args.board_z is not None else 'auto, densest low band in the box; pass --board-z if wrong'}); "
          f"front face y = {front_y:.4f} m; trim {trim}; depth scale {args.depth_scale}.")
    if entry:
        print(f"Catalog {args.product}: tall {tall*1000:.0f} wide {wide*1000:.0f} deep {deep*1000:.0f} mm.")
    print()
    print(" height   n     width  vis.depth  2x vis  fitted depth (rms)   front y   centre x")
    print("   mm             mm      mm       mm       mm      mm         m          m")
    for r in rows:
        print(f"  {r['height']*1000:5.0f} {r['n']:5d}  {r['width']*1000:6.1f}  {r['visible_depth']*1000:6.1f}"
              f"  {2*r['visible_depth']*1000:6.1f}   {r['fitted_depth']*1000:6.1f} ({r['fit_rms']*1000:4.1f})"
              f"  {r['front_y']:8.4f}  {r['centre_x']:8.4f}")

    if entry:
        fraction = float(entry.get("grasp_height_fraction", p["grasp_height_fraction"]))
        gh = fraction * tall
        near = min(rows, key=lambda r: abs(r["height"] - gh))
        print()
        print(f"Front grasp height: {fraction} x {tall*1000:.0f} mm = {gh*1000:.0f} mm above the board "
              f"(nearest slice {near['height']*1000:.0f} mm).")
        cases = [("catalog", deep, wide)]
        cases.append(("measured, depth 2x visible", 2 * near["visible_depth"], near["width"]))
        if np.isfinite(near["fitted_depth"]):
            cases.append(("measured, fitted depth", near["fitted_depth"], near["width"]))
        print("  size used                    wide   deep  tip reach  capped  grasp_link   tips past   tips short")
        print("                               mm     mm      mm               past front   front face  of back face")
        for name, dd, ww in cases:
            g, reach, capped = grasp_link_from_front(dd, ww, p)
            print(f"  {name:27s} {ww*1000:5.1f} {dd*1000:6.1f}   {reach*1000:6.1f}    {'yes' if capped else 'no ':3s}"
                  f"    {g*1000:6.1f}      {(g + reach)*1000:6.1f}      {(dd - g - reach)*1000:6.1f}")
        print("  grasp_link past front: where Compute 2F-85 Shelf Grasps puts grasp_link, along the approach, from")
        print("  the front face (negative: in front of it). Tips: the finger tips closed onto that width. Everything")
        print("  between the front face and the tips is what the fingers hold; the pads' middle is front_pad_offset")
        print(f"  ({p['front_pad_offset']*1000:.0f} mm) short of grasp_link, so a capped grasp holds on the tips only.")

    # Overlay: kept points on the image, coloured by height; the box; the grasp height slice in white.
    img = cv2.imread(str(d / "color.png"))
    cv2.rectangle(img, (int(u0), int(v0)), (int(u1), int(v1)), (0, 255, 255), 1)
    inv = np.linalg.inv(data["world_from_cam"][0])
    pc = (inv[:3, :3] @ pts.T).T + inv[:3, 3]
    uv, _ = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
    hmax = max(r["height"] for r in rows)
    for (uu, vv), zz in zip(uv.reshape(-1, 2), pts[:, 2] - board_z):
        t = np.clip(zz / max(hmax, 1e-3), 0, 1)
        colour = (int(255 * (1 - t)), 64, int(255 * t))
        if entry and abs(zz - gh) < args.slice / 2:
            colour = (255, 255, 255)
        cv2.circle(img, (int(uu), int(vv)), 1, colour, -1)
    cv2.imwrite(str(d / "overlay.png"), img)

    # Profile plot: width and depths against height, catalog values dashed.
    W, H, m = 640, 480, 50
    plot = np.full((H, W, 3), 255, np.uint8)
    xmax = max(0.12, max(r["width"] for r in rows) * 1.2)
    ymax = max(r["height"] for r in rows) * 1.05

    def px(val, height):
        return int(m + val / xmax * (W - 2 * m)), int(H - m - height / ymax * (H - 2 * m))

    cv2.line(plot, px(0, 0), px(xmax, 0), (0, 0, 0), 1)
    cv2.line(plot, px(0, 0), px(0, ymax), (0, 0, 0), 1)
    for mm in range(0, int(xmax * 1000) + 1, 20):
        cv2.putText(plot, str(mm), (px(mm / 1000, 0)[0] - 8, H - m + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
    for mm in range(0, int(ymax * 1000) + 1, 20):
        cv2.putText(plot, str(mm), (5, px(0, mm / 1000)[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
    series = [("width", (200, 0, 0)), ("fitted_depth", (0, 150, 0)), ("visible_depth", (0, 0, 200))]
    for key, colour in series:
        pts_line = [px(r[key] * (2 if key == "visible_depth" else 1), r["height"]) for r in rows if np.isfinite(r[key])]
        for a_, b_ in zip(pts_line, pts_line[1:]):
            cv2.line(plot, a_, b_, colour, 2)
    if entry:
        for val, colour in ((wide, (200, 0, 0)), (deep, (0, 150, 0))):
            for hh in np.arange(0, ymax, 0.01):
                cv2.line(plot, px(val, hh), px(val, hh + 0.005), colour, 1)
        cv2.line(plot, px(0, gh), px(xmax, gh), (120, 120, 120), 1)
    labels = ["width (blue)", "fitted depth (green)", "2x visible depth (red)", "dashed: catalog; grey: grasp height"]
    for i, text in enumerate(labels):
        cv2.putText(plot, text, (W - 250, 20 + 15 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    cv2.putText(plot, "mm across / deep", (W // 2 - 50, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    cv2.putText(plot, "height above board, mm", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    cv2.imwrite(str(d / "profile.png"), plot)

    with open(d / "profile.csv", "w") as f:
        keys = list(rows[0].keys())
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join(f"{r[k]:.5f}" if isinstance(r[k], float) else str(r[k]) for k in keys) + "\n")
    print(f"\nWrote {d}/overlay.png (kept points; white = grasp height), profile.png, profile.csv.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("snap", help="grab image, camera_info and clouds (needs ROS)")
    s.add_argument("--out", required=True, help="directory to write color.png, clouds.npz, meta.json")
    s.add_argument("--frames", type=int, default=5, help="clouds to grab and stack (arm still)")
    s.add_argument("--timeout", type=float, default=15.0)
    m = sub.add_parser("measure", help="slice the product's points (offline)")
    m.add_argument("snapshot", help="directory written by snap")
    m.add_argument("--roi", type=float, nargs=4, required=True, metavar=("U0", "V0", "U1", "V1"),
                   help="pixel box around the product in color.png (raw, distorted image)")
    m.add_argument("--product", help="exemplars.yaml name, for the catalog size and grasp height")
    m.add_argument("--slice", type=float, default=0.005, help="slice thickness, m")
    m.add_argument("--min-points", type=int, default=40, help="fewest points for a slice to be reported")
    m.add_argument("--board-z", type=float, help="board top in world z, m (default: lowest points in the box)")
    m.add_argument("--board-clear", type=float, default=0.01, help="drop points this far above the board, m")
    m.add_argument("--depth-gate", type=float, default=0.035,
                   help="keep points this far behind the front face, m; less than the product's depth, so a "
                        "product standing behind it is left out")
    m.add_argument("--min-range", type=float, default=0.2,
                   help="drop points nearer the camera than this, m (the gripper's fingers)")
    m.add_argument("--trim", type=float, help="trim fraction per end (default: the grasp XML's lateral_trim_fraction)")
    m.add_argument("--depth-scale", type=float, default=1.0,
                   help="scale the cloud's range before measuring, e.g. 1.014 to try undoing the Orbbec's short depth")
    args = ap.parse_args()
    snap(args) if args.cmd == "snap" else measure(args)


if __name__ == "__main__":
    main()
