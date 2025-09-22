#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import print_function

import os, sys, csv, math, shutil, errno
import cv2
import rosbag
import numpy as np
import matplotlib
matplotlib.use('Agg')   # headless
import matplotlib.pyplot as plt

# ====== CONFIG =========================================================
EXTRACTION_DATE = "16092025"   # e.g., run date token you want in the root folder name
ROSBAG_FILE = "/media/avresearch/RouteData/perception_output_2025-09-11_15-09-03/rosbag_perception_output_2025-09-09_12-43-25_2025-09-09-12-43-27_0.bag"
# ROSBAG_FILE = "/media/avresearch/RouteData/0828_Route2_Rosbags/Trial_1_2024-08-28-11/2024-08-28-11-23-33_2.bag"

# Topics in your bag
TOPIC_ODOM        = "/novatel/oem7/odom"             # nav_msgs/Odometry
TOPIC_FUSED_BBOX  = "/yolov9/fused_bbox"             # jsk_recognition_msgs/BoundingBoxArray
TOPIC_CAMERA_IMG  = "/yolov9/published_image"        # sensor_msgs/Image
TOPIC_METADATA    = "/rosbag_metadata"               # std_msgs/String
# TOPIC_LIDAR_PTS = "/front_lidar/points"            # sensor_msgs/PointCloud2  (NOT in this bag)

# Association / filtering / video params
MAX_SPEED_M_S     = 20.0     # gating speed upper bound
BASE_GATING_M     = 2.0      # extra gating slack (meters)
MIN_TRACK_LIFETIME_S = 1.0   # min duration to keep a track
SMOOTH_WINDOW_K   = 5        # moving-average window (odd recommended)
FRAME_RATE        = 10.0     # output video FPS

# Verbosity and debugging
VERBOSE                = True        # print progress messages during processing
DEBUG_FIRST_N_OBJECTS  = 2           # set to >0 to process only first N objects (by sorted ID)

# Static-object handling: For these labels, replace all x,y with their global average
# NOTE: Update this set to match your detector's numeric label IDs for static objects
# Examples (YOU SHOULD CONFIRM IDs): cone, traffic light, stop sign, speed limit sign
STATIC_LABEL_IDS = set([
    #  Example placeholders; replace with your system's static class IDs
    #  e.g., cone=80, fire_hydrant=10, traffic_light=9, stop_sign=11,
    80, 10, 9, 11, 9999
])

# ====== PATHS ==========================================================
def ensure_dir(path):
    try:
        os.makedirs(path)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

BAG_BASENAME = os.path.splitext(os.path.basename(ROSBAG_FILE))[0]
ROOT_OUT     = "Extracted_data_{}".format(EXTRACTION_DATE)
BAG_OUT_DIR  = os.path.join(ROOT_OUT, "{}_extracted_data".format(BAG_BASENAME))
ensure_dir(BAG_OUT_DIR)
INTERMEDIATE_OUT = "Intermediate_data_{}".format(EXTRACTION_DATE)
ensure_dir(INTERMEDIATE_OUT)

# ====== UTILITIES ======================================================
try:
    from cv_bridge import CvBridge
    _BRIDGE = CvBridge()
except Exception as _e:
    _BRIDGE = None

def convert_img_to_cv2(msg):
    """Try cv_bridge, fallback to raw uint8 decode (assumes 8UC3 BGR)."""
    if _BRIDGE is not None:
        try:
            return _BRIDGE.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print("[WARN] cv_bridge conversion failed: {}".format(e))
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    return arr.reshape((msg.height, msg.width, -1))

def yaw_from_quat(qx, qy, qz, qw):
    """Extract yaw (heading) from quaternion in ENU/UTM-ish (z-up)."""
    # yaw from quaternion (Z rotation)
    # ref: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)

def rot2d(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c*x - s*y, s*x + c*y)

def moving_average(vals, k):
    if k <= 1 or len(vals) == 0:
        return vals[:]
    half = k // 2
    out = []
    for i in range(len(vals)):
        lo = max(0, i - half)
        hi = min(len(vals), i + half + 1)
        out.append(sum(vals[lo:hi]) / float(hi - lo))
    return out

def dist2d(a, b):
    dx, dy = a[0] - b[0], a[1] - b[1]
    return math.sqrt(dx*dx + dy*dy)

def choose_fourcc_h264():
    return cv2.VideoWriter_fourcc(*'avc1')  # try H.264 first

def plot_xy(ts, xs, ys, out_path, title):
    plt.figure()
    plt.plot(xs, ys, '-o', markersize=2)
    plt.xlabel("X"); plt.ylabel("Y"); plt.title(title); plt.tight_layout()
    plt.savefig(out_path, dpi=160); plt.close()

def plot_yt(ts, ys, out_path, title):
    plt.figure()
    plt.plot(ts, ys, '-o', markersize=2)
    plt.xlabel("Time (s)"); plt.ylabel("Y"); plt.title(title); plt.tight_layout()
    plt.savefig(out_path, dpi=160); plt.close()

# # ===== (COMMENTED) PCD writer for future bags ==========================
# import struct
# def save_pointcloud2_as_pcd(msg, filename):
#     pc = []
#     step = msg.point_step
#     data = msg.data
#     for i in range(0, len(data), step):
#         x, y, z = struct.unpack_from('fff', data, offset=i)
#         pc.append((x, y, z))
#     with open(filename, 'w') as f:
#         f.write('# .PCD v0.7 - Point Cloud Data file format\n')
#         f.write('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
#         f.write('WIDTH {}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'.format(len(pc)))
#         f.write('POINTS {}\nDATA ascii\n'.format(len(pc)))
#         for (x,y,z) in pc: f.write('{} {} {}\n'.format(x,y,z))

# ====== STEP 1: READ FUSED BBOX FROM BAG -> CSV ========================
def step1_dump_fused_bbox_csv(bag):
    """
    Read jsk_recognition_msgs/BoundingBoxArray from TOPIC_FUSED_BBOX
    and dump a tidy CSV with timestamp, frame_id, position, dims, label.
    """
    # Save to intermediate results folder parallel to ROOT_OUT
    out_csv = os.path.join(INTERMEDIATE_OUT, "{}_fused_bbox_results.csv".format(BAG_BASENAME))
    wrote_header = False
    msg_count = 0
    with open(out_csv, 'w') as f:
        w = csv.writer(f)
        for topic, msg, t in bag.read_messages(topics=[TOPIC_FUSED_BBOX]):
            msg_count += 1
            # header stamp preferred; fallback to bag time
            ts = msg.header.stamp.to_sec() if hasattr(msg.header, 'stamp') and msg.header.stamp else t.to_sec()
            frame_id = msg.header.frame_id if hasattr(msg.header, 'frame_id') else ""
            for b in msg.boxes:
                bx = b.pose.position.x; by = b.pose.position.y; bz = b.pose.position.z
                dx = b.dimensions.x;    dy = b.dimensions.y;    dz = b.dimensions.z
                lab = getattr(b, 'label', -1)
                if not wrote_header:
                    w.writerow(['frame_id','timestamp','x','y','z','dx','dy','dz','label'])
                    wrote_header = True
                w.writerow([frame_id, ts, bx, by, bz, dx, dy, dz, lab])
            if VERBOSE and (msg_count % 200 == 0):
                print("[INFO] Read {} fused bbox messages...".format(msg_count))
    print("[OK] Fused bbox CSV -> {}".format(out_csv))
    return out_csv

# ====== STEP 2: READ METADATA FROM BAG =================================
def step2_extract_metadata(bag):
    """
    Read std_msgs/String from TOPIC_METADATA and parse metadata fields.
    Returns dict with parsed metadata or empty dict if not found.
    """
    metadata = {}
    for topic, msg, t in bag.read_messages(topics=[TOPIC_METADATA]):
        if hasattr(msg, 'data'):
            data_str = msg.data
            # Parse format: "location: RTA-4 Transit, vehicle: blue, passengers: 2, ..."
            try:
                pairs = [pair.strip() for pair in data_str.split(',')]
                for pair in pairs:
                    if ':' in pair:
                        key, value = pair.split(':', 1)
                        key = key.strip().lower().replace(' ', '_')
                        value = value.strip()
                        metadata[key] = value
                break  # Take first message
            except Exception as e:
                print("[WARN] Failed to parse metadata: {}".format(e))
                break
    if VERBOSE:
        print("[INFO] Extracted metadata: {}".format(metadata))
    return metadata

# ====== STEP 3: READ ODOM FROM BAG -> CSV =============================
def step3_dump_odom_csv(bag):
    """
    Read nav_msgs/Odometry and dump timestamp + (x,y,z) + quaternion.
    """
    out_csv = os.path.join(BAG_OUT_DIR, "{}_novatel_odom_data.csv".format(BAG_BASENAME))
    msg_count = 0
    with open(out_csv, 'w') as f:
        w = csv.writer(f)
        w.writerow(["timestamp","position_x","position_y","position_z",
                    "orientation_x","orientation_y","orientation_z","orientation_w"])
        for topic, msg, t in bag.read_messages(topics=[TOPIC_ODOM]):
            msg_count += 1
            ts = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            p = msg.pose.pose.position; q = msg.pose.pose.orientation
            w.writerow([ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            if VERBOSE and (msg_count % 500 == 0):
                print("[INFO] Read {} odom messages...".format(msg_count))
    print("[OK] Odom CSV -> {}".format(out_csv))
    return out_csv

# ====== STEP 4: CALCULATE KEY METRICS ===================================
def step4_calculate_key_metrics(odom_csv, metadata, trajs):
    """
    Calculate key metrics from odometry and object counts.
    Returns dict with all metrics.
    """
    # Load odometry data
    ego = np.genfromtxt(odom_csv, delimiter=',', names=True)
    ts = ego['timestamp']
    xs = ego['position_x']
    ys = ego['position_y']
    
    # Calculate duration
    duration = ts[-1] - ts[0] if len(ts) > 1 else 0.0
    
    # Calculate distance (cumulative path length)
    distances = []
    for i in range(1, len(xs)):
        dx = xs[i] - xs[i-1]
        dy = ys[i] - ys[i-1]
        distances.append(math.sqrt(dx*dx + dy*dy))
    total_distance = sum(distances)
    
    # Calculate average velocity directly from distance/duration
    avg_velocity = total_distance / duration if duration > 0 else 0.0
    
    # Calculate instantaneous velocities for max speed analysis
    velocities = []
    for i in range(1, len(ts)):
        dt = ts[i] - ts[i-1]
        if dt > 0 and dt < 10.0:  # Filter out large time gaps
            dx = xs[i] - xs[i-1]
            dy = ys[i] - ys[i-1]
            vel = math.sqrt(dx*dx + dy*dy) / dt
            velocities.append(vel)
    
    # Find max velocity < 25 m/s
    valid_velocities = [v for v in velocities if v < 25.0]
    max_velocity = max(valid_velocities) if valid_velocities else 0.0
    
    # Calculate accelerations (m/s^2)
    accelerations = []
    for i in range(1, len(velocities)):
        dt = ts[i+1] - ts[i-1] if i+1 < len(ts) else ts[i] - ts[i-1]
        if dt > 0 and dt < 5.0:  # Filter out large time gaps
            accel = (velocities[i] - velocities[i-1]) / dt
            accelerations.append(accel)
    
    # Find max acceleration < 5 m/s²
    valid_accelerations = [a for a in accelerations if a < 5.0]
    max_acceleration = max(valid_accelerations) if valid_accelerations else 0.0
    
    # Find max deceleration > -5 m/s² (most negative but not too extreme)
    valid_decelerations = [a for a in accelerations if a > -5.0]
    max_deceleration = min(valid_decelerations) if valid_decelerations else 0.0  # Most negative
    
    if VERBOSE:
        print("[INFO] Velocity stats: avg={:.2f} m/s (from distance/duration), max={:.2f} m/s (<25 m/s filter)".format(
            avg_velocity, max_velocity))
        print("[INFO] Acceleration stats: max_accel={:.2f} m/s² (<5 m/s²), max_decel={:.2f} m/s² (>-5 m/s²)".format(
            max_acceleration, max_deceleration))
    
    # Count objects by type
    object_counts = {}
    for tid, rows in trajs.items():
        if not rows:
            continue
        # Use most common label for this track
        labels = [r['label'] for r in rows]
        try:
            from collections import Counter
            dom_label = Counter(labels).most_common(1)[0][0]
        except Exception:
            dom_label = labels[0] if labels else -1
        
        if dom_label not in object_counts:
            object_counts[dom_label] = 0
        object_counts[dom_label] += 1
    
    # Build metrics dict
    metrics = {
        'duration_s': duration,
        'distance_m': total_distance,
        'max_velocity_ms': max_velocity,
        'avg_velocity_ms': avg_velocity,
        'max_acceleration_ms2': max_acceleration,
        'max_deceleration_ms2': max_deceleration,
        'total_objects': len(trajs),
        'object_counts_by_type': object_counts
    }
    
    # Add metadata fields
    for field in ['location', 'vehicle', 'passengers', 'road_type', 'road_condition', 'comments', 'maneuver']:
        metrics[field] = metadata.get(field, '')
    
    if VERBOSE:
        print("[INFO] Calculated key metrics: duration={:.1f}s, distance={:.1f}m, max_vel={:.1f}m/s, objects={}".format(
            duration, total_distance, max_velocity, len(trajs)))
    
    return metrics

# ====== STEP 5: TRAJECTORY EXTRACTION (Notebook -> .py) ===============
def step5_build_trajectories(fused_csv, odom_csv):
    """
    Implements your notebook’s logic:
      - filter fused by frame_id if needed,
      - align with ego trajectory,
      - rotate/translate detections into ego global UTM,
      - associate across time -> IDs,
      - filter short tracks, smooth,
      - write *_trajectories_raw.csv
    Returns: trajs (dict: id -> list of rows) and the output CSV path.
    """
    # --- load ego odom
    ego = np.genfromtxt(odom_csv, delimiter=',', names=True)
    ego_ts = ego['timestamp']
    ego_xy = np.vstack((ego['position_x'], ego['position_y'])).T
    ego_qx, ego_qy, ego_qz, ego_qw = ego['orientation_x'], ego['orientation_y'], ego['orientation_z'], ego['orientation_w']
    ego_yaw = np.array([yaw_from_quat(eqx, eqy, eqz, eqw) for (eqx,eqy,eqz,eqw) in zip(ego_qx, ego_qy, ego_qz, ego_qw)])

    # --- load fused
    fused = np.genfromtxt(fused_csv, delimiter=',', names=True)
    # optional: keep only lidar-aligned frame_id
    # mask = (fused['frame_id'] == 'lidar_tc')  # may fail if dtype isn't string array in np
    # Keep all if dtype mixing; otherwise implement pandas. We'll proceed without frame_id filter.

    f_ts = fused['timestamp']
    f_xyz = np.vstack((fused['x'], fused['y'], fused['z'])).T
    f_lab = fused['label'] if 'label' in fused.dtype.names else np.full(len(f_ts), -1, dtype=np.int32)

    # --- interpolate ego pose at detection times
    def interp_ego(ts_query):
        # linear interp of x, y, yaw
        x = np.interp(ts_query, ego_ts, ego_xy[:,0])
        y = np.interp(ts_query, ego_ts, ego_xy[:,1])
        yaw = np.interp(ts_query, ego_ts, ego_yaw)
        return x, y, yaw

    # --- transform detections into global using ego pose:
    # Here we assume fused positions are already in ego's local/lidar frame (x,y relative to ego),
    # so global = ego_xy + R(yaw) * local
    det_global = []
    if VERBOSE:
        print("[INFO] Transforming {} detections to global frame...".format(len(f_ts)))
    for i in range(len(f_ts)):
        gx, gy, gyaw = interp_ego(f_ts[i])
        lx, ly = f_xyz[i,0], f_xyz[i,1]
        rx, ry = rot2d(lx, ly, gyaw)
        det_global.append((f_ts[i], gx + rx, gy + ry, f_xyz[i,2], int(f_lab[i])))
        if VERBOSE and (i % 5000 == 0) and i > 0:
            print("[INFO] Transformed {} / {} detections...".format(i, len(f_ts)))
    # sort by time
    det_global.sort(key=lambda r: r[0])

    # --- multi-target data association (greedy NN with speed gating)
    tracks = {}     # id -> {'last_xy':(x,y), 't':t, 'path': [(t,x,y,z,label)]}
    next_id = 1

    # group by timestamp (frame-like)
    # (We can scan sequentially and associate per message timestamp)
    from collections import defaultdict
    frame = defaultdict(list)
    for (ts, gx, gy, gz, lab) in det_global:
        frame[ts].append((gx, gy, gz, lab))
    times = sorted(frame.keys())

    if VERBOSE:
        print("[INFO] Associating detections across {} timestamps...".format(len(times)))
    for idx_ts, ts in enumerate(times):
        dets = frame[ts]  # list of (x,y,z,lab)
        unmatched = set(range(len(dets)))
        candidates = []  # (dist, tid, j)
        for tid, rec in tracks.items():
            dt = max(1e-6, ts - rec['t'])
            gate = MAX_SPEED_M_S * dt + BASE_GATING_M
            for j, d in enumerate(dets):
                dxy = (d[0], d[1])
                if dist2d(rec['last_xy'], dxy) <= gate:
                    candidates.append((dist2d(rec['last_xy'], dxy), tid, j))
        candidates.sort(key=lambda x: x[0])

        matched_tids = set(); matched_dets = set()
        for (distv, tid, j) in candidates:
            if tid in matched_tids or j in matched_dets:
                continue
            # match
            gx, gy, gz, lab = dets[j]
            tracks[tid]['path'].append((ts, gx, gy, gz, lab))
            tracks[tid]['last_xy'] = (gx, gy)
            tracks[tid]['t'] = ts
            matched_tids.add(tid); matched_dets.add(j)

        # new tracks for leftovers
        for j in sorted(list(unmatched - matched_dets)):
            gx, gy, gz, lab = dets[j]
            tracks[next_id] = {'last_xy': (gx, gy),
                               't': ts,
                               'path': [(ts, gx, gy, gz, lab)]}
            next_id += 1
        if VERBOSE and (idx_ts % 200 == 0) and idx_ts > 0:
            print("[INFO] Associated up to timestamp index {} / {} (tracks so far: {})".format(idx_ts, len(times), len(tracks)))

    # --- filter short tracks & smooth
    trajs = {}  # id -> list of dict rows
    if VERBOSE:
        print("[INFO] Built {} tentative tracks. Filtering and smoothing...".format(len(tracks)))
    for tid, rec in tracks.items():
        path = rec['path']
        if len(path) < 2:
            continue
        t0, t1 = path[0][0], path[-1][0]
        if (t1 - t0) < MIN_TRACK_LIFETIME_S:
            continue
        ts = [p[0] for p in path]
        xs = [p[1] for p in path]
        ys = [p[2] for p in path]
        zs = [p[3] for p in path]
        labs = [p[4] for p in path]
        # Decide smoothing strategy: if static object label, use global average for x,y
        # Determine dominant label for this track
        try:
            from collections import Counter
            dom_label = Counter(labs).most_common(1)[0][0]
        except Exception:
            dom_label = labs[0]

        # Heuristic: treat as static either by label or by small spatial spread
        is_label_static = (dom_label in STATIC_LABEL_IDS)
        span_dist = 0.0
        if len(xs) > 1:
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            span_dist = math.hypot(max_x - min_x, max_y - min_y)
        is_heuristic_static = (span_dist < 1.0)  # within 1 m total span

        if (is_label_static or (len(STATIC_LABEL_IDS) == 0 and is_heuristic_static)) and len(xs) > 0:
            avg_x = sum(xs) / float(len(xs))
            avg_y = sum(ys) / float(len(ys))
            xs_s = [avg_x] * len(xs)
            ys_s = [avg_y] * len(ys)
            if VERBOSE:
                reason = "label" if is_label_static else "heuristic"
                print("[INFO] Track {} flagged static ({}). Using avg position ({:.2f},{:.2f}).".format(tid, reason, avg_x, avg_y))
        else:
            xs_s = moving_average(xs, SMOOTH_WINDOW_K)
            ys_s = moving_average(ys, SMOOTH_WINDOW_K)
        zs_s = moving_average(zs, SMOOTH_WINDOW_K)
        rows = []
        for i in range(len(ts)):
            rows.append({'ID': tid,
                         'time': ts[i],
                         'rosbagtime_int': int(ts[i]),  # used for slicing ranges
                         'x': xs_s[i], 'y': ys_s[i], 'z': zs_s[i],
                         'label': labs[i]})
        trajs[tid] = rows

    # --- write trajectories_raw.csv
    # Save to intermediate results folder parallel to ROOT_OUT
    out_csv = os.path.join(INTERMEDIATE_OUT, "{}_trajectories_raw.csv".format(BAG_BASENAME))
    with open(out_csv, 'w') as f:
        w = csv.writer(f)
        w.writerow(['ID','time','rosbagtime_int','x','y','z','label'])
        for tid in sorted(trajs.keys()):
            for r in trajs[tid]:
                w.writerow([r['ID'], r['time'], r['rosbagtime_int'], r['x'], r['y'], r['z'], r['label']])
    print("[OK] Trajectories CSV -> {}".format(out_csv))
    return trajs, out_csv

# ====== STEP 6: WRITE KEY METRICS CSV ===================================
def step6_write_key_metrics_csv(metrics):
    """
    Write key metrics to CSV file in BAG_OUT_DIR.
    """
    out_csv = os.path.join(BAG_OUT_DIR, "{}_key_metrics.csv".format(BAG_BASENAME))
    
    with open(out_csv, 'w') as f:
        w = csv.writer(f)
        
        # Write header
        header = ['metric', 'value']
        w.writerow(header)
        
        # Write odometry-based metrics
        w.writerow(['duration_s', metrics['duration_s']])
        w.writerow(['distance_m', metrics['distance_m']])
        w.writerow(['max_velocity_ms', metrics['max_velocity_ms']])
        w.writerow(['avg_velocity_ms', metrics['avg_velocity_ms']])
        w.writerow(['max_acceleration_ms2', metrics['max_acceleration_ms2']])
        w.writerow(['max_deceleration_ms2', metrics['max_deceleration_ms2']])
        w.writerow(['total_objects', metrics['total_objects']])
        
        # Write object counts by type
        for label, count in metrics['object_counts_by_type'].items():
            w.writerow(['objects_type_{}'.format(label), count])
        
        # Write metadata fields
        for field in ['location', 'vehicle', 'passengers', 'road_type', 'road_condition', 'comments', 'maneuver']:
            w.writerow([field, metrics[field]])
    
    print("[OK] Key metrics CSV -> {}".format(out_csv))
    return out_csv

# ====== STEP 7: EXTRACT FRAMES PER OBJECT (±2s buffer) ================
def step7_extract_frames(bag, trajs):
    # Build per-object time windows
    ranges = {}
    for tid, rows in trajs.items():
        if not rows:
            continue
        tmins = [r['rosbagtime_int'] for r in rows]
        t0, t1 = min(tmins) - 2, max(tmins) + 2
        ranges[tid] = (t0, t1)

    # Walk the camera stream once and write frames for tids in range
    if VERBOSE:
        print("[INFO] Extracting frames for {} objects...".format(len(ranges)))
    for topic, msg, t in bag.read_messages(topics=[TOPIC_CAMERA_IMG]):
        ts = t.to_sec(); ts_i = int(ts); ms = int(ts * 1000.0)
        for tid, (lo, hi) in ranges.items():
            if lo <= ts_i <= hi:
                obj_dir = os.path.join(BAG_OUT_DIR, "{}_{}".format(BAG_BASENAME, tid))
                cam_dir = os.path.join(obj_dir, "camera")
                ensure_dir(cam_dir)
                img = convert_img_to_cv2(msg)
                out_path = os.path.join(cam_dir, "frame_{}.png".format(ms))
                if not cv2.imwrite(out_path, img):
                    print("[WARN] Failed to write {}".format(out_path))
    print("[OK] Frame extraction complete.")

# ====== STEP 7.5: MAKE PER-OBJECT FOLDERS, CSV, PLOTS =================
def step7p_finalize_objects(trajs):
    if VERBOSE:
        print("[INFO] Finalizing {} objects (CSV + plots)...".format(len(trajs)))
    for tid in sorted(trajs.keys()):
        obj_dir = os.path.join(BAG_OUT_DIR, "{}_{}".format(BAG_BASENAME, tid))
        cam_dir = os.path.join(obj_dir, "camera")
        ensure_dir(obj_dir); ensure_dir(cam_dir)
        # per-object CSV
        csv_path = os.path.join(obj_dir, "smoothed_trajectory_{}.csv".format(tid))
        with open(csv_path, 'w') as f:
            w = csv.writer(f)
            w.writerow(['timestamp','x','y','z','label'])
            for r in trajs[tid]:
                w.writerow([r['time'], r['x'], r['y'], r['z'], r['label']])
        # plots
        ts = [r['time'] for r in trajs[tid]]
        xs = [r['x'] for r in trajs[tid]]
        ys = [r['y'] for r in trajs[tid]]
        plot_xy(ts, xs, ys, os.path.join(obj_dir, "x-y-{}-cone.png".format(tid)), "Object {} (x-y)".format(tid))
        plot_yt(ts, ys, os.path.join(obj_dir, "y-t-{}-cone.png".format(tid)), "Object {} (y-t)".format(tid))

# ====== STEP 8: MAKE WEB-PLAYABLE MP4s ================================
def step8_make_videos_and_copy_odom():
    # Videos
    for name in sorted(os.listdir(BAG_OUT_DIR)):
        obj_dir = os.path.join(BAG_OUT_DIR, name)
        if not os.path.isdir(obj_dir): continue
        cam_dir = os.path.join(obj_dir, "camera")
        if not os.path.isdir(cam_dir): continue
        images = sorted([f for f in os.listdir(cam_dir) if f.endswith(".png")])
        if not images: continue

        first = cv2.imread(os.path.join(cam_dir, images[0]))
        if first is None: continue
        h, w = first.shape[:2]
        out_mp4 = os.path.join(obj_dir, "{}.mp4".format(os.path.basename(obj_dir)))
        fourcc = choose_fourcc_h264()
        writer = cv2.VideoWriter(out_mp4, fourcc, FRAME_RATE, (w, h))
        if not writer.isOpened():
            # fallback to mp4v
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(out_mp4, fourcc, FRAME_RATE, (w, h))

        for fn in images:
            frame = cv2.imread(os.path.join(cam_dir, fn))
            if frame is None: continue
            writer.write(frame)
        writer.release()
        print("[OK] Video -> {}".format(out_mp4))

    # Copy odom CSV to the bag folder (already there, but ensure presence)
    src = os.path.join(BAG_OUT_DIR, "{}_novatel_odom_data.csv".format(BAG_BASENAME))
    dst = os.path.join(BAG_OUT_DIR, os.path.basename(src))
    if os.path.exists(src):
        try:
            shutil.copy(src, dst)
        except shutil.Error:
            pass

# ====== MAIN ==========================================================
def main():
    print("=== Pipeline start ===")
    print("Bag : {}".format(ROSBAG_FILE))
    print("Out : {}".format(BAG_OUT_DIR))
    with rosbag.Bag(ROSBAG_FILE, 'r') as bag:
        fused_csv = step1_dump_fused_bbox_csv(bag)
        metadata = step2_extract_metadata(bag)
        odom_csv  = step3_dump_odom_csv(bag)
        trajs, traj_csv = step5_build_trajectories(fused_csv, odom_csv)
        # If debugging, keep only first N objects by sorted ID
        if DEBUG_FIRST_N_OBJECTS and DEBUG_FIRST_N_OBJECTS > 0:
            keep_ids = sorted(trajs.keys())[:DEBUG_FIRST_N_OBJECTS]
            trajs = {tid: trajs[tid] for tid in keep_ids}
            if VERBOSE:
                print("[INFO] DEBUG: Restricting to first {} objects: {}".format(DEBUG_FIRST_N_OBJECTS, keep_ids))
        metrics = step4_calculate_key_metrics(odom_csv, metadata, trajs)
        step6_write_key_metrics_csv(metrics)
        step7p_finalize_objects(trajs)
        step7_extract_frames(bag, trajs)
    step8_make_videos_and_copy_odom()
    print("=== Done. Results under: {} ===".format(BAG_OUT_DIR))

if __name__ == "__main__":
    main()
