## AVA Perception Data Pipeline

This repository contains a Python-based pipeline to extract, associate, and package perception outputs from ROS bag files into per-object artifacts (CSVs, plots, camera frame dumps, and videos), along with ego odometry. It also writes intermediate CSVs for sanity checks.

### Prerequisites
- ROS1 environment with `rosbag` available in Python (typically Python 2 for Kinetic/Melodic). Noetic uses Python 3; this script currently targets Python 2.
- Python packages: `numpy`, `matplotlib`, `opencv-python` (`cv2`), and optionally `cv_bridge` for faster image conversion.
- Access to bag topics:
  - `/yolov9/fused_bbox` (jsk_recognition_msgs/BoundingBoxArray)
  - `/novatel/oem7/odom` (nav_msgs/Odometry)
  - `/yolov9/published_image` (sensor_msgs/Image)

### Configuration
Edit `data-pipeline.py` top-level constants to match your environment:
- `EXTRACTION_DATE`: Tag used in output folder names.
- `ROSBAG_FILE`: Absolute path to the input `.bag` file.
- Topic names: `TOPIC_ODOM`, `TOPIC_FUSED_BBOX`, `TOPIC_CAMERA_IMG`.
- Association/smoothing params: `MAX_SPEED_M_S`, `BASE_GATING_M`, `MIN_TRACK_LIFETIME_S`, `SMOOTH_WINDOW_K`, `FRAME_RATE`.
- Verbosity/debug:
  - `VERBOSE = True` for progress messages.
  - `DEBUG_FIRST_N_OBJECTS = 0` to process all objects; set to a small number (e.g., `2`) for a quick debugging run.
- Static object smoothing:
  - `STATIC_LABEL_IDS = { ... }` set of numeric labels treated as static (cones/signs/lights). If left empty, a heuristic marks tracks with <1 m total span as static.

### Processing Steps (High-Level Logic)
1) Fused Bounding Boxes → CSV
   - Reads `BoundingBoxArray` messages from `/yolov9/fused_bbox`.
   - Writes per-detection rows (timestamp, position, dimensions, label) to `{INTERMEDIATE_OUT}/{bag}_fused_bbox_results.csv`.

2) NovAtel Odometry → CSV
   - Reads `nav_msgs/Odometry` from `/novatel/oem7/odom`.
   - Writes `{BAG_OUT_DIR}/{bag}_novatel_odom_data.csv`.

3) Trajectories Build (Association + Smoothing)
   - Interpolates ego pose to detection timestamps.
   - Transforms detections from ego frame into a global frame.
   - Associates detections over time via greedy nearest-neighbor with speed gating.
   - Filters very short-lived tracks.
   - Smooths each track:
     - If label is in `STATIC_LABEL_IDS` (or heuristic says static), replaces all x,y with the track’s global average position.
     - Else uses moving average (`SMOOTH_WINDOW_K`).
   - Writes `{INTERMEDIATE_OUT}/{bag}_trajectories_raw.csv`.

3.5) Per-Object Finalization
   - For each object ID, writes `smoothed_trajectory_{id}.csv`.
   - Generates `x-y-{id}-*.png` and `y-t-{id}-*.png` plots.

4) Camera Frame Extraction
   - Dumps camera frames into each object’s folder for a time window spanning ±2 s around the object’s time range.

5) MP4 Generation
   - Converts per-object frames to MP4 videos using H.264 (`avc1`) or `mp4v` fallback.

### How to Run (Python 2)
Ensure you are in the ROS Python 2 environment where `rosbag` is available.

```bash
python2 data-pipeline.py
```

Notes:
- If using ROS Noetic (Python 3), adjust to `python3` and ensure dependencies are available.
- Verbose progress is enabled by default (`VERBOSE = True`).
- To process all objects, set `DEBUG_FIRST_N_OBJECTS = 0`. For a quick test, set it to a small number (e.g., `2`).

### Output Structure

Given a bag basename `{bag}` and the configured `EXTRACTION_DATE`, the pipeline writes two sibling folders in the current working directory:

```text
.
├── Extracted_data_{EXTRACTION_DATE}/
│   ├── {bag}_extracted_data/
│   │   ├── {bag}_novatel_odom_data.csv
│   │   ├── {bag}_{object_id}/
│   │   │   ├── camera/
│   │   │   │   ├── frame_*.png
│   │   │   │   └── ...
│   │   │   ├── smoothed_trajectory_{object_id}.csv
│   │   │   ├── x-y-{object_id}-*.png
│   │   │   ├── y-t-{object_id}-*.png
│   │   │   └── {bag}_{object_id}.mp4
│   │   └── {bag}_{object_id}/
│   │       └── ...
│   └── {bag}_extracted_data/
│       └── ...
└── Intermediate_data_{EXTRACTION_DATE}/
    ├── {bag}_fused_bbox_results.csv
    ├── {bag}_trajectories_raw.csv
    └── ...
```

Where:
- `Extracted_data_{EXTRACTION_DATE}/{bag}_extracted_data/` contains the per-object deliverables and odometry CSV.
- `Intermediate_data_{EXTRACTION_DATE}/` contains the intermediate CSVs for sanity checks.

### Tips
- If camera images fail to save, verify `cv2` is installed and images are decodable; the script will log warnings when it can’t write files.
- If you see few or no tracks, check topic names, labels, and gating parameters.
- Update `STATIC_LABEL_IDS` with your detector’s numeric labels for static objects to enforce average-position smoothing on those classes.


