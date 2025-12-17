## AVA Data Pipelines

This repository contains Python-based pipelines to extract and analyze data from ROS bag files:

1. **Perception Data Pipeline** (`data-pipeline.py`): Extracts, associates, and packages perception outputs into per-object artifacts (CSVs, plots, camera frame dumps, and videos), along with ego odometry.

2. **Control Data Pipeline** (`control-data-pipeline.py`): Extracts control system data including lateral control performance, velocity/acceleration commands, autonomous mode metrics, and vehicle odometry.

**Note**: The Perception Data Pipeline writes intermediate CSVs for sanity checks. The Control Data Pipeline saves all CSV files directly in the bag output directory (no intermediate folder).

### Prerequisites
- ROS1 environment with `rosbag` available in Python (typically Python 2 for Kinetic/Melodic). Noetic uses Python 3; this script currently targets Python 2.
- Python packages: `numpy`, `matplotlib`, `opencv-python` (`cv2`), and optionally `cv_bridge` for faster image conversion.

### Perception Data Pipeline Topics
- `/yolov9/fused_bbox` (jsk_recognition_msgs/BoundingBoxArray)
- `/novatel/oem7/odom` (nav_msgs/Odometry)
- `/yolov9/published_image` (sensor_msgs/Image)
- `/rosbag_metadata` (std_msgs/String) - contains metadata in format: "location: RTA-4 Transit, vehicle: blue, passengers: 2, road_type: gravel, road_condition: dry, comments: No comments, maneuver: manual driving, categories: perception_output"

### Control Data Pipeline Topics
- `novatel/oem7/odom` (nav_msgs/Odometry) - vehicle odometry (position, orientation, velocities)
- `lat_ctrl_perf` (geometry_msgs/Vector3Stamped) - y=cross-track error, z=yaw error
- `ctrl_ref_twist` (geometry_msgs/TwistStamped) - velocity commanded
- `lat_ctrl_cmd` (geometry_msgs/Vector3Stamped) - acceleration commanded
- `ctrl_ref_curv` (geometry_msgs/PointStamped) - curvature reference
- `steer_ctrl_cmd` (geometry_msgs/Vector3Stamped) - autonomous mode indicator

### Configuration

#### Perception Data Pipeline (`data-pipeline.py`)
Edit `data-pipeline.py` top-level constants to match your environment:
- `EXTRACTION_DATE`: Tag used in output folder names.
- **Input Configuration** (choose one):
  - **Batch Processing**: Set `ROSBAG_FOLDER` to the path containing multiple `.bag` files, and `ROSBAG_FILE = None`
  - **Single File**: Set `ROSBAG_FILE` to the absolute path of a single `.bag` file, and `ROSBAG_FOLDER = None`
- **Folder Search Options**:
  - `SEARCH_RECURSIVELY = True`: Search for `.bag` files in all subdirectories (default)
  - `SEARCH_RECURSIVELY = False`: Search only in the top-level folder
- Topic names: `TOPIC_ODOM`, `TOPIC_FUSED_BBOX`, `TOPIC_CAMERA_IMG`.
- Association/smoothing params: `MAX_SPEED_M_S`, `BASE_GATING_M`, `MIN_TRACK_LIFETIME_S`, `SMOOTH_WINDOW_K`, `FRAME_RATE`.
- Verbosity/debug:
  - `VERBOSE = True` for progress messages.
  - `DEBUG_FIRST_N_OBJECTS = 0` to process all objects; set to a small number (e.g., `2`) for a quick debugging run.
- Static object smoothing:
  - `STATIC_LABEL_IDS = { ... }` set of numeric labels treated as static (cones/signs/lights). If left empty, a heuristic marks tracks with <1 m total span as static.

#### Control Data Pipeline (`control-data-pipeline.py`)
Edit `control-data-pipeline.py` top-level constants to match your environment:
- `EXTRACTION_DATE`: Tag used in output folder names.
- **Input Configuration** (choose one):
  - **Batch Processing**: Set `ROSBAG_FOLDER` to the path containing multiple `.bag` files, and `ROSBAG_FILE = None`
  - **Single File**: Set `ROSBAG_FILE` to the absolute path of a single `.bag` file, and `ROSBAG_FOLDER = None`
- **Folder Search Options**:
  - `SEARCH_RECURSIVELY = True`: Search for `.bag` files in all subdirectories (default)
  - `SEARCH_RECURSIVELY = False`: Search only in the top-level folder
- Topic names: `TOPIC_ODOM`, `TOPIC_LAT_CTRL_PERF`, `TOPIC_CTRL_REF_TWIST`, `TOPIC_LAT_CTRL_CMD`, `TOPIC_CTRL_REF_CURV`, `TOPIC_STEER_CTRL_CMD`
- Verbosity/debug:
  - `VERBOSE = True` for progress messages.
  - `DEBUG_FIRST_N_MESSAGES = 0` to process all messages; set to a small number for debugging.

### Processing Steps (High-Level Logic)

#### Perception Data Pipeline
1) Fused Bounding Boxes → CSV
   - Reads `BoundingBoxArray` messages from `/yolov9/fused_bbox`.
   - Writes per-detection rows (timestamp, position, dimensions, label) to `{INTERMEDIATE_OUT}/{bag}_fused_bbox_results.csv`.

2) Metadata Extraction
   - Reads `std_msgs/String` from `/rosbag_metadata`.
   - Parses metadata fields: location, vehicle, passengers, road_type, road_condition, comments, maneuver.

3) NovAtel Odometry → CSV
   - Reads `nav_msgs/Odometry` from `/novatel/oem7/odom`.
   - Writes `{BAG_OUT_DIR}/{bag}_novatel_odom_data.csv`.

4) Key Metrics Calculation
   - Calculates duration, distance, max/average velocity, max acceleration/deceleration from odometry.
   - Uses downsampling to reduce noise in velocity/acceleration calculations (aims for ~100 samples).
   - Counts detected objects by type from trajectories.
   - Combines with metadata fields.
   - Writes `{BAG_OUT_DIR}/{bag}_key_metrics.csv`.

5) Trajectories Build (Association + Smoothing)
   - Interpolates ego pose to detection timestamps.
   - Transforms detections from ego frame into a global frame.
   - Associates detections over time via greedy nearest-neighbor with speed gating.
   - Filters very short-lived tracks.
   - Smooths each track:
     - If label is in `STATIC_LABEL_IDS` (or heuristic says static), replaces all x,y with the track's global average position.
     - Else uses moving average (`SMOOTH_WINDOW_K`).
   - Writes `{INTERMEDIATE_OUT}/{bag}_trajectories_raw.csv`.

6) Per-Object Finalization
   - For each object ID, writes `smoothed_trajectory_{id}.csv`.
   - Generates `x-y-{id}-*.png` and `y-t-{id}-*.png` plots.

7) Camera Frame Extraction
   - Dumps camera frames into each object's folder for a time window spanning ±2 s around the object's time range.

8) MP4 Generation
   - Converts per-object frames to MP4 videos using H.264 (`avc1`) or `mp4v` fallback.

#### Control Data Pipeline
1) Lateral Control Performance → CSV
   - Reads `geometry_msgs/Vector3Stamped` from `lat_ctrl_perf`.
   - Extracts cross-track error (vector.y) and yaw error (vector.z).
   - Writes to `{BAG_OUT_DIR}/{bag}_lat_ctrl_perf.csv`.

2) Velocity Commanded → CSV
   - Reads `geometry_msgs/TwistStamped` from `ctrl_ref_twist`.
   - Calculates velocity magnitude from linear components.
   - Writes to `{BAG_OUT_DIR}/{bag}_velocity_cmd.csv`.

3) Acceleration Commanded → CSV
   - Reads `geometry_msgs/Vector3Stamped` from `lat_ctrl_cmd`.
   - Calculates acceleration magnitude from vector components.
   - Writes to `{BAG_OUT_DIR}/{bag}_acceleration_cmd.csv`.

4) Curvature Reference → CSV
   - Reads `geometry_msgs/PointStamped` from `ctrl_ref_curv`.
   - Extracts curvature reference (point.x).
   - Writes to `{BAG_OUT_DIR}/{bag}_curvature_ref.csv`.

5) Autonomous Mode Detection → CSV
   - Reads `geometry_msgs/Vector3Stamped` from `steer_ctrl_cmd`.
   - If messages exist, vehicle is in autonomous mode.
   - Writes to `{BAG_OUT_DIR}/{bag}_autonomous_mode.csv`.

6) Odometry Extraction → CSV
   - Reads `nav_msgs/Odometry` from `novatel/oem7/odom`.
   - Extracts position (x, y, z), orientation quaternion (x, y, z, w), linear velocity, angular velocity.
   - Writes to `{BAG_OUT_DIR}/{bag}_odom.csv`.

7) Control Key Metrics Calculation
   - Calculates autonomous duration from first to last `steer_ctrl_cmd` timestamp.
   - Estimates autonomous distance as average commanded speed during autonomous window × duration.
   - Computes max cross-track error, max yaw error, max velocity/acceleration/deceleration commanded.
   - Writes to `{BAG_OUT_DIR}/{bag}_control_key_metrics.csv`.

8) Control Plots Generation
   - Creates time-series plots: lateral error, yaw error, velocity commanded, acceleration commanded.
   - Saves to `{BAG_OUT_DIR}/control_plots/`.

### How to Run (Python 2)
Ensure you are in the ROS Python 2 environment where `rosbag` is available.

#### Perception Data Pipeline
**For Batch Processing (Multiple Rosbags):**
```python
# In data-pipeline.py, set:
ROSBAG_FOLDER = "/path/to/your/rosbag/folder/"
ROSBAG_FILE = None
```

**For Single File Processing:**
```python
# In data-pipeline.py, set:
ROSBAG_FILE = "/path/to/single/rosbag.bag"
ROSBAG_FOLDER = None
```

Then run:
```bash
python2 /Users/ylin/Research/AVA/data-pipeline.py
```

#### Control Data Pipeline
**For Batch Processing (Multiple Rosbags):**
```python
# In control-data-pipeline.py, set:
ROSBAG_FOLDER = "/path/to/your/control/rosbag/folder/"
ROSBAG_FILE = None
```

**For Single File Processing:**
```python
# In control-data-pipeline.py, set:
ROSBAG_FILE = "/path/to/single/control/rosbag.bag"
ROSBAG_FOLDER = None
```

Then run:
```bash
python2 /Users/ylin/Research/AVA/control-data-pipeline.py
```

Notes:
- If using ROS Noetic (Python 3), adjust to `python3` and ensure dependencies are available.
- Verbose progress is enabled by default (`VERBOSE = True`).
- To process all objects, set `DEBUG_FIRST_N_OBJECTS = 0`. For a quick test, set it to a small number (e.g., `2`).
- **Batch processing**: The pipeline will automatically discover all `.bag` files in the specified folder and process them sequentially.
- **Error handling**: If individual bags fail to process, the pipeline continues with remaining bags and provides a summary at the end.

### Output Structure

Given a bag basename `{bag}` and the configured `EXTRACTION_DATE`, the pipelines write sibling folders in the current working directory:

#### Perception Data Pipeline Output

**For Single File Processing:**
```text
.
├── Extracted_data_{EXTRACTION_DATE}/
│   └── {bag}_extracted_data/
│       ├── {bag}_novatel_odom_data.csv
│       ├── {bag}_key_metrics.csv
│       └── {bag}_{id}/
│           ├── camera/
│           │   └── frame_*.png
│           ├── smoothed_trajectory_{id}.csv
│           ├── x-y-{id}-{object_type}.png
│           ├── y-t-{id}-{object_type}.png
│           └── {bag}_{id}.mp4
└── Intermediate_data_{EXTRACTION_DATE}/
    ├── {bag}_fused_bbox_results.csv
    └── {bag}_trajectories_raw.csv
```

**For Batch Processing (Multiple Rosbags):**
```text
.
├── Extracted_data_{EXTRACTION_DATE}/
│   ├── {bag1}_extracted_data/
│   │   ├── {bag1}_novatel_odom_data.csv
│   │   ├── {bag1}_key_metrics.csv
│   │   └── {bag1}_{id}/
│   │       ├── camera/
│   │       │   └── frame_*.png
│   │       ├── smoothed_trajectory_{id}.csv
│   │       ├── x-y-{id}-{object_type}.png
│   │       ├── y-t-{id}-{object_type}.png
│   │       └── {bag1}_{id}.mp4
│   ├── {bag2}_extracted_data/
│   │   └── ...
│   └── {bag3}_extracted_data/
│       └── ...
└── Intermediate_data_{EXTRACTION_DATE}/
    ├── {bag1}_fused_bbox_results.csv
    ├── {bag1}_trajectories_raw.csv
    ├── {bag2}_fused_bbox_results.csv
    ├── {bag2}_trajectories_raw.csv
    └── ...
```

Where:
- `Extracted_data_{EXTRACTION_DATE}/{bag}_extracted_data/` contains the per-object deliverables, odometry CSV, and key metrics CSV for each bag.
- `Intermediate_data_{EXTRACTION_DATE}/` contains the intermediate CSVs for sanity checks for all processed bags.
- **Batch processing**: Each rosbag gets its own output directory with the same structure, allowing you to process multiple bags while maintaining organized outputs.

#### Control Data Pipeline Output

**For Single File Processing:**
```text
.
└── Control_data_{EXTRACTION_DATE}/
    └── {bag}_control_data/
        ├── {bag}_lat_ctrl_perf.csv
        ├── {bag}_velocity_cmd.csv
        ├── {bag}_acceleration_cmd.csv
        ├── {bag}_curvature_ref.csv
        ├── {bag}_autonomous_mode.csv
        ├── {bag}_odom.csv
        ├── {bag}_control_key_metrics.csv
        └── control_plots/
            ├── lateral_error_vs_time.png
            ├── yaw_error_vs_time.png
            ├── velocity_commanded_vs_time.png
            └── acceleration_commanded_vs_time.png
```

**For Batch Processing (Multiple Rosbags):**
```text
.
└── Control_data_{EXTRACTION_DATE}/
    ├── {bag1}_control_data/
    │   ├── {bag1}_lat_ctrl_perf.csv
    │   ├── {bag1}_velocity_cmd.csv
    │   ├── {bag1}_acceleration_cmd.csv
    │   ├── {bag1}_curvature_ref.csv
    │   ├── {bag1}_autonomous_mode.csv
    │   ├── {bag1}_odom.csv
    │   ├── {bag1}_control_key_metrics.csv
    │   └── control_plots/
    │       ├── lateral_error_vs_time.png
    │       ├── yaw_error_vs_time.png
    │       ├── velocity_commanded_vs_time.png
    │       └── acceleration_commanded_vs_time.png
    ├── {bag2}_control_data/
    │   └── ...
    └── {bag3}_control_data/
        └── ...
```

Where:
- `Control_data_{EXTRACTION_DATE}/{bag}_control_data/` contains all CSV files (control data, odometry, and key metrics) and plots for each bag.
- All output files are organized together in each bag's control data directory (no intermediate folder).
- **Batch processing**: Each rosbag gets its own output directory with the same structure, allowing you to process multiple bags while maintaining organized outputs.

### Key Metrics CSV Format

#### Perception Data Pipeline
The `{bag}_key_metrics.csv` file contains:
- **Odometry metrics**: duration_s, distance_m, max_velocity_ms, avg_velocity_ms, max_acceleration_ms2, max_deceleration_ms2
- **Object counts**: total_objects, objects_type_{label} for each detected object type
- **Metadata fields**: location, vehicle, passengers, road_type, road_condition, comments, maneuver

#### Control Data Pipeline
The `{bag}_control_key_metrics.csv` file contains:
- **Autonomous mode metrics**: autonomous_duration_s, autonomous_distance_m
- **Control performance metrics**: max_cross_track_error, max_yaw_error
- **Commanded metrics**: max_velocity_commanded, max_acceleration_commanded, max_deceleration_commanded

**Velocity/Acceleration Processing (Perception Pipeline):**
- **Average velocity**: Calculated directly from total distance divided by duration
- **Max velocity**: Uses downsampled data (every Nth point) with < 25 m/s filter
- **Max acceleration**: Uses downsampled velocities with < 5 m/s² filter  
- **Max deceleration**: Uses downsampled velocities with > -5 m/s² filter
- **Noise reduction**: Downsampling reduces high-frequency sensor noise for realistic measurements

**Autonomous Distance Estimation (Control Pipeline):**
- **Autonomous distance**: Estimated as average commanded speed during autonomous window × autonomous duration
- **No odometry dependency**: Uses only control topics for distance estimation

#### Example Control Key Metrics CSV:

| Metric | Value |
|--------|-------|
| autonomous_duration_s | 45.2 |
| autonomous_distance_m | 123.4 |
| max_cross_track_error | 0.15 |
| max_yaw_error | 0.08 |
| max_velocity_commanded | 8.5 |
| max_acceleration_commanded | 1.2 |
| max_deceleration_commanded | -2.1 |

#### Example Perception Key Metrics CSV:

| Metric | Value |
|--------|-------|
| duration_s | 45.2 |
| distance_m | 123.4 |
| max_velocity_ms | 8.5 |
| avg_velocity_ms | 2.7 |
| max_acceleration_ms2 | 1.2 |
| max_deceleration_ms2 | -2.1 |
| total_objects | 15 |
| objects_type_cone | 3 |
| objects_type_traffic_light | 2 |
| location | RTA-4 Transit |
| vehicle | blue |
| passengers | 2 |
| road_type | gravel |
| road_condition | dry |
| comments | No comments |
| maneuver | manual driving |

### Tips

#### General Tips
- **Batch processing**: Both pipelines process bags sequentially and provide progress updates. Failed bags are logged but don't stop the entire batch.
- **Memory usage**: For large batches, consider processing smaller subsets if memory becomes an issue.
- **Output organization**: Each bag maintains its own output directory, making it easy to identify results from specific rosbags.

#### Perception Data Pipeline Tips
- If camera images fail to save, verify `cv2` is installed and images are decodable; the script will log warnings when it can't write files.
- If you see few or no tracks, check topic names, labels, and gating parameters.
- Update `STATIC_LABEL_IDS` with your detector's numeric labels for static objects to enforce average-position smoothing on those classes.

#### Control Data Pipeline Tips
- If CSV files are empty, check that topic names match exactly (no leading `/` for control topics; odometry topic may or may not have leading `/`).
- The script includes debug output to show available topics in your rosbag - use this to verify topic names.
- Autonomous mode detection relies on the presence of `steer_ctrl_cmd` messages.
- Distance estimation uses average commanded speed during autonomous mode - this is an approximation.
- For more accurate distance calculations, use the odometry CSV file (`{bag}_odom.csv`) to compute actual traveled distance.
- **Odometry data**: Position is in global coordinates (typically UTM/ENU frame). Orientation is provided as quaternion - extract yaw angle if needed for heading analysis.


