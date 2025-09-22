## AVA Perception Data Pipeline

This repository contains a Python-based pipeline to extract, associate, and package perception outputs from ROS bag files into per-object artifacts (CSVs, plots, camera frame dumps, and videos), along with ego odometry. It also writes intermediate CSVs for sanity checks.

### Prerequisites
- ROS1 environment with `rosbag` available in Python (typically Python 2 for Kinetic/Melodic). Noetic uses Python 3; this script currently targets Python 2.
- Python packages: `numpy`, `matplotlib`, `opencv-python` (`cv2`), and optionally `cv_bridge` for faster image conversion.
- Access to bag topics:
  - `/yolov9/fused_bbox` (jsk_recognition_msgs/BoundingBoxArray)
  - `/novatel/oem7/odom` (nav_msgs/Odometry)
  - `/yolov9/published_image` (sensor_msgs/Image)
  - `/rosbag_metadata` (std_msgs/String) - contains metadata in format: "location: RTA-4 Transit, vehicle: blue, passengers: 2, road_type: gravel, road_condition: dry, comments: No comments, maneuver: manual driving, categories: perception_output"

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

2) Metadata Extraction
   - Reads `std_msgs/String` from `/rosbag_metadata`.
   - Parses metadata fields: location, vehicle, passengers, road_type, road_condition, comments, maneuver.

3) NovAtel Odometry → CSV
   - Reads `nav_msgs/Odometry` from `/novatel/oem7/odom`.
   - Writes `{BAG_OUT_DIR}/{bag}_novatel_odom_data.csv`.

4) Key Metrics Calculation
   - Calculates duration, distance, max/average velocity, max acceleration/deceleration from odometry.
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

### How to Run (Python 2)
Ensure you are in the ROS Python 2 environment where `rosbag` is available.

```bash
python2 /Users/ylin/Research/AVA/data-pipeline.py
```

Notes:
- If using ROS Noetic (Python 3), adjust to `python3` and ensure dependencies are available.
- Verbose progress is enabled by default (`VERBOSE = True`).
- To process all objects, set `DEBUG_FIRST_N_OBJECTS = 0`. For a quick test, set it to a small number (e.g., `2`).

### Output Structure

Given a bag basename `{bag}` and the configured `EXTRACTION_DATE`, the pipeline writes two sibling folders in the current working directory:

```text
/Users/ylin/Research/AVA/
  Extracted_data_{EXTRACTION_DATE}/
    {bag}_extracted_data/
      {bag}_novatel_odom_data.csv
      {bag}_key_metrics.csv
      {bag}_{id}/
        camera/
          frame_*.png
        smoothed_trajectory_{id}.csv
        x-y-{id}-*.png
        y-t-{id}-*.png
        {bag}_{id}.mp4

  Intermediate_data_{EXTRACTION_DATE}/
    {bag}_fused_bbox_results.csv
    {bag}_trajectories_raw.csv
```

Where:
- `Extracted_data_{EXTRACTION_DATE}/{bag}_extracted_data/` contains the per-object deliverables, odometry CSV, and key metrics CSV.
- `Intermediate_data_{EXTRACTION_DATE}/` contains the intermediate CSVs for sanity checks.

### Key Metrics CSV Format
The `{bag}_key_metrics.csv` file contains:
- **Odometry metrics**: duration_s, distance_m, max_velocity_ms, avg_velocity_ms, max_acceleration_ms2, max_deceleration_ms2
- **Object counts**: total_objects, objects_type_{label} for each detected object type
- **Metadata fields**: location, vehicle, passengers, road_type, road_condition, comments, maneuver

Example:
```csv
metric,value
duration_s,45.2
distance_m,123.4
max_velocity_ms,8.5
avg_velocity_ms,2.7
max_acceleration_ms2,1.2
max_deceleration_ms2,-2.1
total_objects,15
objects_type_80,3
objects_type_9,2
location,RTA-4 Transit
vehicle,blue
passengers,2
road_type,gravel
road_condition,dry
comments,No comments
maneuver,manual driving
```

### Tips
- If camera images fail to save, verify `cv2` is installed and images are decodable; the script will log warnings when it can’t write files.
- If you see few or no tracks, check topic names, labels, and gating parameters.
- Update `STATIC_LABEL_IDS` with your detector’s numeric labels for static objects to enforce average-position smoothing on those classes.


