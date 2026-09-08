# mycobot_yolo_assets

Version-controlled YOLO assets for Project 17 (cube detection):

| Path under `assets/` | Contents |
|----------------------|----------|
| `yolo11n.pt` | Ultralytics base model |
| `dataset/` | Sim training images, labels, `data.yaml` |
| `runs/cube_detector/` | Trained weights (`best.pt`, `last.pt`) and metrics |

**Demo:** [Dataset generation screencast](media/yolo_dataset_generation.webm)

After build:

```bash
source install/setup.bash
python3 -c "from mycobot_yolo_assets.paths import default_model_path; print(default_model_path())"
```

Full workflow: [YOLO_VISION_GUIDED_PICK_AND_PLACE.md](../mycobot_sim_projects/YOLO_VISION_GUIDED_PICK_AND_PLACE.md).
