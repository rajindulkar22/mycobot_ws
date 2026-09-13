# mycobot_yolo_assets

Version-controlled **YOLO11n** assets for Project 17 (AI-based cube detection in simulation):

| Path under `assets/` | Contents |
|----------------------|----------|
| `yolo11n.pt` | Ultralytics base model |
| `dataset/` | 310 sim images, labels, `data.yaml` (243 train / 67 val) |
| `runs/cube_detector/` | Trained weights (`best.pt`, `last.pt`), metrics, plots |

**Validated model (sim):** mAP50 **0.995**, precision **0.999**, recall **1.000** — see [YOLO handoff](../mycobot_sim_projects/YOLO_VISION_GUIDED_PICK_AND_PLACE.md).

**Demo:** [Dataset generation screencast](media/yolo_dataset_generation.webm)

After build:

```bash
source install/setup.bash
python3 -c "from mycobot_yolo_assets.paths import default_model_path; print(default_model_path())"
```

Full workflow: [YOLO_VISION_GUIDED_PICK_AND_PLACE.md](../mycobot_sim_projects/YOLO_VISION_GUIDED_PICK_AND_PLACE.md).
