"""Resolve installed YOLO asset paths from the package share directory."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

_SOURCE_ASSETS_DIR = Path(__file__).resolve().parents[1] / "assets"


def yolo_assets_dir() -> Path:
    try:
        return Path(get_package_share_directory("mycobot_yolo_assets")) / "assets"
    except PackageNotFoundError:
        return _SOURCE_ASSETS_DIR


def default_model_path() -> str:
    return str(
        yolo_assets_dir()
        / "runs"
        / "cube_detector"
        / "weights"
        / "best.pt"
    )


def default_dataset_dir() -> str:
    return str(yolo_assets_dir() / "dataset")


def default_dataset_yaml() -> str:
    return str(yolo_assets_dir() / "dataset" / "data.yaml")


def default_base_model_path() -> str:
    return str(yolo_assets_dir() / "yolo11n.pt")
