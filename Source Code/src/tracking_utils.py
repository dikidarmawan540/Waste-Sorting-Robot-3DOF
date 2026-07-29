"""Helper tracking ByteTrack untuk main.py dan YOLO.py.

File ini sengaja dipisah supaya main.py tidak lagi bergantung pada centroid
tracker manual saat backend BYTETRACK aktif.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any


def tracking_backend(config: Any, *, test_mode: bool = False) -> str:
    name = "YOLO_TEST_TRACKING_BACKEND" if test_mode else "TRACKING_BACKEND"
    return str(getattr(config, name, "BYTETRACK")).strip().upper()


def use_bytetrack(config: Any, *, test_mode: bool = False) -> bool:
    return tracking_backend(config, test_mode=test_mode) in {"BYTETRACK", "BYTE_TRACK", "BYTE-TRACK"}


def bytetrack_config_path(config: Any) -> str:
    path = str(getattr(config, "BYTETRACK_CONFIG_PATH", "bytetrack_conveyor.yaml"))
    if not path:
        return "bytetrack.yaml"
    p = Path(path)
    if p.exists():
        return str(p)
    # Fallback ke tracker bawaan ultralytics jika file lokal tidak ditemukan.
    return "bytetrack.yaml"


def bytetrack_predict_conf(config: Any) -> float:
    return float(getattr(config, "BYTETRACK_YOLO_CONF_THRESHOLD", getattr(config, "CONF_THRESHOLD", 0.25)))


def _boxes_track_ids(boxes_data: Any):
    if boxes_data is None or not hasattr(boxes_data, "id"):
        return None
    ids = boxes_data.id
    if ids is None:
        return None
    try:
        return ids.detach().cpu().numpy().astype(int).tolist()
    except Exception:
        try:
            return ids.cpu().numpy().astype(int).tolist()
        except Exception:
            try:
                return [int(x) for x in ids]
            except Exception:
                return None


def _class_name_from_config(config: Any, cls_id: int, fallback: str | int = "") -> str:
    class_names = getattr(config, "WASTE_CLASS_NAMES", [])
    if isinstance(class_names, dict):
        return str(class_names.get(cls_id, fallback if fallback != "" else cls_id))
    if 0 <= cls_id < len(class_names):
        return str(class_names[cls_id])
    return str(fallback if fallback != "" else cls_id)


def update_tracks_from_bytetrack(
    centroids_all: list[dict],
    boxes_data: Any,
    tracks: dict[int, dict],
    frame_index: int,
    config: Any,
) -> bool:
    """Tempel ID ByteTrack dari result.boxes.id ke list centroid.

    Return True jika ID ByteTrack tersedia. Jika False, pemanggil boleh fallback ke
    tracker internal lama.
    """
    track_ids = _boxes_track_ids(boxes_data)
    if not track_ids:
        return False

    purge_missed = int(getattr(config, "TRACK_PURGE_MISSED_FRAMES", 160))
    class_vote_min = int(getattr(config, "TRACK_CLASS_UPDATE_MIN_VOTES", 3))

    for tr in tracks.values():
        tr["visible"] = False
        tr["matched_this_frame"] = False

    matched_ids: set[int] = set()

    for local_idx, obj in enumerate(centroids_all):
        det_idx = int(obj.get("det_index", local_idx))
        if det_idx < 0 or det_idx >= len(track_ids):
            continue

        track_id = int(track_ids[det_idx])
        if track_id <= 0:
            continue

        raw_cx = float(obj.get("cx", 0.0))
        raw_cy = float(obj.get("cy", 0.0))
        det_cls = int(obj.get("cls_id", -1))
        det_class_name = str(obj.get("class_name", det_cls))
        conf = float(obj.get("conf", 0.0))
        in_pick_zone = bool(obj.get("in_pick_zone", False))
        touches_stop_line = bool(obj.get("touches_pick_stop_line", False))

        tr = tracks.get(track_id)
        if tr is None:
            tr = {
                "track_id": track_id,
                "first_seen_frame": frame_index,
                "first_seen_order": track_id,
                "seen_count": 0,
                "visible_streak": 0,
                "miss_count": 0,
                "skipped": False,
                "class_votes": {},
                "conf_history": [],
            }
            tracks[track_id] = tr

        class_votes = tr.get("class_votes") or {}
        class_votes[str(det_cls)] = int(class_votes.get(str(det_cls), 0)) + 1
        tr["class_votes"] = class_votes
        best_cls_str, best_votes = max(class_votes.items(), key=lambda item: int(item[1]))
        stable_cls = int(best_cls_str) if int(best_votes) >= class_vote_min else int(tr.get("cls_id", det_cls))
        stable_class_name = _class_name_from_config(config, stable_cls, det_class_name)

        old_cx = float(tr.get("cx", raw_cx))
        old_cy = float(tr.get("cy", raw_cy))
        vx = raw_cx - old_cx
        vy = raw_cy - old_cy

        conf_history = list(tr.get("conf_history", []))
        conf_history.append(conf)
        max_hist = int(getattr(config, "TRACK_CONF_HISTORY_SIZE", 8))
        if len(conf_history) > max_hist:
            conf_history = conf_history[-max_hist:]
        avg_conf = sum(conf_history) / len(conf_history) if conf_history else conf

        obj["track_id"] = track_id
        obj["raw_cx"] = raw_cx
        obj["raw_cy"] = raw_cy
        obj["cx"] = raw_cx
        obj["cy"] = raw_cy
        obj["det_cls_id"] = det_cls
        obj["cls_id"] = stable_cls
        obj["class_name"] = stable_class_name
        obj["acc"] = conf * 100.0
        obj["avg_acc"] = avg_conf * 100.0
        obj["in_pick_zone"] = in_pick_zone
        obj["touches_pick_stop_line"] = touches_stop_line

        if in_pick_zone:
            tr["last_in_pick_zone_frame"] = frame_index
        if touches_stop_line:
            tr["last_pick_stop_line_frame"] = frame_index

        tr.update({
            "cls_id": stable_cls,
            "det_cls_id": det_cls,
            "class_name": str(stable_class_name),
            "conf": conf,
            "avg_conf": avg_conf,
            "acc": conf * 100.0,
            "avg_acc": avg_conf * 100.0,
            "cx": raw_cx,
            "cy": raw_cy,
            "raw_cx": raw_cx,
            "raw_cy": raw_cy,
            "x1": float(obj.get("x1", raw_cx)),
            "y1": float(obj.get("y1", raw_cy)),
            "x2": float(obj.get("x2", raw_cx)),
            "y2": float(obj.get("y2", raw_cy)),
            "bbox": obj.get("bbox", [raw_cx, raw_cy, raw_cx, raw_cy]),
            "in_pick_zone": in_pick_zone,
            "touches_pick_stop_line": touches_stop_line,
            "vx": vx,
            "vy": vy,
            "last_seen_frame": frame_index,
            "miss_count": 0,
            "visible": True,
            "matched_this_frame": True,
            "seen_count": int(tr.get("seen_count", 0)) + 1,
            "visible_streak": int(tr.get("visible_streak", 0)) + 1,
            "conf_history": conf_history,
        })
        matched_ids.add(track_id)

    for track_id, tr in list(tracks.items()):
        if track_id not in matched_ids:
            tr["miss_count"] = int(tr.get("miss_count", 0)) + 1
            tr["visible"] = False
            tr["visible_streak"] = 0
            tr["in_pick_zone"] = False
            tr["touches_pick_stop_line"] = False
        if int(tr.get("miss_count", 0)) > purge_missed:
            del tracks[track_id]

    return True
