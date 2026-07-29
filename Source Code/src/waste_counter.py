"""waste_counter.py
=================
Helper counting per-kelas untuk stream kamera 5 kelas
(Kaca, Kertas, Logam, Plastik, Organik).

Aturan counting:
- Kelas ANORGANIK (Kaca/Kertas/Logam/Plastik) di-counting HANYA saat
  objek tersebut BENAR-BENAR sudah masuk penampungan (bin success),
  yaitu saat firmware ESP32 mengirim EVENT:SORT_DONE tanpa indikasi FAIL.
- Kelas ORGANIK (virtual, tidak dideteksi YOLO) di-counting saat sebuah
  track melewati/keluar dari ujung PALING KIRI ROI tanpa pernah berhasil
  di-bin-kan sebagai anorganik. Ini mewakili objek yang lolos sampai ujung
  conveyor (tidak diambil robot) dan otomatis jatuh ke jalur organik.

Setiap track_id hanya boleh berkontribusi PALING BANYAK 1 kali counting
(baik sebagai anorganik lewat bin success, ATAU sebagai organik lewat
ujung kiri ROI) -- state ini dijaga lewat parameter `counted_track_ids`.
"""
from __future__ import annotations

from typing import Any

ORGANIC_CLASS_NAME_DEFAULT = "Organik"


def organic_class_name(config: Any) -> str:
    return str(getattr(config, "ORGANIC_CLASS_NAME", ORGANIC_CLASS_NAME_DEFAULT))


def init_class_counts(config: Any) -> dict[str, int]:
    """Buat dict counting awal (semua 0) untuk kelas anorganik + organik."""
    names = list(getattr(config, "WASTE_CLASS_NAMES", []))
    counts: dict[str, int] = {str(name): 0 for name in names}
    counts[organic_class_name(config)] = 0
    return counts


def count_increment(config: Any) -> int:
    return int(getattr(config, "CLASS_COUNT_INCREMENT", 1))


def normalize_class_name(class_name: str, config: Any) -> str | None:
    """Kembalikan nama kelas kanonis sesuai config.WASTE_CLASS_NAMES.

    Pencocokan dibuat case-insensitive agar variasi kapitalisasi tidak
    menghasilkan key counter baru yang tidak tampil pada overlay.
    """
    raw = str(class_name or "").strip()
    if not raw:
        return None

    configured = [str(name) for name in getattr(config, "WASTE_CLASS_NAMES", [])]
    by_lower = {name.casefold(): name for name in configured}
    canonical = by_lower.get(raw.casefold())
    if canonical is not None:
        return canonical

    # Fallback bila class_name yang tersimpan ternyata berupa class-id.
    try:
        class_id = int(raw)
    except (TypeError, ValueError):
        class_id = -1
    if 0 <= class_id < len(configured):
        return configured[class_id]
    return None


def register_bin_success(
    track_id: int,
    class_name: str,
    class_counts: dict[str, int],
    counted_track_ids: set[int],
    config: Any,
) -> bool:
    """Panggil saat firmware mengonfirmasi objek berhasil masuk bin (anorganik).

    Return True jika counting benar-benar bertambah (mencegah double count
    per track_id yang sama).
    """
    if track_id in counted_track_ids:
        return False

    name = normalize_class_name(class_name, config)
    if name is None:
        print(
            f"[COUNTING] ID{track_id} BIN SUCCESS tidak dihitung: "
            f"nama kelas tidak valid/track sudah hilang ({class_name!r})."
        )
        return False

    if name not in class_counts:
        class_counts[name] = 0
    class_counts[name] += count_increment(config)
    counted_track_ids.add(track_id)
    print(f"[COUNTING] ID{track_id} -> BIN SUCCESS kelas '{name}' = {class_counts[name]}")
    return True


def register_organic_left_edge_exits(
    object_tracks: dict[int, dict],
    roi_left_x: float,
    class_counts: dict[str, int],
    counted_track_ids: set[int],
    config: Any,
) -> list[int]:
    """Cek semua track yang posisi TERAKHIRNYA (cx) sudah melewati ujung
    Return list track_id yang baru saja di-counting sebagai organik pada
    pemanggilan ini.
    """
    margin = float(getattr(config, "CLASS_COUNT_LEFT_EDGE_MARGIN_PX", 15.0))
    threshold_x = float(roi_left_x) + margin
    organic_name = organic_class_name(config)
    newly_counted: list[int] = []

    for track_id, tr in object_tracks.items():
        if track_id in counted_track_ids:
            continue
        cx = tr.get("cx")
        if cx is None:
            continue
        cx = float(cx)
        if cx > threshold_x:
            continue

        if organic_name not in class_counts:
            class_counts[organic_name] = 0
        class_counts[organic_name] += count_increment(config)
        counted_track_ids.add(track_id)
        newly_counted.append(track_id)
        print(
            f"[COUNTING] ID{track_id} -> lewat ujung kiri ROI (cx={cx:.0f} <= {threshold_x:.0f}), "
            f"kelas '{organic_name}' = {class_counts[organic_name]}"
        )

    return newly_counted


def cleanup_counted_ids(object_tracks: dict[int, dict], counted_track_ids: set[int]) -> None:
    """Buang track_id lama dari counted_track_ids yang track-nya sudah
    benar-benar hilang dari object_tracks, supaya set ini tidak membesar
    tanpa batas selama program berjalan lama."""
    for old_id in list(counted_track_ids):
        if old_id not in object_tracks:
            counted_track_ids.discard(old_id)


def write_class_count_csv(
    class_counts: dict[str, int],
    class_order: list[str],
    config: Any,
) -> str | None:
    """Tulis ulang (overwrite) CSV rekap counting per kelas.

    Kolom: kelas, jumlah_sukses, target_percobaan, rasio
    rasio = jumlah_sukses / target_percobaan (mis. 8 dari 10 -> 0.8).

    Dipanggil setiap kali ada counting baru (bin success ANORGANIK atau
    exit ORGANIK), file selalu berisi rekap TERKINI (bukan history baris
    per event).

    Return path CSV yang ditulis, atau None kalau gagal.
    """
    import csv
    import os

    path = str(getattr(config, "CLASS_COUNT_CSV_PATH", ""))
    if not path:
        return None

    target = int(getattr(config, "CLASS_COUNT_TARGET_PER_CLASS", 10))

    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["kelas", "jumlah_sukses", "target_percobaan", "rasio"])
            for name in class_order:
                jumlah = int(class_counts.get(name, 0))
                rasio = (jumlah / target) if target > 0 else 0.0
                writer.writerow([name, jumlah, target, f"{rasio:.2f}"])
        return path
    except Exception as e:
        print(f"[COUNTING] Gagal menulis CSV rekap counting: {e}")
        return None
