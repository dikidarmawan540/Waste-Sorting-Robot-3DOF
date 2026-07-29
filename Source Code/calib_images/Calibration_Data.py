import cv2
import numpy as np
import os
import pickle
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# =========================
# CONFIG
# =========================
corners_x    = 4
corners_y    = 6
marker_dict  = cv2.aruco.DICT_4X4_50
square_length = 39      # mm
marker_length = 29.3    # mm
ERROR_WARN    = 1.0
DATA_FILE     = "charuco_data.pkl"
OUTPUT_PNG    = "reprojection_error_chart.png"

# =========================
# LOAD DATA
# =========================
if not os.path.exists(DATA_FILE):
    print(f"[ERROR] '{DATA_FILE}' tidak ditemukan.")
    exit(1)

with open(DATA_FILE, "rb") as f:
    all_corners, all_ids, all_filenames = pickle.load(f)

print(f"[LOADED] {len(all_corners)} captures")

# =========================
# BOARD & IMG SIZE
# =========================
dictionary = cv2.aruco.getPredefinedDictionary(marker_dict)
board = cv2.aruco.CharucoBoard(
    (corners_x + 1, corners_y + 1),
    square_length / 1000.0,
    marker_length / 1000.0,
    dictionary
)

img_size = None
for fname in all_filenames:
    if os.path.exists(fname):
        img = cv2.imread(fname)
        if img is not None:
            img_size = (img.shape[1], img.shape[0])
            break
if img_size is None:
    img_size = (1280, 720)
    print("[WARN] Pakai default 1280x720")

# =========================
# KALIBRASI
# =========================
print("[INFO] Kalibrasi...")
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    all_corners, all_ids, board, img_size, None, None
)
print(f"[INFO] Overall RMS: {ret:.4f} px")

# =========================
# ERROR PER IMAGE
# =========================
errors, labels = [], []
for i in range(len(all_corners)):
    try:
        obj_pts, img_pts = board.matchImagePoints(all_corners[i], all_ids[i])
        proj, _ = cv2.projectPoints(obj_pts, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        e = cv2.norm(img_pts, proj, cv2.NORM_L2) / np.sqrt(len(img_pts))
        errors.append(e)
    except Exception:
        errors.append(None)
    fname = all_filenames[i] if i < len(all_filenames) else f"img_{i:03d}"
    labels.append(os.path.splitext(os.path.basename(fname))[0])

valid_idx = [i for i, e in enumerate(errors) if e is not None]
vals  = [errors[i] for i in valid_idx]
lbls  = [labels[i] for i in valid_idx]
colors = ["#e74c3c" if e > ERROR_WARN else "#f39c12" if e > 0.5 else "#2ecc71" for e in vals]

# =========================
# PLOT 4:3
# =========================
fig, ax = plt.subplots(figsize=(10, 7.5))

ax.bar(range(len(vals)), vals, color=colors, edgecolor="white", linewidth=0.5, width=0.7)

ax.axhline(y=ERROR_WARN, color="#e74c3c", linestyle="--", linewidth=1.2)
ax.axhline(y=0.5,        color="#f39c12", linestyle=":",  linewidth=1.2)
ax.axhline(y=ret,        color="#3498db", linestyle="-.", linewidth=1.5,
           label=f"Overall RMS = {ret:.4f} px")

ax.set_xticks(range(len(vals)))
ax.set_xticklabels(lbls, rotation=90, fontsize=6.5)
ax.set_ylabel("Reprojection Error (px)", fontsize=11, y=0.65)
ax.set_xlabel("Image", fontsize=11)
ax.set_title("Per-Image Reprojection Error — ChArUco Calibration",
             fontsize=12, fontweight="bold", pad=10)
ax.set_ylim(0, max(vals) * 1.2)
ax.yaxis.grid(True, linestyle="--", alpha=0.4)
ax.set_axisbelow(True)

patch_good = mpatches.Patch(color="#2ecc71", label="GOOD  (≤ 0.50 px)")
patch_ok   = mpatches.Patch(color="#f39c12", label="OK      (0.50–1.00 px)")
patch_high = mpatches.Patch(color="#e74c3c", label="HIGH   (> 1.00 px)")
ax.legend(handles=[patch_good, patch_ok, patch_high, ax.get_lines()[2]],
          fontsize=8.5, loc="upper right", framealpha=0.9)

plt.tight_layout()
plt.savefig(OUTPUT_PNG, dpi=200, bbox_inches="tight")
plt.close()
print(f"[SAVED] {OUTPUT_PNG}")
