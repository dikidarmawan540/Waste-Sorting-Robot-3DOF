import cv2
import numpy as np
import os
import pickle
import time

from camera_source import open_camera

# =========================
# CONFIG BOARD
# =========================
corners_x = 4
corners_y = 6

marker_dict = cv2.aruco.DICT_4X4_50

square_length = 39   # mm
marker_length = 29.3   # mm

MAX_CORNERS = corners_x * corners_y
MIN_CORNERS = 24

CAPTURE_DELAY = 1.0

ERROR_WARN_THRESHOLD = 1.0   # flag ⚠️  jika error > nilai ini

# =========================
# OUTPUT
# =========================
save_dir = "calib_images"
os.makedirs(save_dir, exist_ok=True)

DATA_FILE = "charuco_data.pkl"

# =========================
# LOAD PREVIOUS DATA
# =========================
if os.path.exists(DATA_FILE):

    with open(DATA_FILE, "rb") as f:
        all_corners, all_ids, all_filenames = pickle.load(f)

    print(f"[LOADED] Previous captures: {len(all_corners)}")

else:

    all_corners   = []
    all_ids       = []
    all_filenames = []

# =========================
# GLOBAL
# =========================
img_size          = None
img_id            = len(all_corners)
last_capture_time = 0

# =========================
# CHARUCO DETECTION
# =========================
def detect_charuco(frame):

    dictionary = cv2.aruco.getPredefinedDictionary(marker_dict)

    board = cv2.aruco.CharucoBoard(
        (corners_x + 1, corners_y + 1),
        square_length / 1000.0,
        marker_length / 1000.0,
        dictionary
    )

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detector = cv2.aruco.ArucoDetector(dictionary)

    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None or len(ids) == 0:
        return None, None

    retval, charuco_corners, charuco_ids = (
        cv2.aruco.interpolateCornersCharuco(
            corners,
            ids,
            gray,
            board
        )
    )

    if charuco_ids is None:
        return None, None

    return charuco_corners, charuco_ids

# =========================
# DRAW
# =========================
def draw(frame, corners, ids):

    if ids is not None:

        for c, i in zip(corners, ids):

            x = int(c[0][0])
            y = int(c[0][1])

            cv2.circle(
                frame,
                (x, y),
                4,
                (0, 255, 255),
                -1
            )

            cv2.putText(
                frame,
                str(int(i[0])),
                (x + 5, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 255),
                1
            )

    return frame

# =========================
# SAVE DATA
# =========================
def save_dataset():

    with open(DATA_FILE, "wb") as f:
        pickle.dump(
            (all_corners, all_ids, all_filenames),
            f
        )

# =========================
# PER-IMAGE ERROR
# =========================
def compute_per_image_error(corners_list, ids_list, img_size):
    """
    Jalankan calibrasi sementara lalu hitung
    reprojection error untuk setiap gambar.
    Return: (overall_rms, [error_per_image])
    """

    if len(corners_list) < 3:
        return None, None

    dictionary = cv2.aruco.getPredefinedDictionary(marker_dict)

    board = cv2.aruco.CharucoBoard(
        (corners_x + 1, corners_y + 1),
        square_length / 1000.0,
        marker_length / 1000.0,
        dictionary
    )

    try:
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = (
            cv2.aruco.calibrateCameraCharuco(
                corners_list,
                ids_list,
                board,
                img_size,
                None,
                None
            )
        )
    except Exception as e:
        print(f"\n  [WARN] Calibrasi sementara gagal: {e}")
        return None, None

    # Hitung error per image
    per_image_errors = []

    for i in range(len(corners_list)):

        try:
            obj_points, img_points = board.matchImagePoints(
                corners_list[i],
                ids_list[i]
            )
        except Exception:
            per_image_errors.append(None)
            continue

        if obj_points is None or len(obj_points) == 0:
            per_image_errors.append(None)
            continue

        projected, _ = cv2.projectPoints(
            obj_points,
            rvecs[i],
            tvecs[i],
            camera_matrix,
            dist_coeffs
        )

        error = cv2.norm(img_points, projected, cv2.NORM_L2)
        error /= np.sqrt(len(img_points))

        per_image_errors.append(error)

    return ret, per_image_errors

# =========================
# PRINT ERROR TABLE
# =========================
def print_error_table(overall_err, per_errors, latest_idx=None):

    print(f"\n  {'='*50}")
    print(f"  {'IDX':<8} {'FILENAME':<25} {'ERROR':>8}  {'STATUS'}")
    print(f"  {'-'*50}")

    for idx, e in enumerate(per_errors):

        fname = (
            os.path.basename(all_filenames[idx])
            if idx < len(all_filenames)
            else f"img_{idx:03d}"
        )

        tag = ""
        if idx == latest_idx:
            tag += " <-- latest"

        if e is None:
            status = "N/A"
            print(f"  {idx:<8} {fname:<25} {'---':>8}  {status}{tag}")
        else:
            if e > ERROR_WARN_THRESHOLD:
                status = "⚠️  HIGH"
            elif e > 0.5:
                status = "OK"
            else:
                status = "✓  GOOD"

            print(f"  {idx:<8} {fname:<25} {e:>8.4f}  {status}{tag}")

    print(f"  {'-'*50}")
    print(f"  Overall RMS Error : {overall_err:.4f}")
    print(f"  {'='*50}\n")

# =========================
# DELETE LAST CAPTURE
# =========================
def delete_last_capture():

    global img_id

    if len(all_corners) == 0:
        print("\n[WARN] Tidak ada capture untuk dihapus")
        return

    removed_file = (
        all_filenames[-1]
        if all_filenames
        else "unknown"
    )

    all_corners.pop()
    all_ids.pop()

    if all_filenames:
        all_filenames.pop()

    # Hapus file gambar jika ada
    if os.path.exists(removed_file):
        os.remove(removed_file)
        print(f"\n[DELETED] File: {removed_file}")
    else:
        print(f"\n[DELETED] Data (file tidak ditemukan: {removed_file})")

    img_id = len(all_corners)

    save_dataset()

    print(f"  Sisa captures : {len(all_corners)}")

    # Tampilkan ulang error table jika masih ada data
    if len(all_corners) >= 3 and img_size is not None:

        overall_err, per_errors = compute_per_image_error(
            all_corners,
            all_ids,
            img_size
        )

        if per_errors is not None:
            print("  [UPDATE] Error setelah delete:")
            print_error_table(overall_err, per_errors)

# =========================
# CALIBRATION
# =========================
def run_calibration():

    global img_size

    if len(all_corners) < 10:
        print("\n[ERROR] Need at least 10 captures")
        return

    dictionary = cv2.aruco.getPredefinedDictionary(marker_dict)

    board = cv2.aruco.CharucoBoard(
        (corners_x + 1, corners_y + 1),
        square_length / 1000.0,
        marker_length / 1000.0,
        dictionary
    )

    print("\n[INFO] Calibrating...")

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = (
        cv2.aruco.calibrateCameraCharuco(
            all_corners,
            all_ids,
            board,
            img_size,
            None,
            None
        )
    )

    # Hitung per-image error final
    per_errors = []

    for i in range(len(all_corners)):

        try:
            obj_points, img_points = board.matchImagePoints(
                all_corners[i],
                all_ids[i]
            )
        except Exception:
            per_errors.append(None)
            continue

        if obj_points is None or len(obj_points) == 0:
            per_errors.append(None)
            continue

        projected, _ = cv2.projectPoints(
            obj_points,
            rvecs[i],
            tvecs[i],
            camera_matrix,
            dist_coeffs
        )

        error = cv2.norm(img_points, projected, cv2.NORM_L2)
        error /= np.sqrt(len(img_points))

        per_errors.append(error)

    print("\n========== CALIBRATION RESULT ==========")
    print(f"Images Used : {len(all_corners)}")
    print_error_table(ret, per_errors)

    print("\nCamera Matrix:")
    print(camera_matrix)

    print("\nDistortion Coefficients:")
    print(dist_coeffs)

    fs = cv2.FileStorage(
        "camera_calibration.yaml",
        cv2.FILE_STORAGE_WRITE
    )

    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeffs", dist_coeffs)

    fs.release()

    print("\n[SAVED] camera_calibration.yaml")

# =========================
# MAIN
# =========================
def main():

    global img_id
    global img_size
    global last_capture_time

    try:
        cap = open_camera()
    except RuntimeError as exc:
        print(f"Camera not opened: {exc}")
        return

    print("\n========== CHARUCO CALIBRATION ==========")
    print("C = capture")
    print("D = delete last capture")
    print("K = calibrate & save")
    print("Q = quit")
    print(f"Max corners         : {MAX_CORNERS}")
    print(f"Min corners         : {MIN_CORNERS}")
    print(f"Error warn threshold: {ERROR_WARN_THRESHOLD}")
    print("==========================================\n")

    while True:

        ret, frame = cap.read()

        if not ret:
            continue

        if img_size is None:

            img_size = (
                frame.shape[1],
                frame.shape[0]
            )

        corners, ids = detect_charuco(frame)

        corner_count = 0

        if ids is not None:
            corner_count = len(ids)

        print(
            f"\rCorners: {corner_count}/{MAX_CORNERS} | "
            f"Saved: {len(all_corners)}",
            end=""
        )

        frame = draw(frame, corners, ids)

        color = (
            (0, 255, 0)
            if corner_count >= MIN_CORNERS
            else (0, 255, 255)
        )

        cv2.putText(
            frame,
            f"Corners: {corner_count}/{MAX_CORNERS}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            color,
            2
        )

        cv2.putText(
            frame,
            f"Saved: {len(all_corners)}",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 0),
            2
        )

        cv2.putText(
            frame,
            "C:capture | D:delete | K:calibrate | Q:quit",
            (20, frame.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1
        )

        cv2.imshow(
            "Charuco Calibration",
            frame
        )

        key = cv2.waitKey(1) & 0xFF

        # =========================
        # CAPTURE
        # =========================
        if key == ord('c'):

            now = time.time()

            if now - last_capture_time < CAPTURE_DELAY:
                print("\n[WAIT] Cooldown aktif, tunggu sebentar...")
                continue

            if (
                corners is not None
                and corner_count >= MIN_CORNERS
            ):

                filename = f"{save_dir}/img_{img_id:03d}.png"

                cv2.imwrite(filename, frame)

                all_corners.append(corners)
                all_ids.append(ids)
                all_filenames.append(filename)

                save_dataset()

                latest_idx = len(all_corners) - 1

                print(
                    f"\n[SAVED] {filename} "
                    f"| corners={corner_count}"
                )

                # Hitung dan tampilkan error per image
                if len(all_corners) >= 3:

                    overall_err, per_errors = compute_per_image_error(
                        all_corners,
                        all_ids,
                        img_size
                    )

                    if per_errors is not None:
                        print_error_table(
                            overall_err,
                            per_errors,
                            latest_idx=latest_idx
                        )

                        # Peringatan jika capture terbaru error tinggi
                        latest_err = per_errors[latest_idx]
                        if (
                            latest_err is not None
                            and latest_err > ERROR_WARN_THRESHOLD
                        ):
                            print(
                                f"  [HINT] Error capture terbaru tinggi "
                                f"({latest_err:.4f}). "
                                f"Tekan D untuk menghapusnya.\n"
                            )

                else:
                    print(
                        "  [INFO] Butuh minimal 3 captures "
                        "untuk menghitung error\n"
                    )

                img_id += 1
                last_capture_time = now

            else:

                print(
                    f"\n[SKIP] Hanya {corner_count}/{MAX_CORNERS} "
                    f"corners terdeteksi (min: {MIN_CORNERS})\n"
                )

        # =========================
        # DELETE LAST
        # =========================
        elif key == ord('d'):

            delete_last_capture()

        # =========================
        # CALIBRATION
        # =========================
        elif key == ord('k'):

            run_calibration()

        # =========================
        # QUIT
        # =========================
        elif key == ord('q'):

            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()