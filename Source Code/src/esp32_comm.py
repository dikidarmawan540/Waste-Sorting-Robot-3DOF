"""
esp32_comm.py
=============

Komunikasi serial dua arah Python <-> ESP32.

Mode 1 - dipakai main.py:
    YOLO -> koordinat robot -> SORT/G1 ke ESP32, lalu respons ESP32 dibaca
    ringan di loop utama main.py.

Mode 2 - dijalankan langsung:
    python src/esp32_comm.py
    Console manual untuk kirim command ke ESP32 tanpa kamera/YOLO.
    Console ini bersifat dua arah: command dikirim ke ESP32, respons dibaca,
    dan history command/respons disimpan di sisi Python.

Jangan membuka Arduino Serial Monitor bersamaan dengan main.py/esp32_comm.py,
karena COM port biasanya hanya bisa dipakai satu aplikasi.
"""

from __future__ import annotations

import time
import threading
import os
import csv
import re
import math
import zipfile
from datetime import datetime
from xml.sax.saxutils import escape
from xml.etree import ElementTree as ET
from collections import deque
from dataclasses import dataclass, field

import config

try:
    import serial
    from serial.tools import list_ports
except Exception:
    serial = None
    list_ports = None


LOCAL_HELP_TEXT = """
================ ESP32 COMMAND CONSOLE ================
Mode ini untuk mengirim command langsung ke ESP32 dari VS Code,
tanpa kamera, tanpa YOLO, dan tanpa main.py.

Cara jalankan:
  python src/esp32_comm.py

Command lokal console:
  HELP / ?          Tampilkan bantuan lokal + kirim HELP ke firmware ESP32
  ESPHELP           Kirim HELP firmware ESP32 saja
  LOCALHELP         Tampilkan bantuan lokal saja
  READ              Baca output serial selama 2 detik
  READ 10           Baca output serial selama 10 detik
  PORTS             Tampilkan daftar COM port yang terbaca Windows
  STATUS            Status koneksi Python ke ESP32
  POS / M114        Cek posisi/status robot dari firmware
  INFO              Ambil info lengkap: M114, M119, INA, YOLO
  HISTORY           Tampilkan 20 histori command terakhir
  HISTORY 50        Tampilkan 50 histori command terakhir
  LAST              Tampilkan command terakhir + responsnya
  CLEARHISTORY      Hapus histori lokal Python
  RECONNECT         Tutup dan buka ulang koneksi serial
  EXIT / QUIT       Keluar dari console

Command ESP32 yang umum dipakai:
  HELP              Tampilkan HELP firmware ESP32
  HOME / G28        Homing robot
  M114              Cek posisi/status robot
  M119              Cek limit switch
  INA               Cek arus INA219
  YOLO              Cek apakah koordinat YOLO/SORT terakhir sudah diterima
  PUMP ON           Nyalakan pump
  PUMP OFF          Matikan pump

Gerakan manual:
  G0 X0 Y180 Z150            Gerak cepat ke koordinat aman
  G1 X150 Y230 Z40           Gerak linear/presisi ke koordinat
  MOVEJ A0 B90 C0            Gerak joint/manual jika firmware mendukung

Command pick and place manual:
  SORT X0 Y180 Z40 B0
  SORT X126.10 Y168.30 Z40 B0

Perilaku dua arah:
  - Setelah command dikirim, respons ESP32 dicetak sebagai [ESP32] << ...
  - Untuk G0/G1/G28/MOVEJ/SORT, console otomatis kirim M114 setelah command selesai,
    supaya posisi/status terbaru terlihat.
  - Semua command dan respons disimpan di HISTORY lokal Python.

Urutan aman tes manual:
  PORTS
  HOME
  M114
  YOLO
  G1 X150 Y230 Z40
  HISTORY

Catatan:
  - Jika firmware mewajibkan homing, SORT/G0/G1/MOVEJ tidak jalan sebelum G28.
  - Kalau YOLO status RECEIVED=NO, berarti ESP32 belum menerima command SORT.
  - Kalau main.py sedang jalan, esp32_comm.py tidak bisa memakai COM yang sama.
=======================================================
""".strip()


@dataclass
class CommandRecord:
    timestamp: float
    command: str
    responses: list[str] = field(default_factory=list)
    followups: list[tuple[str, list[str]]] = field(default_factory=list)

    def time_text(self) -> str:
        return time.strftime("%H:%M:%S", time.localtime(self.timestamp))


class ESP32Comm:
    def __init__(self, auto_connect: bool = True, history_size: int = 200):
        self._ser = None
        self._lock = threading.Lock()
        self._last_send = 0.0
        self._connected = False
        self._last_command = ""
        self._serial_enable = bool(getattr(config, "SERIAL_ENABLE", False))
        self.history: deque[CommandRecord] = deque(maxlen=history_size)

        # Kolom log pengujian (revisi):
        # Percobaan | Kelas | Touch Current | Pick X | Pick Y | Actual X | Actual Y |
        # Latency YOLO (ms) | Latency Decision (ms) | Latency Serial (ms) | Latency Total (ms) |
        # Cycle Time (ms) | Status
        # Actual X/Y sengaja dikosongkan untuk pengukuran manual (opsional, tidak dipakai
        # untuk perhitungan apa pun lagi). Kolom Error dan baris ringkasan MAE dihapus
        # dari file pengambilan data ini. Status Success/Fail diambil dari indikator
        # sensor arus INA219 (deteksi objek tertahan di gripper) yang dikirim firmware.
        # Cycle Time (ms) = waktu siklus penuh pick->place dari firmware ESP32
        # (ended_ms - started_ms pada satu siklus SORT), independen dari Latency Total
        # (Latency Total hanya mengukur waktu sisi komputasi/komunikasi Python<->ESP32
        # sebelum gerakan robot dimulai).
        self._pick_place_log_fields = [
            "Percobaan",
            "Kelas",
            "Touch Current",
            "Pick X",
            "Pick Y",
            "Actual X",
            "Actual Y",
            "Latency YOLO (ms)",
            "Latency Decision (ms)",
            "Latency Serial (ms)",
            "Latency Total (ms)",
            "Cycle Time (ms)",
            "Status",
        ]
        self._pick_place_log_col_count = len(self._pick_place_log_fields)
        self._pending_sort_sent_perf: float | None = None
        self._pending_sort_yolo_latency_ms: str = ""
        self._pending_sort_decision_latency_ms: str = ""
        self._pending_sort_serial_latency_ms: str = ""
        self._pending_sort_class_name: str = ""
        self._pending_sort_object_id: int | None = None
        # Atribut lama tetap dipertahankan untuk kompatibilitas, tetapi tidak dipakai
        # sebagai sumber Actual X/Y pada mode pengukuran manual.
        self._pending_sort_actual_xy: tuple[float, float] | None = None
        self._tracking_actual_by_id: dict[int, tuple[float, float, float]] = {}
        self._pick_place_log_enable = bool(getattr(config, "PICK_PLACE_LOG_ENABLE", True))
        self._pick_place_log_dir = str(getattr(config, "PICK_PLACE_LOG_DIR", os.path.join(os.getcwd(), "logs")))
        self._pick_place_csv_base_name = str(getattr(config, "PICK_PLACE_LOG_CSV_BASE_NAME", "PERCOBAAN"))
        self._pick_place_csv_single_file = str(getattr(config, "PICK_PLACE_LOG_CSV_SINGLE_FILE", "hasil_pick_place.csv"))
        self._pick_place_xlsx_single_file = str(getattr(config, "PICK_PLACE_LOG_XLSX_SINGLE_FILE", "hasil_pick_place.xlsx"))
        self._pick_place_rows_per_file = max(1, int(getattr(config, "PICK_PLACE_LOG_ROWS_PER_FILE", 5)))
        self._pick_place_log_write_title_row = bool(getattr(config, "PICK_PLACE_LOG_WRITE_TITLE_ROW", True))
        self._pick_place_log_write_xlsx = bool(getattr(config, "PICK_PLACE_LOG_WRITE_XLSX", True))
        self._csv_formula_separator = str(getattr(config, "PICK_PLACE_CSV_FORMULA_SEPARATOR", ";") or ";")
        if self._pick_place_log_enable:
            print(f"[PICK_PLACE_LOG] Folder output: {self._pick_place_log_dir}")
            print(f"[PICK_PLACE_LOG] CSV : {self._pick_place_csv_single_file}")
            if self._pick_place_log_write_xlsx:
                print(f"[PICK_PLACE_LOG] XLSX: {self._pick_place_xlsx_single_file}")
            print(
                f"[PICK_PLACE_LOG] Kolom: {', '.join(self._pick_place_log_fields)}. "
                f"Blok baru setiap {self._pick_place_rows_per_file} percobaan."
            )

        if auto_connect:
            if self._serial_enable and serial is not None:
                self._connect_once()
            else:
                reason = "SERIAL_ENABLE=False" if not self._serial_enable else "pyserial belum terpasang"
                print(f"[ESP32] STUB mode ({reason}). Port target: {config.SERIAL_PORT} @ {config.SERIAL_BAUDRATE}")

    @property
    def status(self) -> str:
        if self._connected:
            return "SERIAL OK"
        return "STUB" if not self._serial_enable or serial is None else "DISCONNECTED"

    @property
    def is_connected(self) -> bool:
        return bool(self._connected and self._ser is not None)

    @property
    def last_command(self) -> str:
        return self._last_command

    @staticmethod
    def list_serial_ports() -> list[str]:
        if list_ports is None:
            return []
        ports = []
        try:
            for p in list_ports.comports():
                ports.append(f"{p.device} | {p.description} | HWID={p.hwid}")
        except Exception:
            pass
        return ports

    def print_serial_ports(self) -> None:
        ports = self.list_serial_ports()
        print("[PORTS] COM port yang terbaca:")
        if not ports:
            print("  (tidak ada / pyserial list_ports tidak tersedia)")
            return
        for item in ports:
            print(f"  - {item}")

    def _connect_once(self) -> None:
        if not self._serial_enable:
            print("[ESP32] SERIAL_ENABLE=False. Tidak membuka serial.")
            return
        if serial is None:
            print("[ESP32] pyserial belum terpasang. Install: pip install pyserial")
            return

        print(f"[ESP32] Membuka serial {config.SERIAL_PORT} @ {config.SERIAL_BAUDRATE}...")
        self.print_serial_ports()

        try:
            self._ser = serial.Serial(
                port=config.SERIAL_PORT,
                baudrate=config.SERIAL_BAUDRATE,
                timeout=float(getattr(config, "SERIAL_TIMEOUT", 1.0)),
                write_timeout=2.0,
                rtscts=False,
                dsrdtr=False,
            )
            # Hindari kondisi RTS/DTR menggantung setelah port dibuka.
            # Sebagian board ESP32 tetap reset saat port dibuka; karena itu tetap tunggu boot.
            try:
                self._ser.setDTR(False)
                self._ser.setRTS(False)
            except Exception:
                pass

            time.sleep(2.5)  # ESP32 sering reset saat serial dibuka
            try:
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
            except Exception:
                pass
            self._connected = True
            print(f"[ESP32] TERHUBUNG di {config.SERIAL_PORT} @ {config.SERIAL_BAUDRATE}")
            print("[ESP32] Ketik HOME atau G28 untuk homing. Ketik READ 3 untuk baca output boot/firmware.")
        except Exception as e:
            self._connected = False
            self._ser = None
            print(f"[ESP32] GAGAL konek serial: {e}")
            print("[ESP32] Cek: COM benar, kabel data bukan kabel charge, driver CH340/CP210x, dan tidak ada aplikasi lain memakai COM.")

    def reconnect(self) -> bool:
        self.close()
        time.sleep(float(getattr(config, "SERIAL_RECONNECT_DELAY", 1.0)))
        self._connect_once()
        return self.is_connected

    def read_available(self, duration: float = 0.25, echo: bool = True) -> list[str]:
        """Baca output serial ESP32 yang tersedia selama durasi tertentu."""
        lines: list[str] = []
        if not self.is_connected:
            return lines

        end_time = time.perf_counter() + max(0.0, float(duration))
        while time.perf_counter() < end_time:
            try:
                waiting = int(self._ser.in_waiting) if self._ser is not None else 0
            except Exception:
                waiting = 0

            if waiting > 0:
                try:
                    raw = self._ser.readline()
                    text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                    if text:
                        lines.append(text)
                        self._capture_sort_ack(text)
                        self._capture_pick_place_csv(text)
                        if echo:
                            print(f"[ESP32] << {text}")
                except Exception as e:
                    print(f"[ESP32] Gagal baca serial: {e}")
                    self._connected = False
                    break
            else:
                time.sleep(0.02)
        return lines

    def send_line(self, line: str, force: bool = False) -> bool:
        """Kirim 1 baris command ke ESP32, otomatis tambah newline."""
        now = time.perf_counter()
        if not force and now - self._last_send < float(getattr(config, "SEND_INTERVAL", 0.1)):
            return False

        clean = line.strip()
        if not clean:
            return False

        self._last_command = clean
        msg = clean + "\n"

        if self.is_connected:
            with self._lock:
                try:
                    payload = msg.encode("ascii", errors="ignore")
                    n = self._ser.write(payload)
                    self._ser.flush()
                    self._last_send = now
                    print(f"[ESP32] >> {clean}  ({n} bytes)")
                    return True
                except Exception as e:
                    print(f"[ESP32] Gagal kirim: {e}")
                    self._connected = False
                    try:
                        self._ser.close()
                    except Exception:
                        pass
                    self._ser = None
                    return False

        print(f"[ESP32 STUB] >> {clean}")
        self._last_send = now
        return True

    def send_command(self, line: str, wait: float | None = None, force: bool = True, echo: bool = True) -> list[str]:
        """Kirim command lalu baca respons ESP32. Respons tidak otomatis masuk history."""
        ok = self.send_line(line, force=force)
        if not ok:
            return []
        if wait is None:
            wait = self._suggest_wait_seconds(line)
        return self.read_available(duration=wait, echo=echo)

    def send_command_tracked(
        self,
        line: str,
        wait: float | None = None,
        force: bool = True,
        echo: bool = True,
        auto_m114: bool = True,
    ) -> CommandRecord | None:
        """Kirim command, baca respons, simpan history, dan opsional M114 setelah motion."""
        clean = line.strip()
        if not clean:
            return None

        record = CommandRecord(timestamp=time.time(), command=clean)
        responses = self.send_command(clean, wait=wait, force=force, echo=echo)
        record.responses.extend(responses)

        if auto_m114 and self._should_auto_m114(clean):
            # jeda pendek agar firmware sempat menyelesaikan gerakan/log terakhir
            time.sleep(0.15)
            print("[AUTO] Motion command selesai/timeout baca. Meminta posisi terbaru: M114")
            pos_lines = self.send_command("M114", wait=1.5, force=True, echo=echo)
            record.followups.append(("M114", pos_lines))

        self.history.append(record)
        return record

    def _should_auto_m114(self, line: str) -> bool:
        cmd = line.strip().upper()
        return cmd.startswith(("G0", "G1", "G28", "MOVEJ", "SORT"))

    def _suggest_wait_seconds(self, line: str) -> float:
        cmd = line.strip().upper()
        if cmd.startswith("G28"):
            return 15.0
        if cmd.startswith("SORT"):
            return 25.0
        if cmd.startswith("G0") or cmd.startswith("G1") or cmd.startswith("MOVEJ"):
            return 8.0
        if cmd.startswith("HELP"):
            return 2.0
        if cmd.startswith("YOLO") or cmd.startswith("M114") or cmd.startswith("M119") or cmd.startswith("INA"):
            return 1.5
        return 1.0

    def home(self) -> bool:
        return self.send_line("G28", force=True)

    def pump(self, on: bool) -> bool:
        return self.send_line("PUMP ON" if on else "PUMP OFF", force=True)

    def move_g0(self, x_mm: float | None = None, y_mm: float | None = None, z_mm: float | None = None) -> bool:
        return self._send_move("G0", x_mm, y_mm, z_mm)

    def move_g1(self, x_mm: float | None = None, y_mm: float | None = None, z_mm: float | None = None) -> bool:
        return self._send_move("G1", x_mm, y_mm, z_mm)

    def _send_move(self, code: str, x_mm=None, y_mm=None, z_mm=None) -> bool:
        parts = [code]
        if x_mm is not None:
            parts.append(f"X{float(x_mm):.2f}")
        if y_mm is not None:
            parts.append(f"Y{float(y_mm):.2f}")
        if z_mm is not None:
            parts.append(f"Z{float(z_mm):.2f}")
        return self.send_line(" ".join(parts))

    def update_tracking_actual(self, object_id: int | None, x_mm: float | None, y_mm: float | None) -> None:
        """Kompatibilitas lama untuk cache tracking.

        Pada konfigurasi revisi, PICK_PLACE_MAE_ACTUAL_FROM_TRACKING=False sehingga
        data ini tidak pernah dipakai untuk mengisi Actual X/Y pada CSV atau XLSX.
        """
        if not bool(getattr(config, "PICK_PLACE_MAE_ACTUAL_FROM_TRACKING", False)):
            return
        if object_id is None:
            return
        try:
            track_id = int(object_id)
            if track_id <= 0:
                return
            x = float(x_mm)
            y = float(y_mm)
        except Exception:
            return
        now = time.perf_counter()
        self._tracking_actual_by_id[track_id] = (x, y, now)

    def _latest_tracking_actual_xy(self, object_id: int | None, fallback_x: float | None = None, fallback_y: float | None = None) -> tuple[float, float] | None:
        """Ambil koordinat tracking ID terbaru, tanpa median beberapa frame."""
        if object_id is None:
            return None
        try:
            track_id = int(object_id)
        except Exception:
            return None
        max_age_s = float(getattr(config, "PICK_PLACE_TRACKING_ACTUAL_MAX_AGE_SEC", 10.0))
        now = time.perf_counter()
        ref = self._tracking_actual_by_id.get(track_id)
        if ref is not None:
            xh, yh, ts = ref
            if max_age_s <= 0 or (now - float(ts)) <= max_age_s:
                return float(xh), float(yh)
        if fallback_x is not None and fallback_y is not None:
            try:
                return float(fallback_x), float(fallback_y)
            except Exception:
                return None
        return None

    def set_pending_sort_actual(self, object_id: int | None, x_mm: float | None, y_mm: float | None) -> None:
        """Kompatibilitas lama. Mode manual tidak mengunci Actual X/Y dari tracking."""
        if not bool(getattr(config, "PICK_PLACE_MAE_ACTUAL_FROM_TRACKING", False)):
            self._pending_sort_actual_xy = None
            return
        self.update_tracking_actual(object_id, x_mm, y_mm)
        if bool(getattr(config, "PICK_PLACE_ACTUAL_FREEZE_ON_SORT", True)):
            try:
                self._pending_sort_actual_xy = (float(x_mm), float(y_mm))
            except Exception:
                self._pending_sort_actual_xy = self._latest_tracking_actual_xy(object_id, x_mm, y_mm)
        else:
            self._pending_sort_actual_xy = None

    def send_sort(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float | None = None,
        bin_index: int | None = None,
        object_id: int | None = None,
        yolo_latency_ms: float | None = None,
        decision_latency_ms: float | None = None,
        class_name: str | None = None,
    ) -> bool:
        """Kirim perintah SORT ke firmware ESP32, opsional membawa ID objek dari tracker Python."""
        if z_mm is None:
            z_mm = float(getattr(config, "ROBOT_PICK_Z_MM", 40.0))
        if bin_index is None:
            bin_index = int(getattr(config, "ROBOT_BIN_INDEX", 0))
        line = f"SORT X{float(x_mm):.2f} Y{float(y_mm):.2f} Z{float(z_mm):.2f} B{int(bin_index)}"
        if object_id is not None:
            line += f" ID{int(object_id)}"
        ok = self.send_line(line)
        if ok:
            # Jika send_sort dipanggil langsung tanpa set_pending_sort_actual(),
            # tetap kunci actual langsung dari koordinat SORT saat ini.
            if bool(getattr(config, "PICK_PLACE_ACTUAL_FREEZE_ON_SORT", True)) and self._pending_sort_actual_xy is None:
                try:
                    self._pending_sort_actual_xy = (float(x_mm), float(y_mm))
                except Exception:
                    self._pending_sort_actual_xy = self._latest_tracking_actual_xy(object_id, x_mm, y_mm)
            # Dipakai untuk latency komunikasi: SORT terkirim -> ACK koordinat dari ESP32.
            self._pending_sort_sent_perf = time.perf_counter()
            self._pending_sort_yolo_latency_ms = self._format_ms(yolo_latency_ms)
            self._pending_sort_decision_latency_ms = self._format_ms(decision_latency_ms)
            self._pending_sort_serial_latency_ms = ""
            self._pending_sort_class_name = str(class_name).strip() if class_name else ""
            try:
                self._pending_sort_object_id = int(object_id) if object_id is not None else None
            except Exception:
                self._pending_sort_object_id = None
        return ok

    # Nama lama dipertahankan supaya main.py lama tetap kompatibel.
    def send_coordinate(self, x_mm: float, y_mm: float, z_mm: float = 0.0, elbow_up: bool = True) -> bool:
        return self.move_g1(x_mm=x_mm, y_mm=y_mm, z_mm=z_mm)

    def send_coordinate_px(self, cx_px: int, cy_px: int, center_x: int, center_y: int) -> None:
        dx = cx_px - center_x
        dy = cy_px - center_y
        self.send_line(f"; DEBUG_PIXEL dx={dx} dy={dy}")

    def print_history(self, limit: int = 20) -> None:
        records = list(self.history)[-max(1, int(limit)):]
        if not records:
            print("[HISTORY] Belum ada command tercatat.")
            return
        print(f"\n[HISTORY] Menampilkan {len(records)} command terakhir")
        for idx, rec in enumerate(records, start=max(1, len(self.history) - len(records) + 1)):
            print(f"\n#{idx} [{rec.time_text()}] >> {rec.command}")
            if rec.responses:
                for line in rec.responses:
                    print(f"   << {line}")
            else:
                print("   << (tidak ada respons terbaca)")
            for fcmd, lines in rec.followups:
                print(f"   [AUTO] >> {fcmd}")
                if lines:
                    for line in lines:
                        print(f"          << {line}")
                else:
                    print("          << (tidak ada respons terbaca)")

    def print_last(self) -> None:
        if not self.history:
            print("[LAST] Belum ada command tercatat.")
            return
        last = self.history[-1]
        print(f"\n[LAST] [{last.time_text()}] >> {last.command}")
        for line in last.responses:
            print(f"       << {line}")
        for fcmd, lines in last.followups:
            print(f"       [AUTO] >> {fcmd}")
            for line in lines:
                print(f"              << {line}")

    def clear_history(self) -> None:
        self.history.clear()
        print("[HISTORY] Histori lokal Python dibersihkan.")


    def _capture_pick_place_csv(self, text: str) -> None:
        """Tangkap satu hasil pick-place dan simpan ke CSV serta XLSX.

        Kolom Actual X dan Actual Y sengaja dibiarkan kosong karena diukur manual.
        Kolom Error X, Error Y, dan baris MAE ditulis sebagai rumus.
        """
        if not self._pick_place_log_enable:
            return

        prefix = "DATA:PICK_PLACE_CSV,"
        if not text.startswith(prefix):
            return

        payload = text[len(prefix):]
        try:
            parsed = next(csv.reader([payload]))
        except Exception as e:
            print(f"[PICK_PLACE_LOG] Gagal parse CSV firmware: {e} | line={text}")
            return

        compact = self._normalize_pick_place_payload(parsed)
        if compact is None:
            print(f"[PICK_PLACE_LOG] Format DATA:PICK_PLACE_CSV tidak dikenali: {parsed}")
            return

        row = self._build_pick_place_log_row(compact)
        try:
            path, batch_no, local_trial_id = self._append_pick_place_csv(row)
            print(
                f"[PICK_PLACE_LOG] Tersimpan: {os.path.basename(path)} | "
                f"blok={batch_no} percobaan={local_trial_id} | Actual X/Y menunggu input manual"
            )
        except Exception as e:
            print(f"[PICK_PLACE_LOG] Gagal simpan log pick-place: {e}")

    def _build_pick_place_log_row(self, compact: list[str]) -> list[str]:
        """Bangun baris dasar sebelum nomor baris ditambahkan (di _append_pick_place_csv).

        Data otomatis: Percobaan, Kelas, Touch Current, Pick X, Pick Y, Latency
        (YOLO/Decision/Serial/Total), dan Status (Success/Fail berdasarkan indikator
        sensor arus INA219 saat gripper menyentuh/menahan objek). Actual X/Y
        dikosongkan untuk pengukuran manual opsional.
        """
        values = [str(v).strip() for v in compact]
        while len(values) < 12:
            values.append("")

        # Format ringkas 12 kolom firmware:
        # trial_id,started_ms,ended_ms,duration_ms,reason,pick_x,pick_y,bin_name,
        # ina_detected,touch_z,touch_current_A,status
        trial_id = values[0]
        started_ms, ended_ms, duration_ms = values[1], values[2], values[3]
        pick_x, pick_y = values[5], values[6]
        ina_detected = values[8]
        touch_current_A = values[10]
        status_raw = values[11]

        yolo_ms, decision_ms, serial_ms, total_ms = self._consume_latency_columns()
        class_name = self._consume_pending_class_name()
        status_text = self._resolve_pick_status(status_raw, ina_detected)
        cycle_time_ms = self._resolve_cycle_time_ms(started_ms, ended_ms, duration_ms)

        return [
            trial_id,
            class_name,
            touch_current_A,
            pick_x,
            pick_y,
            "",
            "",
            yolo_ms,
            decision_ms,
            serial_ms,
            total_ms,
            cycle_time_ms,
            status_text,
        ]

    def _resolve_cycle_time_ms(self, started_ms: str, ended_ms: str, duration_ms: str) -> str:
        """Cycle Time (ms) = waktu siklus penuh pick->place satu percobaan di firmware.

        Prioritas: pakai `duration_ms` langsung dari firmware. Kalau kosong/tidak
        valid, hitung dari `ended_ms - started_ms` sebagai fallback.
        """
        d = self._to_float_or_none(duration_ms)
        if d is not None:
            return f"{d:.1f}"
        s = self._to_float_or_none(started_ms)
        e = self._to_float_or_none(ended_ms)
        if s is not None and e is not None:
            return f"{max(0.0, e - s):.1f}"
        return ""

    def _consume_pending_class_name(self) -> str:
        name = self._pending_sort_class_name
        self._pending_sort_class_name = ""
        return name

    @staticmethod
    def _resolve_pick_status(status_raw: str, ina_detected_raw: str) -> str:
        """Tentukan Success/Fail dari status firmware atau indikator INA/touch.

        Prioritas utama: field `status` yang dikirim firmware (mis. "SUCCESS",
        "FAIL", "PICK_RESULT=FAIL"). Jika kosong/tidak dikenali, gunakan
        `ina_detected` (1/true/yes = objek terdeteksi tertahan saat touch = Success).
        """
        text = str(status_raw or "").strip().upper()
        if text:
            if "FAIL" in text:
                return "Fail"
            if "SUCCESS" in text or "OK" in text:
                return "Success"

        ina_text = str(ina_detected_raw or "").strip().upper()
        if ina_text in {"1", "TRUE", "YES", "Y", "DETECTED"}:
            return "Success"
        if ina_text in {"0", "FALSE", "NO", "N", "NOT_DETECTED"}:
            return "Fail"

        return "Fail" if not text else text.title()

    @staticmethod
    def _format_ms(value) -> str:
        try:
            if value is None:
                return ""
            return f"{float(value):.1f}"
        except Exception:
            return ""

    def _capture_sort_ack(self, text: str) -> None:
        """Catat latency serial SORT -> ACK dari ESP32.

        ACK tercepat firmware untuk data SORT adalah EVENT:YOLO_COORD_RECEIVED.
        Jika event itu terlewat, EVENT:YOLO_COORD_STORED atau
        EVENT:SORT_MOTION_ACCEPTED_PUMP_WAIT_XY dipakai sebagai fallback.
        """
        if self._pending_sort_sent_perf is None or self._pending_sort_serial_latency_ms:
            return
        upper = str(text).upper()
        ack_markers = (
            "EVENT:YOLO_COORD_RECEIVED",
            "EVENT:YOLO_COORD_STORED",
            "EVENT:SORT_MOTION_ACCEPTED",
        )
        if any(marker in upper for marker in ack_markers):
            serial_latency = max(0.0, (time.perf_counter() - self._pending_sort_sent_perf) * 1000.0)
            self._pending_sort_serial_latency_ms = f"{serial_latency:.1f}"

    def _consume_latency_columns(self) -> tuple[str, str, str, str]:
        """Ambil latency komputasi/komunikasi untuk satu baris log.

        yolo_latency_ms     : waktu inference YOLO.
        decision_latency_ms : waktu dari hasil YOLO sampai SORT dikirim.
        serial_latency_ms   : waktu dari SORT dikirim sampai ACK koordinat diterima.
        total_latency_ms    : jumlah dari tiga latency di atas.
        """
        yolo = self._pending_sort_yolo_latency_ms
        decision = self._pending_sort_decision_latency_ms
        serial_ms = self._pending_sort_serial_latency_ms

        numbers = [self._to_float_or_none(yolo), self._to_float_or_none(decision), self._to_float_or_none(serial_ms)]
        if all(v is not None for v in numbers):
            total = f"{sum(numbers):.1f}"
        else:
            total = ""

        self._pending_sort_sent_perf = None
        self._pending_sort_yolo_latency_ms = ""
        self._pending_sort_decision_latency_ms = ""
        self._pending_sort_serial_latency_ms = ""
        self._pending_sort_object_id = None
        self._pending_sort_actual_xy = None
        return yolo, decision, serial_ms, total

    @staticmethod
    def _to_float_or_none(value) -> float | None:
        if value is None:
            return None
        try:
            text = str(value).strip()
            if not text or text.lower() in {"none", "nan", "null"}:
                return None
            return float(text)
        except Exception:
            return None

    def _resolve_tracking_actual_xy(self, object_id: int | None) -> tuple[float, float] | None:
        """Ambil actual_x/actual_y otomatis dari tracking ID.

        Data ini di-update oleh main.py setiap frame dari centroid/track ID yang
        masih terlihat, setelah dikonversi ke koordinat robot memakai homography.
        """
        if not bool(getattr(config, "PICK_PLACE_MAE_ENABLE", True)):
            return None
        if not bool(getattr(config, "PICK_PLACE_MAE_ACTUAL_FROM_TRACKING", True)):
            return None
        if object_id is None:
            return None
        try:
            track_id = int(object_id)
        except Exception:
            return None
        # Prioritas utama: nilai yang sudah di-freeze saat SORT dikirim.
        # Ini menjaga CSV tidak berubah akibat flicker atau tracking update saat robot bergerak.
        if bool(getattr(config, "PICK_PLACE_ACTUAL_FREEZE_ON_SORT", True)) and self._pending_sort_actual_xy is not None:
            return float(self._pending_sort_actual_xy[0]), float(self._pending_sort_actual_xy[1])

        ref = self._latest_tracking_actual_xy(track_id)
        if ref is None:
            return None
        return float(ref[0]), float(ref[1])

    def _resolve_mae_reference_xy(self, bin_name: str) -> tuple[float, float] | None:
        """Fallback opsional jika tracking actual tidak tersedia.

        Default sistem sekarang memakai tracking ID otomatis. Referensi manual hanya
        dipakai sebagai cadangan jika kamu sengaja mengisi config lama.
        """
        if not bool(getattr(config, "PICK_PLACE_MAE_ENABLE", True)):
            return None

        refs_by_bin = getattr(config, "PICK_PLACE_MAE_REFERENCE_BY_BIN_NAME", {}) or {}
        if isinstance(refs_by_bin, dict) and bin_name in refs_by_bin:
            ref = refs_by_bin.get(bin_name)
            if isinstance(ref, dict):
                rx = self._to_float_or_none(ref.get("x"))
                ry = self._to_float_or_none(ref.get("y"))
            elif isinstance(ref, (list, tuple)) and len(ref) >= 2:
                rx = self._to_float_or_none(ref[0])
                ry = self._to_float_or_none(ref[1])
            else:
                rx = ry = None
            if rx is not None and ry is not None:
                return rx, ry

        rx = self._to_float_or_none(getattr(config, "PICK_PLACE_MAE_REF_X_MM", None))
        ry = self._to_float_or_none(getattr(config, "PICK_PLACE_MAE_REF_Y_MM", None))
        if rx is None or ry is None:
            return None
        return rx, ry

    def _compute_mae_columns(self, pick_x: str, pick_y: str, bin_name: str, object_id: int | None = None) -> tuple[str, str, str, str, str]:
        """Kompatibilitas lama.

        Perhitungan error tidak lagi dilakukan dari tracking ID. Actual X/Y diinput
        manual dan rumus Error X/Y dibuat langsung di CSV/XLSX saat baris ditulis.
        """
        return "", "", "", "", ""

    def _normalize_pick_place_payload(self, parsed: list[str]) -> list[str] | None:
        '''Ubah payload firmware menjadi format CSV ringkas terbaru.

        Mendukung dua format agar aman saat firmware lama belum di-upload:
        1) Format lama 20 kolom:
           trial_id,object_id,started_ms,ended_ms,duration_ms,status,reason,pick_x,pick_y,
           pick_z_req,bin_index,bin_name,bin_x,bin_y,bin_z,ina_ready,ina_detected,
           offset_ok,touch_z,touch_current_A
        2) Format baru 12 kolom:
           trial_id,started_ms,ended_ms,duration_ms,reason,pick_x,pick_y,bin_name,
           ina_detected,touch_z,touch_current_A,status
        '''
        parsed = [str(v).strip() for v in parsed]

        # Firmware lama: buang kolom yang tidak diperlukan dan pindahkan status ke kanan.
        if len(parsed) >= 20:
            return [
                parsed[0],   # trial_id, nanti di-reset per blok CSV
                parsed[2],   # started_ms
                parsed[3],   # ended_ms
                parsed[4],   # duration_ms
                parsed[6],   # reason
                parsed[7],   # pick_x
                parsed[8],   # pick_y
                parsed[11],  # bin_name
                parsed[16],  # ina_detected
                parsed[18],  # touch_z
                parsed[19],  # touch_current_A
                parsed[5],   # status di paling kanan
            ]

        # Firmware baru: sudah sesuai urutan ringkas.
        if len(parsed) >= 12:
            return parsed[:12]

        return None

    def _pick_place_single_csv_path(self) -> str:
        directory = os.path.abspath(self._pick_place_log_dir)
        os.makedirs(directory, exist_ok=True)
        return os.path.join(directory, self._pick_place_csv_single_file)

    def _pick_place_xlsx_path(self) -> str:
        directory = os.path.abspath(self._pick_place_log_dir)
        os.makedirs(directory, exist_ok=True)
        return os.path.join(directory, self._pick_place_xlsx_single_file)

    @staticmethod
    def _indonesian_ordinal(n: int) -> str:
        words = {
            1: "PERTAMA",
            2: "KEDUA",
            3: "KETIGA",
            4: "KEEMPAT",
            5: "KELIMA",
            6: "KEENAM",
            7: "KETUJUH",
            8: "KEDELAPAN",
            9: "KESEMBILAN",
            10: "KESEPULUH",
            11: "KESEBELAS",
            12: "KEDUA BELAS",
            13: "KETIGA BELAS",
            14: "KEEMPAT BELAS",
            15: "KELIMA BELAS",
            16: "KEENAM BELAS",
            17: "KETUJUH BELAS",
            18: "KEDELAPAN BELAS",
            19: "KESEMBILAN BELAS",
            20: "KEDUA PULUH",
        }
        return words.get(int(n), f"KE-{int(n)}")

    def _pick_place_section_title(self, batch_no: int) -> str:
        return f"{self._pick_place_csv_base_name} {self._indonesian_ordinal(batch_no)}"

    def _scan_single_csv_state(self, path: str) -> tuple[int, int]:
        """Return (batch_no_terakhir, jumlah_data_di_batch_terakhir).

        Dipakai untuk menentukan nomor Percobaan berikutnya dan kapan blok
        PERCOBAAN baru harus dibuat (setiap PICK_PLACE_LOG_ROWS_PER_FILE baris).
        """
        if not os.path.exists(path) or os.path.getsize(path) <= 0:
            return 0, 0

        batch_no = 0
        row_count = 0
        try:
            with open(path, "r", newline="", encoding="utf-8") as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row or not any(str(cell).strip() for cell in row):
                        continue
                    first = str(row[0]).strip().upper()
                    if row == self._pick_place_log_fields:
                        # Baris header kolom (mis. "Percobaan,...") harus dicek lebih dulu,
                        # karena kolom pertamanya juga diawali "PERCOBAAN" seperti judul blok.
                        continue
                    if first.startswith(self._pick_place_csv_base_name.upper()):
                        batch_no += 1
                        row_count = 0
                        continue
                    # Baris data pick-and-place.
                    if batch_no <= 0:
                        batch_no = 1
                    row_count += 1
        except Exception:
            return 0, 0
        return batch_no, row_count

    def _append_pick_place_csv(self, row: list[str]) -> tuple[str, int, int]:
        """Tambahkan satu percobaan dan rumus Error/MAE ke CSV.

        Nomor baris fisik CSV dibuat sama dengan nomor baris Excel. Dengan demikian,
        formula di CSV tetap benar saat file dibuka langsung melalui Microsoft Excel.
        """
        path = self._pick_place_single_csv_path()
        batch_no, row_count = self._scan_single_csv_state(path)

        file_exists = os.path.exists(path) and os.path.getsize(path) > 0
        need_new_section = (not file_exists) or batch_no <= 0 or row_count >= self._pick_place_rows_per_file
        if need_new_section:
            batch_no = max(1, batch_no + 1)
            row_count = 0

        local_trial_id = row_count + 1
        col_count = self._pick_place_log_col_count
        row = list(row[:col_count]) + [""] * max(0, col_count - len(row))
        row = row[:col_count]
        row[0] = str(local_trial_id)

        with open(path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if need_new_section:
                if file_exists:
                    writer.writerow([])
                if self._pick_place_log_write_title_row:
                    writer.writerow([self._pick_place_section_title(batch_no)])
                writer.writerow(self._pick_place_log_fields)
            writer.writerow(row)

        if self._pick_place_log_write_xlsx:
            self._rewrite_pick_place_xlsx_from_csv()

        return path, batch_no, local_trial_id

    @staticmethod
    def _read_shared_strings(zf: zipfile.ZipFile) -> list[str]:
        try:
            root = ET.fromstring(zf.read("xl/sharedStrings.xml"))
        except Exception:
            return []
        ns = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
        result: list[str] = []
        for si in root.findall("m:si", ns):
            result.append("".join(t.text or "" for t in si.findall(".//m:t", ns)))
        return result

    @classmethod
    def _read_manual_actual_from_xlsx(cls, xlsx_path: str) -> dict[int, dict[str, str]]:
        """Baca kembali input manual kolom E/F agar tidak hilang saat XLSX diperbarui."""
        if not os.path.exists(xlsx_path):
            return {}
        try:
            with zipfile.ZipFile(xlsx_path, "r") as zf:
                shared = cls._read_shared_strings(zf)
                root = ET.fromstring(zf.read("xl/worksheets/sheet1.xml"))
        except Exception:
            return {}

        ns = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
        values: dict[int, dict[str, str]] = {}
        for cell in root.findall(".//m:c", ns):
            ref = str(cell.attrib.get("r", ""))
            match = re.fullmatch(r"([A-Z]+)(\d+)", ref)
            if not match:
                continue
            col, row_text = match.groups()
            if col not in {"F", "G"}:
                continue
            row_num = int(row_text)
            cell_type = cell.attrib.get("t", "")
            value = ""
            if cell_type == "inlineStr":
                value = "".join(t.text or "" for t in cell.findall(".//m:t", ns))
            else:
                node = cell.find("m:v", ns)
                if node is not None and node.text is not None:
                    value = node.text
                    if cell_type == "s":
                        try:
                            value = shared[int(value)]
                        except Exception:
                            value = ""
            value = str(value).strip()
            if value:
                values.setdefault(row_num, {})[col] = value
        return values

    def _rewrite_pick_place_xlsx_from_csv(self) -> None:
        csv_path = self._pick_place_single_csv_path()
        xlsx_path = self._pick_place_xlsx_path()
        if not os.path.exists(csv_path):
            return

        with open(csv_path, "r", newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))

        existing_actual = self._read_manual_actual_from_xlsx(xlsx_path)
        csv_changed = False
        for row_num, actual in existing_actual.items():
            idx = row_num - 1
            if idx < 0 or idx >= len(rows):
                continue
            row = rows[idx]
            col_count = self._pick_place_log_col_count
            if len(row) < col_count or not str(row[0]).strip().isdigit():
                continue
            while len(row) < col_count:
                row.append("")
            for col, col_idx in (("F", 5), ("G", 6)):
                if not str(row[col_idx]).strip() and actual.get(col):
                    row[col_idx] = actual[col]
                    csv_changed = True

        if csv_changed:
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                csv.writer(f).writerows(rows)

        self._write_minimal_xlsx(xlsx_path, rows)

    @staticmethod
    def _xlsx_col_name(index0: int) -> str:
        n = index0 + 1
        name = ""
        while n > 0:
            n, rem = divmod(n - 1, 26)
            name = chr(65 + rem) + name
        return name

    @staticmethod
    def _xml_safe(value: str) -> str:
        return re.sub(r"[\x00-\x08\x0B\x0C\x0E-\x1F]", "", value)

    @classmethod
    def _cell_xml(cls, row_num: int, col_index0: int, value: str, style_id: int = 0) -> str:
        cell_ref = f"{cls._xlsx_col_name(col_index0)}{row_num}"
        text = cls._xml_safe(str(value))
        style_attr = f' s="{int(style_id)}"' if style_id else ""

        if text.startswith("="):
            formula = escape(text[1:].replace(";", ","))
            return f'<c r="{cell_ref}"{style_attr}><f>{formula}</f><v></v></c>'
        if re.fullmatch(r"-?\d+(\.\d+)?", text or ""):
            return f'<c r="{cell_ref}"{style_attr}><v>{text}</v></c>'
        return f'<c r="{cell_ref}"{style_attr} t="inlineStr"><is><t>{escape(text)}</t></is></c>'

    @classmethod
    def _write_minimal_xlsx(cls, xlsx_path: str, rows: list[list[str]]) -> None:
        fields = [
            "Percobaan", "Kelas", "Touch Current", "Pick X", "Pick Y",
            "Actual X", "Actual Y", "Latency YOLO (ms)", "Latency Decision (ms)",
            "Latency Serial (ms)", "Latency Total (ms)", "Cycle Time (ms)", "Status",
        ]
        col_count = len(fields)
        last_col = cls._xlsx_col_name(col_count - 1)  # "M"
        title_prefix = "PERCOBAAN"

        sheet_rows: list[str] = []
        merges: list[str] = []
        for r_idx, source_row in enumerate(rows, start=1):
            row = list(source_row)
            while len(row) < col_count:
                row.append("")
            first = str(row[0]).strip()
            upper_first = first.upper()
            is_blank = not any(str(v).strip() for v in row)
            is_header = row[:col_count] == fields
            is_title = upper_first.startswith(title_prefix) and not is_header
            is_data = first.isdigit()
            status_val = str(row[12]).strip().upper() if len(row) > 12 else ""

            cells: list[str] = []
            if not is_blank:
                for c_idx, value in enumerate(row[:col_count]):
                    if is_title:
                        style = 1
                    elif is_header:
                        style = 2
                    elif is_data:
                        if c_idx in (5, 6):
                            style = 4  # Actual X/Y - kuning (isi manual)
                        elif c_idx in (7, 8, 9, 10, 11):
                            style = 3  # kolom latency + cycle time
                        elif c_idx == 12:
                            style = 6 if status_val == "FAIL" else 5  # Status - merah/hijau
                        else:
                            style = 3
                    else:
                        style = 3
                    cells.append(cls._cell_xml(r_idx, c_idx, value, style_id=style))

            if is_title:
                merges.append(f"A{r_idx}:{last_col}{r_idx}")

            height = "24" if is_title else "34" if is_header else "22"
            sheet_rows.append(f'<row r="{r_idx}" ht="{height}" customHeight="1">' + "".join(cells) + "</row>")

        max_rows = max(len(rows), 1)
        dimension = f"A1:{last_col}{max_rows}"
        merge_xml = ""
        if merges:
            merge_xml = f'<mergeCells count="{len(merges)}">' + "".join(f'<mergeCell ref="{m}"/>' for m in merges) + "</mergeCells>"

        sheet_xml = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <dimension ref="{dimension}"/>
  <sheetViews><sheetView showGridLines="0" workbookViewId="0"/></sheetViews>
  <sheetFormatPr defaultRowHeight="18"/>
  <cols>
    <col min="1" max="1" width="12" customWidth="1"/>
    <col min="2" max="2" width="14" customWidth="1"/>
    <col min="3" max="3" width="16" customWidth="1"/>
    <col min="4" max="7" width="13" customWidth="1"/>
    <col min="8" max="12" width="16" customWidth="1"/>
    <col min="13" max="13" width="12" customWidth="1"/>
  </cols>
  <sheetData>{''.join(sheet_rows)}</sheetData>
  {merge_xml}
  <pageMargins left="0.4" right="0.4" top="0.5" bottom="0.5" header="0.2" footer="0.2"/>
  <pageSetup orientation="landscape" fitToWidth="1" fitToHeight="0"/>
</worksheet>'''

        styles_xml = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">
  <numFmts count="1"><numFmt numFmtId="164" formatCode="0.00"/></numFmts>
  <fonts count="3">
    <font><sz val="11"/><name val="Calibri"/></font>
    <font><b/><color rgb="FFFFFFFF"/><sz val="11"/><name val="Calibri"/></font>
    <font><b/><color rgb="FF1F2937"/><sz val="11"/><name val="Calibri"/></font>
  </fonts>
  <fills count="7">
    <fill><patternFill patternType="none"/></fill>
    <fill><patternFill patternType="gray125"/></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FF1F4E78"/><bgColor indexed="64"/></patternFill></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FF5B9BD5"/><bgColor indexed="64"/></patternFill></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FFFFF2CC"/><bgColor indexed="64"/></patternFill></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FFE2F0D9"/><bgColor indexed="64"/></patternFill></fill>
    <fill><patternFill patternType="solid"><fgColor rgb="FFFCE4D6"/><bgColor indexed="64"/></patternFill></fill>
  </fills>
  <borders count="2">
    <border><left/><right/><top/><bottom/><diagonal/></border>
    <border>
      <left style="thin"><color rgb="FF7F8C8D"/></left>
      <right style="thin"><color rgb="FF7F8C8D"/></right>
      <top style="thin"><color rgb="FF7F8C8D"/></top>
      <bottom style="thin"><color rgb="FF7F8C8D"/></bottom>
      <diagonal/>
    </border>
  </borders>
  <cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs>
  <cellXfs count="8">
    <xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/>
    <xf numFmtId="0" fontId="1" fillId="2" borderId="1" xfId="0" applyAlignment="1"><alignment horizontal="center" vertical="center"/></xf>
    <xf numFmtId="0" fontId="1" fillId="3" borderId="1" xfId="0" applyAlignment="1"><alignment horizontal="center" vertical="center" wrapText="1"/></xf>
    <xf numFmtId="164" fontId="0" fillId="0" borderId="1" xfId="0" applyNumberFormat="1" applyAlignment="1"><alignment horizontal="center" vertical="center"/></xf>
    <xf numFmtId="164" fontId="0" fillId="4" borderId="1" xfId="0" applyNumberFormat="1" applyAlignment="1"><alignment horizontal="center" vertical="center"/></xf>
    <xf numFmtId="164" fontId="0" fillId="5" borderId="1" xfId="0" applyNumberFormat="1" applyAlignment="1"><alignment horizontal="center" vertical="center"/></xf>
    <xf numFmtId="0" fontId="2" fillId="6" borderId="1" xfId="0" applyAlignment="1"><alignment horizontal="center" vertical="center"/></xf>
    <xf numFmtId="164" fontId="2" fillId="6" borderId="1" xfId="0" applyNumberFormat="1" applyAlignment="1"><alignment horizontal="center" vertical="center"/></xf>
  </cellXfs>
  <cellStyles count="1"><cellStyle name="Normal" xfId="0" builtinId="0"/></cellStyles>
</styleSheet>'''

        workbook_xml = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <sheets><sheet name="Pengambilan Data" sheetId="1" r:id="rId1"/></sheets>
  <calcPr calcId="191029" calcMode="auto" fullCalcOnLoad="1" forceFullCalc="1"/>
</workbook>'''
        workbook_rels = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>
</Relationships>'''
        rels = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>
  <Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>
</Relationships>'''
        content_types = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>
  <Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
  <Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>
  <Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
  <Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>
</Types>'''
        now = datetime.utcnow().isoformat(timespec="seconds") + "Z"
        core = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" xmlns:dc="http://purl.org/dc/elements/1.1/" xmlns:dcterms="http://purl.org/dc/terms/" xmlns:dcmitype="http://purl.org/dc/dcmitype/" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <dc:title>Pengambilan Data Pick and Place</dc:title>
  <dc:creator>YOLO Robot Logger</dc:creator>
  <dcterms:created xsi:type="dcterms:W3CDTF">{now}</dcterms:created>
  <dcterms:modified xsi:type="dcterms:W3CDTF">{now}</dcterms:modified>
</cp:coreProperties>'''
        app = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties" xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">
  <Application>Python</Application>
</Properties>'''

        directory = os.path.dirname(os.path.abspath(xlsx_path))
        if directory:
            os.makedirs(directory, exist_ok=True)
        with zipfile.ZipFile(xlsx_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            zf.writestr("[Content_Types].xml", content_types)
            zf.writestr("_rels/.rels", rels)
            zf.writestr("xl/workbook.xml", workbook_xml)
            zf.writestr("xl/_rels/workbook.xml.rels", workbook_rels)
            zf.writestr("xl/worksheets/sheet1.xml", sheet_xml)
            zf.writestr("xl/styles.xml", styles_xml)
            zf.writestr("docProps/core.xml", core)
            zf.writestr("docProps/app.xml", app)

    def close(self) -> None:
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
            self._ser = None
            self._connected = False


# =========================
# STANDALONE CONSOLE MODE
# =========================

def print_local_help() -> None:
    print(LOCAL_HELP_TEXT)


def _read_seconds_from_line(line: str, default: float = 2.0) -> float:
    parts = line.split()
    if len(parts) < 2:
        return default
    try:
        return max(0.1, float(parts[1]))
    except Exception:
        return default


def _history_limit_from_line(line: str, default: int = 20) -> int:
    parts = line.split()
    if len(parts) < 2:
        return default
    try:
        return max(1, int(parts[1]))
    except Exception:
        return default


def _run_info_bundle(comm: ESP32Comm) -> None:
    print("[INFO] Mengambil status dari ESP32: M114, M119, INA, YOLO")
    for cmd in ("M114", "M119", "INA", "YOLO"):
        comm.send_command_tracked(cmd, wait=1.5, force=True, echo=True, auto_m114=False)


def run_console() -> None:
    print("\n[MODE] ESP32 Command Console dua arah tanpa YOLO")
    print(f"[PORT] {getattr(config, 'SERIAL_PORT', 'COM?')} @ {getattr(config, 'SERIAL_BAUDRATE', 115200)}")
    print("[INFO] Tutup Arduino Serial Monitor sebelum menjalankan mode ini.\n")

    comm = ESP32Comm(auto_connect=True)
    print_local_help()

    if comm.is_connected:
        comm.read_available(duration=2.0, echo=True)

    while True:
        try:
            line = input("\nESP32> ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\n[EXIT] Console dihentikan.")
            break

        if not line:
            continue

        upper = line.upper()

        if upper in {"EXIT", "QUIT", "Q"}:
            break

        if upper in {"LOCALHELP", "LHELP"}:
            print_local_help()
            continue

        if upper in {"HELP", "?"}:
            print_local_help()
            if comm.is_connected:
                comm.send_command_tracked("HELP", wait=2.0, force=True, echo=True, auto_m114=False)
            else:
                print("[ESP32] Belum terhubung, HELP firmware tidak bisa dibaca.")
            continue

        if upper == "ESPHELP":
            comm.send_command_tracked("HELP", wait=2.0, force=True, echo=True, auto_m114=False)
            continue

        if upper.startswith("READ"):
            seconds = _read_seconds_from_line(line, default=2.0)
            print(f"[ESP32] Membaca output serial selama {seconds:.1f} detik...")
            comm.read_available(duration=seconds, echo=True)
            continue

        if upper == "PORTS":
            comm.print_serial_ports()
            continue

        if upper in {"HOME", "HOMING"}:
            comm.send_command_tracked("G28", wait=20.0, force=True, echo=True, auto_m114=True)
            continue

        if upper == "STATUS":
            print(f"[STATUS] {comm.status} | last_command={comm.last_command or '-'} | history={len(comm.history)}")
            continue

        if upper in {"POS", "POSITION"}:
            comm.send_command_tracked("M114", wait=1.5, force=True, echo=True, auto_m114=False)
            continue

        if upper == "INFO":
            _run_info_bundle(comm)
            continue

        if upper.startswith("HISTORY"):
            comm.print_history(_history_limit_from_line(line, default=20))
            continue

        if upper == "LAST":
            comm.print_last()
            continue

        if upper == "CLEARHISTORY":
            comm.clear_history()
            continue

        if upper == "RECONNECT":
            comm.reconnect()
            if comm.is_connected:
                comm.read_available(duration=2.0, echo=True)
            continue

        # Selain command lokal, semua input dikirim apa adanya ke ESP32.
        if not comm.is_connected and bool(getattr(config, "SERIAL_ENABLE", False)):
            print("[ESP32] Belum terhubung. Mencoba reconnect...")
            comm.reconnect()

        # Ini yang membuat console terasa dua arah:
        # command dikirim -> respons dibaca -> untuk motion otomatis tanya M114.
        comm.send_command_tracked(line, wait=None, force=True, echo=True, auto_m114=True)

    comm.close()
    print("[ESP32] Serial ditutup.")


if __name__ == "__main__":
    run_console()
