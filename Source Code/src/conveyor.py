"""
conveyor.py
===========

Controller terpisah untuk conveyor.

Default:
  - Robot ESP32 tetap di-handle oleh esp32_comm.py / COM15.
  - Conveyor ESP32 di-handle oleh file ini / COM10.
  - main.py cukup memanggil start(), stop(), close().

Catatan penting untuk firmware conveyor lama:
  Sketch Conveyor.ino awal hanya mengenal command 'F' dan 'R', serta motor
  SELALU mengeluarkan pulse. Dengan firmware seperti itu, Python bisa
  mengubah arah, tetapi TIDAK bisa stop motor secara software karena firmware
  tidak punya handler STOP/START. Jika firmware conveyor sudah mendukung
  START/STOP, class ini akan langsung cocok.

Mode kontrol:
  - CONVEYOR_CONTROL_MODE = "AUTO"            -> coba konek sesuai priority, pakai yang berhasil
  - CONVEYOR_CONTROL_MODE = "ESP32"           -> pakai ESP32 serial saja
  - CONVEYOR_CONTROL_MODE = "PLC_MODBUS_TCP"  -> pakai PLC Modbus TCP saja
  - CONVEYOR_CONTROL_MODE = "PLC_MODBUS_RTU"  -> pakai PLC Modbus RTU saja

Untuk AUTO, urutan default:
  1. ESP32 conveyor serial
  2. PLC Modbus TCP
  3. PLC Modbus RTU

Untuk PLC dengan register D0 sebagai command conveyor, gunakan:
  - CONVEYOR_PLC_WRITE_MODE = "REGISTER"
  - CONVEYOR_PLC_RUN_REGISTER_ADDRESS = 0

Mapping default register:
  - D0 = 1  -> Conveyor RUN
  - D0 = 0  -> Conveyor STOP

Agar tidak perlu mengubah config.py, nilai default bisa diganti di konstanta
bagian atas file ini, atau lewat environment variable Windows, contoh:
  set CONVEYOR_CONTROL_MODE=AUTO
  set CONVEYOR_AUTO_PRIORITY=ESP32,PLC_MODBUS_TCP
  set CONVEYOR_PORT=COM10
  set CONVEYOR_START_COMMAND=R
  set CONVEYOR_STOP_COMMAND=STOP
"""

from __future__ import annotations

import os
import time
import threading
from dataclasses import dataclass

try:
    import config
except Exception:  # pragma: no cover
    config = None

try:
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover
    serial = None
    list_ports = None

# pymodbus bersifat opsional. Install jika mode PLC dipakai:
#   pip install pymodbus
try:
    from pymodbus.client import ModbusTcpClient, ModbusSerialClient
except Exception:  # pragma: no cover
    ModbusTcpClient = None
    ModbusSerialClient = None


# =========================
# DEFAULT SETTING CONVEYOR
# =========================

CONVEYOR_CONTROL_MODE = os.getenv(
    "CONVEYOR_CONTROL_MODE",
    getattr(config, "CONVEYOR_CONTROL_MODE", "AUTO") if config is not None else "AUTO",
).upper()

# Mode AUTO: sistem akan mencoba koneksi sesuai urutan ini, lalu memakai yang pertama berhasil.
# Contoh: "ESP32,PLC_MODBUS_TCP,PLC_MODBUS_RTU"
CONVEYOR_AUTO_PRIORITY = os.getenv(
    "CONVEYOR_AUTO_PRIORITY",
    getattr(config, "CONVEYOR_AUTO_PRIORITY", "ESP32,PLC_MODBUS_TCP,PLC_MODBUS_RTU") if config is not None else "ESP32,PLC_MODBUS_TCP,PLC_MODBUS_RTU",
)

CONVEYOR_SERIAL_PORT = os.getenv(
    "CONVEYOR_PORT",
    getattr(config, "CONVEYOR_SERIAL_PORT", "COM10") if config is not None else "COM10",
)

CONVEYOR_SERIAL_BAUDRATE = int(os.getenv(
    "CONVEYOR_BAUDRATE",
    str(getattr(config, "CONVEYOR_SERIAL_BAUDRATE", 115200) if config is not None else 115200),
))

CONVEYOR_SERIAL_TIMEOUT = float(os.getenv(
    "CONVEYOR_TIMEOUT",
    str(getattr(config, "CONVEYOR_SERIAL_TIMEOUT", 0.2) if config is not None else 0.2),
))

# Untuk firmware lama, command arah yang dikenal adalah F/R.
# Default diset R karena firmware Conveyor.ino original memakai LOW/Reverse
# sebagai arah awal. Jika arah aktual masih terbalik, ganti menjadi "F".
CONVEYOR_START_COMMAND = os.getenv(
    "CONVEYOR_START_COMMAND",
    getattr(config, "CONVEYOR_START_COMMAND", "R") if config is not None else "R",
)

# Untuk firmware yang sudah support START/STOP, command ini akan bekerja.
# Untuk firmware lama yang hanya F/R, STOP akan diabaikan oleh ESP32.
CONVEYOR_RUN_COMMAND = os.getenv(
    "CONVEYOR_RUN_COMMAND",
    getattr(config, "CONVEYOR_RUN_COMMAND", "START") if config is not None else "START",
)
CONVEYOR_STOP_COMMAND = os.getenv(
    "CONVEYOR_STOP_COMMAND",
    getattr(config, "CONVEYOR_STOP_COMMAND", "STOP") if config is not None else "STOP",
)

CONVEYOR_AUTO_START_ON_CONNECT = bool(int(os.getenv(
    "CONVEYOR_AUTO_START_ON_CONNECT",
    "0",
)))

# PLC TCP default. Ubah di config.py/env jika mode PLC dipakai.
PLC_HOST = os.getenv("CONVEYOR_PLC_HOST", getattr(config, "CONVEYOR_PLC_HOST", "192.168.1.10") if config is not None else "192.168.1.10")
PLC_PORT = int(os.getenv("CONVEYOR_PLC_PORT", str(getattr(config, "CONVEYOR_PLC_PORT", 502) if config is not None else 502)))
PLC_UNIT_ID = int(os.getenv("CONVEYOR_PLC_UNIT_ID", str(getattr(config, "CONVEYOR_PLC_UNIT_ID", 1) if config is not None else 1)))

# PLC RTU default.
PLC_RTU_PORT = os.getenv("CONVEYOR_PLC_RTU_PORT", getattr(config, "CONVEYOR_PLC_RTU_PORT", "COM10") if config is not None else "COM10")
PLC_RTU_BAUDRATE = int(os.getenv("CONVEYOR_PLC_RTU_BAUDRATE", str(getattr(config, "CONVEYOR_PLC_RTU_BAUDRATE", 9600) if config is not None else 9600)))
PLC_RTU_PARITY = os.getenv("CONVEYOR_PLC_RTU_PARITY", getattr(config, "CONVEYOR_PLC_RTU_PARITY", "N") if config is not None else "N")
PLC_RTU_STOPBITS = int(os.getenv("CONVEYOR_PLC_RTU_STOPBITS", str(getattr(config, "CONVEYOR_PLC_RTU_STOPBITS", 1) if config is not None else 1)))
PLC_RTU_BYTESIZE = int(os.getenv("CONVEYOR_PLC_RTU_BYTESIZE", str(getattr(config, "CONVEYOR_PLC_RTU_BYTESIZE", 8) if config is not None else 8)))

# Mode penulisan PLC:
PLC_WRITE_MODE = os.getenv(
    "CONVEYOR_PLC_WRITE_MODE",
    getattr(config, "CONVEYOR_PLC_WRITE_MODE", "REGISTER") if config is not None else "REGISTER",
).upper()

# Register PLC untuk RUN conveyor. Default address 0 = D0 pada banyak mapping PLC.
# Jika PLC/HMI kamu memetakan D0 ke alamat lain, ubah nilainya di config.py.
PLC_RUN_REGISTER_ADDRESS = int(os.getenv(
    "CONVEYOR_PLC_RUN_REGISTER",
    str(getattr(config, "CONVEYOR_PLC_RUN_REGISTER_ADDRESS", 0) if config is not None else 0),
))
PLC_RUN_VALUE = int(os.getenv(
    "CONVEYOR_PLC_RUN_VALUE",
    str(getattr(config, "CONVEYOR_PLC_RUN_VALUE", 1) if config is not None else 1),
))
PLC_STOP_VALUE = int(os.getenv(
    "CONVEYOR_PLC_STOP_VALUE",
    str(getattr(config, "CONVEYOR_PLC_STOP_VALUE", 0) if config is not None else 0),
))

# Coil PLC untuk RUN conveyor. Dipakai hanya jika CONVEYOR_PLC_WRITE_MODE="COIL".
PLC_RUN_COIL_ADDRESS = int(os.getenv("CONVEYOR_PLC_RUN_COIL", str(getattr(config, "CONVEYOR_PLC_RUN_COIL_ADDRESS", 0) if config is not None else 0)))
PLC_FORWARD_COIL_ADDRESS = int(os.getenv("CONVEYOR_PLC_FORWARD_COIL", str(getattr(config, "CONVEYOR_PLC_FORWARD_COIL_ADDRESS", 1) if config is not None else 1)))

# Opsional: pulse D0 sesaat. Tidak dipakai oleh start()/stop(), tetapi tersedia untuk console/test.
PLC_PULSE_DURATION_SEC = float(os.getenv(
    "CONVEYOR_PLC_PULSE_DURATION_SEC",
    str(getattr(config, "CONVEYOR_PLC_PULSE_DURATION_SEC", 0.20) if config is not None else 0.20),
))


@dataclass
class ConveyorStatus:
    mode: str
    connected: bool
    running: bool
    detail: str = ""


class ConveyorController:
    def __init__(self, auto_connect: bool = True):
        # requested_mode = mode dari config/env. Jika AUTO, self.mode akan berubah
        # menjadi mode aktual yang berhasil terkoneksi, misalnya ESP32 atau PLC_MODBUS_TCP.
        self.requested_mode = CONVEYOR_CONTROL_MODE
        self.mode = CONVEYOR_CONTROL_MODE
        self._ser = None
        self._client = None
        self._lock = threading.Lock()
        self._connected = False
        self.running = False
        self.last_command = ""

        if auto_connect:
            self.connect()

    @property
    def is_connected(self) -> bool:
        return bool(self._connected)

    @property
    def status(self) -> str:
        state = "RUN" if self.running else "STOP"
        prefix = f"AUTO->{self.mode}" if self.requested_mode == "AUTO" and self.mode != "AUTO" else self.mode
        if self._connected:
            return f"{prefix} OK | {state}"
        return f"{prefix} DISCONNECTED | {state}"

    @staticmethod
    def list_serial_ports() -> list[str]:
        if list_ports is None:
            return []
        try:
            return [f"{p.device} | {p.description} | HWID={p.hwid}" for p in list_ports.comports()]
        except Exception:
            return []

    def connect(self) -> bool:
        # Selalu mulai dari requested_mode, supaya reconnect AUTO bisa memilih ulang
        # jika koneksi sebelumnya putus.
        self.mode = self.requested_mode

        if self.mode == "AUTO":
            return self._connect_auto()
        if self.mode == "ESP32":
            return self._connect_esp32()
        if self.mode == "PLC_MODBUS_TCP":
            return self._connect_plc_tcp()
        if self.mode == "PLC_MODBUS_RTU":
            return self._connect_plc_rtu()
        print(f"[CONVEYOR] Mode tidak dikenal: {self.mode}")
        return False

    def _connect_auto(self) -> bool:
        raw_modes = [m.strip().upper() for m in str(CONVEYOR_AUTO_PRIORITY).split(",") if m.strip()]
        modes = []
        for m in raw_modes:
            if m in {"ESP32", "PLC_MODBUS_TCP", "PLC_MODBUS_RTU"} and m not in modes:
                modes.append(m)
        if not modes:
            modes = ["ESP32", "PLC_MODBUS_TCP", "PLC_MODBUS_RTU"]

        print(f"[CONVEYOR AUTO] Mencari koneksi conveyor. Priority: {', '.join(modes)}")
        for candidate in modes:
            self._close_transport_only()
            self.mode = candidate
            print(f"[CONVEYOR AUTO] Coba {candidate}...")
            if candidate == "ESP32":
                ok = self._connect_esp32()
            elif candidate == "PLC_MODBUS_TCP":
                ok = self._connect_plc_tcp()
            elif candidate == "PLC_MODBUS_RTU":
                ok = self._connect_plc_rtu()
            else:
                ok = False

            if ok:
                print(f"[CONVEYOR AUTO] Terpilih: {candidate}")
                return True

        self.mode = "AUTO"
        self._connected = False
        print("[CONVEYOR AUTO] Tidak ada koneksi conveyor yang berhasil. Cek ESP32/PLC dan config.py.")
        return False

    def _connect_esp32(self) -> bool:
        serial_enable = bool(getattr(config, "SERIAL_ENABLE", True)) if config is not None else True
        if not serial_enable:
            print(f"[CONVEYOR] STUB mode karena SERIAL_ENABLE=False. Target {CONVEYOR_SERIAL_PORT}")
            self._connected = False
            return False
        if serial is None:
            print("[CONVEYOR] pyserial belum terpasang. Install: pip install pyserial")
            self._connected = False
            return False

        print(f"[CONVEYOR] Membuka ESP32 conveyor {CONVEYOR_SERIAL_PORT} @ {CONVEYOR_SERIAL_BAUDRATE}...")
        try:
            self._ser = serial.Serial(
                port=CONVEYOR_SERIAL_PORT,
                baudrate=CONVEYOR_SERIAL_BAUDRATE,
                timeout=CONVEYOR_SERIAL_TIMEOUT,
                write_timeout=1.0,
                rtscts=False,
                dsrdtr=False,
            )
            try:
                self._ser.setDTR(False)
                self._ser.setRTS(False)
            except Exception:
                pass
            time.sleep(2.0)
            try:
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
            except Exception:
                pass
            self._connected = True
            print(f"[CONVEYOR] TERHUBUNG di {CONVEYOR_SERIAL_PORT}")
            if CONVEYOR_AUTO_START_ON_CONNECT:
                self.start()
            return True
        except Exception as e:
            self._connected = False
            self._ser = None
            print(f"[CONVEYOR] Gagal konek ESP32 conveyor: {e}")
            print("[CONVEYOR] Cek COM10, kabel, driver, dan pastikan tidak dibuka Arduino Serial Monitor.")
            return False

    def _connect_plc_tcp(self) -> bool:
        if ModbusTcpClient is None:
            print("[CONVEYOR PLC] pymodbus belum terpasang. Install: pip install pymodbus")
            return False
        try:
            self._client = ModbusTcpClient(host=PLC_HOST, port=PLC_PORT, timeout=1.0)
            self._connected = bool(self._client.connect())
            print(f"[CONVEYOR PLC] TCP {PLC_HOST}:{PLC_PORT} connected={self._connected}")
            return self._connected
        except Exception as e:
            self._connected = False
            print(f"[CONVEYOR PLC] Gagal konek TCP: {e}")
            return False

    def _connect_plc_rtu(self) -> bool:
        if ModbusSerialClient is None:
            print("[CONVEYOR PLC] pymodbus belum terpasang. Install: pip install pymodbus")
            return False
        try:
            self._client = ModbusSerialClient(
                port=PLC_RTU_PORT,
                baudrate=PLC_RTU_BAUDRATE,
                parity=PLC_RTU_PARITY,
                stopbits=PLC_RTU_STOPBITS,
                bytesize=PLC_RTU_BYTESIZE,
                timeout=1.0,
            )
            self._connected = bool(self._client.connect())
            print(f"[CONVEYOR PLC] RTU {PLC_RTU_PORT} connected={self._connected}")
            return self._connected
        except Exception as e:
            self._connected = False
            print(f"[CONVEYOR PLC] Gagal konek RTU: {e}")
            return False

    def _close_transport_only(self) -> None:
        """Tutup port/client tanpa mengirim STOP. Dipakai saat AUTO mencoba beberapa koneksi."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
        self._client = None
        self._connected = False

    def reconnect(self) -> bool:
        self.close()
        time.sleep(0.5)
        return self.connect()

    def send_raw(self, command: str) -> bool:
        command = str(command).strip()
        if not command:
            return False

        self.last_command = command
        if self.mode == "ESP32":
            if not self._connected or self._ser is None:
                print(f"[CONVEYOR] Tidak terkoneksi, command tidak dikirim: {command}")
                return False
            try:
                with self._lock:
                    self._ser.write((command + "\n").encode("utf-8"))
                    self._ser.flush()
                print(f"[CONVEYOR -> ESP32] {command}")
                return True
            except Exception as e:
                self._connected = False
                print(f"[CONVEYOR] Gagal kirim '{command}': {e}")
                return False

        print(f"[CONVEYOR] send_raw hanya untuk mode ESP32. Mode aktif: {self.mode}")
        return False

    def read_available(self, duration: float = 0.05, echo: bool = True) -> list[str]:
        lines: list[str] = []
        if self.mode != "ESP32" or not self._connected or self._ser is None:
            return lines
        end = time.perf_counter() + max(0.0, duration)
        while time.perf_counter() < end:
            try:
                waiting = int(self._ser.in_waiting)
            except Exception:
                waiting = 0
            if waiting <= 0:
                time.sleep(0.01)
                continue
            try:
                text = self._ser.readline().decode("utf-8", errors="replace").strip()
                if text:
                    lines.append(text)
                    if echo:
                        print(f"[CONVEYOR] << {text}")
            except Exception as e:
                self._connected = False
                print(f"[CONVEYOR] Gagal baca serial: {e}")
                break
        return lines

    def start(self) -> bool:
        """Jalankan conveyor."""
        if self.mode == "ESP32":
            ok = True
            # Kirim arah dulu. Untuk firmware lama, command F/R ini sekaligus
            # cukup karena pulse motor memang selalu berjalan.
            if CONVEYOR_START_COMMAND:
                ok = self.send_raw(CONVEYOR_START_COMMAND) and ok
                time.sleep(0.03)
            # Untuk firmware baru yang mendukung START/STOP.
            if CONVEYOR_RUN_COMMAND:
                ok = self.send_raw(CONVEYOR_RUN_COMMAND) and ok
            self.running = True if ok or self._connected else self.running
            return ok

        if self.mode in {"PLC_MODBUS_TCP", "PLC_MODBUS_RTU"}:
            return self._plc_write_run(True)

        return False

    def stop(self) -> bool:
        """Berhentikan conveyor."""
        if self.mode == "ESP32":
            ok = self.send_raw(CONVEYOR_STOP_COMMAND)
            if ok:
                self.running = False
            return ok

        if self.mode in {"PLC_MODBUS_TCP", "PLC_MODBUS_RTU"}:
            return self._plc_write_run(False)

        return False

    def _plc_call_with_unit(self, func, *args, **kwargs):
        """Pymodbus v2/v3 compatibility: coba slave= dulu, lalu unit=."""
        try:
            return func(*args, slave=PLC_UNIT_ID, **kwargs)
        except TypeError:
            return func(*args, unit=PLC_UNIT_ID, **kwargs)

    @staticmethod
    def _modbus_failed(result) -> bool:
        try:
            return bool(result.isError())
        except Exception:
            return False

    def _plc_write_run(self, run: bool) -> bool:
        """
        Tulis command conveyor ke PLC.

        Default mode REGISTER:
            D0 = 1 -> conveyor RUN
            D0 = 0 -> conveyor STOP

        Mode COIL lama tetap tersedia lewat CONVEYOR_PLC_WRITE_MODE="COIL".
        """
        if self._client is None or not self._connected:
            print("[CONVEYOR PLC] Tidak terkoneksi.")
            return False

        if PLC_WRITE_MODE in {"REGISTER", "D", "D0", "D_REGISTER", "HOLDING_REGISTER"}:
            value = PLC_RUN_VALUE if run else PLC_STOP_VALUE
            try:
                result = self._plc_call_with_unit(
                    self._client.write_register,
                    PLC_RUN_REGISTER_ADDRESS,
                    int(value),
                )
                if self._modbus_failed(result):
                    print(f"[CONVEYOR PLC] Gagal write D0/register {PLC_RUN_REGISTER_ADDRESS} = {value}: {result}")
                    return False
                self.running = bool(run)
                print(f"[CONVEYOR PLC] D0/register {PLC_RUN_REGISTER_ADDRESS} = {value} ({'RUN' if run else 'STOP'})")
                return True
            except Exception as e:
                self._connected = False
                print(f"[CONVEYOR PLC] Error write D0/register RUN={run}: {e}")
                return False

        if PLC_WRITE_MODE == "COIL":
            try:
                # Set arah forward, lalu RUN coil.
                try:
                    self._plc_call_with_unit(self._client.write_coil, PLC_FORWARD_COIL_ADDRESS, True)
                except Exception:
                    # Tidak semua ladder membutuhkan coil arah. Abaikan jika tidak tersedia.
                    pass
                result = self._plc_call_with_unit(self._client.write_coil, PLC_RUN_COIL_ADDRESS, bool(run))

                if self._modbus_failed(result):
                    print(f"[CONVEYOR PLC] Gagal write coil RUN={run}: {result}")
                    return False
                self.running = bool(run)
                print(f"[CONVEYOR PLC] RUN coil {PLC_RUN_COIL_ADDRESS} = {bool(run)}")
                return True
            except Exception as e:
                self._connected = False
                print(f"[CONVEYOR PLC] Error write coil RUN={run}: {e}")
                return False

        print(f"[CONVEYOR PLC] CONVEYOR_PLC_WRITE_MODE tidak dikenal: {PLC_WRITE_MODE}")
        return False

    def pulse_d0(self, duration_sec: float | None = None) -> bool:
        """
        Kirim pulse ke D0: 1 sebentar lalu 0.
        Pakai hanya jika ladder PLC memang didesain membaca pulse. Untuk conveyor,
        start()/stop() level-signal tetap lebih aman.
        """
        if self.mode not in {"PLC_MODBUS_TCP", "PLC_MODBUS_RTU"}:
            print(f"[CONVEYOR PLC] pulse_d0 hanya untuk mode PLC. Mode aktif: {self.mode}")
            return False
        duration = PLC_PULSE_DURATION_SEC if duration_sec is None else float(duration_sec)
        ok1 = self._plc_write_run(True)
        time.sleep(max(0.01, duration))
        ok2 = self._plc_write_run(False)
        return ok1 and ok2

    def close(self) -> None:
        try:
            # Usahakan stop dulu, tetapi jangan memaksa jika firmware lama tidak support STOP.
            if self.running:
                self.stop()
                time.sleep(0.05)
        except Exception:
            pass

        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
        self._client = None
        self._connected = False


def _console() -> None:
    conveyor = ConveyorController(auto_connect=True)
    print("\n[CONVEYOR CONSOLE]")
    print("  START   : jalankan conveyor / tulis D0=1 jika mode PLC")
    print("  STOP    : hentikan conveyor / tulis D0=0 jika mode PLC")
    print("  PULSE   : pulse D0=1 lalu D0=0, khusus mode PLC")
    print("  F / R   : kirim arah langsung ke ESP32")
    print("  STATUS  : tampilkan status dan mode aktif")
    print("  PORTS   : daftar COM port")
    print("  READ    : baca serial conveyor")
    print("  RECONNECT: konek ulang")
    print("  EXIT    : keluar\n")

    try:
        while True:
            cmd = input("CONVEYOR> ").strip()
            if not cmd:
                continue
            upper = cmd.upper()
            if upper in {"EXIT", "QUIT", "Q"}:
                break
            if upper == "START":
                conveyor.start()
            elif upper == "STOP":
                conveyor.stop()
            elif upper == "PULSE":
                conveyor.pulse_d0()
            elif upper == "STATUS":
                print(f"[CONVEYOR] {conveyor.status}")
            elif upper == "PORTS":
                ports = ConveyorController.list_serial_ports()
                if not ports:
                    print("[CONVEYOR] Tidak ada port terbaca / pyserial belum tersedia.")
                for p in ports:
                    print("  -", p)
            elif upper == "READ":
                conveyor.read_available(duration=2.0, echo=True)
            elif upper == "RECONNECT":
                conveyor.reconnect()
            else:
                conveyor.send_raw(cmd)
                conveyor.read_available(duration=0.2, echo=True)
    finally:
        conveyor.close()


if __name__ == "__main__":
    _console()
