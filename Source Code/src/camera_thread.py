import sys
import threading

from camera_source import open_camera


class CameraThread:
    """
    Membaca frame OBS Virtual Camera di thread terpisah agar loop utama tidak
    diblokir oleh latensi I/O kamera dan buffer selalu memakai frame terbaru.
    """

    def __init__(self, src: int, width: int, height: int):
        # Parameter src dipertahankan agar pemanggilan lama tetap kompatibel.
        # Pemilihan perangkat sekarang dipusatkan pada camera_source.py.
        del src
        try:
            self.cap = open_camera(width, height)
        except RuntimeError as exc:
            print(f"ERROR: {exc}")
            sys.exit(1)

        self.frame = None
        self.lock = threading.Lock()
        self.running = True

        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def read(self):
        """Kembalikan salinan frame terakhir, atau None jika belum ada."""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def release(self):
        self.running = False
        self._thread.join(timeout=2)
        self.cap.release()
