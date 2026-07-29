from ultralytics import YOLO
import torch

print(f"CUDA available : {torch.cuda.is_available()}")
print(f"GPU            : {torch.cuda.get_device_name(0)}")

MODEL_PATH = r"D:\Documents\Yolov11-seg Skripsi\model\best.pt"

model = YOLO(MODEL_PATH)

print("\nMulai export TensorRT... (proses 2-10 menit, harap tunggu)\n")

model.export(
    format   = "engine",
    half     = True,      # FP16
    device   = 0,
    workspace= 4,         # GB — sesuai VRAM GTX 1650
    simplify = True,
    imgsz    = 640,
)

print("\nExport selesai! File best.engine tersimpan di folder model.")