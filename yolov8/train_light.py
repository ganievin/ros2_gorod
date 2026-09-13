from ultralytics import YOLO
import torch

print("Проверка системы...")
print(f"PyTorch версия: {torch.__version__}")
print(f"CUDA доступна: {torch.cuda.is_available()}")

# Проверка свободной памяти
import psutil
ram = psutil.virtual_memory()
print(f"Доступно RAM: {ram.available / (1024**3):.1f} GB")
print(f"Всего RAM: {ram.total / (1024**3):.1f} GB")

print("\nЗагрузка YOLOv8n (самая маленькая модель)...")
model = YOLO('yolov8n.pt')

print("\nНачало обучения с оптимизированными параметрами...")
results = model.train(
    data='my_dataset_yolov8_vseznaki/data.yaml',
    epochs=50,               # всего 20 эпох
    imgsz=320,               # уменьшили размер (было 640)
    batch=2,                 # уменьшили батч (было 4)
    workers=0,               # отключили workers
    device='cpu',           
    cache=False,            
    optimizer='SGD',        
    lr0=0.01,               
    augment=False,          
    exist_ok=True,
    name='yolo_light',
    verbose=True
)

print("\nОбучение завершено!")
