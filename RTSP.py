import cv2
import requests
import numpy as np

ESP32_IP = "192.168.x.x" 
STREAM_URL = f"http://{ESP32_IP}:81/stream"

print(f"Conectando al flujo de datos en: {STREAM_URL}")

stream = requests.get(STREAM_URL, stream=True)

if stream.status_code != 200:
    print("Error: No se pudo conectar a la cámara. Verifica la IP.")
    exit()

print("Conexión exitosa. Presiona 'q' para cerrar las ventanas.")

bytes_data = b""
kernel = np.ones((5, 5), np.uint8) 

for chunk in stream.iter_content(chunk_size=1024):
    bytes_data += chunk
    
    a = bytes_data.find(b'\xff\xd8')
    b = bytes_data.find(b'\xff\xd9')
    
    if a != -1 and b != -1:
        if a < b:
            jpg = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]
            
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            if frame is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                _, binary = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
                processed = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
                
                cv2.imshow("1. Canal de Video Local (ESP32)", frame)
                cv2.imshow("2. Procesamiento OpenCV en la PC", processed)
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            bytes_data = bytes_data[b+2:]

stream.close()
cv2.destroyAllWindows()
print("Flujo finalizado.")
cv2.destroyAllWindows()
print("Flujo finalizado.")