// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "camera_index.h"
#include "board_config.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";
static const char *_MORPH_INDEX_HTML = R"rawliteral(
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root {
      color-scheme: dark;
      --bg: #111827;
      --panel: #1f2937;
      --panel-2: #0f172a;
      --text: #e5e7eb;
      --muted: #9ca3af;
      --accent: #22c55e;
      --border: #334155;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Arial, sans-serif;
      background: linear-gradient(160deg, #0f172a, #111827 45%, #1e293b);
      color: var(--text);
    }
    .wrap {
      max-width: 1100px;
      margin: 0 auto;
      padding: 24px;
    }
    h1, h2, p { margin: 0; }
    .hero {
      display: grid;
      gap: 12px;
      margin-bottom: 24px;
    }
    .hero p {
      color: var(--muted);
      line-height: 1.5;
    }
    .grid {
      display: grid;
      grid-template-columns: minmax(280px, 1.2fr) minmax(280px, 0.8fr);
      gap: 20px;
    }
    .compare-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 20px;
      margin-top: 20px;
    }
    .panel {
      background: rgba(15, 23, 42, 0.82);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 12px 30px rgba(0, 0, 0, 0.25);
    }
    .panel h2 {
      font-size: 18px;
      margin-bottom: 12px;
    }
    .video-box, .image-box {
      background: #020617;
      border: 1px solid #1e293b;
      border-radius: 14px;
      overflow: hidden;
      aspect-ratio: 4 / 3;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    img, canvas {
      width: 100%;
      height: 100%;
      object-fit: contain;
      display: block;
      background: #000;
    }
    .stack {
      display: grid;
      gap: 16px;
    }
    .controls {
      display: grid;
      gap: 12px;
    }
    .two-col {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    .row {
      display: grid;
      gap: 8px;
    }
    label {
      font-size: 14px;
      color: var(--muted);
    }
    select, input, button {
      width: 100%;
      border-radius: 12px;
      border: 1px solid var(--border);
      padding: 12px 14px;
      background: var(--panel);
      color: var(--text);
      font-size: 15px;
    }
    button {
      background: var(--accent);
      color: #052e16;
      font-weight: bold;
      cursor: pointer;
      border: none;
    }
    button.secondary {
      background: #334155;
      color: var(--text);
      border: 1px solid var(--border);
    }
    .note {
      color: var(--muted);
      font-size: 14px;
      line-height: 1.5;
    }
    .status {
      min-height: 24px;
      color: #86efac;
      font-size: 14px;
    }
    .camera-controls {
      margin-top: 18px;
      padding-top: 18px;
      border-top: 1px solid var(--border);
      display: grid;
      gap: 12px;
    }
    @media (max-width: 900px) {
      .grid {
        grid-template-columns: 1fr;
      }
      .compare-grid {
        grid-template-columns: 1fr;
      }
      .two-col {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="grid">
      <div class="panel stack">
        <div class="video-box">
          <img id="stream" alt="Stream de la camara">
        </div>
      </div>

      <div class="panel controls">
        <div>
          <h2>Procesamiento</h2>
        </div>
        <div class="row">
          <label for="operation">Operacion</label>
          <select id="operation">
            <option value="erode">Erosion</option>
            <option value="dilate">Dilatacion</option>
            <option value="open">Apertura</option>
            <option value="close">Cierre</option>
          </select>
        </div>
        <div class="row">
          <label for="threshold">Umbral binario</label>
          <input id="threshold" type="range" min="0" max="255" value="120">
        </div>
        <div class="row">
          <label for="kernel">Tamano de kernel</label>
          <select id="kernel">
            <option value="3">3 x 3</option>
            <option value="5">5 x 5</option>
          </select>
        </div>
        <button id="captureBtn">Tomar foto</button>
        <button id="processBtn" class="secondary">Reprocesar</button>
        <div id="status" class="status"></div>
        <div class="camera-controls">
          <div>
            <h2>Ajustes de camara</h2>
            <p class="note">Modifica la imagen en vivo antes de capturarla.</p>
          </div>
          <div class="row">
            <label for="framesize">Resolucion</label>
            <select id="framesize">
              <option value="10">UXGA 1600x1200</option>
              <option value="9">SXGA 1280x1024</option>
              <option value="8">XGA 1024x768</option>
              <option value="7">SVGA 800x600</option>
              <option value="6">VGA 640x480</option>
              <option value="5">CIF 400x296</option>
              <option value="4">QVGA 320x240</option>
              <option value="3">HQVGA 240x176</option>
              <option value="0">QQVGA 160x120</option>
              <option value="0">QQVGA 128x128</option>
              <option value="0">QQVGA 96x96</option>
            </select>
          </div>
          <div class="row">
            <label for="quality">Calidad JPEG</label>
            <input id="quality" type="range" min="10" max="63" value="10">
          </div>
          <div class="two-col">
            <div class="row">
              <label for="brightness">Brillo</label>
              <input id="brightness" type="range" min="-2" max="2" value="0">
            </div>
            <div class="row">
              <label for="contrast">Contraste</label>
              <input id="contrast" type="range" min="-2" max="2" value="0">
            </div>
          </div>
          <div class="two-col">
            <div class="row">
              <label for="saturation">Saturacion</label>
              <input id="saturation" type="range" min="-2" max="2" value="0">
            </div>
            <div class="row">
              <label for="ae_level">Nivel AE</label>
              <input id="ae_level" type="range" min="-2" max="2" value="0">
            </div>
          </div>
          <div class="two-col">
            <div class="row">
              <label for="awb">AWB</label>
              <select id="awb">
                <option value="1">Activado</option>
                <option value="0">Desactivado</option>
              </select>
            </div>
            <div class="row">
              <label for="agc">AGC</label>
              <select id="agc">
                <option value="1">Activado</option>
                <option value="0">Desactivado</option>
              </select>
            </div>
          </div>
          <div class="two-col">
            <div class="row">
              <label for="aec">AEC</label>
              <select id="aec">
                <option value="1">Activado</option>
                <option value="0">Desactivado</option>
              </select>
            </div>
            <div class="row">
              <label for="vflip">Volteo vertical</label>
              <select id="vflip">
                <option value="0">Normal</option>
                <option value="1">Volteado</option>
              </select>
            </div>
          </div>
          <div class="two-col">
            <div class="row">
              <label for="hmirror">Espejo horizontal</label>
              <select id="hmirror">
                <option value="0">Normal</option>
                <option value="1">Espejo</option>
              </select>
            </div>
            <div class="row">
              <label for="special_effect">Efecto</label>
              <select id="special_effect">
                <option value="0">Ninguno</option>
                <option value="1">Negativo</option>
                <option value="2">Escala de grises</option>
                <option value="3">Tinte rojo</option>
                <option value="4">Tinte verde</option>
                <option value="5">Tinte azul</option>
                <option value="6">Sepia</option>
              </select>
            </div>
          </div>
          <button id="refreshStatusBtn" class="secondary">Cargar ajustes actuales</button>
        </div>
      </div>
    </div>

    <div class="compare-grid">
      <div class="panel">
        <div class="stack">
          <div>
            <h2>Foto original</h2>
          </div>
          <div class="image-box">
            <img id="captured" alt="Foto capturada">
          </div>
        </div>
      </div>

      <div class="panel">
        <div class="stack">
          <div>
            <h2>Foto procesada</h2>
          </div>
          <div class="image-box">
            <canvas id="processed"></canvas>
          </div>
        </div>
      </div>
    </div>
  </div>

  <script>
    const streamImg = document.getElementById("stream");
    const capturedImg = document.getElementById("captured");
    const processedCanvas = document.getElementById("processed");
    const processBtn = document.getElementById("processBtn");
    const captureBtn = document.getElementById("captureBtn");
    const operationSelect = document.getElementById("operation");
    const thresholdInput = document.getElementById("threshold");
    const kernelSelect = document.getElementById("kernel");
    const statusLabel = document.getElementById("status");
    const refreshStatusBtn = document.getElementById("refreshStatusBtn");
    const cameraControls = [
      "framesize", "quality", "brightness", "contrast", "saturation",
      "ae_level", "awb", "agc", "aec", "vflip", "hmirror", "special_effect"
    ];

    function getStreamUrl() {
      // Si estamos usando Ngrok, el host no tendrá puertos y manejará el flujo directo
      if (window.location.hostname.includes("ngrok")) {
        return window.location.protocol + "//" + window.location.host + "/stream";
      }
      // Si estamos en red local, mantiene tu lógica original del puerto + 1 (Puerto 81)
      const streamPort = Number(window.location.port || 80) + 1;
      return window.location.protocol + "//" + window.location.hostname + ":" + streamPort + "/stream";
    }

    function setStatus(text) {
      statusLabel.textContent = text;
    }

    async function setCameraVar(variable, value) {
      const response = await fetch(`/control?var=${encodeURIComponent(variable)}&val=${encodeURIComponent(value)}`);
      if (!response.ok) {
        throw new Error("No se pudo aplicar " + variable);
      }
    }

    async function loadCameraStatus() {
      try {
        const response = await fetch("/status");
        const data = await response.json();
        cameraControls.forEach((id) => {
          const el = document.getElementById(id);
          if (el && Object.prototype.hasOwnProperty.call(data, id)) {
            el.value = String(data[id]);
          }
        });
        setStatus("Ajustes de camara cargados.");
      } catch (error) {
        setStatus("No se pudieron leer los ajustes.");
      }
    }

    function imageToBinary(imageData, threshold) {
      const src = imageData.data;
      const binary = new Uint8Array(imageData.width * imageData.height);
      for (let i = 0, p = 0; i < src.length; i += 4, p++) {
        const gray = Math.round(src[i] * 0.299 + src[i + 1] * 0.587 + src[i + 2] * 0.114);
        binary[p] = gray >= threshold ? 255 : 0;
      }
      return binary;
    }

    function erode(binary, width, height, kernelSize) {
      const radius = Math.floor(kernelSize / 2);
      const out = new Uint8Array(binary.length);
      for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
          let keep = 255;
          for (let ky = -radius; ky <= radius && keep; ky++) {
            for (let kx = -radius; kx <= radius; kx++) {
              const nx = x + kx;
              const ny = y + ky;
              if (nx < 0 || ny < 0 || nx >= width || ny >= height || binary[ny * width + nx] === 0) {
                keep = 0;
                break;
              }
            }
          }
          out[y * width + x] = keep;
        }
      }
      return out;
    }

    function dilate(binary, width, height, kernelSize) {
      const radius = Math.floor(kernelSize / 2);
      const out = new Uint8Array(binary.length);
      for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
          let fill = 0;
          for (let ky = -radius; ky <= radius && !fill; ky++) {
            for (let kx = -radius; kx <= radius; kx++) {
              const nx = x + kx;
              const ny = y + ky;
              if (nx >= 0 && ny >= 0 && nx < width && ny < height && binary[ny * width + nx] === 255) {
                fill = 255;
                break;
              }
            }
          }
          out[y * width + x] = fill;
        }
      }
      return out;
    }

    function applyMorphology(binary, width, height, operation, kernelSize) {
      if (operation === "erode") return erode(binary, width, height, kernelSize);
      if (operation === "dilate") return dilate(binary, width, height, kernelSize);
      if (operation === "open") return dilate(erode(binary, width, height, kernelSize), width, height, kernelSize);
      if (operation === "close") return erode(dilate(binary, width, height, kernelSize), width, height, kernelSize);
      return binary;
    }

    function drawBinary(binary, width, height) {
      processedCanvas.width = width;
      processedCanvas.height = height;
      const ctx = processedCanvas.getContext("2d");
      const output = ctx.createImageData(width, height);
      for (let i = 0, p = 0; p < binary.length; i += 4, p++) {
        output.data[i] = binary[p];
        output.data[i + 1] = binary[p];
        output.data[i + 2] = binary[p];
        output.data[i + 3] = 255;
      }
      ctx.putImageData(output, 0, 0);
    }

    function processCurrentImage() {
      if (!capturedImg.complete || !capturedImg.naturalWidth) {
        setStatus("Primero toma una foto.");
        return;
      }

      const width = capturedImg.naturalWidth;
      const height = capturedImg.naturalHeight;
      const tempCanvas = document.createElement("canvas");
      tempCanvas.width = width;
      tempCanvas.height = height;
      const tempCtx = tempCanvas.getContext("2d");
      tempCtx.drawImage(capturedImg, 0, 0, width, height);

      const threshold = Number(thresholdInput.value);
      const kernelSize = Number(kernelSelect.value);
      const operation = operationSelect.value;
      const imageData = tempCtx.getImageData(0, 0, width, height);
      const binary = imageToBinary(imageData, threshold);
      const result = applyMorphology(binary, width, height, operation, kernelSize);
      drawBinary(result, width, height);
      setStatus("Procesamiento aplicado: " + operation + ".");
    }

    async function captureImage() {
      setStatus("Tomando foto...");
      const url = "/capture?_ts=" + Date.now();
      capturedImg.onload = () => {
        processCurrentImage();
      };
      capturedImg.onerror = () => {
        setStatus("No se pudo cargar la foto.");
      };
      capturedImg.src = url;
    }

    async function onCameraControlChange(event) {
      const element = event.target;
      try {
        await setCameraVar(element.id, element.value);
        setStatus("Ajuste aplicado: " + element.id + ".");
        if (element.id === "framesize") {
          streamImg.src = getStreamUrl() + "?_ts=" + Date.now();
        }
      } catch (error) {
        setStatus(error.message);
      }
    }

    streamImg.src = getStreamUrl();
    captureBtn.addEventListener("click", captureImage);
    processBtn.addEventListener("click", processCurrentImage);
    operationSelect.addEventListener("change", processCurrentImage);
    thresholdInput.addEventListener("input", processCurrentImage);
    kernelSelect.addEventListener("change", processCurrentImage);
    refreshStatusBtn.addEventListener("click", loadCameraStatus);
    cameraControls.forEach((id) => {
      const el = document.getElementById(id);
      if (!el) return;
      el.addEventListener("change", onCameraControlChange);
      if (el.tagName === "INPUT") {
        el.addEventListener("mouseup", onCameraControlChange);
        el.addEventListener("touchend", onCameraControlChange);
      }
    });
    loadCameraStatus();
  </script>
</body>
</html>
)rawliteral";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

typedef struct {
  size_t size;   //number of values used for filtering
  size_t index;  //current value index
  size_t count;  //value count
  int sum;
  int *values;  //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values) {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
static int ra_filter_run(ra_filter_t *filter, int value) {
  if (!filter->values) {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
    filter->count++;
  }
  return filter->sum / filter->count;
}
#endif

static esp_err_t bmp_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_start = esp_timer_get_time();
#endif
  fb = esp_camera_fb_get();
  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/x-windows-bmp");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

  uint8_t *buf = NULL;
  size_t buf_len = 0;
  bool converted = frame2bmp(fb, &buf, &buf_len);
  esp_camera_fb_return(fb);
  if (!converted) {
    log_e("BMP Conversion failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  res = httpd_resp_send(req, (const char *)buf, buf_len);
  free(buf);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_end = esp_timer_get_time();
#endif
  log_i("BMP: %llums, %uB", (uint64_t)((fr_end - fr_start) / 1000), buf_len);
  return res;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_start = esp_timer_get_time();
#endif

  fb = esp_camera_fb_get();

  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  size_t fb_len = 0;
#endif
  if (fb->format == PIXFORMAT_JPEG) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = fb->len;
#endif
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    jpg_chunking_t jchunk = {req, 0};
    res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = jchunk.len;
#endif
  }
  esp_camera_fb_return(fb);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_end = esp_timer_get_time();
#endif
  log_i("JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[128];

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      log_e("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          log_e("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      log_e("Send frame failed");
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;

    frame_time /= 1000;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
#endif
    log_i(
      "MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)", (uint32_t)(_jpg_buf_len), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, avg_frame_time,
      1000.0 / avg_frame_time
    );
  }

  return res;
}

static esp_err_t parse_get(httpd_req_t *req, char **obuf) {
  char *buf = NULL;
  size_t buf_len = 0;

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      *obuf = buf;
      return ESP_OK;
    }
    free(buf);
  }
  httpd_resp_send_404(req);
  return ESP_FAIL;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf = NULL;
  char variable[32];
  char value[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK || httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int val = atoi(value);
  log_i("%s = %d", variable, val);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(variable, "framesize")) {
    if (s->pixformat == PIXFORMAT_JPEG) {
      res = s->set_framesize(s, (framesize_t)val);
    }
  } else if (!strcmp(variable, "quality")) {
    res = s->set_quality(s, val);
  } else if (!strcmp(variable, "contrast")) {
    res = s->set_contrast(s, val);
  } else if (!strcmp(variable, "brightness")) {
    res = s->set_brightness(s, val);
  } else if (!strcmp(variable, "saturation")) {
    res = s->set_saturation(s, val);
  } else if (!strcmp(variable, "gainceiling")) {
    res = s->set_gainceiling(s, (gainceiling_t)val);
  } else if (!strcmp(variable, "colorbar")) {
    res = s->set_colorbar(s, val);
  } else if (!strcmp(variable, "awb")) {
    res = s->set_whitebal(s, val);
  } else if (!strcmp(variable, "agc")) {
    res = s->set_gain_ctrl(s, val);
  } else if (!strcmp(variable, "aec")) {
    res = s->set_exposure_ctrl(s, val);
  } else if (!strcmp(variable, "hmirror")) {
    res = s->set_hmirror(s, val);
  } else if (!strcmp(variable, "vflip")) {
    res = s->set_vflip(s, val);
  } else if (!strcmp(variable, "awb_gain")) {
    res = s->set_awb_gain(s, val);
  } else if (!strcmp(variable, "agc_gain")) {
    res = s->set_agc_gain(s, val);
  } else if (!strcmp(variable, "aec_value")) {
    res = s->set_aec_value(s, val);
  } else if (!strcmp(variable, "aec2")) {
    res = s->set_aec2(s, val);
  } else if (!strcmp(variable, "dcw")) {
    res = s->set_dcw(s, val);
  } else if (!strcmp(variable, "bpc")) {
    res = s->set_bpc(s, val);
  } else if (!strcmp(variable, "wpc")) {
    res = s->set_wpc(s, val);
  } else if (!strcmp(variable, "raw_gma")) {
    res = s->set_raw_gma(s, val);
  } else if (!strcmp(variable, "lenc")) {
    res = s->set_lenc(s, val);
  } else if (!strcmp(variable, "special_effect")) {
    res = s->set_special_effect(s, val);
  } else if (!strcmp(variable, "wb_mode")) {
    res = s->set_wb_mode(s, val);
  } else if (!strcmp(variable, "ae_level")) {
    res = s->set_ae_level(s, val);
  }
  else {
    log_i("Unknown command: %s", variable);
    res = -1;
  }

  if (res < 0) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static int print_reg(char *p, sensor_t *s, uint16_t reg, uint32_t mask) {
  return sprintf(p, "\"0x%x\":%u,", reg, s->get_reg(s, reg, mask));
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];

  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';

  if (s->id.PID == OV5640_PID || s->id.PID == OV3660_PID) {
    for (int reg = 0x3400; reg < 0x3406; reg += 2) {
      p += print_reg(p, s, reg, 0xFFF);  //12 bit
    }
    p += print_reg(p, s, 0x3406, 0xFF);

    p += print_reg(p, s, 0x3500, 0xFFFF0);  //16 bit
    p += print_reg(p, s, 0x3503, 0xFF);
    p += print_reg(p, s, 0x350a, 0x3FF);   //10 bit
    p += print_reg(p, s, 0x350c, 0xFFFF);  //16 bit

    for (int reg = 0x5480; reg <= 0x5490; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }

    for (int reg = 0x5380; reg <= 0x538b; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }

    for (int reg = 0x5580; reg < 0x558a; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }
    p += print_reg(p, s, 0x558a, 0x1FF);  //9 bit
  } else if (s->id.PID == OV2640_PID) {
    p += print_reg(p, s, 0xd3, 0xFF);
    p += print_reg(p, s, 0x111, 0xFF);
    p += print_reg(p, s, 0x132, 0xFF);
  }

  p += sprintf(p, "\"xclk\":%u,", s->xclk_freq_hz / 1000000);
  p += sprintf(p, "\"pixformat\":%u,", s->pixformat);
  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
  p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
  p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
  p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
  p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
  p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
  p += sprintf(p, "\"awb\":%u,", s->status.awb);
  p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
  p += sprintf(p, "\"aec\":%u,", s->status.aec);
  p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
  p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
  p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
  p += sprintf(p, "\"agc\":%u,", s->status.agc);
  p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
  p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
  p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
  p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
  p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
  p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
  p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
  p += sprintf(p, "\"vflip\":%u,", s->status.vflip);
  p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
  p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
  p += sprintf(p, ",\"led_intensity\":%d", -1);
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t xclk_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _xclk[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "xclk", _xclk, sizeof(_xclk)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int xclk = atoi(_xclk);
  log_i("Set XCLK: %d MHz", xclk);

  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_xclk(s, LEDC_TIMER_0, xclk);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t reg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];
  char _val[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK
      || httpd_query_key_value(buf, "val", _val, sizeof(_val)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);
  int mask = atoi(_mask);
  int val = atoi(_val);
  log_i("Set Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, val);

  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_reg(s, reg, mask, val);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t greg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);
  int mask = atoi(_mask);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->get_reg(s, reg, mask);
  if (res < 0) {
    return httpd_resp_send_500(req);
  }
  log_i("Get Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, res);

  char buffer[20];
  const char *val = itoa(res, buffer, 10);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, val, strlen(val));
}

static int parse_get_var(char *buf, const char *key, int def) {
  char _int[16];
  if (httpd_query_key_value(buf, key, _int, sizeof(_int)) != ESP_OK) {
    return def;
  }
  return atoi(_int);
}

static esp_err_t pll_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }

  int bypass = parse_get_var(buf, "bypass", 0);
  int mul = parse_get_var(buf, "mul", 0);
  int sys = parse_get_var(buf, "sys", 0);
  int root = parse_get_var(buf, "root", 0);
  int pre = parse_get_var(buf, "pre", 0);
  int seld5 = parse_get_var(buf, "seld5", 0);
  int pclken = parse_get_var(buf, "pclken", 0);
  int pclk = parse_get_var(buf, "pclk", 0);
  free(buf);

  log_i("Set Pll: bypass: %d, mul: %d, sys: %d, root: %d, pre: %d, seld5: %d, pclken: %d, pclk: %d", bypass, mul, sys, root, pre, seld5, pclken, pclk);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_pll(s, bypass, mul, sys, root, pre, seld5, pclken, pclk);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t win_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }

  int startX = parse_get_var(buf, "sx", 0);
  int startY = parse_get_var(buf, "sy", 0);
  int endX = parse_get_var(buf, "ex", 0);
  int endY = parse_get_var(buf, "ey", 0);
  int offsetX = parse_get_var(buf, "offx", 0);
  int offsetY = parse_get_var(buf, "offy", 0);
  int totalX = parse_get_var(buf, "tx", 0);
  int totalY = parse_get_var(buf, "ty", 0);  // codespell:ignore totaly
  int outputX = parse_get_var(buf, "ox", 0);
  int outputY = parse_get_var(buf, "oy", 0);
  bool scale = parse_get_var(buf, "scale", 0) == 1;
  bool binning = parse_get_var(buf, "binning", 0) == 1;
  free(buf);

  log_i(
    "Set Window: Start: %d %d, End: %d %d, Offset: %d %d, Total: %d %d, Output: %d %d, Scale: %u, Binning: %u", startX, startY, endX, endY, offsetX, offsetY,
    totalX, totalY, outputX, outputY, scale, binning  // codespell:ignore totaly
  );
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);  // codespell:ignore totaly
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    if (s->id.PID == OV3660_PID) {
      return httpd_resp_send(req, (const char *)index_ov3660_html_gz, index_ov3660_html_gz_len);
    } else if (s->id.PID == OV5640_PID) {
      return httpd_resp_send(req, (const char *)index_ov5640_html_gz, index_ov5640_html_gz_len);
    } else {
      return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
    }
  } else {
    log_e("Camera sensor not found");
    return httpd_resp_send_500(req);
  }
}

static esp_err_t morph_index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, _MORPH_INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 16;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t morph_index_uri = {
    .uri = "/morph",
    .method = HTTP_GET,
    .handler = morph_index_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t bmp_uri = {
    .uri = "/bmp",
    .method = HTTP_GET,
    .handler = bmp_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t xclk_uri = {
    .uri = "/xclk",
    .method = HTTP_GET,
    .handler = xclk_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t reg_uri = {
    .uri = "/reg",
    .method = HTTP_GET,
    .handler = reg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t greg_uri = {
    .uri = "/greg",
    .method = HTTP_GET,
    .handler = greg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t pll_uri = {
    .uri = "/pll",
    .method = HTTP_GET,
    .handler = pll_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t win_uri = {
    .uri = "/resolution",
    .method = HTTP_GET,
    .handler = win_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  ra_filter_init(&ra_filter, 20);

  log_i("Starting web server on port: '%d'", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &morph_index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &bmp_uri);

    httpd_register_uri_handler(camera_httpd, &stream_uri);

    httpd_register_uri_handler(camera_httpd, &xclk_uri);
    httpd_register_uri_handler(camera_httpd, &reg_uri);
    httpd_register_uri_handler(camera_httpd, &greg_uri);
    httpd_register_uri_handler(camera_httpd, &pll_uri);
    httpd_register_uri_handler(camera_httpd, &win_uri);
    
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  log_i("Starting stream server on port: '%d'", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setupLedFlash() {
  log_i("LED flash is disabled -> LED_GPIO_NUM undefined");
}
