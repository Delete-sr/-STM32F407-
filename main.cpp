  #include <Arduino.h>
  #include "esp_camera.h"
  #include <WiFi.h>
  #include <WebServer.h>
  #include <HTTPClient.h>
  #include <WiFiClientSecure.h>   // 用于 HTTPS Server酱推送走 https

  // =========================
  // WiFi
  // =========================
  const char* ssid = "Sakura";
  const char* password = "11111111";

  // 电脑 Flask 服务地址 改成你电脑的局域网IP ESP32 把图发到这里做识别
  const char* PC_BASE_URL = "http://172.20.10.5:5000";

  // =========================
  // Server酱推送 缺货时往微信推消息
  // =========================
  const char* SERVERCHAN_SENDKEY = "SCT354001TiPQ5GCcs0DN3WuTnvw7fBnao";

  // =========================
  // STM32 UART 与底盘主控通信的串口
  // =========================
  #define STM32_UART_RX_PIN  20
  #define STM32_UART_TX_PIN  19
  HardwareSerial STM32Serial(1);   // 使用 ESP32 的 UART1 不占用调试用的 UART0

  // =========================
  // 摄像头接线 各数据/控制引脚到 GPIO 的映射 按模块实际接线填写
  // =========================
  #define PWDN_GPIO_NUM      -1     // -1 表示该引脚未使用
  #define RESET_GPIO_NUM     -1
  #define XCLK_GPIO_NUM      -1
  #define SIOD_GPIO_NUM       4
  #define SIOC_GPIO_NUM       5

  #define Y2_GPIO_NUM        11
  #define Y3_GPIO_NUM         9
  #define Y4_GPIO_NUM         8
  #define Y5_GPIO_NUM        10
  #define Y6_GPIO_NUM        12
  #define Y7_GPIO_NUM        18
  #define Y8_GPIO_NUM        17
  #define Y9_GPIO_NUM        16
  #define VSYNC_GPIO_NUM      6
  #define HREF_GPIO_NUM       7
  #define PCLK_GPIO_NUM      13

  WebServer server(80);   // ESP32 自身的 Web 服务 用于查看状态/结果

  // =========================
  // 参数
  // =========================
  const unsigned long SCAN_INTERVAL_MS = 100;         // 扫码节流 两次扫码最小间隔
  const unsigned long WAIT_DETECT_TIMEOUT_MS = 10000; // 扫到码后等 STM32 下发检测指令的超时
  const unsigned long IGNORE_SHELF_MS = 10000;        // 检测完成后 同一货架的冷却忽略时间
  const unsigned long DETECT_IGNORE_MS = 5000;        // 检测失败后 同一货架的冷却忽略时间

  // =========================
  // 工作状态机
  // =========================
  enum WorkState {
    STATE_SCAN_QR = 0,        // 持续扫码找货架
    STATE_WAIT_DETECT,        // 已扫到码 等待 STM32 下发 DETECT 指令
    STATE_LIFTING_FOR_L2      // 第一层完成 正在上升找第二层
  };

  WorkState g_state = STATE_SCAN_QR;

  String currentShelf = "";          // 当前正在处理的货架
  String latestShelf = "";           // 最近一次检测的货架 供 /result 查询
  String latestCounts = "";          // 最近一次检测结果 "box=3,sack=2,bucket=1"

  unsigned long lastScanMs = 0;          // 上次扫码时刻 配合 SCAN_INTERVAL_MS 节流
  unsigned long waitDetectStartMs = 0;   // 进入等待检测状态的时刻 用于超时判断

  // 冷却机制 刚处理完的货架在一段时间内忽略 避免对同一货架重复触发
  String ignoreShelf = "";
  unsigned long ignoreShelfUntilMs = 0;

  // 串口接收缓冲 按行 \n 或 \r 拼出完整指令
  String stm32Line = "";

  // 多层货架升降相关
  String layer1Shelf = "";               // 第一层已扫的货架号 上升时忽略它
  unsigned long liftScanLastMs = 0;
  const unsigned long LIFT_SCAN_INTERVAL_MS = 300;  // 上升时扫码间隔
  bool g_lowering = false;               // 正在下降归位中

  // =========================
  // 工具函数
  // =========================
  String makeUrl(const char* path) {
    // 拼出完整请求地址 如 http://172.20.10.5:5000/detect
    return String(PC_BASE_URL) + String(path);
  }

  bool shouldIgnoreShelf(const String& shelf) {
    // 该货架是否处于冷却期内 在冷却期则跳过 避免重复处理
    return (ignoreShelf == shelf && millis() < ignoreShelfUntilMs);
  }

  void markIgnoreShelf(const String& shelf, unsigned long ms) {
    // 把某货架标记为冷却 持续 ms 毫秒
    ignoreShelf = shelf;
    ignoreShelfUntilMs = millis() + ms;
  }

  void uartSendLine(const String& s) {
    // 向 STM32 发送一行指令 以 \n 结尾 并在调试串口回显
    STM32Serial.print(s);
    STM32Serial.print("\n");
    Serial.print("[UART->STM32] ");
    Serial.println(s);
  }

  String urlEncode(const String& str) {
    // URL 编码 把非安全字符转成 %XX 用于 Server酱 推送的表单内容 含中文
    String encoded = "";
    char c;
    char buf[4];

    for (size_t i = 0; i < str.length(); i++) {
      c = str.charAt(i);

      // 字母、数字和 -_.~ 不需要编码
      if ((c >= 'a' && c <= 'z') ||
          (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') ||
          c == '-' || c == '_' || c == '.' || c == '~') {
        encoded += c;
      } else if (c == ' ') {
        encoded += "%20";
      } else if (c == '\n') {
        encoded += "%0A";          // 换行编码 Server酱用 \n\n 分段
      } else if (c == '\r') {
        // 回车直接丢弃
      } else {
        // 其余字符 含中文 UTF-8 字节 按字节转 %XX
        sprintf(buf, "%%%02X", (unsigned char)c);
        encoded += buf;
      }
    }

    return encoded;
  }

  String countsToReadable(const String& countsStr) {
    if (countsStr.length() == 0) return "无物品";
    String result = "";
    String temp = countsStr;
    int pos = 0;
    while ((pos = temp.indexOf(',')) > 0) {
      String pair = temp.substring(0, pos);
      pair.trim();
      pair.replace("=", "  ");
      pair += "个";
      result += pair + "\n";
      temp = temp.substring(pos + 1);
    }
    temp.trim();
    temp.replace("=", "  ");
    temp += "个";
    result += temp;
    return result;
  }

  bool initCamera() {
    // 初始化摄像头 填充引脚配置 按是否有 PSRAM 选择不同的缓冲策略
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    // 8 位并口数据线 D0~D7
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    // 时钟与同步信号
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;

    // SCCB 类 I2C 控制总线
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn  = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;          // 20MHz 主时钟
    config.pixel_format = PIXFORMAT_JPEG;    // 直接输出 JPEG 省内存、方便上传

    // 有 PSRAM 时可用双缓冲、抓最新帧 无 PSRAM 时降级为单缓冲
    bool hasPsram = psramFound();
    if (hasPsram) {
      config.frame_size   = FRAMESIZE_SVGA;   // 800x600
      config.jpeg_quality = 12;               // 数值越小画质越高
      config.fb_count     = 2;                // 双帧缓冲
      config.fb_location  = CAMERA_FB_IN_PSRAM;
      config.grab_mode    = CAMERA_GRAB_LATEST;     // 总是取最新帧 降低延迟
      Serial.println("[CAM] use PSRAM, VGA(640x480), fb_count=2");
    } else {
      config.frame_size   = FRAMESIZE_SVGA;
      config.jpeg_quality = 12;
      config.fb_count     = 1;                // 单帧缓冲
      config.fb_location  = CAMERA_FB_IN_DRAM;
      config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY; // 缓冲空了才抓新帧
      Serial.println("[CAM] no PSRAM, VGA(640x480), fb_count=1");
    }

    Serial.println("[CAM] init...");
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("[CAM] init failed: 0x%x\n", err);
      return false;
    }

    // 初始化后再对传感器做画质微调
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
      s->set_framesize(s, FRAMESIZE_SVGA);
      s->set_quality(s, 10);
      s->set_brightness(s, 0);
      s->set_contrast(s, 1);
      s->set_saturation(s, 0);
      s->set_sharpness(s, 2);   // 提高锐度 利于二维码识别
    }

    Serial.println("[CAM] init ok");
    return true;
  }

  camera_fb_t* captureFrame() {
    // 抓取一帧 返回帧缓冲指针 用完必须 esp_camera_fb_return 归还
    return esp_camera_fb_get();
  }

  bool postImageMultipart(const String& url, camera_fb_t* fb, const String& shelfId, String& responseText) {
    // 以 multipart/form-data 把图片 和可选的 shelf_id POST 给 Flask
    if (!fb) return false;

    String boundary = "----ESP32Boundary7MA4YWxkTrZu0gW";   // 分隔各表单字段的边界串
    String head = "";
    String tail = "";

    // 可选字段 shelf_id
    if (shelfId.length() > 0) {
      head += "--" + boundary + "\r\n";
      head += "Content-Disposition: form-data; name=\"shelf_id\"\r\n\r\n";
      head += shelfId + "\r\n";
    }

    // 图片字段头 紧跟其后的是 JPEG 二进制数据
    head += "--" + boundary + "\r\n";
    head += "Content-Disposition: form-data; name=\"image\"; filename=\"frame.jpg\"\r\n";
    head += "Content-Type: image/jpeg\r\n\r\n";

    tail += "\r\n--" + boundary + "--\r\n";   // 结束边界

    // 在堆上拼出完整请求体 head + 图片字节 + tail
    size_t totalLen = head.length() + fb->len + tail.length();
    uint8_t* payload = (uint8_t*)malloc(totalLen);
    if (!payload) {
      Serial.println("[HTTP] malloc failed");
      return false;
    }

    size_t offset = 0;
    memcpy(payload + offset, head.c_str(), head.length());
    offset += head.length();

    memcpy(payload + offset, fb->buf, fb->len);   // 图片二进制原样拷贝
    offset += fb->len;

    memcpy(payload + offset, tail.c_str(), tail.length());
    offset += tail.length();

    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http.setTimeout(10000);

    int code = http.POST(payload, totalLen);
    free(payload);   // 发完立即释放 避免内存泄漏

    if (code > 0) {
      responseText = http.getString();
      Serial.printf("[HTTP] POST %s -> %d, resp=%s\n", url.c_str(), code, responseText.c_str());
      http.end();
      return code == 200;
    } else {
      Serial.printf("[HTTP] POST %s failed: %s\n", url.c_str(), http.errorToString(code).c_str());
      http.end();
      return false;
    }
  }

  // 新格式 RESULT:shelf:cls1=n1,cls2=n2
  bool parseDetectResult(const String& resp, String& shelf, String& countsStr) {
    if (!resp.startsWith("RESULT:")) return false;

    int p1 = resp.indexOf(':');
    int p2 = resp.indexOf(':', p1 + 1);

    if (p1 < 0 || p2 < 0) return false;   // 格式不完整

    shelf = resp.substring(p1 + 1, p2);
    countsStr = resp.substring(p2 + 1);

    shelf.trim();
    countsStr.trim();

    return true;
  }

  // 每个货架都推送微信 列出所有物品
  bool pushWechatReport(const String& shelf, const String& countsStr) {
    if (strlen(SERVERCHAN_SENDKEY) == 0) {
      Serial.println("[WX] SERVERCHAN_SENDKEY empty, skip push");
      return false;
    }

    WiFiClientSecure client;
    client.setInsecure();   // 跳过证书校验 简化处理 生产环境不建议

    HTTPClient http;
    String url = "https://sctapi.ftqq.com/" + String(SERVERCHAN_SENDKEY) + ".send";

    if (!http.begin(client, url)) {
      Serial.println("[WX] begin failed");
      return false;
    }

    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String title = "Shelf Report";
    String desp  = "货架 " + shelf + "\n\n"
                   + countsToReadable(countsStr);

    String body = "title=" + urlEncode(title) + "&desp=" + urlEncode(desp);

    int code = http.POST(body);
    String resp = http.getString();

    Serial.printf("[WX] code=%d resp=%s\n", code, resp.c_str());

    http.end();
    return code == 200;
  }

  void resetStateAll() {
    // 全量复位状态机和所有缓存 新一趟巡检或收到 RESET 时调用
    g_state = STATE_SCAN_QR;
    currentShelf = "";
    latestShelf = "";
    latestCounts = "";
    ignoreShelf = "";
    ignoreShelfUntilMs = 0;
    stm32Line = "";
    layer1Shelf = "";
    g_lowering = false;
  }

  void backToScanAfterShelf(const String& shelf, unsigned long ignoreMs) {
    // 处理完 或失败 后回到扫码状态 并把该货架设为冷却 避免立即重复触发
    currentShelf = "";
    g_state = STATE_SCAN_QR;
    if (shelf.length() > 0) {
      markIgnoreShelf(shelf, ignoreMs);
    }
  }

  void handleRoot() {
    // ESP32 自带网页首页 提供 /status 和 /result 链接
    String html =
      "<!doctype html><html><head><meta charset='utf-8'><title>ESP32 Auto QR Detect</title></head>"
      "<body style='font-family:Arial'>"
      "<h2>ESP32 Auto QR + Detect Bridge</h2>"
      "<p><a href='/status'>/status</a></p>"
      "<p><a href='/result'>/result</a></p>"
      "</body></html>";
    server.send(200, "text/html", html);
  }

  void handleStatus() {
    // 返回运行状态 JSON IP、状态机、当前货架、信号强度、剩余内存等
    String json = "{";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"state\":" + String((int)g_state) + ",";
    json += "\"current_shelf\":\"" + currentShelf + "\",";
    json += "\"latest_shelf\":\"" + latestShelf + "\",";
    json += "\"latest_counts\":\"" + latestCounts + "\",";
    json += "\"ignore_shelf\":\"" + ignoreShelf + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    json += "\"heap\":" + String(ESP.getFreeHeap());
    json += "}";
    server.send(200, "application/json", json);
  }

  void handleResult() {
    // 返回最近一次检测结果 JSON
    String json = "{";
    json += "\"shelf\":\"" + latestShelf + "\",";
    json += "\"counts\":\"" + latestCounts + "\"";
    json += "}";
    server.send(200, "application/json", json);
  }

  void handleReset() {
    // HTTP 触发复位
    resetStateAll();
    server.send(200, "text/plain", "OK");
  }

  void processDetectCommand(const String& shelf) {
    // 收到检测指令后的完整流程 拍图 -> 上传识别 -> 解析 -> 回传 STM32 -> 微信推送
    camera_fb_t* fb = captureFrame();
    if (!fb) {
      Serial.println("[DETECT] capture failed");
      uartSendLine("DETECT_FAIL:" + shelf);
      backToScanAfterShelf(shelf, DETECT_IGNORE_MS);
      return;
    }

    String resp;
    bool ok = postImageMultipart(makeUrl("/detect"), fb, shelf, resp);
    esp_camera_fb_return(fb);   // 帧缓冲用完立即归还

    if (!ok) {
      Serial.println("[DETECT] HTTP failed");
      uartSendLine("DETECT_FAIL:" + shelf);
      backToScanAfterShelf(shelf, DETECT_IGNORE_MS);
      return;
    }

    resp.trim();

    // 解析识别返回 新格式 RESULT:shelf:cls1=n1,cls2=n2
    String parsedShelf, parsedCounts;
    bool parseOk = parseDetectResult(resp, parsedShelf, parsedCounts);

    if (!parseOk) {
      Serial.println("[DETECT] bad response format");
      uartSendLine("DETECT_FAIL:" + shelf);
      backToScanAfterShelf(shelf, DETECT_IGNORE_MS);
      return;
    }

    // 缓存结果
    latestShelf = parsedShelf;
    latestCounts = parsedCounts;

    uartSendLine(resp);   // 原样把结果转发给 STM32

    // 每个货架都推送微信报告
    pushWechatReport(parsedShelf, parsedCounts);

    // 第一层处理完成 → 进入上升找第二层模式
    layer1Shelf = parsedShelf;
    g_state = STATE_LIFTING_FOR_L2;
    liftScanLastMs = 0;
    waitDetectStartMs = millis();            // 记录开始时间 用于超时判断
    uartSendLine("LIFT_START");              // 通知 STM32 开始上升
  }

  void processStm32Command(const String& line) {
    // 解析 STM32 发来的一行指令
    Serial.print("[UART<-STM32] ");
    Serial.println(line);

    // DETECT:货架号 -> 触发检测
    if (line.startsWith("DETECT:")) {
      String shelf = line.substring(strlen("DETECT:"));
      shelf.trim();

      if (shelf.length() == 0) {
        uartSendLine("DETECT_FAIL:UNKNOWN");
        backToScanAfterShelf(currentShelf, DETECT_IGNORE_MS);
        return;
      }

      processDetectCommand(shelf);
      return;
    }

    // RESET -> 全量复位
    if (line == "RESET") {
      resetStateAll();
      return;
    }
  }

  void readStm32Uart() {
    // 读串口 按 \n / \r 分行 拼出完整指令后交给 processStm32Command
    while (STM32Serial.available()) {
      char c = (char)STM32Serial.read();
      if (c == '\n' || c == '\r') {
        if (stm32Line.length() > 0) {
          processStm32Command(stm32Line);
          stm32Line = "";
        }
      } else {
        stm32Line += c;
        // 防止异常数据撑爆缓冲 超长则丢弃重来
        if (stm32Line.length() > 120) {
          stm32Line = "";
        }
      }
    }
  }

  void scanQrTask () {
    // 扫码任务 仅在扫码态执行 按间隔拍图发给 /scan_qr
    if (g_state != STATE_SCAN_QR) return;
    if (g_lowering) return;                  // 下降归位时不扫新货架
    if (millis() - lastScanMs < SCAN_INTERVAL_MS) return;   // 节流

    lastScanMs = millis();

    camera_fb_t* fb = captureFrame();
    if (!fb) {
      Serial.println("[SCAN] capture failed");
      return;
    }

    String resp;
    bool ok = postImageMultipart(makeUrl("/scan_qr"), fb, "", resp);   // 扫码不带 shelf_id
    esp_camera_fb_return(fb);

    if (!ok) {
      Serial.println("[SCAN] post failed");
      return;
    }

    resp.trim();

    // Flask 返回 FOUND:货架号:Y=0.xx 表示二维码已对正 (含纵向位置)
    if (resp.startsWith("FOUND:")) {
      String payload = resp.substring(strlen("FOUND:"));
      // 解析 Y 坐标 (新格式: SHELF_A1:Y=0.52)
      int yPos = payload.indexOf(":Y=");
      if (yPos > 0) {
        payload = payload.substring(0, yPos);   // 只取货架号部分
      }
      String shelf = payload;
      shelf.trim();

      if (shelf.length() == 0) return;

      // 刚处理过的货架在冷却期内 跳过
      if (shouldIgnoreShelf(shelf)) {
        Serial.println("[SCAN] same shelf ignored by cooldown");
        return;
      }

      // 锁定货架 切到等待检测态 并记开始时间用于超时
      currentShelf = shelf;
      g_state = STATE_WAIT_DETECT;
      waitDetectStartMs = millis();

      uartSendLine("QR_FOUND:" + shelf);   // 通知 STM32 已找到货架
    }
  }

  void liftScanTask() {
    // 上升找第二层任务 仅在 STATE_LIFTING_FOR_L2 状态执行
    if (g_state != STATE_LIFTING_FOR_L2) return;
    if (millis() - liftScanLastMs < LIFT_SCAN_INTERVAL_MS) return;

    liftScanLastMs = millis();

    camera_fb_t* fb = captureFrame();
    if (!fb) return;

    String resp;
    bool ok = postImageMultipart(makeUrl("/scan_qr?region=upper"), fb, "", resp);
    esp_camera_fb_return(fb);

    if (!ok) return;
    resp.trim();

    // 上升时只看 Y 轴居中，不管是 FOUND 还是 OFFCENTER
    String shelf;
    float yRatio = 0.0;

    if (resp.startsWith("FOUND:")) {
      String payload = resp.substring(strlen("FOUND:"));
      int yPos = payload.indexOf(":Y=");
      if (yPos > 0) {
        shelf = payload.substring(0, yPos);
        yRatio = payload.substring(yPos + 3).toFloat();
      }
    } else if (resp.startsWith("OFFCENTER:")) {
      String payload = resp.substring(strlen("OFFCENTER:"));
      int yPos = payload.indexOf(":Y=");
      if (yPos > 0) {
        shelf = payload.substring(0, yPos);
        yRatio = payload.substring(yPos + 3).toFloat();
      }
    } else {
      return;
    }

    shelf.trim();
    if (shelf.length() == 0) return;

    // 忽略第一层已经扫过的货架(同一个二维码正在远离)
    if (shelf == layer1Shelf) return;

    // 忽略冷却期内的货架
    if (shouldIgnoreShelf(shelf)) return;

    // 只看 Y 轴居中 (上半画面中间)
    if (yRatio < 0.09 || yRatio > 0.25) {
      Serial.printf("[LIFT] shelf %s Y=%.2f not centered\n",
                    shelf.c_str(), yRatio);
      return;
    }

    // 第二层 Y 轴居中! 停止上升
    Serial.printf("[LIFT] layer 2 found: %s at Y=%.2f\n", shelf.c_str(), yRatio);
    uartSendLine("LIFT_STOP");               // 通知 STM32 停止上升
    delay(500);                               // 等机械稳定

    // 拍照识别第二层
    camera_fb_t* fb2 = captureFrame();
    if (fb2) {
      String detectResp;
      bool detectOk = postImageMultipart(makeUrl("/detect?region=upper"), fb2, shelf, detectResp);
      esp_camera_fb_return(fb2);

      if (detectOk) {
        detectResp.trim();
        String parsedShelf, parsedCounts;
        if (parseDetectResult(detectResp, parsedShelf, parsedCounts)) {
          latestShelf = parsedShelf;
          latestCounts = parsedCounts;
          uartSendLine(detectResp);  // 转发结果给 STM32

          // 第二层也推送微信报告
          pushWechatReport(parsedShelf, parsedCounts);
        }
      } else {
        uartSendLine("DETECT_FAIL:" + shelf);
      }
    }

    // 第二层完成 → 开始下降归位, 等扫到第一层二维码再停
    g_lowering = true;
    g_state = STATE_SCAN_QR;
    uartSendLine("LOWER_START");            // 通知 STM32 开始下降
    markIgnoreShelf(shelf, IGNORE_SHELF_MS);
    markIgnoreShelf(layer1Shelf, IGNORE_SHELF_MS);
    // 保留 layer1Shelf, lowerScanTask 需要它识别归位点
  }

  void lowerScanTask() {
    // 下降归位时扫码: 扫到第一层二维码 Y 居中就停
    if (!g_lowering) return;
    if (millis() - liftScanLastMs < 200) return;
    liftScanLastMs = millis();

    camera_fb_t* fb = captureFrame();
    if (!fb) return;
    String resp;
    bool ok = postImageMultipart(makeUrl("/scan_qr"), fb, "", resp);
    esp_camera_fb_return(fb);
    if (!ok) return;
    resp.trim();

    float yRatio = 0.0;
    String shelf;
    if (resp.startsWith("FOUND:")) {
      String p = resp.substring(strlen("FOUND:"));
      int yp = p.indexOf(":Y=");
      if (yp > 0) { shelf = p.substring(0, yp); yRatio = p.substring(yp+3).toFloat(); }
    } else if (resp.startsWith("OFFCENTER:")) {
      String p = resp.substring(strlen("OFFCENTER:"));
      int yp = p.indexOf(":Y=");
      if (yp > 0) { shelf = p.substring(0, yp); yRatio = p.substring(yp+3).toFloat(); }
    } else return;
    shelf.trim();
    if (shelf.length() == 0) return;
    if (shelf != layer1Shelf) return;   // 只认第一层

    // Y 居中(下半画面中间, 匹配下降方向)
    if (yRatio < 0.25 || yRatio > 0.5) return;

    // 归位! 停止下降
    Serial.printf("[LOWER] back to layer1: %s Y=%.2f\n", shelf.c_str(), yRatio);
    uartSendLine("LIFT_STOP");
    g_lowering = false;
    layer1Shelf = "";
    lastScanMs = 0;           // 立刻允许 scanQrTask 扫描
    markIgnoreShelf(shelf, IGNORE_SHELF_MS);
  }

  void timeoutTask() {
    // 等待检测超时保护 超时没等到 DETECT 指令就回扫码态 避免卡死
    if (g_state == STATE_WAIT_DETECT) {
      if (millis() - waitDetectStartMs > WAIT_DETECT_TIMEOUT_MS) {
        Serial.println("[STATE] wait detect timeout, back to scan");
        markIgnoreShelf(currentShelf, DETECT_IGNORE_MS);
        currentShelf = "";
        g_state = STATE_SCAN_QR;
      }
    }
  // 上升找第二层超时: 25秒没找到就降回 (单层货架)
    if (g_state == STATE_LIFTING_FOR_L2) {
      if (millis() - waitDetectStartMs > 25000) {
        Serial.println("[LIFT] timeout 25s, no layer2, lowering");
        uartSendLine("LIFT_STOP");
        delay(500);
        g_lowering = true;
        uartSendLine("LOWER_START");
        g_state = STATE_SCAN_QR;
      }
    }
  }

  void setup() {
    Serial.begin(115200);   // 调试串口
    delay(1500);
    Serial.println("Boot...");

    Serial.printf("[SYS] PSRAM found=%d size=%u, freeHeap=%u\n",
                  psramFound(), (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreeHeap());

    // 启动与 STM32 通信的串口
    STM32Serial.begin(115200, SERIAL_8N1, STM32_UART_RX_PIN, STM32_UART_TX_PIN);
    Serial.println("[UART] STM32 serial started");

    // 摄像头初始化失败则停在这里 不再往下跑
    if (!initCamera()) {
      Serial.println("[SYS] camera init failed");
      while (true) delay(1000);
    }

    // 连接 WiFi 阻塞直到连上
    WiFi.begin(ssid, password);
    Serial.print("[WiFi] connecting");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println();
    Serial.println("[WiFi] connected");
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());

    // 注册 Web 路由并启动服务
    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/result", HTTP_GET, handleResult);
    server.on("/reset", HTTP_GET, handleReset);
    server.on("/reset", HTTP_POST, handleReset);
    server.on("/lower", HTTP_GET, [](){ uartSendLine("LIFT_STOP"); delay(200); uartSendLine("LOWER_START"); server.send(200, "text/plain", "ok"); });
    server.begin();

    Serial.println("[SYS] web server started");
  }

  void loop() {
    // 主循环 处理网页请求、读串口、扫码、上升找第二层、下降归位
    server.handleClient();
    readStm32Uart();
    scanQrTask();
    liftScanTask();      // 上升找第二层
    lowerScanTask();     // 下降归位: 扫到第一层二维码就停
    timeoutTask();
  }