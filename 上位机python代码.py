
# =========================
# 配置
# =========================
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 5000

MODEL_PATH = "best(1).pt"

# 摄像头方向设置
CAMERA_FLIP_MODE = 1

# 中间竖带比例
CENTER_BAND_RATIO = 0.18

# 升降扫码区域 (避免两层货架二维码互相干扰)
UPPER_RATIO = 0.45   # 上升时只看顶部
LOWER_RATIO = 0.45   # 下降时只看底部

# =========================
# 货架配置
# =========================
TEST_MODE = False
TEST_SHELF = "SHELF_A1"
ALL_SHELVES = {"SHELF_A1", "SHELF_A2", "SHELF_B1", "SHELF_B2"}

if TEST_MODE:
    VALID_SHELVES = {TEST_SHELF}
else:
    VALID_SHELVES = ALL_SHELVES

# 图片保存设置
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SAVE_ROOT = os.path.join(BASE_DIR, "saved_images")
SCAN_SAVE_DIR = os.path.join(SAVE_ROOT, "scan_qr")
DETECT_SAVE_DIR = os.path.join(SAVE_ROOT, "detect")

os.makedirs(SCAN_SAVE_DIR, exist_ok=True)
os.makedirs(DETECT_SAVE_DIR, exist_ok=True)

# =========================
# 初始化
# =========================
app = Flask(__name__)

# YOLO 模型加载 (文件不存在时跳过, /scan_qr 仍可用)
model = None
if os.path.exists(MODEL_PATH):
    model = YOLO(MODEL_PATH)
    print(f"[INFO] YOLO模型已加载: {MODEL_PATH}")
else:
    print(f"[WARN] 模型文件不存在: {MODEL_PATH}, /detect 接口将不可用")

qr_detector = cv2.QRCodeDetector()

state_lock = threading.Lock()

current_shelf = ""
last_qr_data = ""
last_qr_in_center = False
last_detect_result = {
    "shelf": "",
    "counts": {}
}

# =========================
# 工具函数
# =========================
def make_timestamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def clear_scan_images():
    if not os.path.isdir(SCAN_SAVE_DIR):
        return 0
    deleted = 0
    for name in os.listdir(SCAN_SAVE_DIR):
        path = os.path.join(SCAN_SAVE_DIR, name)
        if os.path.isfile(path) and name.lower().endswith((".jpg", ".jpeg", ".png")):
            try:
                os.remove(path)
                deleted += 1
            except OSError as e:
                print(f"[CLEAR_SCAN] 删除失败 {path}: {e}")
    print(f"[CLEAR_SCAN] 已清空上一趟扫码图 {deleted} 张，detect 图保留")
    return deleted


def fix_camera_image(img):
    if img is None:
        return img
    if CAMERA_FLIP_MODE == 1:
        return cv2.flip(img, 0)
    elif CAMERA_FLIP_MODE == 2:
        return cv2.flip(img, 1)
    elif CAMERA_FLIP_MODE == 3:
        return cv2.flip(img, -1)
    return img


def save_debug_images(image_bytes, img_fixed, save_dir, prefix, annotated_img=None):
    ts = make_timestamp()
    fixed_path = os.path.join(save_dir, f"{prefix}_{ts}_fixed.jpg")
    annotated_path = os.path.join(save_dir, f"{prefix}_{ts}_annotated.jpg")
    if img_fixed is not None:
        cv2.imwrite(fixed_path, img_fixed)
        print(f"[SAVE] 处理图已保存: {fixed_path}")
    if annotated_img is not None:
        cv2.imwrite(annotated_path, annotated_img)
        print(f"[SAVE] 标注图已保存: {annotated_path}")
    else:
        annotated_path = ""
    return fixed_path, annotated_path


def decode_image_from_request():
    if "image" not in request.files:
        return None, None
    file = request.files["image"]
    image_bytes = file.read()
    if not image_bytes:
        return None, None
    np_arr = np.frombuffer(image_bytes, np.uint8)
    img_raw = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if img_raw is None:
        return None, None
    img_fixed = fix_camera_image(img_raw)
    return img_fixed, image_bytes


def get_center_band(w, h):
    band_w = int(w * CENTER_BAND_RATIO)
    band_h = int(h * CENTER_BAND_RATIO)
    cx, cy = w // 2, h // 2
    x1 = max(0, cx - band_w // 2)
    x2 = min(w - 1, cx + band_w // 2)
    y1 = max(0, cy - band_h // 2)
    y2 = min(h - 1, cy + band_h // 2)
    return x1, y1, x2, y2


def preprocess_for_qr(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    candidates = [gray]
    big = cv2.resize(gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
    blur = cv2.GaussianBlur(big, (0, 0), 3)
    sharp = cv2.addWeighted(big, 2.0, blur, -1.0, 0)
    candidates.append(sharp)
    _, otsu = cv2.threshold(sharp, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    candidates.append(otsu)
    return candidates


def analyze_qr(img):
    h, w = img.shape[:2]
    x1, y1, x2, y2 = get_center_band(w, h)
    result = {
        "found": False,
        "data": "",
        "points": None,
        "center": None,
        "in_center": False,
        "y_ratio": 0.0,
        "band": (x1, y1, x2, y2)
    }
    for cand in preprocess_for_qr(img):
        decoded = zbar_decode(cand, symbols=[ZBarSymbol.QRCODE])
        if decoded:
            obj = decoded[0]
            cand_h, cand_w = cand.shape[:2]
            scale_x = w / cand_w
            scale_y = h / cand_h
            pts = np.array([[p.x, p.y] for p in obj.polygon], dtype=np.float64)
            if len(pts) < 4:
                rx, ry, rw, rh = obj.rect
                pts = np.array([[rx, ry], [rx + rw, ry],
                                [rx + rw, ry + rh], [rx, ry + rh]], dtype=np.float64)
            pts[:, 0] *= scale_x
            pts[:, 1] *= scale_y
            pts = pts.astype(int)
            qr_center_x = int(np.mean(pts[:, 0]))
            qr_center_y = int(np.mean(pts[:, 1]))
            result.update({
                "found": True,
                "data": obj.data.decode("utf-8").strip(),
                "points": pts,
                "center": (qr_center_x, qr_center_y),
                "in_center": x1 <= qr_center_x <= x2,
                "y_ratio": qr_center_y / h
            })
            return result
    data, points, _ = qr_detector.detectAndDecode(img)
    if data and points is not None:
        pts = points[0].astype(int)
        qr_center_x = int(np.mean(pts[:, 0]))
        qr_center_y = int(np.mean(pts[:, 1]))
        result.update({
            "found": True,
            "data": data.strip(),
            "points": pts,
            "center": (qr_center_x, qr_center_y),
            "in_center": x1 <= qr_center_x <= x2,
            "y_ratio": qr_center_y / h
        })
    return result


def count_objects(results):
    """统计所有检测到的类别及数量"""
    counts = {}
    boxes = results[0].boxes
    if boxes is None or boxes.cls is None:
        return counts
    class_ids = boxes.cls.cpu().numpy().astype(int)
    for cls_id in class_ids:
        class_name = model.names[int(cls_id)]
        counts[class_name] = counts.get(class_name, 0) + 1
    return counts


def counts_to_string(counts):
    """将 {cls: n} 转为 'cls1=n1,cls2=n2' 字符串"""
    if not counts:
        return "none=0"
    return ",".join(f"{k}={v}" for k, v in sorted(counts.items()))


def draw_center_band(img):
    if img is None:
        return img
    h, w = img.shape[:2]
    x1, y1, x2, y2 = get_center_band(w, h)
    out = img.copy()
    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 2)
    return out


def draw_qr_result(img, qr_result):
    if img is None:
        return img
    out = img.copy()
    band = qr_result.get("band")
    if band:
        x1, y1, x2, y2 = band
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 2)
    if qr_result.get("found") and qr_result.get("points") is not None:
        pts = qr_result["points"]
        for i in range(4):
            p1 = tuple(pts[i])
            p2 = tuple(pts[(i + 1) % 4])
            cv2.line(out, p1, p2, (0, 255, 0), 2)
        center = qr_result.get("center")
        if center:
            cv2.circle(out, center, 5, (0, 0, 255), -1)
        text = f"{qr_result.get('data', '')} center={qr_result.get('in_center', False)}"
        cv2.putText(out, text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return out


def draw_detect_result(img, results, shelf_id, counts):
    if img is None:
        return img
    out = img.copy()
    import random
    colors = {}
    if results and len(results) > 0:
        res = results[0]
        boxes = res.boxes
        if boxes is not None and boxes.xyxy is not None and boxes.cls is not None:
            xyxy = boxes.xyxy.cpu().numpy().astype(int)
            cls_ids = boxes.cls.cpu().numpy().astype(int)
            for i, box in enumerate(xyxy):
                x1, y1, x2, y2 = box
                cls_id = cls_ids[i]
                class_name = model.names[int(cls_id)]
                if class_name not in colors:
                    random.seed(hash(class_name) % 256)
                    colors[class_name] = (random.randint(64, 255),
                                          random.randint(64, 255),
                                          random.randint(64, 255))
                color = colors[class_name]
                cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
                cv2.putText(out, class_name, (x1, max(25, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    text = f"{shelf_id}  {counts_to_string(counts)}"
    cv2.putText(out, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    return out


# =========================
# 接口
# =========================

@app.route("/", methods=["GET"])
def home():
    return (
        "Flask server is running<br>"
        f"VALID_SHELVES: {sorted(list(VALID_SHELVES))}<br>"
        f"SCAN_SAVE_DIR: {SCAN_SAVE_DIR}<br>"
        f"DETECT_SAVE_DIR: {DETECT_SAVE_DIR}"
    )


@app.route("/scan_qr", methods=["POST"])
def scan_qr():
    """
    扫码接口: 返回 FOUND:shelf_id:Y=0.xx
    - FOUND: 二维码在中心带内 (X方向)
    - OFFCENTER: 二维码存在但不在中心带内
    - NOT_FOUND: 没有二维码
    Y值: 二维码中心纵坐标/图像高度 (0=顶部, 1=底部)
    """
    global current_shelf, last_qr_data, last_qr_in_center

    img, image_bytes = decode_image_from_request()
    if img is None:
        return "BAD_IMAGE", 400

    # region 过滤: 上升只看顶部, 下降只看底部
    region = request.args.get("region", "all")
    scan_img = img
    y_offset = 0.0
    if region == "upper":
        full_h = img.shape[0]
        scan_img = img[0:int(full_h * UPPER_RATIO), :]
    elif region == "lower":
        full_h = img.shape[0]
        cut = int(full_h * LOWER_RATIO)
        scan_img = img[full_h - cut:full_h, :]
        y_offset = 1.0 - LOWER_RATIO

    qr_result = analyze_qr(scan_img)
    # Y 坐标修正回全图比例
    if region in ("upper", "lower") and qr_result.get("found"):
        crop_ratio = UPPER_RATIO if region == "upper" else LOWER_RATIO
        qr_result["y_ratio"] = y_offset + qr_result["y_ratio"] * crop_ratio
    if region == "upper" and qr_result.get("found"):
        qr_result["y_ratio"] = qr_result["y_ratio"] * UPPER_RATIO
    annotated = draw_qr_result(img, qr_result)
    save_debug_images(
        image_bytes=image_bytes,
        img_fixed=img,
        save_dir=SCAN_SAVE_DIR,
        prefix="scan",
        annotated_img=annotated
    )

    if not qr_result["found"]:
        with state_lock:
            last_qr_data = ""
            last_qr_in_center = False
        return "NOT_FOUND", 200

    shelf_id = qr_result["data"].strip()
    y_ratio = qr_result.get("y_ratio", 0.0)

    if VALID_SHELVES and shelf_id not in VALID_SHELVES:
        with state_lock:
            last_qr_data = shelf_id
            last_qr_in_center = qr_result["in_center"]
        print(f"[SCAN_QR] INVALID shelf={shelf_id}")
        return "NOT_FOUND", 200

    with state_lock:
        last_qr_data = shelf_id
        last_qr_in_center = qr_result["in_center"]

    if not qr_result["in_center"]:
        print(f"[SCAN_QR] OFFCENTER: {shelf_id} Y={y_ratio:.2f}")
        return f"OFFCENTER:{shelf_id}:Y={y_ratio:.2f}", 200

    with state_lock:
        current_shelf = shelf_id

    print(f"[SCAN_QR] FOUND: {shelf_id} Y={y_ratio:.2f}")
    return f"FOUND:{shelf_id}:Y={y_ratio:.2f}", 200


@app.route("/detect", methods=["POST"])
def detect():
    """
    检测接口: 返回 RESULT:shelf_id:cls1=n1,cls2=n2
    ?region=upper 时只识别顶部 UPPER_RATIO 区域 (用于第二层)
    """
    global current_shelf, last_detect_result
    shelf_id = request.form.get("shelf_id", "").strip()
    if not shelf_id:
        with state_lock:
            shelf_id = current_shelf if current_shelf else "UNKNOWN"
    img, image_bytes = decode_image_from_request()
    if img is None:
        return "BAD_IMAGE", 400

    if model is None:
        return "NO_MODEL", 503
    results = model(img, verbose=False)

    # 第二层只取图像顶部区域
    region = request.args.get("region", "all")
    if region == "upper" and results and len(results) > 0:
        h = img.shape[0]
        boxes = results[0].boxes
        if boxes is not None and boxes.xyxy is not None:
            keep = boxes.xyxy[:, 1] < (h * UPPER_RATIO)  # Y1 在阈值以上的保留
            results[0].boxes = boxes[keep]

    counts = count_objects(results)
    counts_str = counts_to_string(counts)
    annotated = draw_detect_result(img, results, shelf_id, counts)
    save_debug_images(
        image_bytes=image_bytes,
        img_fixed=img,
        save_dir=DETECT_SAVE_DIR,
        prefix=f"detect_{shelf_id}",
        annotated_img=annotated
    )
    with state_lock:
        current_shelf = shelf_id
        last_detect_result = {
            "shelf": shelf_id,
            "counts": counts
        }
    print(f"[DETECT] shelf={shelf_id}, counts={counts}")
    return f"RESULT:{shelf_id}:{counts_str}", 200


@app.route("/status", methods=["GET"])
def status():
    with state_lock:
        return jsonify({
            "current_shelf": current_shelf,
            "last_qr_data": last_qr_data,
            "last_qr_in_center": last_qr_in_center,
            "last_detect_result": last_detect_result,
            "valid_shelves": sorted(list(VALID_SHELVES))
        }), 200


@app.route("/reset", methods=["POST", "GET"])
def reset_state():
    global current_shelf, last_qr_data, last_qr_in_center, last_detect_result
    with state_lock:
        current_shelf = ""
        last_qr_data = ""
        last_qr_in_center = False
        last_detect_result = {
            "shelf": "",
            "counts": {}
        }
    print("[RESET] 状态已重置")
    return jsonify({"ok": True, "message": "状态已重置"}), 200


if __name__ == "__main__":
    clear_scan_images()
    print(f"Flask 服务启动: http://127.0.0.1:{SERVER_PORT}")
    print("接口:")
    print("  GET  /")
    print("  POST /scan_qr  → FOUND:shelf_id:Y=0.xx")
    print("  POST /detect   → RESULT:shelf_id:cls1=n1,cls2=n2")
    print("  GET  /status")
    print("  POST /reset")
    print(f"当前允许识别货架: {sorted(list(VALID_SHELVES))}")

    app.run(host=SERVER_HOST, port=SERVER_PORT, debug=False, threaded=True)
