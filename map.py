import streamlit as st
from streamlit_drawable_canvas import st_canvas
import cv2
from PIL import Image
import numpy as np
import heapq
import json
import socket

st.set_page_config(layout="wide")

# ===== 이미지 불러오기 =====
IMAGE_PATH = "D:/world.png"
CAR_ICON_PATH = "D:/car_icon.png"

image = cv2.imread(IMAGE_PATH)
if image is None:
    st.error("이미지를 불러올 수 없습니다. 경로를 확인하세요.")
    st.stop()
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

car_icon = cv2.imread(CAR_ICON_PATH, cv2.IMREAD_UNCHANGED)
car_icon = cv2.resize(car_icon, (30, 30))

# ===== 도로 마스크 생성 =====
hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 50])
road_mask = cv2.inRange(hsv, lower_black, upper_black)

# ===== 상태 저장 =====
if "mode" not in st.session_state:
    st.session_state.mode = "route"
if "start" not in st.session_state:
    st.session_state.start = None
if "end" not in st.session_state:
    st.session_state.end = None
if "path" not in st.session_state:
    st.session_state.path = []
if "reset_key" not in st.session_state:
    st.session_state.reset_key = 0
if "sent_to_gazebo" not in st.session_state:
    st.session_state.sent_to_gazebo = False

# ===== 픽셀 → Gazebo 좌표 변환 
def pixel_to_gazebo(px):
    px_x, px_y = px
    W = 768  # world.png의 가로 픽셀 수
    H = 825  # world.png의 세로 픽셀 수
    gazebo_x = -48 + (px_x / W) * 171
    gazebo_y = 103 - (px_y / H) * 206
    return {"x": round(gazebo_x, 2), "y": round(gazebo_y, 2)}

# ===== UI 버튼 =====
col1, col2, col3 = st.columns(3)
with col1:
    if st.button("🟢 출발지 찍기"):
        st.session_state.mode = "start"
with col2:
    if st.button("🔴 도착지 찍기"):
        st.session_state.mode = "end"
with col3:
    if st.button("🧹 초기화"):
        st.session_state.start = None
        st.session_state.end = None
        st.session_state.path = []
        st.session_state.sent_to_gazebo = False
        st.session_state.mode = "route"
        st.session_state.reset_key += 1
        st.experimental_rerun()

st.write(f"📌 현재 모드: **{st.session_state.mode.upper()}**")

# ===== 캔버스 클릭 =====
canvas_result = st_canvas(
    fill_color="rgba(0, 0, 255, 0.3)",
    stroke_width=10,
    stroke_color="#0000FF",
    background_image=Image.fromarray(image),
    update_streamlit=True,
    height=image.shape[0],
    width=image.shape[1],
    drawing_mode="point",
    key=f"canvas_{st.session_state.reset_key}"
)

# ===== 도로 위 가장 가까운 픽셀 찾기 =====
def find_nearest_white(mask, x, y):
    h, w = mask.shape
    visited = set()
    q = [(0, x, y)]
    while q:
        dist, cx, cy = heapq.heappop(q)
        if (cx, cy) in visited or not (0 <= cx < w and 0 <= cy < h):
            continue
        visited.add((cx, cy))
        if mask[cy, cx] > 0:
            return (cx, cy)
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            heapq.heappush(q, (dist+1, cx+dx, cy+dy))
    return (x, y)

# ===== A* 경로 탐색 =====
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(mask, start, end):
    h, w = mask.shape
    open_set = [(0 + heuristic(start, end), 0, start)]
    came_from = {}
    g_score = {start: 0}
    while open_set:
        _, cost, current = heapq.heappop(open_set)
        if current == end:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = current[0]+dx, current[1]+dy
            if 0 <= nx < w and 0 <= ny < h and mask[ny, nx] > 0:
                neighbor = (nx, ny)
                tentative = g_score[current] + 1
                if tentative < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    heapq.heappush(open_set, (tentative + heuristic(neighbor, end), tentative, neighbor))
    return []

# ===== 클릭 좌표 처리 =====
if canvas_result.json_data and "objects" in canvas_result.json_data:
    objects = canvas_result.json_data["objects"]
    if len(objects) > 0:
        last = objects[-1]
        if last["type"] == "circle":
            x = int(last["left"] + last["radius"])
            y = int(last["top"] + last["radius"])
            cx, cy = find_nearest_white(road_mask, x, y)
            if st.session_state.mode == "start":
                st.session_state.start = (cx, cy)
                st.session_state.sent_to_gazebo = False  # 다시 클릭 시 초기화
            elif st.session_state.mode == "end":
                st.session_state.end = (cx, cy)
                st.session_state.sent_to_gazebo = False  # 다시 클릭 시 초기화

# ===== 경로 탐색 및 자동 Gazebo 전송 =====
if st.session_state.start and st.session_state.end:
    st.session_state.path = astar(road_mask, st.session_state.start, st.session_state.end)

    if len(st.session_state.path) > 1 and not st.session_state.sent_to_gazebo:
        start_gazebo = pixel_to_gazebo(st.session_state.start)
        end_gazebo = pixel_to_gazebo(st.session_state.end)
        path_gazebo = [pixel_to_gazebo(p) for p in st.session_state.path]
        data = {
            "start": start_gazebo,
            "end": end_gazebo,
            "path": path_gazebo
        }
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(("localhost", 9999))  # Gazebo 측 노드 포트
            s.send(json.dumps(data).encode())
            s.close()
            st.session_state.sent_to_gazebo = True
            st.toast("📡 출발/도착 설정 완료 → Gazebo로 경로 자동 전송됨")
        except Exception as e:
            st.error(f"❌ Gazebo 전송 실패: {e}")

# ===== 시각화 =====
img_draw = image.copy()
if st.session_state.start:
    cv2.circle(img_draw, st.session_state.start, 8, (0, 255, 0), -1)
    cv2.putText(img_draw, "Start", st.session_state.start, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
if st.session_state.end:
    cv2.circle(img_draw, st.session_state.end, 8, (0, 0, 255), -1)
    cv2.putText(img_draw, "End", st.session_state.end, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

for i in range(len(st.session_state.path) - 1):
    cv2.line(img_draw, st.session_state.path[i], st.session_state.path[i + 1], (255, 0, 0), 2)

# ===== 차량 아이콘 표시 =====
def paste_icon(base_img, icon, center):
    x, y = center
    h, w = icon.shape[:2]
    top_left_x = max(0, x - w // 2)
    top_left_y = max(0, y - h // 2)
    icon_roi = icon[:min(h, base_img.shape[0]-top_left_y), :min(w, base_img.shape[1]-top_left_x)]
    alpha_s = icon_roi[:, :, 3] / 255.0
    alpha_l = 1.0 - alpha_s
    for c in range(3):
        base_img[top_left_y:top_left_y+icon_roi.shape[0], top_left_x:top_left_x+icon_roi.shape[1], c] = (
            alpha_s * icon_roi[:, :, c] +
            alpha_l * base_img[top_left_y:top_left_y+icon_roi.shape[0], top_left_x:top_left_x+icon_roi.shape[1], c]
        )
    return base_img

if st.session_state.start:
    img_draw = paste_icon(img_draw, car_icon, st.session_state.start)

st.image(img_draw, caption="🗺️ 도로 기반 경로 탐색 + 차량 시작 위치", use_column_width=True)
