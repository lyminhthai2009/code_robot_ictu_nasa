#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import os
from PIL import Image, ImageDraw, ImageFont

PROJECT_DIR = os.path.expanduser("~/ICTU_Robotics_2026")
WORLDS_DIR = os.path.join(PROJECT_DIR, "worlds")
TEXTURES_DIR = os.path.join(WORLDS_DIR, "textures")
CTRL_DIR = os.path.join(PROJECT_DIR, "controllers", "cobot_cpp")

SCALE = 2
TEX_W, TEX_H = 2378 * SCALE, 1682 * SCALE

COL_BG = (252, 252, 252)
COL_LINE = (18, 18, 18)
COL_START = (0, 185, 55)
COL_FINISH = (225, 25, 25)
COL_BUMP = (15, 65, 225)
COL_MARKER_BLUE = (20, 90, 235)
# Màu xanh lá có kênh R = 115 để tránh kích hoạt ngưỡng nhận diện line của cảm biến IR Webots
COL_ARROW = (115, 215, 115)


def world_to_px(x_m, y_m):
  px = int(round(((x_m + 1.189) / 2.378) * TEX_W))
  py = int(round(((0.841 - y_m) / 1.682) * TEX_H))
  return (px, py)


def mm_to_px(width_mm):
  return max(2, int(round(width_mm * SCALE)))


def get_font(size_px, bold=True):
  font_candidates = [
      (
          "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
          if bold
          else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
      ),
      (
          "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf"
          if bold
          else "/usr/share/fonts/TTF/DejaVuSans.ttf"
      ),
      "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
      "/usr/share/fonts/noto/NotoSans-Bold.ttf",
  ]
  for path in font_candidates:
    if os.path.exists(path):
      return ImageFont.truetype(path, size_px)
  return ImageFont.load_default()


def draw_thick_line(draw, x1, y1, x2, y2, width_mm, color=COL_LINE):
  w_px = mm_to_px(width_mm)
  p1 = world_to_px(x1, y1)
  p2 = world_to_px(x2, y2)
  draw.line([p1, p2], fill=color, width=w_px)
  r_px = w_px // 2
  for p in (p1, p2):
    draw.ellipse([p[0] - r_px, p[1] - r_px, p[0] + r_px, p[1] + r_px], fill=color)


def draw_thick_arc(
    draw, cx_m, cy_m, r_m, start_deg, end_deg, width_mm, color=COL_LINE
):
  w_px = mm_to_px(width_mm)
  steps = max(180, int(abs(end_deg - start_deg) * 3))
  pts = []
  for i in range(steps + 1):
    ang = math.radians(start_deg + (end_deg - start_deg) * i / steps)
    x = cx_m + r_m * math.cos(ang)
    y = cy_m + r_m * math.sin(ang)
    pts.append(world_to_px(x, y))
  for p1, p2 in zip(pts[:-1], pts[1:]):
    draw.line([p1, p2], fill=color, width=w_px)
  r_px = w_px // 2
  for p in pts[::2]:
    draw.ellipse([p[0] - r_px, p[1] - r_px, p[0] + r_px, p[1] + r_px], fill=color)


def draw_arrow(
    draw, x1, y1, x2, y2, color=COL_ARROW, width_mm=6, head_len_m=0.028
):
  draw_thick_line(draw, x1, y1, x2, y2, width_mm, color=color)
  ang = math.atan2(y2 - y1, x2 - x1)
  for delta in (math.radians(150), math.radians(-150)):
    hx = x2 + head_len_m * math.cos(ang + delta)
    hy = y2 + head_len_m * math.sin(ang + delta)
    draw_thick_line(draw, x2, y2, hx, hy, width_mm, color=color)


def draw_label_badge(
    draw,
    x_m,
    y_m,
    text,
    font_size_mm=22,
    text_color=(20, 20, 20),
    box_fill=(255, 255, 255),
    border_color=(180, 180, 180),
):
  font = get_font(mm_to_px(font_size_mm), bold=True)
  cx, cy = world_to_px(x_m, y_m)
  bbox = draw.textbbox((0, 0), text, font=font)
  tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
  pad_x, pad_y = mm_to_px(10), mm_to_px(6)
  rect = [
      cx - tw // 2 - pad_x,
      cy - th // 2 - pad_y,
      cx + tw // 2 + pad_x,
      cy + th // 2 + pad_y,
  ]
  if box_fill:
    draw.rounded_rectangle(
        rect,
        radius=mm_to_px(8),
        fill=box_fill,
        outline=border_color,
        width=max(1, SCALE),
    )
  draw.text(
      (cx - tw // 2 - bbox[0], cy - th // 2 - bbox[1]),
      text,
      font=font,
      fill=text_color,
  )


def draw_teardrop_loop(
    draw,
    x_in,
    y_top,
    y_bot,
    r_loop,
    r_trans,
    line_width_mm,
    direction=1,
    allow_shortcut=False,
    label_text="",
):
  """Vẽ cụm Ngã tư chữ X + Vòng tròn tiếp tuyến C1 mượt mà."""
  y_mid = 0.5 * (y_top + y_bot)
  half_h = 0.5 * (y_top - y_bot)
  s45 = math.sqrt(0.5)

  # 1. Cung bo chuyển tiếp vào chữ X
  if direction == 1:
    draw_thick_arc(draw, x_in, y_top - r_trans, r_trans, 45, 90, line_width_mm)
    draw_thick_arc(draw, x_in, y_bot + r_trans, r_trans, -90, -45, line_width_mm)
  else:
    draw_thick_arc(draw, x_in, y_top - r_trans, r_trans, 90, 135, line_width_mm)
    draw_thick_arc(
        draw, x_in, y_bot + r_trans, r_trans, -135, -90, line_width_mm
    )

  # 2. Giao lộ chữ X
  dy_to_mid = half_h - r_trans * (1.0 - s45)
  x_cross = x_in + direction * (r_trans * s45 + dy_to_mid)
  cx_loop = x_cross + direction * (r_loop * math.sqrt(2.0))

  x_tan_start = x_in + direction * (r_trans * s45)
  y_tan_top = y_top - r_trans * (1.0 - s45)
  y_tan_bot = y_bot + r_trans * (1.0 - s45)

  x_tan_end = x_cross + direction * (r_loop * s45)
  y_loop_bot = y_mid - r_loop * s45
  y_loop_top = y_mid + r_loop * s45

  draw_thick_line(
      draw, x_tan_start, y_tan_top, x_tan_end, y_loop_bot, line_width_mm
  )
  draw_thick_line(
      draw, x_tan_end, y_loop_top, x_tan_start, y_tan_bot, line_width_mm
  )

  # 3. Vòng tròn 270 độ
  if direction == 1:
    draw_thick_arc(draw, cx_loop, y_mid, r_loop, -135, 135, line_width_mm)
  else:
    draw_thick_arc(draw, cx_loop, y_mid, r_loop, 45, 315, line_width_mm)

  # 4. Mũi tên chỉ hướng cách line 5cm
  def uv_to_xy(u1, u2):
    x = x_cross + direction * s45 * (u1 + u2)
    y = y_mid + s45 * (-u1 + u2)
    return x, y

  ax1, ay1 = uv_to_xy(-0.135, 0.050)
  ax2, ay2 = uv_to_xy(-0.055, 0.050)
  draw_arrow(draw, ax1, ay1, ax2, ay2)

  bx1, by1 = uv_to_xy(0.055, 0.050)
  bx2, by2 = uv_to_xy(0.135, 0.050)
  draw_arrow(draw, bx1, by1, bx2, by2)

  if allow_shortcut:
    cx1, cy1 = uv_to_xy(-0.050, -0.055)
    cx2, cy2 = uv_to_xy(-0.050, -0.135)
    draw_arrow(draw, cx1, cy1, cx2, cy2)

  if label_text:
    draw_label_badge(draw, cx_loop, y_mid, label_text, font_size_mm=20)


def generate_vong_bang_texture(filename, line_width_mm=22):
  img = Image.new("RGB", (TEX_W, TEX_H), COL_BG)
  draw = ImageDraw.Draw(img)

  # Tọa độ 4 tầng ngang cân đối
  y1, y2, y3, y4 = 0.62, 0.20, -0.20, -0.62

  # Tọa độ X chuẩn hóa toàn sân
  x_left_s = -0.88  # Dời cụm chữ S sang trái (cách tường 10cm)
  x_right_in = 0.36  # Nhánh phải chuyển vào Vòng tròn 1 & 2
  x_c_turn = -0.06  # Tâm Cua chữ C đặt tại -0.06m (cách chữ S hơn 31cm!)

  # 1. Đoạn thẳng Tầng 1: START -> Vòng tròn 1
  draw_thick_line(draw, x_left_s, y1, x_right_in, y1, line_width_mm)

  # 2. Cụm Vòng tròn 1 (Bắt buộc đi vòng)
  draw_teardrop_loop(
      draw,
      x_in=x_right_in,
      y_top=y1,
      y_bot=y2,
      r_loop=0.21,
      r_trans=0.13,
      line_width_mm=line_width_mm,
      direction=1,
      allow_shortcut=False,
      label_text="Vòng tròn 1",
  )

  # 3. Đoạn Tầng 2: Vòng tròn 1 -> Gờ giảm tốc 1 -> Cua chữ C
  draw_thick_line(draw, x_right_in, y2, x_c_turn, y2, line_width_mm)

  # 4. Cua chữ C chuyển tầng giữa sân (R = 0.20m, đỉnh lồi vươn tới x = -0.26m)
  r_c = 0.5 * (y2 - y3)
  y_c_mid = 0.5 * (y2 + y3)
  draw_thick_arc(draw, x_c_turn, y_c_mid, r_c, 90, 270, line_width_mm)

  # 5. Đoạn Tầng 3: Cua chữ C -> Gờ giảm tốc 2 -> Vòng tròn 2
  draw_thick_line(draw, x_c_turn, y3, x_right_in, y3, line_width_mm)

  # 6. Cụm Vòng tròn 2 (Được phép đi tắt)
  draw_teardrop_loop(
      draw,
      x_in=x_right_in,
      y_top=y3,
      y_bot=y4,
      r_loop=0.20,
      r_trans=0.13,
      line_width_mm=line_width_mm,
      direction=1,
      allow_shortcut=True,
      label_text="Vòng tròn 2",
  )

  # 7. Đoạn Tầng 4: Vòng tròn 2 -> Dốc cầu -> Chữ S
  draw_thick_line(draw, x_right_in, y4, x_left_s, y4, line_width_mm)

  # 8. Cụm Cua chữ S bên trái (Khoảng cách với Cua chữ C là ~31.3cm)
  r_s = (y1 - y4) / 6.0  # ~0.2067m
  x_s_inner = -0.78  # Đỉnh trong chữ S chỉ tới x = -0.573m
  draw_thick_arc(draw, x_left_s, y4 + r_s, r_s, 90, 270, line_width_mm)
  draw_thick_line(
      draw, x_left_s, y4 + 2 * r_s, x_s_inner, y4 + 2 * r_s, line_width_mm
  )
  draw_thick_arc(draw, x_s_inner, 0.0, r_s, -90, 90, line_width_mm)
  draw_thick_line(
      draw, x_s_inner, y1 - 2 * r_s, x_left_s, y1 - 2 * r_s, line_width_mm
  )
  draw_thick_arc(draw, x_left_s, y1 - r_s, r_s, 90, 270, line_width_mm)

  # 9. Vạch FINISH & START
  x_finish, x_start = -0.58, -0.24
  draw_thick_line(
      draw, x_finish, y1, x_start, y1, line_width_mm, color=COL_FINISH
  )
  draw_thick_line(
      draw, x_start, y1, x_start + 0.035, y1, line_width_mm, color=COL_START
  )
  draw_thick_line(
      draw, x_finish, y1 - 0.11, x_finish, y1 + 0.02, 5, color=COL_LINE
  )
  draw_thick_line(
      draw, x_start, y1 - 0.11, x_start, y1 + 0.02, 5, color=COL_LINE
  )
  draw_label_badge(
      draw,
      x_finish + 0.04,
      y1 - 0.08,
      "FINISH",
      font_size_mm=18,
      text_color=COL_FINISH,
  )
  draw_label_badge(
      draw,
      x_start + 0.04,
      y1 - 0.08,
      "START",
      font_size_mm=18,
      text_color=COL_START,
  )

  # 10. Gờ giảm tốc 1 & 2 (5 vạch xanh chuẩn vị trí)
  for bx in [0.06, 0.12, 0.18, 0.24, 0.30]:
    draw_thick_line(draw, bx, y2 - 0.045, bx, y2 + 0.045, 16, color=COL_BUMP)
    draw_thick_line(draw, bx, y3 - 0.045, bx, y3 + 0.045, 16, color=COL_BUMP)
  draw_label_badge(draw, 0.18, y2 + 0.095, "Gờ giảm tốc 1", font_size_mm=20)
  draw_label_badge(draw, 0.18, y3 + 0.095, "Gờ giảm tốc 2", font_size_mm=20)

  # 11. Khu vực Dốc cầu (Tâm x = -0.20m, dài 60cm từ -0.50m đến +0.10m)
  draw_thick_line(draw, -0.50, y4 - 0.022, 0.10, y4 - 0.022, 8, color=COL_BUMP)
  draw_thick_line(draw, -0.50, y4 + 0.022, 0.10, y4 + 0.022, 8, color=COL_BUMP)
  draw_label_badge(draw, -0.20, y4 + 0.075, "Dốc cầu", font_size_mm=22)

  img.save(filename)


def generate_vong_chung_ket_texture(filename, line_width_mm=22):
  img = Image.new("RGB", (TEX_W, TEX_H), COL_BG)
  draw = ImageDraw.Draw(img)

  y1, y2, y3, y4 = 0.62, 0.22, -0.22, -0.62
  x_start_base = 0.36
  x_c_center = 0.56
  x_loop_in = 0.70

  for s in [1, -1]:
    draw_thick_line(
        draw, s * x_start_base, s * y1, s * x_loop_in, s * y1, line_width_mm
    )
    draw_thick_line(
        draw,
        s * x_start_base,
        s * y1,
        s * (x_start_base + 0.04),
        s * y1,
        line_width_mm,
        color=COL_START,
    )
    draw_thick_line(
        draw,
        s * (x_loop_in - 0.08),
        s * y1,
        s * (x_loop_in - 0.03),
        s * y1,
        line_width_mm,
        color=COL_MARKER_BLUE,
    )
    draw_thick_line(
        draw,
        s * x_start_base,
        s * (y1 - 0.05),
        s * x_start_base,
        s * (y1 + 0.08),
        5,
        color=COL_LINE,
    )
    draw_label_badge(
        draw,
        s * x_start_base,
        s * (y1 + 0.11),
        "START",
        font_size_mm=16,
        text_color=COL_START,
    )

    y_top_1 = y1 if s == 1 else -y2
    y_bot_1 = y2 if s == 1 else -y1
    draw_teardrop_loop(
        draw,
        x_in=s * x_loop_in,
        y_top=y_top_1,
        y_bot=y_bot_1,
        r_loop=0.14,
        r_trans=0.09,
        line_width_mm=line_width_mm,
        direction=s,
        allow_shortcut=False,
        label_text="Vòng tròn 1",
    )

    draw_thick_line(
        draw, s * x_loop_in, s * y2, s * x_c_center, s * y2, line_width_mm
    )
    for bx in [0.58, 0.61, 0.64, 0.67, 0.70]:
      draw_thick_line(
          draw,
          s * bx,
          s * y2 - 0.04,
          s * bx,
          s * y2 + 0.04,
          12,
          color=COL_BUMP,
      )
    draw_label_badge(
        draw,
        s * 0.64,
        s * y2 + (0.08 if s == 1 else -0.08),
        "Gờ giảm tốc 1",
        font_size_mm=16,
    )

    c_ang1, c_ang2 = (90, 270) if s == 1 else (-90, 90)
    draw_thick_arc(
        draw, s * x_c_center, 0.0, 0.22, c_ang1, c_ang2, line_width_mm
    )

    draw_thick_line(
        draw, s * x_c_center, s * y3, s * x_loop_in, s * y3, line_width_mm
    )
    for bx in [0.58, 0.61, 0.64, 0.67, 0.70]:
      draw_thick_line(
          draw,
          s * bx,
          s * y3 - 0.04,
          s * bx,
          s * y3 + 0.04,
          12,
          color=COL_BUMP,
      )
    draw_label_badge(
        draw,
        s * 0.64,
        s * y3 + (0.08 if s == 1 else -0.08),
        "Gờ giảm tốc 2",
        font_size_mm=16,
    )
    draw_thick_line(
        draw,
        s * (x_loop_in - 0.05),
        s * y3,
        s * x_loop_in,
        s * y3,
        line_width_mm,
        color=COL_START,
    )

    y_top_2 = y3 if s == 1 else -y4
    y_bot_2 = y4 if s == 1 else -y3
    draw_teardrop_loop(
        draw,
        x_in=s * x_loop_in,
        y_top=y_top_2,
        y_bot=y_bot_2,
        r_loop=0.14,
        r_trans=0.09,
        line_width_mm=line_width_mm,
        direction=s,
        allow_shortcut=True,
        label_text="Vòng tròn 2",
    )

    x_s_entry = 0.22
    draw_thick_line(
        draw, s * x_loop_in, s * y4, s * x_s_entry, s * y4, line_width_mm
    )
    draw_thick_line(
        draw,
        s * 0.66,
        s * y4,
        s * 0.70,
        s * y4,
        line_width_mm,
        color=COL_START,
    )
    draw_thick_line(
        draw,
        s * 0.24,
        s * y4,
        s * 0.28,
        s * y4,
        line_width_mm,
        color=COL_START,
    )
    draw_label_badge(
        draw,
        s * 0.48,
        s * y4 + (0.07 if s == 1 else -0.07),
        "Dốc cầu",
        font_size_mm=16,
    )

    draw_thick_arc(
        draw, s * x_s_entry, s * (-0.44), 0.20, c_ang1, c_ang2, line_width_mm
    )
    opp_ang1, opp_ang2 = (-90, 90) if s == 1 else (90, 270)
    draw_thick_arc(
        draw, s * 0.14, s * (-0.06), 0.18, opp_ang1, opp_ang2, line_width_mm
    )
    draw_thick_line(
        draw, s * 0.22, s * (-0.24), s * 0.14, s * (-0.24), line_width_mm
    )
    draw_thick_arc(
        draw, s * 0.08, s * 0.06, 0.06, c_ang1, c_ang2, line_width_mm
    )
    draw_thick_line(
        draw, s * 0.14, s * 0.12, s * 0.08, s * 0.12, line_width_mm
    )
    draw_thick_line(draw, s * 0.08, 0.0, 0.0, 0.0, line_width_mm)

  draw_thick_line(
      draw, -0.03, 0.0, 0.03, 0.0, line_width_mm, color=COL_FINISH
  )
  draw_thick_line(draw, 0.0, -0.09, 0.0, 0.09, 6, color=COL_FINISH)
  draw_label_badge(
      draw, 0.0, -0.14, "FINISH", font_size_mm=18, text_color=COL_FINISH
  )

  img.save(filename)


def create_single_bridge(name, x, y, rot_z=0.0):
  """Dốc cầu 3D đúng quy chuẩn BTC: dài 60cm (20-20-20), cao 5cm."""
  return f"""
    Solid {{
      translation {x} {y} 0
      rotation 0 0 1 {rot_z}
      name "{name}"
      children [
        Pose {{
          translation 0 0 0.025
          children [
            Shape {{
              appearance PBRAppearance {{ baseColor 0.12 0.35 0.85 roughness 0.6 metalness 0 }}
              geometry Box {{ size 0.20 0.16 0.05 }}
            }}
          ]
        }}
        Pose {{
          translation 0 0 0.0505
          children [
            Shape {{
              appearance PBRAppearance {{ baseColor 0.05 0.05 0.05 roughness 0.9 metalness 0 }}
              geometry Box {{ size 0.20 0.024 0.001 }}
            }}
          ]
        }}
        Pose {{
          translation 0.20 0 0.022
          rotation 0 1 0 0.24498
          children [
            Shape {{
              appearance PBRAppearance {{ baseColor 0.14 0.38 0.9 roughness 0.6 metalness 0 }}
              geometry Box {{ size 0.208 0.16 0.006 }}
            }}
            Pose {{
              translation 0 0 0.0035
              children [
                Shape {{
                  appearance PBRAppearance {{ baseColor 0.05 0.05 0.05 roughness 0.9 metalness 0 }}
                  geometry Box {{ size 0.208 0.024 0.001 }}
                }}
              ]
            }}
          ]
        }}
        Pose {{
          translation -0.20 0 0.022
          rotation 0 1 0 -0.24498
          children [
            Shape {{
              appearance PBRAppearance {{ baseColor 0.14 0.38 0.9 roughness 0.6 metalness 0 }}
              geometry Box {{ size 0.208 0.16 0.006 }}
            }}
            Pose {{
              translation 0 0 0.0035
              children [
                Shape {{
                  appearance PBRAppearance {{ baseColor 0.05 0.05 0.05 roughness 0.9 metalness 0 }}
                  geometry Box {{ size 0.208 0.024 0.001 }}
                }}
              ]
            }}
          ]
        }}
      ]
      boundingObject Group {{
        children [
          Pose {{ translation 0 0 0.025 children [ Box {{ size 0.20 0.16 0.05 }} ] }}
          Pose {{ translation 0.20 0 0.022 rotation 0 1 0 0.24498 children [ Box {{ size 0.208 0.16 0.006 }} ] }}
          Pose {{ translation -0.20 0 0.022 rotation 0 1 0 -0.24498 children [ Box {{ size 0.208 0.16 0.006 }} ] }}
        ]
      }}
    }}
    """


def create_bridge_and_bumps_vrml(is_final=False):
  vrml = ""
  if not is_final:
    # 5 thanh gờ giảm tốc vòng bảng đã cập nhật tọa độ đồng bộ với texture
    for idx, bx in enumerate([0.06, 0.12, 0.18, 0.24, 0.30]):
      for g_idx, by in enumerate([0.20, -0.20]):
        vrml += f"""
    Solid {{
      translation {bx} {by} 0.001
      name "bump_vb_{g_idx+1}_{idx+1}"
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor 0.08 0.25 0.85 roughness 0.5 metalness 0 }}
          geometry Box {{ size 0.016 0.09 0.002 }}
        }}
      ]
      boundingObject Box {{ size 0.016 0.09 0.002 }}
    }}
    """
    vrml += create_single_bridge(
        "doc_cau_vong_bang", -0.20, -0.62, rot_z=0.0
    )
  else:
    for s in [1, -1]:
      for idx, bx in enumerate([0.58, 0.61, 0.64, 0.67, 0.70]):
        for g_idx, by in enumerate([0.22, -0.22]):
          vrml += f"""
    Solid {{
      translation {s*bx} {s*by} 0.001
      name "bump_ck_{'right' if s==1 else 'left'}_{g_idx}_{idx}"
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor 0.08 0.25 0.85 roughness 0.5 metalness 0 }}
          geometry Box {{ size 0.012 0.08 0.002 }}
        }}
      ]
      boundingObject Box {{ size 0.012 0.08 0.002 }}
    }}
    """
      vrml += create_single_bridge(
          f"doc_cau_chung_ket_{'1' if s==1 else '2'}",
          s * 0.48,
          s * (-0.64),
          rot_z=(0.0 if s == 1 else math.pi),
      )
  return vrml


def create_cobot_robot_vrml(
    name="COBOT_ICTU", x=-0.24, y=0.62, z=0.021, yaw=0.0, ctrl="cobot_cpp"
):
  return f"""
    DEF {name} Robot {{
      translation {x} {y} {z}
      rotation 0 0 1 {yaw}
      name "{name}"
      controller "{ctrl}"
      supervisor TRUE
      contactMaterial "caster_slide"
      children [
        Solid {{
          translation 0.02 0 0.008
          name "chassis_kmm_11_100"
          children [
            Shape {{
              appearance PBRAppearance {{ baseColor 0.05 0.55 0.28 roughness 0.4 metalness 0.2 }}
              geometry Box {{ size 0.14 0.10 0.004 }}
            }}
            Pose {{
              translation -0.01 0 0.012
              children [
                Shape {{
                  appearance PBRAppearance {{ baseColor 0.15 0.15 0.15 roughness 0.6 metalness 0.1 }}
                  geometry Box {{ size 0.058 0.062 0.018 }}
                }}
              ]
            }}
            Pose {{
              translation 0.04 0 0.008
              children [
                Shape {{
                  appearance PBRAppearance {{ baseColor 0.1 0.25 0.8 roughness 0.4 metalness 0.3 }}
                  geometry Box {{ size 0.035 0.055 0.012 }}
                }}
              ]
            }}
            Pose {{
              translation 0.07 0 -0.018
              children [
                Shape {{
                  appearance PBRAppearance {{ baseColor 0.85 0.85 0.9 roughness 0.1 metalness 0.9 }}
                  geometry Sphere {{ radius 0.010 }}
                }}
              ]
            }}
          ]
        }}
        HingeJoint {{
          jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 0.06 0 }}
          device [
            RotationalMotor {{ name "left_motor" maxVelocity 31.416 maxTorque 0.55 }}
          ]
          endPoint Solid {{
            translation 0 0.06 0
            rotation 1 0 0 1.5708
            name "left_wheel"
            contactMaterial "wheel_grip"
            children [
              Shape {{
                appearance PBRAppearance {{ baseColor 0.95 0.8 0.05 roughness 0.5 metalness 0.1 }}
                geometry Cylinder {{ height 0.018 radius 0.021 }}
              }}
            ]
            boundingObject Cylinder {{ height 0.018 radius 0.021 }}
            physics Physics {{ density -1 mass 0.04 }}
          }}
        }}
        HingeJoint {{
          jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 -0.06 0 }}
          device [
            RotationalMotor {{ name "right_motor" maxVelocity 31.416 maxTorque 0.55 }}
          ]
          endPoint Solid {{
            translation 0 -0.06 0
            rotation 1 0 0 1.5708
            name "right_wheel"
            contactMaterial "wheel_grip"
            children [
              Shape {{
                appearance PBRAppearance {{ baseColor 0.95 0.8 0.05 roughness 0.5 metalness 0.1 }}
                geometry Cylinder {{ height 0.018 radius 0.021 }}
              }}
            ]
            boundingObject Cylinder {{ height 0.018 radius 0.021 }}
            physics Physics {{ density -1 mass 0.04 }}
          }}
        }}
        DistanceSensor {{
          translation 0.085 0.027 -0.006
          rotation 0 1 0 1.5708
          name "ir0"
          type "infra-red"
          lookupTable [ 0 0 0, 0.05 1024 0 ]
        }}
        DistanceSensor {{
          translation 0.085 0.009 -0.006
          rotation 0 1 0 1.5708
          name "ir1"
          type "infra-red"
          lookupTable [ 0 0 0, 0.05 1024 0 ]
        }}
        DistanceSensor {{
          translation 0.085 -0.009 -0.006
          rotation 0 1 0 1.5708
          name "ir2"
          type "infra-red"
          lookupTable [ 0 0 0, 0.05 1024 0 ]
        }}
        DistanceSensor {{
          translation 0.085 -0.027 -0.006
          rotation 0 1 0 1.5708
          name "ir3"
          type "infra-red"
          lookupTable [ 0 0 0, 0.05 1024 0 ]
        }}
        Camera {{
          translation 0.090 0 -0.004
          rotation 0 1 0 1.5708
          name "color_sensor"
          width 4
          height 4
          fieldOfView 0.35
        }}
      ]
      boundingObject Group {{
        children [
          Pose {{ translation 0.02 0 0.010 children [ Box {{ size 0.14 0.10 0.020 }} ] }}
          Pose {{ translation 0.07 0 -0.011 children [ Sphere {{ radius 0.010 }} ] }}
          Pose {{ translation -0.04 0 -0.011 children [ Sphere {{ radius 0.010 }} ] }}
        ]
      }}
      physics Physics {{
        density -1
        mass 0.30
        centerOfMass [ 0.01 0 -0.005 ]
      }}
    }}
    """


def create_world_file(filepath, texture_rel_path, is_final=False):
  if not is_final:
    robots_vrml = create_cobot_robot_vrml(
        "COBOT_ICTU", x=-0.24, y=0.62, z=0.021, yaw=0.0
    )
  else:
    robots_vrml = create_cobot_robot_vrml(
        "COBOT_ICTU_1", x=0.36, y=0.62, z=0.021, yaw=0.0
    )
    robots_vrml += "\n" + create_cobot_robot_vrml(
        "COBOT_ICTU_2", x=-0.36, y=-0.62, z=0.021, yaw=math.pi
    )

  content = f"""#VRML_SIM R2025a utf8
WorldInfo {{
  info [
    "Cuoc thi Sang tao Robot THPT Tinh Thai Nguyen 2026 - Duong toi Tan Cuong Lan 2 (ICTU)"
  ]
  title "ICTU Robotics Contest 2026 - {'Vong Chung Ket' if is_final else 'Vong Bang'}"
  basicTimeStep 8
  contactProperties [
    ContactProperties {{
      material1 "wheel_grip"
      coulombFriction [ 2.4 ]
      bounce 0.02
    }}
    ContactProperties {{
      material1 "caster_slide"
      coulombFriction [ 0.0 ]
      bounce 0.0
    }}
  ]
}}
Viewpoint {{
  orientation -0.38928 0.38928 0.83482 1.75033
  position 0.0 -1.52 1.82
  follow "{'COBOT_ICTU' if not is_final else 'COBOT_ICTU_1'}"
  followType "None"
}}
Background {{
  skyColor [ 0.14 0.18 0.24 ]
}}
DEF ENV_LIGHT DirectionalLight {{
  ambientIntensity 0.65
  direction -0.3 -0.4 -1.0
  intensity 1.8
  castShadows TRUE
}}
DEF ARENA_FLOOR Solid {{
  translation 0 0 0
  name "sa_hinh_4A0"
  children [
    DEF FLOOR_SHAPE Shape {{
      appearance PBRAppearance {{
        baseColorMap DEF MAP_TEXTURE ImageTexture {{
          url [ "{texture_rel_path}" ]
          filtering 5
        }}
        roughness 0.85
        metalness 0.0
      }}
      geometry Plane {{
        size 2.378 1.682
      }}
    }}
    Pose {{ translation 0 0.851 0.03 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.35 0.75 }} geometry Box {{ size 2.40 0.02 0.06 }} }} ] }}
    Pose {{ translation 0 -0.851 0.03 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.35 0.75 }} geometry Box {{ size 2.40 0.02 0.06 }} }} ] }}
    Pose {{ translation 1.199 0 0.03 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.35 0.75 }} geometry Box {{ size 0.02 1.72 0.06 }} }} ] }}
    Pose {{ translation -1.199 0 0.03 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.35 0.75 }} geometry Box {{ size 0.02 1.72 0.06 }} }} ] }}
  ]
  boundingObject Plane {{ size 2.378 1.682 }}
}}
{create_bridge_and_bumps_vrml(is_final)}
{robots_vrml}
"""
  with open(filepath, "w", encoding="utf-8") as f:
    f.write(content)


def main():
  os.makedirs(TEXTURES_DIR, exist_ok=True)
  os.makedirs(CTRL_DIR, exist_ok=True)
  for w in [16, 22, 30]:
    generate_vong_bang_texture(
        os.path.join(TEXTURES_DIR, f"vong_bang_{w}mm.png"), line_width_mm=w
    )
    generate_vong_chung_ket_texture(
        os.path.join(TEXTURES_DIR, f"vong_chung_ket_{w}mm.png"),
        line_width_mm=w,
    )
  create_world_file(
      os.path.join(WORLDS_DIR, "vong_bang.wbt"),
      "textures/vong_bang_22mm.png",
      is_final=False,
  )
  create_world_file(
      os.path.join(WORLDS_DIR, "vong_chung_ket.wbt"),
      "textures/vong_chung_ket_22mm.png",
      is_final=True,
  )
  print(
      "✅ ĐÃ CẬP NHẬT SA HÌNH CHUẨN THỂ LỆ ICTU 2026: Khe hở mở rộng 31.3cm"
      " hoàn hảo!"
  )


if __name__ == "__main__":
  main()
