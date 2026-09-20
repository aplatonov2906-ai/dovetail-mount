"""
Кронштейн на боковую планку «ласточкин хвост» (АК-тип) с верхней и боковой
планками Пикатинни. Параметрическая модель CadQuery для 3D-печати.

Система координат (как на оружии):
  X — вдоль ствола, 0 = задний торец кронштейна, +X вперёд
  Y — поперёк, 0 = ось ствола, −Y = левая сторона (там планка)
  Z — вверх, 0 = середина призмы «ласточкин хвост» на коробке

Запуск:  python mount.py   → out/*.stl, out/*.step
"""
import math
import os
import cadquery as cq

# ───────── ПАРАМЕТРЫ — правь под свою реплику ─────────
# «Ласточкин хвост» на коробке (проверить штангенциркулем!)
DT_FACE  = 14.0   # высота лицевой (узкой) грани призмы, мм
DT_DEPTH = 3.5    # глубина призмы от основания до лицевой грани, мм
DT_ANGLE = 45.0   # угол скосов, градусов
DT_CLEAR = 0.3    # зазор на печать по каждой грани, мм

# Коробка
RECV_HALF_W = 17.0  # половина ширины ствольной коробки, мм
PLATE_BASE  = 3.5   # толщина основания планки под призмой (от коробки), мм
RECV_TOP    = 33.0  # от середины призмы до верха крышки коробки (по рулетке ≈33), мм
TOP_CLEAR   = 2.5   # зазор между крышкой и верхней полкой кронштейна, мм

# Тело
L        = 150.0  # длина кронштейна, мм
WALL_T   = 12.0   # толщина боковой стенки, мм
PLATE_T  = 6.0    # толщина верхней полки, мм
GUSSET   = 4.0    # косынка стенка/полка, мм
BODY_Z0  = -5.0   # низ тела (плоскость разъёма с губкой), мм

# Нижняя губка (отдельная деталь, 2 болта М4)
JAW_L    = 40.0   # длина губки, мм
JAW_X    = 60.0   # центр губки по X, мм
JAW_H    = 17.0   # высота губки, мм
JAW_GAP  = 0.8    # зазор губка/тело в затянутом состоянии (натяг), мм
BOSS_OUT = 5.5    # утолщение тела наружу в зоне губки, мм
BOSS_TOP = 8.0    # верх утолщения, мм
BOLT_DX  = 13.0   # болты на ±BOLT_DX от центра губки, мм
BOLT_D   = 4.4    # отверстие под М4
HEAD_D, HEAD_H = 8.0, 3.0      # цековка под головку DIN 912
NUT_AF, NUT_H  = 7.2, 3.4      # паз под гайку М4 (7 мм по граням + зазор)
NUT_Z0   = 2.0    # низ паза гайки, мм

# Пикатинни (MIL-STD-1913)
RAIL_W, RAIL_NECK_W = 21.2, 15.67
RAIL_CH  = (RAIL_W - RAIL_NECK_W) / 2   # 45° фаска ≈ 2.77
RAIL_NECK_H = 1.5
RAIL_H   = RAIL_CH + RAIL_NECK_H
SLOT_W, SLOT_D, PITCH = 5.23, 3.0, 10.01
TOP_SLOTS = 14

# Боковая планка
SIDE_X0, SIDE_X1 = 55.0, 140.0
SIDE_ZC  = 20.0   # центр боковой планки по Z, мм
SIDE_HOLE_D = 6.0 # облегчающие отверстия (0 — выключить)

# ───────── ПРОИЗВОДНЫЕ ─────────
WALL_IN  = -(RECV_HALF_W + PLATE_BASE)      # внутренняя грань стенки (основание призмы)
WALL_OUT = WALL_IN - WALL_T
BOSS_Y   = WALL_OUT - BOSS_OUT
PLATE_Z0 = RECV_TOP + TOP_CLEAR
PLATE_Z1 = PLATE_Z0 + PLATE_T
PLATE_Y1 = RAIL_W / 2 + 2.0
BOLT_Y   = WALL_OUT - 0.5                    # ось болтов
JAW_X0, JAW_X1 = JAW_X - JAW_L / 2, JAW_X + JAW_L / 2


def yz_prism(points, x0, x1):
    """Полигон в плоскости YZ (точки (y, z)), вытянутый по X от x0 до x1."""
    return (cq.Workplane("YZ", origin=(x0, 0, 0))
            .polyline(points).close().extrude(x1 - x0))


def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY")
            .box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def dovetail_cutter(x0, x1):
    """Паз под призму (охватывающий), режется от внутренней грани стенки."""
    f = DT_FACE / 2 + DT_CLEAR
    b = DT_FACE / 2 + DT_DEPTH * math.tan(math.radians(DT_ANGLE)) + DT_CLEAR
    d = DT_DEPTH + DT_CLEAR
    pts = [(WALL_IN + 5, b), (WALL_IN, b), (WALL_IN - d, f),
           (WALL_IN - d, -f), (WALL_IN, -b), (WALL_IN + 5, -b)]
    return yz_prism(pts, x0, x1)


def picatinny(x0, x1, base_z, center_y, n_slots, direction="up"):
    """Планка Пикатинни вдоль X. direction: 'up' (верх +Z) или 'left' (верх −Y)."""
    length = x1 - x0
    h, w2, n2, ch = RAIL_H, RAIL_W / 2, RAIL_NECK_W / 2, RAIL_CH
    # профиль в локальных (u = поперёк, v = высота от основания)
    prof = [(-n2, 0), (n2, 0), (n2, RAIL_NECK_H), (w2, h), (-w2, h), (-n2, RAIL_NECK_H)]
    if direction == "up":
        pts = [(center_y + u, base_z + v) for u, v in prof]
    else:  # 'left': основание на плоскости Y = base_z... используем center как Z
        pts = [(base_z - v, center_y + u) for u, v in prof]
    rail = yz_prism(pts, x0, x1)
    span = (n_slots - 1) * PITCH
    start = x0 + (length - span) / 2
    for i in range(n_slots):
        xc = start + i * PITCH
        if direction == "up":
            cut = box(xc - SLOT_W / 2, xc + SLOT_W / 2, center_y - w2 - 1, center_y + w2 + 1,
                      base_z + h - SLOT_D, base_z + h + 1)
        else:
            cut = box(xc - SLOT_W / 2, xc + SLOT_W / 2, base_z - h - 1, base_z - h + SLOT_D,
                      center_y - w2 - 1, center_y + w2 + 1)
        rail = rail.cut(cut)
    return rail, start


def build_body():
    wall = box(0, L, WALL_OUT, WALL_IN, BODY_Z0, PLATE_Z1)
    plate = box(0, L, WALL_OUT, PLATE_Y1, PLATE_Z0, PLATE_Z1)
    gusset = yz_prism([(WALL_IN - 0.1, PLATE_Z0 - GUSSET), (WALL_IN - 0.1, PLATE_Z0 + 0.1),
                       (WALL_IN + GUSSET, PLATE_Z0 + 0.1)], 0, L)
    boss = box(JAW_X0, JAW_X1, BOSS_Y, WALL_OUT + 0.1, BODY_Z0, BOSS_TOP)
    body = wall.union(plate).union(gusset).union(boss)

    top_rail, _ = picatinny(0, L, PLATE_Z1 - 0.01, 0.0, TOP_SLOTS, "up")
    body = body.union(top_rail)

    n_side = int((SIDE_X1 - SIDE_X0) // PITCH)
    side_rail, s0 = picatinny(SIDE_X0, SIDE_X1, WALL_OUT + 0.01, SIDE_ZC, n_side, "left")
    body = body.union(side_rail)

    # паз под ласточкин хвост
    body = body.cut(dovetail_cutter(-1, L + 1))

    # облегчающие отверстия в боковой планке — в рёбрах между пазами
    if SIDE_HOLE_D > 0:
        for i in range(n_side - 1):
            xc = s0 + i * PITCH + PITCH / 2
            hole = (cq.Workplane("XZ", origin=(xc, 0, SIDE_ZC))
                    .circle(SIDE_HOLE_D / 2).extrude(60, both=True))
            body = body.cut(hole)

    # болты: отверстия + пазы под гайки (вставляются сбоку снаружи)
    for sx in (-1, 1):
        xc = JAW_X + sx * BOLT_DX
        hole = (cq.Workplane("XY", origin=(xc, BOLT_Y, BODY_Z0 - 1))
                .circle(BOLT_D / 2).extrude(BOSS_TOP + 1 - BODY_Z0 - 0.01))
        body = body.cut(hole)
        nut_ac = NUT_AF / math.cos(math.radians(30))  # по углам
        slot = box(xc - NUT_AF / 2, xc + NUT_AF / 2,
                   BOSS_Y - 1, BOLT_Y + nut_ac / 2, NUT_Z0, NUT_Z0 + NUT_H)
        body = body.cut(slot)

    # скругления внешних углов полки
    try:
        body = body.edges("|Z").edges(cq.selectors.BoxSelector(
            (-1, PLATE_Y1 - 1, PLATE_Z0 - 1), (L + 1, PLATE_Y1 + 1, PLATE_Z1 + 1))).fillet(3)
    except Exception:
        pass
    return body


def build_jaw():
    jaw = box(JAW_X0, JAW_X1, BOSS_Y, WALL_IN, BODY_Z0 - JAW_H, BODY_Z0)
    jaw = jaw.cut(dovetail_cutter(JAW_X0 - 1, JAW_X1 + 1))
    # натяг: срезаем верх губки, чтобы при затяжке оставался зазор
    jaw = jaw.cut(box(JAW_X0 - 1, JAW_X1 + 1, BOSS_Y - 1, WALL_IN + 1, BODY_Z0 - JAW_GAP, BODY_Z0 + 1))
    for sx in (-1, 1):
        xc = JAW_X + sx * BOLT_DX
        z0 = BODY_Z0 - JAW_H
        hole = cq.Workplane("XY", origin=(xc, BOLT_Y, z0 - 1)).circle(BOLT_D / 2).extrude(JAW_H + 2)
        cbore = cq.Workplane("XY", origin=(xc, BOLT_Y, z0 - 1)).circle(HEAD_D / 2).extrude(HEAD_H + 1)
        jaw = jaw.cut(hole).cut(cbore)
    return jaw


def build_test(body, jaw):
    """Короткий тест посадки: только зона губки, без полки и планок."""
    region = box(JAW_X0 - 0.01, JAW_X1 + 0.01, BOSS_Y - 1, WALL_IN + 1, BODY_Z0 - JAW_H - 1, 12)
    return body.intersect(region), jaw.intersect(region)


if __name__ == "__main__":
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
    os.makedirs(out, exist_ok=True)
    body = build_body()
    jaw = build_jaw()
    tbody, tjaw = build_test(body, jaw)

    cq.exporters.export(body, os.path.join(out, "mount_body.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(jaw, os.path.join(out, "mount_jaw.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(tbody, os.path.join(out, "test_body.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(tjaw, os.path.join(out, "test_jaw.stl"), tolerance=0.02, angularTolerance=0.1)

    asm = cq.Assembly()
    asm.add(body, name="body", color=cq.Color(0.6, 0.6, 0.6, 1.0))
    asm.add(jaw, name="jaw", color=cq.Color(0.35, 0.35, 0.35, 1.0))
    asm.save(os.path.join(out, "mount.step"))

    bb = body.val().BoundingBox()
    print(f"body: X {bb.xlen:.1f}  Y {bb.ylen:.1f}  Z {bb.zlen:.1f} mm, volume {body.val().Volume()/1000:.1f} cm3")
    bj = jaw.val().BoundingBox()
    print(f"jaw:  X {bj.xlen:.1f}  Y {bj.ylen:.1f}  Z {bj.zlen:.1f} mm, volume {jaw.val().Volume()/1000:.1f} cm3")
    print("dovetail: face", DT_FACE + 2*DT_CLEAR, "base",
          round(DT_FACE + 2*DT_DEPTH*math.tan(math.radians(DT_ANGLE)) + 2*DT_CLEAR, 2),
          "depth", DT_DEPTH + DT_CLEAR)
