"""
Кронштейн на боковую планку «ласточкин хвост» по образцу кронштейна ТОЧМАШ
для СР-3МП (форма снята с игровой модели ref/proto_sr3mp_mount.obj).
Параметрическая модель CadQuery для 3D-печати.

Схема прототипа: от паза стенка идёт вверх и дугой загибается внутрь, над
коробкой; на вершине дуги верхняя планка Пикатинни, сквозь её основание
ряд из 11 отверстий; боковая планка на 8 пазов на наружной грани спереди;
задний нижний угол срезан наискось, на конце среза упор с отверстием;
нижняя губка отдельной деталью на двух винтах сверху через мостик.

Отличия от прототипа под реплику друга: паз под планку 15 мм с выступом 5,5
(в игровой модели паза нет), нижняя стенка толще (11 мм, чтобы за пазом
осталось тело), дуга поднята — планка на его коробке сидит низко.

Система координат (как на оружии):
  X — вдоль ствола, 0 = задний торец, +X вперёд
  Y — поперёк, 0 = ось ствола, −Y = левая сторона (там планка)
  Z — вверх, 0 = середина планки «ласточкин хвост» на коробке

Запуск:  python mount.py   → out/*.stl, out/*.step
"""
import math
import os
import cadquery as cq

# ───────── ПАРАМЕТРЫ ─────────
# Планка на коробке (замеры друга 21.09.2026)
DT_FACE  = 15.0   # высота планки по лицевой грани, мм
DT_DEPTH = 5.5    # выступ планки над коробкой, мм
DT_ANGLE = 45.0   # угол скосов (не мерили)
DT_CLEAR = 0.25   # зазор на печать по каждой грани, мм

# Коробка
RECV_HALF_W = 17.0  # половина ширины коробки, мм
RECV_TOP    = 50.0  # от середины планки до верха коробки, мм
RELIEF      = 1.5   # отступ стенки от коробки выше зоны планки, мм
RELIEF_Z    = 16.0

# Корпус
L        = 147.0  # длина по верхней планке (как у прототипа), мм
PLATE_L  = 144.0  # длина стенки (планка выступает вперёд на 3 мм)
T_LOW    = 11.0   # толщина нижней стенки (зона паза и боковой планки), мм
T_ARC    = 6.5    # толщина дуги, мм
ARC_R    = 13.0   # радиус дуги по внутренней (вогнутой) стороне, мм
ARC_Z0   = RECV_TOP - 2.0   # где вертикальная стенка переходит в дугу, мм
STEP_Z   = 36.0   # выше этой высоты стенка тонкая (переход фаской), мм
FLANGE_T = 5.0    # полка под верхней планкой, мм
BODY_Z0  = -(DT_FACE / 2) + 1.0   # низ стенки (разъём с губкой)

# Верхняя планка и отверстия в её основании
TOP_SLOTS = 14
BAR_HOLE_D, BAR_HOLES, BAR_HOLE_PITCH = 6.0, 11, 12.0

# Задний срез (силуэт прототипа) и упор
REAR_PTS = [(0.0, 33.0), (33.0, 22.0), (39.0, None)]   # (X, Z) по силуэту; None = до низа стенки
LUG_X0, LUG_L, LUG_H, LUG_HOLE = 37.0, 8.0, 8.0, 4.5

# Губка и мостик
JAW_X0, JAW_X1 = 63.0, 104.0
JAW_H    = 14.0
JAW_GAP  = 0.8
BOSS_OUT = 10.5
BOSS_TOP = 5.0
BOLT_DX  = 13.0
BOLT_D   = 4.4
HEAD_D, HEAD_H = 8.0, 4.5
NUT_AF, NUT_H  = 7.2, 3.4

# Пикатинни (MIL-STD-1913)
RAIL_W, RAIL_NECK_W = 21.2, 15.67
RAIL_CH  = (RAIL_W - RAIL_NECK_W) / 2
RAIL_NECK_H = 1.5
RAIL_H   = RAIL_CH + RAIL_NECK_H
SLOT_W, SLOT_D, PITCH = 5.23, 3.0, 10.01

# Боковая планка (спереди, 8 пазов, 8 отверстий)
SIDE_X0, SIDE_X1 = 55.0, 140.0
SIDE_ZC  = 23.0
SIDE_HOLE_D = 6.0

# ───────── ПРОИЗВОДНЫЕ ─────────
WALL_IN  = -RECV_HALF_W
WALL_REL = WALL_IN - RELIEF
WALL_OUT = WALL_IN - T_LOW
ARC_CY   = WALL_REL + ARC_R          # центр дуги по Y (над коробкой)
APEX_Z   = ARC_Z0 + ARC_R            # вершина внутренней дуги
TOP_Z    = APEX_Z + T_ARC            # верх дуги = основание планки
RAIL_YC  = ARC_CY                    # ось верхней планки
BOSS_Y   = WALL_OUT - BOSS_OUT
BOLT_Y   = WALL_OUT - 6.0
JAW_X    = (JAW_X0 + JAW_X1) / 2
C45, S45 = math.cos(math.radians(45)), math.sin(math.radians(45))


def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def yz_prism(points, x0, x1):
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(points).close().extrude(x1 - x0)


def xz_prism(points, y0, y1):
    return cq.Workplane("XZ", origin=(0, y0, 0)).polyline(points).close().extrude(-(y1 - y0))


def cyl_y(x, z, d, y0=-60, y1=20):
    return cq.Workplane("XZ", origin=(x, y0, z)).circle(d / 2).extrude(-(y1 - y0))


def dovetail_cutter(x0, x1):
    f = DT_FACE / 2 + DT_CLEAR
    b = DT_FACE / 2 + DT_DEPTH * math.tan(math.radians(DT_ANGLE)) + DT_CLEAR
    d = DT_DEPTH + DT_CLEAR
    pts = [(WALL_IN + 5, b), (WALL_IN, b), (WALL_IN - d, f),
           (WALL_IN - d, -f), (WALL_IN, -b), (WALL_IN + 5, -b)]
    return yz_prism(pts, x0, x1)


def picatinny(x0, x1, base, center, n_slots, direction="up"):
    h, w2, n2 = RAIL_H, RAIL_W / 2, RAIL_NECK_W / 2
    prof = [(-n2, 0), (n2, 0), (n2, RAIL_NECK_H), (w2, h), (-w2, h), (-n2, RAIL_NECK_H)]
    pts = [(center + u, base + v) for u, v in prof] if direction == "up" else [(base - v, center + u) for u, v in prof]
    rail = yz_prism(pts, x0, x1)
    start = x0 + ((x1 - x0) - (n_slots - 1) * PITCH) / 2
    for i in range(n_slots):
        xc = start + i * PITCH
        if direction == "up":
            cut = box(xc - SLOT_W / 2, xc + SLOT_W / 2, center - w2 - 1, center + w2 + 1, base + h - SLOT_D, base + h + 1)
        else:
            cut = box(xc - SLOT_W / 2, xc + SLOT_W / 2, base - h - 1, base - h + SLOT_D, center - w2 - 1, center + w2 + 1)
        rail = rail.cut(cut)
    return rail, start


def plate_profile(x0, x1):
    """Сечение стенки: вертикаль у коробки → дуга внутрь над коробкой → вершина под планкой."""
    ri, ro = ARC_R, ARC_R + T_ARC
    wp = (cq.Workplane("YZ", origin=(x0, 0, 0))
          .moveTo(WALL_IN, BODY_Z0)
          .lineTo(WALL_IN, RELIEF_Z)
          .lineTo(WALL_REL, RELIEF_Z)
          .lineTo(WALL_REL, ARC_Z0)
          .threePointArc((ARC_CY - ri * C45, ARC_Z0 + ri * S45), (ARC_CY, APEX_Z))
          .lineTo(ARC_CY, TOP_Z)
          .threePointArc((ARC_CY - ro * C45, ARC_Z0 + ro * S45), (WALL_REL - T_ARC, ARC_Z0))
          .lineTo(WALL_REL - T_ARC, STEP_Z)
          .lineTo(WALL_OUT, STEP_Z - (WALL_REL - T_ARC - WALL_OUT))
          .lineTo(WALL_OUT, BODY_Z0)
          .close())
    return wp.extrude(x1 - x0)


def build_body():
    body = plate_profile(0, PLATE_L)
    # полка под планкой и сама планка — во всю длину
    flange = box(0, L, RAIL_YC - RAIL_W / 2, RAIL_YC + RAIL_W / 2, TOP_Z - FLANGE_T, TOP_Z)
    top_rail, _ = picatinny(0, L, TOP_Z - 0.01, RAIL_YC, TOP_SLOTS, "up")
    body = body.union(flange).union(top_rail)

    # мостик под винты губки
    body = body.union(box(JAW_X0, JAW_X1, BOSS_Y, WALL_OUT + 0.1, BODY_Z0, BOSS_TOP))

    # боковая планка
    n_side = int((SIDE_X1 - SIDE_X0) // PITCH)
    side_rail, s0 = picatinny(SIDE_X0, SIDE_X1, WALL_OUT + 0.01, SIDE_ZC, n_side, "left")
    body = body.union(side_rail)

    # задний срез по силуэту прототипа (планка с полкой остаются во всю длину)
    (xa, za), (xb, zb), (xc, _) = REAR_PTS
    cut = xz_prism([(-1, za), (xb, zb), (xc, BODY_Z0), (xc, BODY_Z0 - 30), (-1, BODY_Z0 - 30)],
                   BOSS_Y - 5, 5)
    body = body.cut(cut)

    # упор с отверстием на конце среза (не касается коробки)
    lug = box(LUG_X0, LUG_X0 + LUG_L, WALL_OUT, WALL_IN - DT_DEPTH - DT_CLEAR - 1.0, BODY_Z0 - LUG_H, BODY_Z0 + 0.1)
    body = body.union(lug).cut(cyl_y(LUG_X0 + LUG_L / 2 + 1.0, BODY_Z0 + 7.0, LUG_HOLE))

    # паз под ласточкин хвост
    body = body.cut(dovetail_cutter(-1, L + 1))

    # 11 отверстий сквозь основание верхней планки
    x0 = (L - (BAR_HOLES - 1) * BAR_HOLE_PITCH) / 2
    for i in range(BAR_HOLES):
        body = body.cut(cyl_y(x0 + i * BAR_HOLE_PITCH, TOP_Z + 1.0, BAR_HOLE_D))

    # 8 отверстий боковой планки: 7 рёбер между пазами + переднее ребро
    for i in range(n_side - 1):
        body = body.cut(cyl_y(s0 + i * PITCH + PITCH / 2, SIDE_ZC, SIDE_HOLE_D))
    body = body.cut(cyl_y(s0 + (n_side - 1) * PITCH + PITCH / 2, SIDE_ZC, SIDE_HOLE_D))

    # винты сверху: цековка + отверстие через мостик
    for sx in (-1, 1):
        xc = JAW_X + sx * BOLT_DX
        body = body.cut(cq.Workplane("XY", origin=(xc, BOLT_Y, BODY_Z0 - 1)).circle(BOLT_D / 2).extrude(BOSS_TOP - BODY_Z0 + 2))
        body = body.cut(cq.Workplane("XY", origin=(xc, BOLT_Y, BOSS_TOP - HEAD_H)).circle(HEAD_D / 2).extrude(HEAD_H + 1))
    return body


def build_jaw():
    z0 = BODY_Z0 - JAW_H
    jaw = box(JAW_X0, JAW_X1, BOSS_Y, WALL_IN, z0, BODY_Z0)
    jaw = jaw.cut(dovetail_cutter(JAW_X0 - 1, JAW_X1 + 1))
    jaw = jaw.cut(box(JAW_X0 - 1, JAW_X1 + 1, BOSS_Y - 1, WALL_IN + 1, BODY_Z0 - JAW_GAP, BODY_Z0 + 1))
    nut_ac = NUT_AF / math.cos(math.radians(30))
    nut_z0 = z0 + 3.5
    for sx in (-1, 1):
        xc = JAW_X + sx * BOLT_DX
        jaw = jaw.cut(cq.Workplane("XY", origin=(xc, BOLT_Y, z0 - 1)).circle(BOLT_D / 2).extrude(JAW_H + 2))
        jaw = jaw.cut(box(xc - NUT_AF / 2, xc + NUT_AF / 2, BOSS_Y - 1, BOLT_Y + nut_ac / 2, nut_z0, nut_z0 + NUT_H))
    return jaw


def build_test(body, jaw):
    region = box(JAW_X0 - 0.01, JAW_X1 + 0.01, BOSS_Y - 1, WALL_IN + 1, BODY_Z0 - JAW_H - 1, RELIEF_Z)
    return body.intersect(region), jaw.intersect(region)


if __name__ == "__main__":
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
    os.makedirs(out, exist_ok=True)
    body, jaw = build_body(), build_jaw()
    tbody, tjaw = build_test(body, jaw)
    for name, obj in (("mount_body", body), ("mount_jaw", jaw), ("test_body", tbody), ("test_jaw", tjaw)):
        cq.exporters.export(obj, os.path.join(out, name + ".stl"), tolerance=0.02, angularTolerance=0.1)
    asm = cq.Assembly()
    asm.add(body, name="body", color=cq.Color(0.6, 0.6, 0.6, 1.0))
    asm.add(jaw, name="jaw", color=cq.Color(0.35, 0.35, 0.35, 1.0))
    asm.save(os.path.join(out, "mount.step"))
    for name, obj in (("body", body), ("jaw", jaw)):
        bb = obj.val().BoundingBox()
        print(f"{name}: X {bb.xlen:.1f}  Y {bb.ylen:.1f}  Z {bb.zlen:.1f} mm, volume {obj.val().Volume()/1000:.1f} cm3")
    print(f"rail axis Y = {RAIL_YC:.1f} (offset from bore {-RAIL_YC:.1f} mm); rail top Z = {TOP_Z + RAIL_H:.1f}; "
          f"above receiver top: {TOP_Z + RAIL_H - RECV_TOP:.1f} mm")
