"""
Кронштейн на боковую планку «ласточкин хвост» с верхней и боковой планками
Пикатинни, по образцу прототипа с фото. Параметрическая модель CadQuery
для 3D-печати.

Схема прототипа: толстая вертикальная пластина стоит на боку коробки;
верхняя планка идёт по её верхнему ребру на брусе со сквозными отверстиями;
боковая планка на наружной грани спереди; задний нижний угол срезан
наискось, на конце среза упор с отверстием; нижняя губка отдельной деталью
на двух винтах сверху через мостик.

Система координат (как на оружии):
  X — вдоль ствола, 0 = задний торец кронштейна, +X вперёд
  Y — поперёк, 0 = ось ствола, −Y = левая сторона (там планка)
  Z — вверх, 0 = середина призмы «ласточкин хвост» на коробке

Запуск:  python mount.py   → out/*.stl, out/*.step
"""
import math
import os
import cadquery as cq

# ───────── ПАРАМЕТРЫ ─────────
# «Ласточкин хвост» на коробке (замеры друга 21.09.2026)
DT_FACE  = 15.0   # высота планки по лицевой грани, мм
DT_DEPTH = 5.5    # выступ планки над коробкой, мм (5–6)
DT_ANGLE = 45.0   # угол скосов, градусов (не мерили)
DT_CLEAR = 0.25   # зазор на печать по каждой грани, мм

# Коробка
RECV_HALF_W = 17.0  # половина ширины ствольной коробки, мм
RECV_TOP    = 50.0  # от середины планки до верха коробки (верх планки 42,5 + 15/2), мм
RELIEF      = 1.5   # отступ пластины от коробки выше зоны планки, мм
RELIEF_Z    = 15.0  # выше этой высоты пластина не касается коробки

# Пластина
L        = 150.0  # длина, мм
PLATE_T  = 12.0   # толщина пластины, мм
BODY_Z0  = -(DT_FACE / 2) + 1.0   # низ пластины (разъём с губкой): чуть выше нижнего скоса

# Брус под верхнюю планку (с 11 сквозными отверстиями, как у прототипа)
BAR_Z0   = RECV_TOP - 8.0   # низ бруса, мм
BAR_H    = 12.0             # высота бруса, мм
BAR_HOLE_D, BAR_HOLES, BAR_HOLE_PITCH = 7.0, 11, 12.0

# Задний срез и упор
REAR_CUT = 40.0            # длина косого среза заднего нижнего угла по X, мм
LUG_L, LUG_H, LUG_HOLE = 12.0, 10.0, 5.0

# Губка (отдельно, 2 винта М4×30 сверху через мостик)
JAW_L    = 42.0
JAW_X    = 86.0   # центр губки от заднего торца, мм
JAW_H    = 16.0
JAW_GAP  = 0.8    # натяг: зазор губка/тело при затяжке, мм
BOSS_OUT = 12.0   # мостик наружу от пластины, мм
BOLT_DX  = 13.0
BOLT_D   = 4.4
HEAD_D, HEAD_H = 8.0, 4.5      # цековка под головку DIN 912
NUT_AF, NUT_H  = 7.2, 3.4      # паз под гайку М4

# Пикатинни (MIL-STD-1913)
RAIL_W, RAIL_NECK_W = 21.2, 15.67
RAIL_CH  = (RAIL_W - RAIL_NECK_W) / 2
RAIL_NECK_H = 1.5
RAIL_H   = RAIL_CH + RAIL_NECK_H
SLOT_W, SLOT_D, PITCH = 5.23, 3.0, 10.01
TOP_SLOTS = 14

# Боковая планка (спереди, 8 пазов, отверстия в рёбрах)
SIDE_LEN = 92.0
SIDE_ZC  = 23.0
SIDE_HOLE_D = 6.0

# ───────── ПРОИЗВОДНЫЕ ─────────
WALL_IN  = -RECV_HALF_W                 # внутренняя грань пластины (сидит на коробке)
WALL_REL = WALL_IN - RELIEF             # внутренняя грань выше зоны планки
WALL_OUT = WALL_IN - PLATE_T
BAR_Z1   = BAR_Z0 + BAR_H
BAR_Y_IN, BAR_Y_OUT = WALL_REL, WALL_REL - RAIL_W   # брус шириной с планку, заподлицо изнутри
RAIL_YC  = (BAR_Y_IN + BAR_Y_OUT) / 2   # ось верхней планки
BOSS_Y   = WALL_OUT - BOSS_OUT
BOLT_Y   = WALL_OUT - 7.0               # ось винтов: снаружи от боковой планки, ключ проходит
BOSS_TOP = BODY_Z0 + 14.5
JAW_X0, JAW_X1 = JAW_X - JAW_L / 2, JAW_X + JAW_L / 2
SIDE_X1, SIDE_X0 = L - 3.0, L - 3.0 - SIDE_LEN


def yz_prism(points, x0, x1):
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(points).close().extrude(x1 - x0)


def xz_prism(points, y0, y1):
    """Полигон в плоскости XZ (точки (x, z)), вытянутый по Y от y0 до y1."""
    return (cq.Workplane("XZ", origin=(0, y0, 0)).polyline(points).close()
            .extrude(-(y1 - y0)))


def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


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
    """Планка вдоль X. 'up': основание на Z=base, ось Y=center. 'left': основание на Y=base, ось Z=center."""
    h, w2, n2 = RAIL_H, RAIL_W / 2, RAIL_NECK_W / 2
    prof = [(-n2, 0), (n2, 0), (n2, RAIL_NECK_H), (w2, h), (-w2, h), (-n2, RAIL_NECK_H)]
    if direction == "up":
        pts = [(center + u, base + v) for u, v in prof]
    else:
        pts = [(base - v, center + u) for u, v in prof]
    rail = yz_prism(pts, x0, x1)
    start = x0 + ((x1 - x0) - (n_slots - 1) * PITCH) / 2
    for i in range(n_slots):
        xc = start + i * PITCH
        if direction == "up":
            cut = box(xc - SLOT_W / 2, xc + SLOT_W / 2, center - w2 - 1, center + w2 + 1,
                      base + h - SLOT_D, base + h + 1)
        else:
            cut = box(xc - SLOT_W / 2, xc + SLOT_W / 2, base - h - 1, base - h + SLOT_D,
                      center - w2 - 1, center + w2 + 1)
        rail = rail.cut(cut)
    return rail, start


def build_body():
    # пластина: внизу прижата к коробке, выше — с отступом
    plate = (box(0, L, WALL_OUT, WALL_IN, BODY_Z0, RELIEF_Z)
             .union(box(0, L, WALL_OUT, WALL_REL, RELIEF_Z - 0.1, BAR_Z0 + 0.1)))
    bar = box(0, L, BAR_Y_OUT, BAR_Y_IN, BAR_Z0, BAR_Z1)
    body = plate.union(bar)

    top_rail, _ = picatinny(0, L, BAR_Z1 - 0.01, RAIL_YC, TOP_SLOTS, "up")
    body = body.union(top_rail)

    # мостик под винты губки
    boss = box(JAW_X0, JAW_X1, BOSS_Y, WALL_OUT + 0.1, BODY_Z0, BOSS_TOP)
    body = body.union(boss)

    # боковая планка спереди
    n_side = int(SIDE_LEN // PITCH)
    side_rail, s0 = picatinny(SIDE_X0, SIDE_X1, WALL_OUT + 0.01, SIDE_ZC, n_side, "left")
    body = body.union(side_rail)

    # косой срез заднего нижнего угла (брус с планкой остаётся во всю длину)
    z_top = BAR_Z0 - 3.0
    slope = REAR_CUT / (z_top - BODY_Z0)
    cut = xz_prism([(-1, z_top), (-1, BODY_Z0 - 30), (REAR_CUT + 30 * slope, BODY_Z0 - 30)],
                   BOSS_Y - 5, WALL_IN + 5)
    body = body.cut(cut)

    # упор с отверстием на конце среза (не касается коробки: заподлицо с дном паза)
    lug = box(REAR_CUT, REAR_CUT + LUG_L, WALL_OUT, WALL_IN - DT_DEPTH - DT_CLEAR - 1.0,
              BODY_Z0 - LUG_H, BODY_Z0 + 0.1)
    body = body.union(lug).cut(cyl_y(REAR_CUT + LUG_L / 2, BODY_Z0 - LUG_H / 2, LUG_HOLE))

    # паз под ласточкин хвост
    body = body.cut(dovetail_cutter(-1, L + 1))

    # сквозные отверстия в брусе
    zc = (BAR_Z0 + BAR_Z1) / 2
    x0 = (L - (BAR_HOLES - 1) * BAR_HOLE_PITCH) / 2
    for i in range(BAR_HOLES):
        body = body.cut(cyl_y(x0 + i * BAR_HOLE_PITCH, zc, BAR_HOLE_D))

    # отверстия в боковой планке — в рёбрах между пазами, насквозь через пластину
    for i in range(n_side - 1):
        body = body.cut(cyl_y(s0 + i * PITCH + PITCH / 2, SIDE_ZC, SIDE_HOLE_D))

    # винты сверху: цековка под головку + отверстие через мостик
    for sx in (-1, 1):
        xc = JAW_X + sx * BOLT_DX
        hole = cq.Workplane("XY", origin=(xc, BOLT_Y, BODY_Z0 - 1)).circle(BOLT_D / 2).extrude(BOSS_TOP - BODY_Z0 + 2)
        cbore = cq.Workplane("XY", origin=(xc, BOLT_Y, BOSS_TOP - HEAD_H)).circle(HEAD_D / 2).extrude(HEAD_H + 1)
        body = body.cut(hole).cut(cbore)
    return body


def build_jaw():
    z0 = BODY_Z0 - JAW_H
    jaw = box(JAW_X0, JAW_X1, BOSS_Y, WALL_IN, z0, BODY_Z0)
    jaw = jaw.cut(dovetail_cutter(JAW_X0 - 1, JAW_X1 + 1))
    jaw = jaw.cut(box(JAW_X0 - 1, JAW_X1 + 1, BOSS_Y - 1, WALL_IN + 1, BODY_Z0 - JAW_GAP, BODY_Z0 + 1))
    nut_ac = NUT_AF / math.cos(math.radians(30))
    nut_z0 = z0 + 4.0
    for sx in (-1, 1):
        xc = JAW_X + sx * BOLT_DX
        hole = cq.Workplane("XY", origin=(xc, BOLT_Y, z0 - 1)).circle(BOLT_D / 2).extrude(JAW_H + 2)
        slot = box(xc - NUT_AF / 2, xc + NUT_AF / 2, BOSS_Y - 1, BOLT_Y + nut_ac / 2, nut_z0, nut_z0 + NUT_H)
        jaw = jaw.cut(hole).cut(slot)
    return jaw


def build_test(body, jaw):
    """Короткий тест посадки: только зона губки с мостиком."""
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
    print("rail axis offset from bore:", round(-RAIL_YC, 1), "mm; rail top Z:", round(BAR_Z1 + RAIL_H, 1))
