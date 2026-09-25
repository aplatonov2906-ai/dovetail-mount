"""
Кронштейн ОДИН В ОДИН с игровой модели прототипа (ref/proto_sr3mp_mount.obj →
ref/proto_solid.stl, см. ref/solidify.py). Форма не перерисовывается: в готовое
тело режется паз под планку, добавляется толщина только за пазом, отделяется
губка, сверлятся отверстия под болты.

Правки относительно оригинала (только в зоне зажима):
  - полоса вдоль низа (Z −6,5…+1) и утолщение снаружи под боковой планкой
    (до Z 12) — иначе за пазом глубиной 5,75 не остаётся стенки (у оригинала 5 мм);
  - в зоне губки мостик выступает наружу на 13,5 вместо 9,5 — под гайки М4;
  - болты М4×20 снизу через губку в гайки, гайки вставляются сбоку в пазы.

Выход: out/exact_body.stl, out/exact_jaw.stl, out/exact_test_body.stl, out/exact_test_jaw.stl
Система координат выхода — как у mount.py: X вперёд, +Y — левая сторона (там планка), Z вверх,
0 по Z = середина планки на коробке.
"""
import math, os, tempfile
import numpy as np
import trimesh
import cadquery as cq

# ───── параметры планки на коробке (замеры друга) ─────
DT_FACE, DT_DEPTH, DT_ANGLE, DT_CLEAR = 15.0, 5.5, 45.0, 0.25
RECV_HALF_W = 19.25

# ───── зона зажима (в системе игровой модели: X вдоль, −Y к коробке, Z вверх) ─────
BODY_Z0 = -(DT_FACE / 2) + 1.0        # разъём тело/губка
JAW_X0, JAW_X1 = -19.0, 22.0          # губка как у оригинала
JAW_H, JAW_GAP = 14.0, 0.8
STRIP_X0, STRIP_X1 = -57.0, 41.0      # полоса вдоль низа: от передка до упора
PAD_TOP = 12.0                        # утолщение снаружи до низа боковой планки
BOSS_OUT_Y, BOSS_TOP = 13.5, 5.0      # мостик в зоне губки
BOLT_DX, BOLT_D = 13.0, 4.4
HEAD_D, HEAD_H = 8.0, 4.5
NUT_AF, NUT_H, NUT_Z0 = 7.2, 3.4, 0.0

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
TMP = tempfile.mkdtemp()


def cq_to_tm(obj):
    p = os.path.join(TMP, f"c{abs(hash(id(obj)))}.stl")
    cq.exporters.export(obj, p, tolerance=0.01, angularTolerance=0.05)
    m = trimesh.load(p, force="mesh")
    assert m.is_watertight
    return m


def box(x0, x1, y0, y1, z0, z1):
    return cq_to_tm(cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0)))


def yz_prism(points, x0, x1):
    return cq_to_tm(cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(points).close().extrude(x1 - x0))


def cyl_z(x, y, d, z0, z1):
    return cq_to_tm(cq.Workplane("XY", origin=(x, y, z0)).circle(d / 2).extrude(z1 - z0))


def union(a, b):  return trimesh.boolean.union([a, b], engine="manifold")
def diff(a, b):   return trimesh.boolean.difference([a, b], engine="manifold")
def inter(a, b):  return trimesh.boolean.intersection([a, b], engine="manifold")


def dovetail_cutter(y_in, x0, x1):
    """Паз, режется от внутренней грани y_in вглубь тела (+Y)."""
    f = DT_FACE / 2 + DT_CLEAR
    b = DT_FACE / 2 + DT_DEPTH * math.tan(math.radians(DT_ANGLE)) + DT_CLEAR
    d = DT_DEPTH + DT_CLEAR
    return yz_prism([(y_in - 5, b), (y_in, b), (y_in + d, f), (y_in + d, -f), (y_in, -b), (y_in - 5, -b)], x0, x1)


def main():
    solid = trimesh.load(os.path.join(HERE, "ref", "proto_solid.stl"), force="mesh")
    print("proto solid:", len(solid.faces), "faces, vol", round(solid.volume / 1000, 1), "cm3")

    # внутренняя/наружная грань нижней стенки — по сечению вне зоны губки
    sec = solid.section(plane_origin=[-40, 0, 0], plane_normal=[1, 0, 0])
    pts = np.vstack(sec.discrete)
    band = pts[(pts[:, 2] > 2) & (pts[:, 2] < 9)]
    y_in, y_out = band[:, 1].min(), band[:, 1].max()
    print(f"lower wall: inner face Y={y_in:.2f}, outer face Y={y_out:.2f}")

    body = solid
    # 1. убрать родную губку (ниже разъёма) — только в её зоне
    body = diff(body, box(JAW_X0 - 1, JAW_X1 + 1, -40, 40, -30, BODY_Z0))
    # 2. полоса вдоль низа + утолщение снаружи под боковой планкой
    body = union(body, box(STRIP_X0, STRIP_X1, y_in, y_out + 7.8, BODY_Z0, 1.0))
    body = union(body, box(STRIP_X0, STRIP_X1, y_out - 0.3, y_out + 7.8, BODY_Z0, PAD_TOP))
    # 3. мостик под гайки в зоне губки
    body = union(body, box(JAW_X0, JAW_X1, y_out - 0.3, BOSS_OUT_Y, BODY_Z0, BOSS_TOP))
    # 4. паз под планку — во всю длину
    body = diff(body, dovetail_cutter(y_in, -70, 90))
    # 5. болты: отверстия снизу, пазы под гайки сбоку снаружи
    bolt_y = y_in + DT_DEPTH + DT_CLEAR + 2.0 + NUT_AF / math.cos(math.radians(30)) / 2
    jaw_xc = (JAW_X0 + JAW_X1) / 2
    for sx in (-1, 1):
        xc = jaw_xc + sx * BOLT_DX
        body = diff(body, cyl_z(xc, bolt_y, BOLT_D, BODY_Z0 - 1, BOSS_TOP - 0.8))
        nut_ac = NUT_AF / math.cos(math.radians(30))
        body = diff(body, box(xc - NUT_AF / 2, xc + NUT_AF / 2, bolt_y - nut_ac / 2, BOSS_OUT_Y + 1, NUT_Z0, NUT_Z0 + NUT_H))
    print(f"bolt axis Y={bolt_y:.2f} (channel floor at {y_in + DT_DEPTH + DT_CLEAR:.2f})")

    # губка
    z0 = BODY_Z0 - JAW_H
    jaw = box(JAW_X0, JAW_X1, y_in, BOSS_OUT_Y, z0, BODY_Z0)
    jaw = diff(jaw, dovetail_cutter(y_in, JAW_X0 - 1, JAW_X1 + 1))
    jaw = diff(jaw, box(JAW_X0 - 1, JAW_X1 + 1, y_in - 1, BOSS_OUT_Y + 1, BODY_Z0 - JAW_GAP, BODY_Z0 + 1))
    for sx in (-1, 1):
        xc = jaw_xc + sx * BOLT_DX
        jaw = diff(jaw, cyl_z(xc, bolt_y, BOLT_D, z0 - 1, BODY_Z0 + 1))
        jaw = diff(jaw, cyl_z(xc, bolt_y, HEAD_D, z0 - 1, z0 + HEAD_H))

    # тест посадки: зона губки
    region = box(JAW_X0 - 0.01, JAW_X1 + 0.01, y_in - 1, BOSS_OUT_Y + 1, z0 - 1, PAD_TOP + 2)
    tbody, tjaw = inter(body, region), inter(jaw, region)

    # в систему оружия: X вперёд (у модели перед — где боковая планка, −X), +Y — левая сторона
    x_rear = body.bounds[1][0]   # задний торец → X = 0
    T = np.array([[-1, 0, 0, x_rear], [0, 1, 0, RECV_HALF_W - y_in], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
    os.makedirs(OUT, exist_ok=True)
    for name, m in (("exact_body", body), ("exact_jaw", jaw), ("exact_test_body", tbody), ("exact_test_jaw", tjaw)):
        m = m.copy(); m.apply_transform(T); m.fix_normals()
        if len(m.faces) > 250000:   # marching cubes даёт избыточную сетку — упрощаем без потери формы
            m = m.simplify_quadric_decimation(face_count=220000)
            m.merge_vertices(); m.fix_normals()
        m.export(os.path.join(OUT, name + ".stl"))
        b = m.bounds
        print(f"{name}: wt={m.is_watertight} faces={len(m.faces)} vol={m.volume/1000:.1f} cm3 "
              f"X {b[0][0]:.1f}..{b[1][0]:.1f} Y {b[0][1]:.1f}..{b[1][1]:.1f} Z {b[0][2]:.1f}..{b[1][2]:.1f}")


if __name__ == "__main__":
    main()
