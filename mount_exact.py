"""
Кронштейн ОДИН В ОДИН с игровой модели прототипа (ref/proto_sr3mp_mount.obj →
ref/proto_solid.stl, см. ref/solidify.py). К форме ничего не добавлено.

У исходника уже есть полный зажим «ласточкин хвост» (ref/proto_native_clamp.png):
зуб на теле сверху, губа на родной губке снизу, канал между ними 14 мм у стенки,
сужается к коробке — планка ШИРЕ у лица (15 мм) и уже у корня. Планка друга
(лицо 15, выступ 5,5) ложится в родной канал почти без реза: снимается ~0,7 мм
с корня зуба и ~0,7 мм с корня губы, чтобы получить 15,5 (15 + зазор). Углы
скосов — родные, с исходника (сверху ~22°, снизу ~16° от лица). Губка отделена по
родному стыку, высота родная. Винты М4×14 (или ×16, кончик выйдет на 1,5 мм) сверху
через родной мостик, гайки в губке.

Выход: out/exact_body.stl, out/exact_jaw.stl, out/exact_test_body.stl, out/exact_test_jaw.stl
Система координат выхода — как у mount.py: X вперёд, +Y — левая сторона (там планка), Z вверх,
0 по Z = середина планки на коробке.
"""
import math, os, tempfile
import numpy as np
import trimesh
import cadquery as cq

# ───── параметры планки на коробке (замеры друга) ─────
DT_FACE, DT_DEPTH, DT_CLEAR = 15.0, 5.5, 0.25
TOP_ANGLE, BOT_ANGLE = 22.0, 16.0   # подрез планки от лица к корню, град.; сняты с родного зуба и губы исходника
RAIL_AXIS_Z = -1.65                 # ось планки в системе исходника: середина родного канала (корень зуба 5.33, корень губы −8.66)
RECV_HALF_W = 19.25

# ───── зона зажима (в системе игровой модели: X вдоль, −Y к коробке, Z вверх) ─────
JAW_X0, JAW_X1 = -19.0, 22.0          # родная губка оригинала
JAW_GAP = 0.8                         # натяг: зазор губка/тело при затяжке
BOLT_DX, BOLT_D = 13.0, 4.4
HEAD_D, HEAD_H = 7.6, 4.5
NUT_AF, NUT_H = 7.2, 3.4

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


def dovetail_cutter(y_in, zc, x0, x1):
    """Паз, режется от внутренней грани y_in вглубь тела (+Y) на DT_ENGAGE; ось паза на Z = zc."""
    f = DT_FACE / 2 + DT_CLEAR
    b = DT_FACE / 2 + DT_ENGAGE * math.tan(math.radians(DT_ANGLE)) + DT_CLEAR
    d = DT_ENGAGE + DT_CLEAR
    return yz_prism([(y_in - 5, zc + b), (y_in, zc + b), (y_in + d, zc + f), (y_in + d, zc - f), (y_in, zc - b), (y_in - 5, zc - b)], x0, x1)


def main():
    solid = trimesh.load(os.path.join(HERE, "ref", "proto_solid.stl"), force="mesh")
    print("proto solid:", len(solid.faces), "faces, vol", round(solid.volume / 1000, 1), "cm3")

    # геометрия низа по сечению вне зоны губки: стенка (дно канала), низ тела; мостик — в зоне губки
    sec = np.vstack(solid.section(plane_origin=[-40, 0, 0], plane_normal=[1, 0, 0]).discrete)
    y_floor = sec[(sec[:, 2] > 0.4) & (sec[:, 2] < 1.8)][:, 1].min()     # стенка под зубом = дно канала
    y_tip = sec[(sec[:, 2] > 3.0) & (sec[:, 2] < 7.0)][:, 1].min()       # кончик зуба
    y_out = sec[(sec[:, 2] > 2) & (sec[:, 2] < 9)][:, 1].max()
    z_bot = sec[sec[:, 1] > -6][:, 2].min()
    sec0 = np.vstack(solid.section(plane_origin=[1.5, 0, 0], plane_normal=[1, 0, 0]).discrete)
    boss = sec0[(sec0[:, 2] > z_bot + 1) & (sec0[:, 2] < z_bot + 9) & (sec0[:, 1] > y_out + 2)]
    y_boss, z_boss = boss[:, 1].max(), boss[:, 2].max()
    y_jaw_in = sec0[(sec0[:, 2] > -13) & (sec0[:, 2] < -3)][:, 1].min()  # внутренняя грань родной губки
    zc = RAIL_AXIS_Z
    fl = y_floor + DT_CLEAR
    f = DT_FACE / 2 + DT_CLEAR
    print(f"floor Y={y_floor:.2f}, tooth tip Y={y_tip:.2f}, jaw inner Y={y_jaw_in:.2f}, outer Y={y_out:.2f}, body bottom Z={z_bot:.2f}, "
          f"boss Y={y_boss:.2f} top Z={z_boss:.2f}; rail axis Z={zc:.2f}, face Z {zc-f:.2f}..{zc+f:.2f} at floor")

    # канал правильной стороны: широкий у дна (лицо планки), сужается к коробке
    tt, tb = math.tan(math.radians(TOP_ANGLE)), math.tan(math.radians(BOT_ANGLE))
    yo = y_jaw_in - 4.0   # резак уходит за внутренние грани (там воздух)
    d = fl - yo
    cutter = lambda x0, x1: yz_prism([(fl, zc + f), (fl, zc - f), (yo, zc - f + d * tb), (yo, zc + f - d * tt)], x0, x1)

    # 1. губка — родной блок, родная высота
    jaw_region = box(JAW_X0 - 1, JAW_X1 + 1, y_jaw_in - 3, y_boss + 2, z_bot - 30, z_bot)
    jaw = inter(solid, jaw_region)
    body = diff(solid, jaw_region)
    # 2. канал — во всю длину, в теле (зуб) и в губке (губа)
    body = diff(body, cutter(-70, 90))
    jaw = diff(jaw, cutter(JAW_X0 - 2, JAW_X1 + 2))
    jaw = diff(jaw, box(JAW_X0 - 2, JAW_X1 + 2, y_jaw_in - 4, y_boss + 3, z_bot - JAW_GAP, z_bot + 1))
    # 3. винты сверху через родной мостик; гайки в губке
    nut_ac = NUT_AF / math.cos(math.radians(30))
    bolt_y = fl + 2.0 + nut_ac / 2
    jaw_xc = (JAW_X0 + JAW_X1) / 2
    z_jaw0 = jaw.bounds[0][2]
    nut_z0 = z_jaw0 + 3.5
    for sx in (-1, 1):
        xc = jaw_xc + sx * BOLT_DX
        body = diff(body, cyl_z(xc, bolt_y, BOLT_D, z_bot - 1, z_boss + 1))
        body = diff(body, cyl_z(xc, bolt_y, HEAD_D, z_boss - HEAD_H, z_boss + 1))
        jaw = diff(jaw, cyl_z(xc, bolt_y, BOLT_D, z_jaw0 - 1, z_bot + 1))
        jaw = diff(jaw, box(xc - NUT_AF / 2, xc + NUT_AF / 2, bolt_y - nut_ac / 2, y_boss + 3, nut_z0, nut_z0 + NUT_H))
    print(f"bolt axis Y={bolt_y:.2f} (head edge {bolt_y + HEAD_D/2:.2f} vs boss {y_boss:.2f}); jaw Z {z_jaw0:.2f}..{z_bot:.2f} (native); "
          f"bolt M4x14: tip at Z={z_boss - HEAD_H - 14:.1f}, nut Z {nut_z0:.1f}..{nut_z0 + NUT_H:.1f}")

    region = box(JAW_X0 - 0.01, JAW_X1 + 0.01, y_jaw_in - 4, y_boss + 3, z_jaw0 - 1, zc + 16)
    tbody, tjaw = inter(body, region), inter(jaw, region)

    # в систему оружия: X вперёд (перед — где боковая планка, −X), +Y — левая сторона;
    # дно канала = лицевая грань планки (RECV_HALF_W + DT_DEPTH), ось планки → Z = 0
    x_rear = body.bounds[1][0]
    dy = RECV_HALF_W + DT_DEPTH - fl
    T = np.array([[-1, 0, 0, x_rear], [0, 1, 0, dy], [0, 0, 1, -zc], [0, 0, 0, 1]], dtype=float)
    os.makedirs(OUT, exist_ok=True)
    np.savetxt(os.path.join(OUT, "exact_transform.txt"), T)
    for name, m in (("exact_body", body), ("exact_jaw", jaw), ("exact_test_body", tbody), ("exact_test_jaw", tjaw)):
        m = m.copy(); m.apply_transform(T); m.fix_normals()
        if len(m.faces) > 250000:
            m = m.simplify_quadric_decimation(face_count=220000)
            m.update_faces(m.nondegenerate_faces()); m.merge_vertices()
            trimesh.repair.fill_holes(m); m.fix_normals()
            assert m.is_watertight, name
        m.export(os.path.join(OUT, name + ".stl"))
        b = m.bounds
        print(f"{name}: wt={m.is_watertight} faces={len(m.faces)} vol={m.volume/1000:.1f} cm3 "
              f"X {b[0][0]:.1f}..{b[1][0]:.1f} Y {b[0][1]:.1f}..{b[1][1]:.1f} Z {b[0][2]:.1f}..{b[1][2]:.1f}")


if __name__ == "__main__":
    main()
