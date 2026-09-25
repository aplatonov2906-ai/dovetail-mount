"""
Кронштейн ОДИН В ОДИН с игровой модели прототипа (ref/proto_sr3mp_mount.obj →
ref/proto_solid.stl, см. ref/solidify.py).

Что есть у исходника и используется как есть:
  - родной зажим «ласточкин хвост»: зуб на теле сверху, губа на родной губке снизу,
    канал 14 мм у стенки, сужается к коробке (планка ШИРЕ у лица). Планка друга
    (лицо 15, выступ 5,5) ложится после снятия ~0,7 мм с корня зуба и губы;
  - родная губка: скруглённый брусок; на его дне два конических гнезда под головки
    винтов, из верха торчат два стержня Ø4,2 (винты) — ровно под пазами боковой планки.
Крепёж по исходнику: два винта М3×25 с потайной головкой (DIN 7991) СНИЗУ, головки в
родных гнёздах на дне губки, стержни видны в зазоре между губкой и телом.
Единственное добавление: над стержнями, под боковой планкой, у исходника пустота (в игре
стержни висят в воздухе) — добавлены две втулки Ø9 с карманом под гайку М3 снизу, чтобы
винтам было во что вкручиваться. Они в тени под планкой, не выходят за силуэт губки.

Выход: out/exact_body.stl, out/exact_jaw.stl, out/exact_test_body.stl, out/exact_test_jaw.stl,
       out/exact_assembly.stl — обе детали в сборе, только для просмотра.
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
BOLT_D = 3.4                          # отверстие под М3
CSK_D, CSK_H = 6.4, 1.5               # потай под головку DIN 7991 М3 (Ø6) на дне губки
NUT_AF, NUT_H = 5.7, 2.7              # карман под гайку М3 (5,5 по граням) во втулке, снизу
COL_D, COL_Z0, COL_Z1 = 9.0, 7.0, 10.8   # втулки над стержнями, под боковой планкой
POST_D = 4.24                         # родные стержни Ø4,2 — заменяются настоящими винтами

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



def main():
    solid = trimesh.load(os.path.join(HERE, "ref", "proto_solid.stl"), force="mesh")
    print("proto solid:", len(solid.faces), "faces, vol", round(solid.volume / 1000, 1), "cm3")

    # геометрия низа по сечению вне зоны губки: стенка (дно канала), кончик зуба, низ тела
    sec = np.vstack(solid.section(plane_origin=[-40, 0, 0], plane_normal=[1, 0, 0]).discrete)
    y_floor = sec[(sec[:, 2] > 0.4) & (sec[:, 2] < 1.8)][:, 1].min()
    y_tip = sec[(sec[:, 2] > 3.0) & (sec[:, 2] < 7.0)][:, 1].min()
    y_out = sec[(sec[:, 2] > 2) & (sec[:, 2] < 9)][:, 1].max()
    z_bot = sec[sec[:, 1] > -6][:, 2].min()
    sec0 = np.vstack(solid.section(plane_origin=[1.5, 0, 0], plane_normal=[1, 0, 0]).discrete)
    jawband = sec0[(sec0[:, 2] > -13) & (sec0[:, 2] < -3)]
    y_jaw_in, y_jaw_out = jawband[:, 1].min(), jawband[:, 1].max()
    # родные стержни (винты): центры по срезу Z=6.5
    posts = []
    for e in solid.section(plane_origin=[0, 0, 6.5], plane_normal=[0, 0, 1]).discrete:
        c = e.mean(0)
        if -30 < c[0] < 35 and c[1] > 3.5 and (e[:, 0].max() - e[:, 0].min()) < 6:
            posts.append((float(c[0]), float(c[1])))
    posts.sort()
    zc = RAIL_AXIS_Z
    fl = y_floor + DT_CLEAR
    f = DT_FACE / 2 + DT_CLEAR
    print(f"floor Y={y_floor:.2f}, tooth tip Y={y_tip:.2f}, jaw Y {y_jaw_in:.2f}..{y_jaw_out:.2f}, wall outer Y={y_out:.2f}, "
          f"body bottom Z={z_bot:.2f}; rail axis Z={zc:.2f}; posts: {[(round(x, 2), round(y, 2)) for x, y in posts]}")
    assert len(posts) == 2

    # канал правильной стороны: широкий у дна (лицо планки), сужается к коробке
    tt, tb = math.tan(math.radians(TOP_ANGLE)), math.tan(math.radians(BOT_ANGLE))
    yo = y_jaw_in - 4.0
    d = fl - yo
    cutter = lambda x0, x1: yz_prism([(fl, zc + f), (fl, zc - f), (yo, zc - f + d * tb), (yo, zc + f - d * tt)], x0, x1)

    # 1. губка — родной брусок целиком: ниже низа тела + его скруглённый верх снаружи стенки (до стержней)
    jaw_region = union(box(JAW_X0 - 1.5, JAW_X1 + 1.5, y_jaw_in - 3, y_jaw_out + 2, z_bot - 30, z_bot),
                       box(JAW_X0 - 1.5, JAW_X1 + 1.5, y_out + 0.3, y_jaw_out + 2, z_bot - 0.1, 4.7))
    jaw = inter(solid, jaw_region)
    body = diff(solid, jaw_region)
    # 2. родные стержни — это винты: убираем их из сетки (их место займут настоящие М3)
    for (px, py) in posts:
        post_cut = cyl_z(px, py, POST_D + 1.2, 4.0, 8.6)
        body = diff(body, post_cut)
        jaw = diff(jaw, post_cut)
    # 3. канал — от передка до заднего упора (упор — родной ограничитель, его не трогаем); натяг губки
    lug_x0 = 37.5
    body = diff(body, cutter(-70, lug_x0))
    jaw = diff(jaw, cutter(JAW_X0 - 2, JAW_X1 + 2))
    jaw = diff(jaw, box(JAW_X0 - 2, JAW_X1 + 2, y_jaw_in - 4, y_out + 0.3, z_bot - JAW_GAP, z_bot + 1))
    # 4. втулки под гайки над стержнями (единственное добавление) + отверстия
    z_jaw0 = jaw.bounds[0][2]
    for (px, py) in posts:
        body = union(body, cyl_z(px, py, COL_D, COL_Z0, COL_Z1))
        body = diff(body, cyl_z(px, py, BOLT_D, COL_Z0 - 1, COL_Z1 + 6))      # сквозь втулку, выходит в паз планки
        hexp = cq_to_tm(cq.Workplane("XY", origin=(px, py, COL_Z0 - 0.1))
                        .polygon(6, NUT_AF / math.cos(math.radians(30))).extrude(NUT_H + 0.1))
        body = diff(body, hexp)                                                 # карман под гайку снизу втулки
        jaw = diff(jaw, cyl_z(px, py, BOLT_D, z_jaw0 - 1, z_bot + 6))          # отверстие сквозь губку
        csk = cq_to_tm(cq.Workplane("XY", origin=(px, py, z_jaw0 - 0.5)).circle(CSK_D / 2 + 0.5)
                       .workplane(offset=CSK_H + 0.5).circle(BOLT_D / 2).loft())
        jaw = diff(jaw, csk)                                                    # потай под головку на дне
    print(f"jaw Z {z_jaw0:.2f}..{jaw.bounds[1][2]:.2f}; screw M3x25 from Z={z_jaw0:.1f}: tip at {z_jaw0 + 25:.1f}, "
          f"nut Z {COL_Z0:.1f}..{COL_Z0 + NUT_H:.1f}; visible shaft Z 4.4..{COL_Z0}")

    region = box(JAW_X0 - 0.01, JAW_X1 + 0.01, y_jaw_in - 4, y_jaw_out + 3, z_jaw0 - 1, zc + 16)
    tbody, tjaw = inter(body, region), inter(jaw, region)

    # в систему оружия: X вперёд (перед — где боковая планка, −X), +Y — левая сторона;
    # дно канала = лицевая грань планки (RECV_HALF_W + DT_DEPTH), ось планки → Z = 0
    x_rear = body.bounds[1][0]
    dy = RECV_HALF_W + DT_DEPTH - fl
    T = np.array([[-1, 0, 0, x_rear], [0, 1, 0, dy], [0, 0, 1, -zc], [0, 0, 0, 1]], dtype=float)
    os.makedirs(OUT, exist_ok=True)
    np.savetxt(os.path.join(OUT, "exact_transform.txt"), T)
    asm = trimesh.util.concatenate([body, jaw])
    for name, m in (("exact_body", body), ("exact_jaw", jaw), ("exact_test_body", tbody),
                    ("exact_test_jaw", tjaw), ("exact_assembly", asm)):
        m = m.copy(); m.apply_transform(T); m.fix_normals()
        if name != "exact_assembly":                              # осколки булевых операций — выбросить
            m = max(m.split(only_watertight=False), key=lambda p: len(p.faces))
            m.update_faces(m.nondegenerate_faces()); m.merge_vertices(); trimesh.repair.fill_holes(m); m.fix_normals()
        if len(m.faces) > 250000:
            m = m.simplify_quadric_decimation(face_count=220000)
            m.update_faces(m.nondegenerate_faces()); m.merge_vertices()
            trimesh.repair.fill_holes(m); m.fix_normals()
            assert m.is_watertight or name == "exact_assembly", name
        m.export(os.path.join(OUT, name + ".stl"))
        b = m.bounds
        print(f"{name}: wt={m.is_watertight} faces={len(m.faces)} vol={m.volume/1000:.1f} cm3 "
              f"X {b[0][0]:.1f}..{b[1][0]:.1f} Y {b[0][1]:.1f}..{b[1][1]:.1f} Z {b[0][2]:.1f}..{b[1][2]:.1f}")


if __name__ == "__main__":
    main()
