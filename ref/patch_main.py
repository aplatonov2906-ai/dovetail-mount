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
    # 3. канал — во всю длину, в теле (зуб) и в губке (губа); натяг губки
    body = diff(body, cutter(-70, 90))
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
        if len(m.faces) > 250000:
            m = m.simplify_quadric_decimation(face_count=220000)
            m.update_faces(m.nondegenerate_faces()); m.merge_vertices()
            trimesh.repair.fill_holes(m); m.fix_normals()
            assert m.is_watertight or name == "exact_assembly", name
        m.export(os.path.join(OUT, name + ".stl"))
        b = m.bounds
        print(f"{name}: wt={m.is_watertight} faces={len(m.faces)} vol={m.volume/1000:.1f} cm3 "
              f"X {b[0][0]:.1f}..{b[1][0]:.1f} Y {b[0][1]:.1f}..{b[1][1]:.1f} Z {b[0][2]:.1f}..{b[1][2]:.1f}")
