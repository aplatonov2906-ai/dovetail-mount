"""Точная разница между исходником (залитое тело игровой модели) и exact_body+exact_jaw:
что убрано, что добавлено, где, сколько."""
import trimesh, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
T = np.loadtxt("out/exact_transform.txt")
orig = trimesh.load("ref/proto_solid.stl", force="mesh"); orig.apply_transform(T); orig.fix_normals()
body = trimesh.load("out/exact_body.stl"); jaw = trimesh.load("out/exact_jaw.stl")
ours = trimesh.boolean.union([body, jaw], engine="manifold")
removed = trimesh.boolean.difference([orig, ours], engine="manifold")
added = trimesh.boolean.difference([ours, orig], engine="manifold")
def report(name, m):
    parts = [p for p in m.split(only_watertight=False) if abs(p.volume) > 8]
    parts.sort(key=lambda p: -abs(p.volume))
    print(f"== {name}: {len(parts)} зон (>8 mm3), всего {sum(abs(p.volume) for p in parts)/1000:.2f} cm3")
    for p in parts[:12]:
        b = p.bounds
        print(f"   {abs(p.volume):7.0f} mm3  X {b[0][0]:6.1f}..{b[1][0]:6.1f}  Y {b[0][1]:5.1f}..{b[1][1]:5.1f}  Z {b[0][2]:6.1f}..{b[1][2]:6.1f}")
    return parts
rem = report("УБРАНО (есть в исходнике, нет у нас)", removed)
add = report("ДОБАВЛЕНО (нет в исходнике, есть у нас)", added)
removed = trimesh.util.concatenate(rem) if rem else trimesh.Trimesh()
added = trimesh.util.concatenate(add) if add else trimesh.Trimesh()
# картинка: исходник серым полупрозрачно, убрано — красным, добавлено — зелёным; вид с торца и сбоку-изнутри
fig = plt.figure(figsize=(16, 7))
light = np.array([0.3, 0.6, 0.75]); light /= np.linalg.norm(light)
os_ = orig.simplify_quadric_decimation(face_count=40000)
for i, (el, az, t) in enumerate([(0, 0, "с торца (спереди)"), (25, -140, "изнутри, со стороны коробки")]):
    ax = fig.add_subplot(1, 2, i + 1, projection="3d")
    for m, c, a in ((os_, (0.75, 0.75, 0.78), 0.25), (removed, (0.9, 0.15, 0.1), 1.0), (added, (0.1, 0.7, 0.2), 1.0)):
        if len(m.faces) == 0: continue
        sh = np.clip(np.abs(m.face_normals @ light), 0, 1) * 0.7 + 0.3
        pc = Poly3DCollection(m.vertices[m.faces], edgecolors="none")
        pc.set_facecolor(np.c_[np.outer(sh, c), np.full(len(sh), a)]); ax.add_collection3d(pc)
    c0 = np.array([83, 27, 0]); r = 30
    ax.set_xlim(c0[0]-r, c0[0]+r); ax.set_ylim(c0[1]-r, c0[1]+r); ax.set_zlim(c0[2]-r, c0[2]+r)
    ax.view_init(el, az); ax.set_axis_off(); ax.set_title(t + "  (красное — убрано, зелёное — добавлено)")
plt.tight_layout(); plt.savefig("out/diff_exact.png", dpi=90)
