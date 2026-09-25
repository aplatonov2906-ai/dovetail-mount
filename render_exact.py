import trimesh, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
body = trimesh.load("out/exact_body.stl"); jaw = trimesh.load("out/exact_jaw.stl")
bs = body.simplify_quadric_decimation(face_count=60000)
views = [("iso_front_left", 25, 135), ("iso_rear_right", 25, -40), ("side_left", 0, 90), ("top", 89, 90), ("front_end", 0, 0)]
fig = plt.figure(figsize=(18, 11))
light = np.array([0.4,0.7,0.6]); light/=np.linalg.norm(light)
for i,(name, el, az) in enumerate(views):
    ax = fig.add_subplot(2, 3, i+1, projection="3d")
    for m, c in ((bs, (0.6,0.6,0.62)), (jaw, (0.85,0.45,0.2))):
        sh = np.clip(np.abs(m.face_normals@light), 0, 1)*0.7+0.3
        pc = Poly3DCollection(m.vertices[m.faces], edgecolors="none")
        pc.set_facecolor(np.c_[np.outer(sh, c), np.ones(len(sh))]); ax.add_collection3d(pc)
    allv = np.vstack([bs.vertices, jaw.vertices]); c0 = allv.mean(0); r = (allv.max(0)-allv.min(0)).max()/2
    ax.set_xlim(c0[0]-r, c0[0]+r); ax.set_ylim(c0[1]-r, c0[1]+r); ax.set_zlim(c0[2]-r, c0[2]+r)
    ax.view_init(el, az); ax.set_axis_off(); ax.set_title(name)
ax = fig.add_subplot(2,3,6)
for m, c in ((body,"0.3"), (jaw,"tab:orange")):
    s = m.section(plane_origin=[83.6,0,0], plane_normal=[1,0,0])
    if s is not None:
        for e in s.discrete: ax.plot(e[:,1], e[:,2], color=c, lw=1.2)
ax.axhline(50, color="tab:blue", lw=1, ls="--"); ax.axvline(19.25, color="tab:blue", lw=1, ls="--")
ax.text(-5, 51, "верх коробки", color="tab:blue", fontsize=8)
ax.set_aspect("equal"); ax.grid(True, lw=0.3); ax.set_title("section YZ через губку (Y across, Z up)")
plt.tight_layout(); plt.savefig("out/preview_exact.png", dpi=90)
