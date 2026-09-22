import trimesh, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
m = trimesh.load("ref/proto_sr3mp_mount.obj", force='mesh'); m.apply_scale(1000)
# в игре: X поперёк, Y вдоль, Z вверх → приводим к нашим осям: X вдоль, Y поперёк, Z вверх
m.vertices = m.vertices[:, [1, 0, 2]]
print("extents", np.round(m.extents,1), "bounds", np.round(m.bounds,1))
views = [("iso", 25, -135), ("iso2", 25, 45), ("side -Y", 0, -90), ("side +Y", 0, 90), ("top", 89, -90), ("end +X", 0, 0)]
fig = plt.figure(figsize=(18, 12))
light = np.array([0.4,-0.7,0.6]); light/=np.linalg.norm(light)
for i,(name, el, az) in enumerate(views):
    ax = fig.add_subplot(3, 3, i+1, projection="3d")
    n = m.face_normals; sh = np.clip(np.abs(n@light), 0, 1)*0.7+0.3
    pc = Poly3DCollection(m.vertices[m.faces], edgecolors="none")
    pc.set_facecolor(np.c_[np.outer(sh, [0.6,0.6,0.62]), np.ones(len(sh))]); ax.add_collection3d(pc)
    c0 = m.bounds.mean(0); r = m.extents.max()/2
    ax.set_xlim(c0[0]-r, c0[0]+r); ax.set_ylim(c0[1]-r, c0[1]+r); ax.set_zlim(c0[2]-r, c0[2]+r)
    ax.view_init(el, az); ax.set_axis_off(); ax.set_title(name)
for j, x in enumerate([m.bounds[0][0]+40, m.bounds[0][0]+75, m.bounds[0][0]+110]):
    ax = fig.add_subplot(3, 3, 7+j)
    s = m.section(plane_origin=[x,0,0], plane_normal=[1,0,0])
    if s is not None:
        for e in s.discrete: ax.plot(e[:,1], e[:,2], color="0.2", lw=1)
    ax.set_aspect("equal"); ax.grid(True, lw=0.3); ax.set_title(f"section X={x:.0f} (Y across, Z up)")
plt.tight_layout(); plt.savefig("ref/proto_views.png", dpi=80)
