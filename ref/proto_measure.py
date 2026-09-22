import trimesh, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
m = trimesh.load("ref/proto_sr3mp_mount.obj", force='mesh'); m.apply_scale(1000)
m.vertices = m.vertices[:, [1, 0, 2]]
V = m.vertices; F = m.faces
def silhouette(a, b, name, ax):
    lo, hi = V[:, [a, b]].min(0) - 2, V[:, [a, b]].max(0) + 2
    res = 4  # px/mm
    W, H = int((hi[0]-lo[0])*res), int((hi[1]-lo[1])*res)
    img = Image.new("1", (W, H), 0); d = ImageDraw.Draw(img)
    for f in F:
        p = [((V[i, a]-lo[0])*res, (hi[1]-V[i, b])*res) for i in f]
        d.polygon(p, fill=1)
    arr = np.array(img)
    ax.imshow(arr, cmap="gray_r", extent=[lo[0], hi[0], lo[1], hi[1]], origin="upper")
    ax.set_xticks(np.arange(np.floor(lo[0]/10)*10, hi[0], 10)); ax.set_yticks(np.arange(np.floor(lo[1]/10)*10, hi[1], 10))
    ax.grid(True, lw=0.4, color="tab:red", alpha=0.6); ax.set_title(name); ax.set_aspect("equal")
    return arr, lo, hi, res
fig, axs = plt.subplots(2, 1, figsize=(16, 14))
arr, lo, hi, res = silhouette(0, 2, "XZ silhouette (X along, Z up), mm", axs[0])
# по столбцам: верх и низ силуэта
xs = np.arange(lo[0], hi[0], 1/res)
print("X   Ztop  Zbot")
for x in range(int(lo[0])+2, int(hi[0])-1, 5):
    col = arr[:, int((x-lo[0])*res)]
    zz = hi[1] - np.where(col)[0]/res
    if len(zz): print(f"{x:5d} {zz.max():6.1f} {zz.min():6.1f}")
silhouette(1, 2, "YZ silhouette (Y across, Z up), mm", axs[1])
plt.tight_layout(); plt.savefig("ref/proto_silhouette.png", dpi=70)

# сечения YZ в нескольких X — как полилинии
fig, axs = plt.subplots(2, 4, figsize=(20, 11))
for ax, x in zip(axs.flat, [-55, -40, -25, -10, 5, 25, 45, 70]):
    s = m.section(plane_origin=[x, 0, 0], plane_normal=[1, 0, 0])
    if s is not None:
        for e in s.discrete: ax.plot(e[:, 1], e[:, 2], color="0.2", lw=1)
    ax.set_xticks(range(-35, 15, 5)); ax.set_yticks(range(-15, 55, 5))
    ax.grid(True, lw=0.4); ax.set_aspect("equal"); ax.set_title(f"X={x}")
plt.tight_layout(); plt.savefig("ref/proto_sections.png", dpi=60)
