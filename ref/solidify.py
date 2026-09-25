"""Игровая оболочка (obj) → сплошное тело (stl): растеризация треугольников в воксели 0.25 мм,
заливка внутренностей, marching cubes, лёгкое сглаживание."""
import trimesh, numpy as np, time
from scipy import ndimage
from skimage import measure
PITCH = 0.25
m = trimesh.load("ref/proto_sr3mp_mount.obj", force='mesh'); m.apply_scale(1000)
m.vertices = m.vertices[:, [1, 0, 2]]
lo = m.bounds[0] - 2; hi = m.bounds[1] + 2
shape = np.ceil((hi - lo) / PITCH).astype(int) + 1
grid = np.zeros(shape, dtype=bool)
t = time.time()
V = m.vertices; F = m.faces
step = PITCH / 2.2
for f in F:
    a, b, c = V[f]
    n = int(np.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / step)) + 1
    u = np.linspace(0, 1, n)
    uu, vv = np.meshgrid(u, u); mask = uu + vv <= 1
    uu, vv = uu[mask], vv[mask]
    pts = a + np.outer(uu, b - a) + np.outer(vv, c - a)
    idx = np.floor((pts - lo) / PITCH).astype(int)
    grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True
print("surface voxels", grid.sum(), round(time.time() - t, 1), "s")
filled = ndimage.binary_fill_holes(grid)
print("filled voxels", filled.sum(), "vol cm3", round(filled.sum() * PITCH ** 3 / 1000, 1))
verts, faces, _, _ = measure.marching_cubes(filled.astype(np.uint8), level=0.5)
verts = verts * PITCH + lo
mm = trimesh.Trimesh(verts, faces); mm.merge_vertices(); mm.fix_normals()
print("mc faces", len(mm.faces), "wt", mm.is_watertight, "vol", round(mm.volume / 1000, 1))
trimesh.smoothing.filter_laplacian(mm, lamb=0.5, iterations=5)
print("smoothed vol", round(mm.volume / 1000, 1), "wt", mm.is_watertight, np.round(mm.bounds, 1))
mm.export("ref/proto_solid.stl")
