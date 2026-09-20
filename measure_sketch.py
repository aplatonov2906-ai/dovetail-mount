import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
fig, ax = plt.subplots(figsize=(7, 8))
# receiver (section), rail strip on the left side
ax.add_patch(Rectangle((0, 0), 34, 60, fc="#bbb", ec="k"))
strip = Polygon([(0, 24), (-2, 23), (-2, 20), (0, 19)], closed=True, fc="#666", ec="k")
ax.add_patch(strip)
def dim(x0, y0, x1, y1, txt, off=(0, 0)):
    ax.annotate("", (x0, y0), (x1, y1), arrowprops=dict(arrowstyle="<->", color="red", lw=1.4))
    ax.text((x0 + x1) / 2 + off[0], (y0 + y1) / 2 + off[1], txt, color="red", fontsize=13, ha="center", va="center",
            bbox=dict(fc="white", ec="none"))
dim(-4.5, 20, -4.5, 23, "A", (-1.5, 0))     # face height
dim(-7.5, 19, -7.5, 24, "B", (-1.5, 0))     # base height
dim(-2, 16.5, 0, 16.5, "C", (0, -1.5))      # protrusion
dim(-11, 21.5, -11, 60, "D", (-1.5, 0))     # to receiver top
dim(0, 63, 34, 63, "E", (0, 1.5))           # receiver width
ax.text(17, 30, "ствольная\nкоробка\n(сечение)", ha="center", va="center", fontsize=11)
ax.text(-2, 27, "полоска\n«ласточкин хвост»", ha="right", va="bottom", fontsize=10)
ax.text(-14, 5, "A — высота полоски по наружной (узкой) грани\n"
                "B — высота у основания (под скосами), если есть поднутрение\n"
                "C — на сколько полоска выступает над коробкой\n"
                "D — от середины полоски до верха коробки\n"
                "E — ширина коробки\n"
                "+ угол скосов, если не 45°, и длина полоски",
        fontsize=9.5, va="top", ha="left")
ax.set_xlim(-16, 40); ax.set_ylim(-14, 67); ax.set_aspect("equal"); ax.axis("off")
ax.set_title("Что замерить штангенциркулем (мм)")
plt.tight_layout(); plt.savefig("ref/measure.png", dpi=110)
