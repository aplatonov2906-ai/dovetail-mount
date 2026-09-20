import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import Polygon, Rectangle, FancyBboxPatch

img = mpimg.imread("ref/frame_side.jpg")
fig = plt.figure(figsize=(13, 7.2))
fig.patch.set_facecolor("white")

# ─── левая панель: его кадр ───
ax = fig.add_axes([0.02, 0.05, 0.50, 0.9])
ax.imshow(img); ax.axis("off")
ax.set_title("Замеры ① и ③ — на боку автомата", fontsize=14, weight="bold")
R = "#ff2a2a"
def arrow(x0, y0, x1, y1):
    ax.annotate("", (x0, y0), (x1, y1), arrowprops=dict(arrowstyle="<->", color=R, lw=3, shrinkA=0, shrinkB=0))
def label(x, y, t, ha="left"):
    ax.text(x, y, t, color="white", fontsize=12, weight="bold", ha=ha, va="center",
            bbox=dict(fc=R, ec="none", boxstyle="round,pad=0.3"))
# полоска
ax.annotate("", (200, 243), (60, 330), arrowprops=dict(arrowstyle="->", color="#ffd400", lw=3))
ax.text(20, 350, "вот эта полоска", color="black", fontsize=12, weight="bold",
        bbox=dict(fc="#ffd400", ec="none", boxstyle="round,pad=0.3"))
# ① высота полоски
ax.plot([130, 175], [236, 236], color=R, lw=2); ax.plot([130, 175], [251, 251], color=R, lw=2)
arrow(140, 236, 140, 251)
label(60, 215, "① высота полоски")
# ③ от верха коробки до полоски
ax.plot([300, 370], [150, 150], color=R, lw=2); ax.plot([300, 370], [243, 243], color=R, lw=2)
arrow(355, 150, 355, 243)
label(300, 120, "③ от самого верха до полоски", ha="center")
ax.text(200, 385, "③ мерить от ВЕРХНЕГО края коробки, не от шва крышки",
        color="black", fontsize=10.5, ha="center", bbox=dict(fc="#eee", ec="none"))

# ─── правая панель: вид с торца ───
bx = fig.add_axes([0.55, 0.05, 0.43, 0.9]); bx.axis("off"); bx.set_xlim(0, 10); bx.set_ylim(0, 10)
bx.set_title("Замер ② — смотреть с торца, где полоска кончается", fontsize=14, weight="bold")
bx.add_patch(Rectangle((5, 1.2), 4, 7.5, fc="#c8c8c8", ec="k", lw=1.5))
bx.text(7, 5, "коробка\n(вид спереди,\nв торец)", ha="center", va="center", fontsize=12)
strip = Polygon([(5, 4.2), (4.2, 4.5), (4.2, 5.5), (5, 5.8)], closed=True, fc="#444", ec="k")
bx.add_patch(strip)
bx.text(4.0, 6.4, "полоска", ha="right", fontsize=12, weight="bold")
bx.annotate("", (4.2, 3.5), (5, 3.5), arrowprops=dict(arrowstyle="<->", color=R, lw=3, shrinkA=0, shrinkB=0))
bx.plot([4.2, 4.2], [3.3, 4.5], color=R, lw=2); bx.plot([5, 5], [3.3, 4.2], color=R, lw=2)
bx.text(4.6, 2.9, "② на сколько торчит", color="white", fontsize=12, weight="bold", ha="center",
        bbox=dict(fc=R, ec="none", boxstyle="round,pad=0.3"))
bx.text(0.2, 1.6, "④ И ОДНО ФОТО: сними полоску с торца,\n"
                  "как на этой картинке (смотришь вдоль\n"
                  "автомата спереди, крупно, с фонариком).\n"
                  "По фото я сам увижу, какой у неё\n"
                  "профиль — прямой или «ёлочкой».",
        fontsize=11.5, va="top", bbox=dict(fc="#fff3b0", ec="#e0c060", boxstyle="round,pad=0.5"))
plt.savefig("ref/measure_simple.png", dpi=110, facecolor="white")
