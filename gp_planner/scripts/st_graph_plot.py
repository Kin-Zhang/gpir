import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import awesome_plot.style.figsize_utils as fu
import sys

print(sys.argv[0])

ref_v = sys.argv[2]

block_region = pd.read_csv("../data/block_region_" + sys.argv[1] + ".csv", sep=",")
st_nodes = pd.read_csv("../data/st_node_" + sys.argv[1] + ".csv", sep=",")

fig, ax = plt.subplots(1, 1, figsize=fu.set_size("ral"))
# axin = ax.inset_axes([0.1, 0.55, 0.35, 0.35])

start_s = 0
start_v = 0

# s-t nodes
for idx, row in st_nodes.iterrows():
    [s0, v0, a, t0, tf] = row[0:5]
    if idx == 0:
        start_s = s0
        start_v = v0
    t = np.linspace(0, tf, 10)
    s = s0 + v0 * t + 0.5 * a * t ** 2
    t += t0
    [color, lw, zorder] = ["gray", 0.4, 1]
    if row[-2] == 1:
        [color, lw, zorder] = ["orangered", 1.0, 120]
    ax.plot(t, s, lw=lw, color=color, zorder=zorder)
    # axin.plot(t, s, lw=lw, color=color, zorder=zorder)

# block region
for idx, row in block_region.iterrows():
    x = row[0:5]
    y = row[5:10]
    ax.plot(x, y, color="b", lw=1)

# boundary
bound_color = "k"
t = np.linspace(0, 6.5, 100)
s_upper = start_s + start_v * t + 0.5 * 2 * t ** 2
ax.plot(t, s_upper, lw=1.5, color=bound_color)
s_max = start_s + start_v ** 2 / 8
ax.plot([start_v / 4, 8], [s_max, s_max], lw=1.5, color=bound_color)
t = np.linspace(0, 5, 100)
s_lower = start_s + start_v * t - 0.5 * 4 * t ** 2
ax.plot(t, s_lower, lw=1.5, color=bound_color)

reference = start_s + float(ref_v) * t
ax.plot(t, reference, lw=1.5, color='b')

zoom_color = "darkblue"

# axin.set_xlim(2.5, 3.5)
# axin.set_ylim(36, 46)
# patch, pp1, pp2 = mark_inset(
#     ax, axin, loc1=3, loc2=1, fc="none", ec=zoom_color, lw=1, zorder=100
# )
# pp1.loc2 = 2
# pp2.loc1 = 4
# pp2.loc2 = 1

fu.set_axis_off(ax)
# fu.set_axis_off(axin)

# fu.set_axis_color(axin, zoom_color)

tex_fonts = {
    # Use LaTeX to write all text
    "text.usetex": True,
    "font.family": "Times New Roman",
    # Use 10pt font in plots, to match 10pt font in document
    "axes.labelsize": 10,
    "font.size": 10,
    # Make the legend/label fonts a little smaller
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
}
plt.rcParams.update(tex_fonts)
# ax.set_xticks(np.linspace(0, 8, 9))
# ax.set_yticks(np.linspace(0, 120, 60))
# ax.grid(alpha=0.3)
ax.text(8, 15, "t")
ax.text(-0.1, 160, "s")

# fig.savefig(
#     "../figure/st_node.pdf", format="pdf", bbox_inches="tight", pad_inches=0, dpi=300
# )

plt.show()
