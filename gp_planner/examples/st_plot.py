import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import awesome_plot.style.figsize_utils as fu

block_region = pd.read_csv("block_region.csv", sep=',')
st_nodes = pd.read_csv("st_node.csv", sep=',')

fig, ax = plt.subplots(1, 1, figsize=fu.set_size('ral'))

for idx, row in block_region.iterrows():
    x = row[0:5]
    y = row[5:10]
    ax.plot(x, y, color='b', lw=1)

for idx, row in st_nodes.iterrows():
  [s0, v0, a, t0, tf] = row[0:5]
  t = np.linspace(0, tf, 10)
  s = s0 + v0 * t  + 0.5 * a * t**2
  t += t0
  ax.plot(t, s, lw=0.4, color='gray')

ax.tick_params(
    axis='both',
    which='both',
    bottom=False,
    top=False,
    labelbottom=False,
    right=False,
    left=False,
    labelleft=False)

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
    "ytick.labelsize": 8
}
plt.rcParams.update(tex_fonts)


ax.text(8, 10, 't')
ax.text(0.2, 100, 's')

fig.savefig("st_node.pdf", format='pdf',
            bbox_inches='tight', pad_inches=0, dpi=300)

plt.show()
