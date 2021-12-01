import pandas as pd
import matplotlib.pyplot as plt
import pathlib
import prettytable
import numpy as np

from awesome_plot.style import figsize_utils
from matplotlib.patches import Polygon
from tqdm import tqdm


def draw_polygon(ax, top_left, bottom_right):
    polygon = [
        top_left,
        [bottom_right[0], top_left[1]],
        bottom_right,
        [top_left[0], bottom_right[1]],
    ]
    ax.add_patch(Polygon(polygon, color="w", fill=True, alpha=1, zorder=100, lw=0.1))
    ax.add_patch(
        Polygon(
            polygon,
            closed=True,
            color="royalblue",
            fill=False,
            hatch="\\\\\\\\\\\\\\\\",
            lw=0.1,
            zorder=101,
        )
    )
    ax.add_patch(
        Polygon(polygon, closed=True, color="k", fill=False, lw=0.15, zorder=102)
    )


root_path = pathlib.Path("/home/cj/research/gpir_ws/src/gpir/gp_planner/data/")
env_path = root_path / "env"
info_path = root_path / "info"
res_path = root_path / "results"

total_trial = len(list(info_path.glob("*")))

plt.style.use(["science"])


success_num = {"GPCur": 0, "DLIAPS": 0, "TDROBCA": 0}
avg_time = {"GPCur": list(), "DLIAPS": list(), "TDROBCA": list()}

for i in tqdm(range(total_trial)):
    file_name = "{}.csv".format(i)
    df_info = pd.read_csv(info_path / file_name, sep=",")
    for index, row in df_info.iterrows():
        success_num[row["planner"]] += row["success"]
        avg_time[row["planner"]].append(row["time"])

    # fig, axes = plt.subplots(2, 1, figsize=figsize_utils.set_size("ral"))
    # df_env = pd.read_csv(env_path / file_name, sep=",")
    # df_res = pd.read_csv(res_path / file_name, sep=",")
    # for planner in df_res.planner.unique():
    #     sub_df = df_res[df_res.planner == planner]
    #     if planner == "GPNoCur":
    #         label = "G3P"
    #     elif planner == "DLIAPS":
    #         label = "DL-IAPS"
    #     elif planner == "GPCur":
    #         label = "G3P-Cur"
    #     elif planner == "TDROBCA":
    #         label = "TDR-OBCA"
    #     axes[0].plot(sub_df.x, sub_df.y, label=label)
    #     axes[1].plot(sub_df.x, sub_df.k, label=label)

    # for index, row in df_env.iterrows():
    #     draw_polygon(
    #         axes[0],
    #         [row["left_top_x"], row["left_top_y"]],
    #         [row["right_bottom_x"], row["right_bottom_y"]],
    #     )

    # plt.rc("legend", fontsize=5)  # legend fontsize
    # plt.rc("font", size=6)  # fontsize of the tick labels
    # plt.rc("font", weight=2)  # fontsize of the tick labels
    # plt.rc("font", family="Times New Roman")  # fontsize of the tick labels

    # for ax in axes:
    #     ax.legend()
    #     ax.xaxis.label.set_size(7)
    #     ax.yaxis.label.set_size(7)
    #     ax.tick_params(axis="both", which="major", labelsize=6)
    #     ax.minorticks_off()

    # plt.show()

for key in avg_time.keys():
    avg_time[key] = np.array(avg_time[key])

tab = prettytable.PrettyTable()
tab.field_names = ["Planner", "Success rate", "Avg. time", "std"]
tab.add_rows(
    [
        [
            "GP-Cur",
            success_num["GPCur"] / total_trial,
            avg_time["GPCur"].mean(),
            avg_time["GPCur"].std(),
        ],
        [
            "DL-IAPS",
            success_num["DLIAPS"] / total_trial,
            avg_time["DLIAPS"].mean(),
            avg_time["DLIAPS"].std(),
        ],
        [
            "TDR-OBCA",
            success_num["TDROBCA"] / total_trial,
            avg_time["TDROBCA"].mean(),
            avg_time["TDROBCA"].std(),
        ],
    ]
)

print(tab)
