import json
import plotly.graph_objs as go
import numpy as np


def get_traces(data: dict, x_axis):

    colors = [
        "rgb(0, 0, 0)",        "rgb(255, 0, 75)",     "rgb(0, 102, 204)", 
        "rgb(1, 150, 180)",    "rgb(204, 0, 204)",    "rgb(102, 51, 153)",
        "rgb(255, 128, 0)",    "rgb(108, 20, 255)",   "rgb(128, 128, 128)", 
        "rgb(255, 204, 0)",    "rgb(255, 153, 51)",   "rgb(153, 0, 0)", 
        "rgb(76, 0, 153)",     "rgb(0, 150, 255)",    "rgb(0, 255, 0)", 
        "rgb(255, 0, 255)",    "rgb(255, 102, 102)",  "rgb(51, 255, 25)",
        "rgb(153, 102, 255)",  "rgb(255, 255, 0)",    "rgb(0, 204, 102)",  
        "rgb(65, 51, 0)",      "rgb(0, 153, 153)", 
        "rgb(0, 51, 102)",     "rgb(102, 204, 0)", 
    ]

    traces = []

    aoi = go.Scattergl(
        x=x_axis,
        y=data["ego"]["aoi"],
        mode="lines+markers",
        name="aoi [m/s]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(aoi)

    trace_decision_time = go.Scattergl(
        x=x_axis,
        y=[x/10000000 for x in data["ego"]["decision_time"]],
        mode="lines+markers",
        name="decision_time [s]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_decision_time)

    trace_tactical_speed = go.Scattergl(
        x=x_axis,
        y=data["ego"]["ego_tactical_speed"],
        mode="lines+markers",
        name="tactical_speed [m/s]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_tactical_speed)

    trace_ego_pos = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_current_pos"],
        mode="lines+markers",
        name="ego_current_pos [m]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_ego_pos)


    trace_ego_pos = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_front_d"],
        mode="lines+markers",
        name="ego_front_d [m]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_ego_pos)

    trace_ego_pred_go_pos = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_pred_go_pos"],
        mode="lines+markers",
        name="ego_pred_go_pos",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_ego_pred_go_pos)

    ego_d_to_cr = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_d_to_cr"],
        mode="lines+markers",
        name="ego_d_to_cr [m]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(ego_d_to_cr)

    ego_ttcr = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_ttcr"],
        mode="lines+markers",
        name="ego_ttcr [s]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(ego_ttcr)

    trace_target_acc = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_acc"],
        mode="lines+markers",
        name="target_acc [m/s2]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_acc)


    trace_target_ttcr = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_ttcr"],
        mode="lines+markers",
        name="target_ttcr [s]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_ttcr)

    trace_target_d_to_cr = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_d_to_cr"],
        mode="lines+markers",
        name="target_d_to_cr [m]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_d_to_cr)

    trace_target_pos = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_pos"],
        mode="lines+markers",
        name="target_pos",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_pos)

    trace_target_d_front = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_d_front"],
        mode="lines+markers",
        name="target_d_front [m]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_d_front)

    target_p_x, target_p_y = zip(*data["ego"]["target_front_coords"])

    trace_target_d_front = go.Scatter(
        x=x_axis,
        y=target_p_y,
        mode="lines+markers",
        name="gt target center [m]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_d_front)

    # trace_ego_d_front = go.Scatter(
    #     x=x_axis,
    #     y=data["debug"]["ego_front_d"],
    #     mode="lines+markers",
    #     name="gt ego front d [m]",
    #     line=dict(color=colors[len(traces)]),
    #     marker=dict(color=colors[len(traces)], size=4),
    # )
    # traces.append(trace_ego_d_front)

    # trace_ego_v = go.Scatter(
    #     x=x_axis,
    #     y=data["debug"]["ego_v"],
    #     mode="lines+markers",
    #     name="ego v [m/s]",
    #     line=dict(color=colors[len(traces)]),
    #     marker=dict(color=colors[len(traces)], size=4),
    # )
    # traces.append(trace_ego_v)

    # trace_target_v = go.Scatter(
    #     x=x_axis,
    #     y=data["debug"]["target_v"],
    #     mode="lines+markers",
    #     name="gt target v [m/s]",
    #     line=dict(color=colors[len(traces)]),
    #     marker=dict(color=colors[len(traces)], size=4),
    # )
    # traces.append(trace_target_v)

    return traces



def plot(file_name):
    with open(file_name, "r") as f:
        parsed = json.load(f)

    # Create the graph layout
    plot_heading = ("Debug chart")

    x_axis = [i for i in range(len(parsed["ego"]["call_time"]))]
    #x_axis = parsed["ego"]["call_time"]
    #traces = get_traces(parsed, x_axis)

    skip = ['call_time', 'version']
    legend_only = []
    fig = go.Figure()

    indices = [i for i, val in enumerate(parsed["ego"]['target_d_front']) if val != -1]


    for key, arr in parsed["ego"].items():
        if key in skip:
            continue

        if key == 'target_front_coords':
            _, arr = zip(*arr)
            x_axis = [x_axis[i] for i in indices]
            name = "target_front_coord_y"

        if key in ['all_rec_msg', 'msg_current']:
            x_axis, arr, _, _ = zip(*arr)

        if key in ['aoi', 'aoi_abs']:
            arr = np.array(arr) / 1e6
            

        name = key
        trace = go.Scatter(
            x=x_axis,
            y=arr,
            mode="lines+markers",
            name=name,
            marker=dict( size=4),
            visible='legendonly' if key in legend_only else True
        )
        fig.add_trace(trace)

    # Create the graph layout

    fig.update_layout(
        title=plot_heading,
        xaxis_title='Time',
        yaxis_title='Value',
        hovermode="x unified",
        xaxis=dict( tickformat=',', separatethousands=True)  
    )
    fig.show()


if __name__ == '__main__':

    fileName = "/home/gianfi/KvaserCar_outdoor_test/test_behaviour/0621/21172500843790.json"
    plot(fileName)