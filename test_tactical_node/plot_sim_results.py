import json
import plotly.graph_objs as go
import numpy as np
from pathlib import Path


def get_traces(data: dict, x_axis, skip_index):

    colors = [
        "rgb(0, 0, 0)", "rgb(255, 0, 0)", "rgb(0, 102, 204)", "rgb(0, 153, 0)", "rgb(204, 0, 204)",
        "rgb(255, 128, 0)", "rgb(128, 0, 255)", "rgb(128, 128, 128)", "rgb(0, 153, 153)", "rgb(255, 204, 0)",
        "rgb(76, 0, 153)", "rgb(0, 0, 255)", "rgb(0, 255, 0)", "rgb(255, 0, 255)", "rgb(255, 102, 102)",
        "rgb(153, 102, 255)", "rgb(255, 255, 0)", "rgb(0, 204, 102)", "rgb(102, 51, 153)", "rgb(255, 51, 0)",
        "rgb(0, 51, 102)", "rgb(102, 204, 0)", "rgb(255, 153, 51)", "rgb(153, 0, 0)", "rgb(51, 255, 255)"
    ]

    traces = []

    aoi = go.Scattergl(
        x=x_axis,
        y=(np.asarray(data["ego"]["aoi"][skip_index:]) / 1000_000),
        mode="lines+markers",
        name="aoi [ms]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(aoi)

    trace_aoi_seconds = go.Scatter(
        x=x_axis,
        y=data["ego"]["aoi_seconds"][skip_index:],
        mode="lines+markers",
        name="aoi_seconds [s]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_aoi_seconds)

    trace_tactical_speed = go.Scattergl(
        x=x_axis,
        y=data["ego"]["ego_tactical_speed"][skip_index:],
        mode="lines+markers",
        name="tactical_speed [m/s]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_tactical_speed)

    trace_ego_pos = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_pos"][skip_index:],
        mode="lines+markers",
        name="ego_pos",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_ego_pos)

    trace_ego_pred_go_pos = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_pred_go_pos"][skip_index:],
        mode="lines+markers",
        name="ego_pred_go_pos",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_ego_pred_go_pos)

    ego_d_to_cr = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_d_to_cr"][skip_index:],
        mode="lines+markers",
        name="ego_d_to_cr [m]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(ego_d_to_cr)

    ego_ttcr = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_ttcr"][skip_index:],
        mode="lines+markers",
        name="ego_ttcr [s]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(ego_ttcr)

    trace_target_acc = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_acc"][skip_index:],
        mode="lines+markers",
        name="target_acc [m/s2]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_acc)


    trace_target_ttcr = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_ttcr"][skip_index:],
        mode="lines+markers",
        name="target_ttcr [s]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_ttcr)

    trace_target_d_to_cr = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_d_to_cr"][skip_index:],
        mode="lines+markers",
        name="target_d_to_cr [m]",
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_d_to_cr)

    trace_target_pos = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_pos"][skip_index:],
        mode="lines+markers",
        name="target_pos",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_pos)

    trace_target_d_front = go.Scatter(
        x=x_axis,
        y=data["ego"]["target_d_front"][skip_index:],
        mode="lines+markers",
        name="target_d_front [m]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_target_d_front)



    trace_ego_d_front = go.Scatter(
        x=x_axis,
        y=data["ego"]["kalman_acc"][skip_index:],
        mode="lines+markers",
        name="kalman_acc [m/s2]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_ego_d_front)

    trace_ego_v = go.Scatter(
        x=x_axis,
        y=data["ego"]["ego_front_d"][skip_index:],
        mode="lines+markers",
        name="ego_front_d [m]",
        visible='legendonly',
        line=dict(color=colors[len(traces)]),
        marker=dict(color=colors[len(traces)], size=4),
    )
    traces.append(trace_ego_v)

    try:

        t_rec_list, id_list, t_msg_list = zip(*data["ego"]["all_rec_msg"])

        trace_all_msg_time = go.Scatter(
            x=(np.asarray(t_rec_list) - data["ego"]["call_time"][0])/1000_000_000,
            y=(np.asarray(t_msg_list) - t_msg_list[0])  /1000_000_000,
            mode="lines+markers",
            name="all_msg_time",
            visible='legendonly',
            line=dict(color=colors[len(traces)]),
            marker=dict(color=colors[len(traces)], size=4),
        )
        traces.append(trace_all_msg_time)

        trace_all_msg_id = go.Scatter(
            x=(np.asarray(t_rec_list)-data["ego"]["call_time"][0])/1000_000_000,
            y=id_list,
            mode="lines+markers",
            name="all_msg_id",
            visible='legendonly',
            line=dict(color=colors[len(traces)]),
            marker=dict(color=colors[len(traces)], size=4),
        )
        traces.append(trace_all_msg_id)

        t_rec_list, id_list, t_msg_list = zip(*data["ego"]["msg_current"])

        trace_msg_id = go.Scatter(
            x=(np.asarray(t_rec_list)- data["ego"]["call_time"][0])/1000_000_000,
            y=id_list ,
            mode="lines+markers",
            name="msg_id",
            visible='legendonly',
            line=dict(color=colors[len(traces)]),
            marker=dict(color=colors[len(traces)], size=4),
        )
        traces.append(trace_msg_id)

        trace_msg_time = go.Scatter(
            x=(np.asarray(t_rec_list) - data["ego"]["call_time"][0])/1000_000_000,
            y=(np.asarray(t_msg_list) - t_msg_list[0] )/1000_000_000,
            mode="lines+markers",
            name="msg_time",
            visible='legendonly',
            line=dict(color=colors[len(traces)]),
            marker=dict(color=colors[len(traces)], size=4),
        )
        traces.append(trace_msg_time)
    except ValueError as e:
        print("No messages received")


    return traces



def plot(parsed):


    # Create the graph layout
    plot_heading = ("Debug chart {0}, cause {1}".format(parsed["exp_name"], parsed["term_cause"]))

    dec_time = [x for x in parsed["ego"]["call_time"] if x>-1]
    #skip_index = len(parsed["ego"]["decision_time"]) - len(dec_time)
    #x_axis = (np.asarray(dec_time) - dec_time[0])/1000_000
    x_axis = (np.asarray(parsed["ego"]["call_time"]) - parsed["ego"]["call_time"][0])/1000_000_000
    traces = get_traces(parsed, x_axis, 0)

    # Create the graph layout
    layout = go.Layout(
        title=plot_heading,
        xaxis=dict(range=[x_axis[0], x_axis[-1]]),
        yaxis=dict(range=[-5, 35]),
        hovermode='x unified'
    )

    fig = go.Figure(data=traces, layout=layout)
    fig.show()



def load_most_recent_json(directory: str) -> dict:
    
    # Create a Path object for the directory
    dir_path = Path(directory)

    # Find all .json files
    json_files = list(dir_path.glob('*.json'))
    if not json_files:
        raise FileNotFoundError(f"No JSON files found in directory: {directory}")

    # Select the file with the latest modification time
    latest_file = max(json_files, key=lambda f: f.stat().st_mtime)

    # Load and return the JSON content
    with latest_file.open('r', encoding='utf-8') as f:
        return json.load(f)

if __name__ == '__main__':

    fileName = "/home/gianfi/KvaserCar_outdoor_test/tactical_log/0703/"
    dir = "/home/gianfi/KvaserCar_outdoor_test/tactical_log/0703/"

    #with open(fileName, "r") as f:
    #    parsed = json.load(f)

    parsed = load_most_recent_json(dir)
    plot(parsed)