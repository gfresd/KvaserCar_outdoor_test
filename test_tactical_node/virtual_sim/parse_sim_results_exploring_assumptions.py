import json
import os
import math
import pandas as pd
import plotly.express as px

# --- Constants ---
PATH = "/home/gianfi/Documents/KvaserCar_outdoor_test/results_1750878135.6633976.json"
OUT_DIR = "plots"

# --- Ensure input exists and output dir is ready ---
if not os.path.exists(PATH):
    print(f"Error: {PATH} not found. Please place it in the correct path and rerun.")
    exit(1)

os.makedirs(OUT_DIR, exist_ok=True)

# --- Load and flatten JSON ---
with open(PATH, 'r') as f:
    data = json.load(f)["out"]

df = pd.DataFrame([{
        'delay': entry['cfg']['obps']['delay'],
        'failure_start': entry['cfg']['obps']['failure_start'],
        'failure_len': entry['cfg']['obps']['failure_len'],
        'outcome': entry['outcome'],
        'distance': entry['distance'],
        'adv_max_acc': entry['cfg']['ego_params']['adv_max_acc'],
        'adv_max_speed': entry['cfg']['ego_params']['adv_max_speed']
    } for entry in data]
)

# Order failure_len for consistent faceting
sorted_lens = sorted(df['failure_len'].unique())
df['failure_len'] = pd.Categorical(df['failure_len'], categories=sorted_lens, ordered=True)

# Compute how many columns (to get two rows)
cols = math.ceil(len(sorted_lens) / 2)

# --- Loop over each adv_max_acc × adv_max_speed combination ---
for acc, speed in df[['adv_max_acc','adv_max_speed']].drop_duplicates().itertuples(index=False):
    subset = df[(df['adv_max_acc'] == acc) & (df['adv_max_speed'] == speed)]
    if subset.empty:
        continue  # just in case

    # Build the scatter plot
    fig = px.scatter(
        subset,
        x='failure_start',
        y='delay',
        color='outcome',
        facet_col='failure_len',
        facet_col_wrap=cols,
        category_orders={'failure_len': sorted_lens},
        color_discrete_map={'PASSED': 'blue', 'CRASH': 'red'},
        hover_data=['distance'],
        labels={
            'failure_start': 'Failure Start (s)',
            'delay': 'Delay (ms)',
            'outcome': 'Outcome',
            'failure_len': 'Failure Length (s)'
        },
        title=f"adv_max_acc={acc}, adv_max_speed={speed}"
    )

    # Save to HTML
    # sanitize floats in filename by replacing dot with ‘p’
    fn_acc = str(acc).replace('.', 'p')
    fn_spd = str(speed).replace('.', 'p')
    filename = os.path.join(OUT_DIR, f"plot_acc_{fn_acc}_speed_{fn_spd}.html")
    fig.write_html(filename, include_plotlyjs='cdn')
    print(f"Saved: {filename}")
