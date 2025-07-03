import json
import pandas as pd
import plotly.express as px
import os
import math

PATH = "results_1750721986.2965846.json"


# Verify that the JSON file exists
if not os.path.exists(PATH):
    print(f"Error: {PATH} not found. Please place it in the current working directory and rerun.")
    exit(0)


# Load the JSON results
with open(PATH, 'r') as f:
    data = json.load(f)["out"]

# Flatten into a DataFrame
df = pd.DataFrame([
    {
        'delay': entry['cfg']['obps']['delay'],
        'failure_start': entry['cfg']['obps']['failure_start'],
        'failure_len': entry['cfg']['obps']['failure_len'],
        'outcome': entry['outcome'],
        'distance': entry['distance']
    }
    for entry in data
])

# Sort failure_len values and make categorical
sorted_lens = sorted(df['failure_len'].unique())
df['failure_len'] = pd.Categorical(df['failure_len'], categories=sorted_lens, ordered=True)

# Determine facet wrapping for two rows
cols = math.ceil(len(sorted_lens) / 2)

# Create scatter plot with facets for failure_len
fig = px.scatter(
    df,
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
    title='Simulation Results: Delay vs Failure Start Faceted by Failure Length'
)

fig.show()
filename = "sim_result.html"
fig.write_html(filename, include_plotlyjs='cdn')
print(f"Saved: {filename}")
