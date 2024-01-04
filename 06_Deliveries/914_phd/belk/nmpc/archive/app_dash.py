from dash import Dash, dcc, html, Input, Output
import plotly.express as px
import numpy as np
import pandas as pd

df = px.data.gapminder()
app = Dash(__name__)

x_history = np.load('data/x_1704396094.8685088.npy')
mydf = pd.DataFrame(x_history[:,0,:].reshape((-1,4)) , columns)


app.layout = html.Div(
    [
        html.H4("Animated GDP and population over decades"),
        html.P("Select an animation:"),
        dcc.RadioItems(
            id="selection",
            options=["GDP - Scatter", "Population - Bar", "My Data"],
            value="GDP - Scatter",
        ),
        dcc.Loading(dcc.Graph(id="graph"), type="cube"),
    ]
)


@app.callback(
    Output("graph", "figure"), Input("selection", "value")
)
def display_animated_graph(selection):
    animations = {
        "My Data": px.line(
            None,
            x=x_history[0,0,:],
            y=x_history[1,0,:],
            animation_frame="year",
            animation_group="country",
            size="pop",
            color="continent",
            hover_name="country",
            log_x=True,
            size_max=55,
            range_x=[100, 100000],
            range_y=[25, 90],
        ),
        "GDP - Scatter": px.scatter(
            df,
            x="gdpPercap",
            y="lifeExp",
            animation_frame="year",
            animation_group="country",
            size="pop",
            color="continent",
            hover_name="country",
            log_x=True,
            size_max=55,
            range_x=[100, 100000],
            range_y=[25, 90],
        ),
        "Population - Bar": px.bar(
            df,
            x="continent",
            y="pop",
            color="continent",
            animation_frame="year",
            animation_group="country",
            range_y=[0, 4000000000],
        ),
    }
    return animations[selection]


if __name__ == "__main__":
    app.run_server(debug=True)