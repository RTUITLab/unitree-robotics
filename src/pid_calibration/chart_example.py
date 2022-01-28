import plotly.express as px
import plotly
import pandas as pd
import time

def drawChart(data, leg):
    df = pd.DataFrame([data[0], data[1]], index=['real', 'expected'])
    fig = px.line(df.T)

    ts = time.time()
    name = f"./plots/leg_{str(leg)}_{str(ts).split('.')[0]}.html"
    plotly.offline.plot(fig, filename=name, auto_open=False)
    print(name)
    # fig.show()
