import numpy as np
import plotly.graph_objects as go
import plotly.io as pio
from plotly.io import write_html

# --- Helper: moving average smoothing ---
def smooth(data, window_size=5):
    window = np.ones(window_size) / window_size
    return np.convolve(data, window, mode='same')

# --- Load CSV ---
data = np.loadtxt("translations.csv", delimiter=",")

# --- Filter out outliers where any component > 25 ---
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]

# mask = (np.sqrt(x**2 + y**2 + z**2) <= 1)
# x = x[mask]
# y = y[mask]
# z = z[mask]

frames = np.arange(len(x))  # frame indices after filtering

# --- Compute cumulative positions ---
x_cum = np.cumsum(x)
y_cum = np.cumsum(y)
z_cum = np.cumsum(z)

# --- Smooth both translations and cumulative positions ---
window_size = 5
x_smooth = smooth(x, window_size)
y_smooth = smooth(y, window_size)
z_smooth = smooth(z, window_size)

x_cum_smooth = np.cumsum(x_smooth)
y_cum_smooth = np.cumsum(y_smooth)
z_cum_smooth = np.cumsum(z_smooth)

# --- Function to create 2D line plots for browser ---
def plot_2d(frames, raw, smooth_data, cum=False, axis_name='X'):
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=frames, y=raw, mode='lines+markers', name='Raw', marker=dict(size=4, color='blue')))
    fig.add_trace(go.Scatter(x=frames, y=smooth_data, mode='lines', name='Smoothed', line=dict(color='orange', width=2)))
    fig.update_layout(
        title=f"{axis_name} {'Cumulative Position' if cum else 'Translation'} over time",
        xaxis_title='Frame',
        yaxis_title=f"{axis_name} {'Position' if cum else 'Translation'}",
        width=900,
        height=500
    )
    return fig

# --- Plot translations in browser ---
write_html(plot_2d(frames, x, x_smooth, cum=False, axis_name='X'), file="./plots_traj/X_2d.html", auto_open=True)
write_html(plot_2d(frames, y, y_smooth, cum=False, axis_name='Y'), file="./plots_traj/Y_2d.html", auto_open=True)
write_html(plot_2d(frames, z, z_smooth, cum=False, axis_name='Z'), file="./plots_traj/Z_2d.html", auto_open=True)




# --- Plot cumulative positions in browser ---
# plot_2d(frames, x_cum, x_cum_smooth, cum=True, axis_name='X')
# plot_2d(frames, y_cum, y_cum_smooth, cum=True, axis_name='Y')
# plot_2d(frames, z_cum, z_cum_smooth, cum=True, axis_name='Z')

# --- 3D Trajectories: raw, smoothed, and cumulative ---
fig3d = go.Figure()

# Raw trajectory
fig3d.add_trace(go.Scatter3d(
    x=x, y=y, z=z,
    mode='lines+markers',
    name='Raw Trajectory',
    marker=dict(size=3, color='blue'),
    line=dict(width=2)
))

# Smoothed trajectory
fig3d.add_trace(go.Scatter3d(
    x=x_smooth, y=y_smooth, z=z_smooth,
    mode='lines+markers',
    name='Smoothed Trajectory',
    marker=dict(size=3, color='green'),
    line=dict(width=2, dash='dash')
))

# Cumulative trajectory
fig3d.add_trace(go.Scatter3d(
    x=x_cum, y=y_cum, z=z_cum,
    mode='lines+markers',
    name='Cumulative Trajectory',
    marker=dict(size=3, color='orange'),
    line=dict(width=3)
))
# Cumulative smooth trajectory
fig3d.add_trace(go.Scatter3d(
    x=x_cum_smooth, y=y_cum_smooth, z=z_cum_smooth,
    mode='lines+markers',
    name='Cumulative Smooth Trajectory',
    marker=dict(size=3, color='black'),
    line=dict(width=3)
))

min_val = 1.1*min(x_cum_smooth.min(), y_cum_smooth.min(), z_cum_smooth.min())
max_val = 1.1*max(x_cum_smooth.max(), y_cum_smooth.max(), z_cum_smooth.max())

fig3d.update_layout(
    title='3D Trajectories: Raw, Smoothed, and Cumulative',
    scene=dict(
        xaxis=dict(range=[min_val, max_val], title='X Axis'),
        yaxis=dict(range=[min_val, max_val], title='Y Axis'),
        zaxis=dict(range=[min_val, max_val], title='Z Axis'),
    ),
    width=1800,
    height=1400
)

# Show 3D interactive plot in browser
# fig3d.show()
fig3d.update_yaxes(
    scaleanchor="x",
    scaleratio=1,
  )
write_html(fig3d, file="./plots_traj/3d.html", auto_open=True)
