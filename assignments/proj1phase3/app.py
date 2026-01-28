from nicegui import ui
import plotly.graph_objects as go
import asyncio
from sim.simulator import SimulationEngine
import numpy as np
from sim.trajectories import (
    MAPS,
    build_generator_from_map,
    build_map,
    build_path_from_map,
    trajectory_fn_from_generator,
)
from sim.visualization import (
    make_3d_trajectory_plot,
    make_position_plot,
    make_rpy_plot,
    make_velocity_plot,
    make_initial_3d_figure,
    make_incremental_3d_figure,
    update_incremental_3d_figure,
)

METHODS = {
    "Smooth Only": "smooth",
    "Minimum Jerk": "jerk",
    "Minimum Snap": "snap",
}

# Styles
ui.add_head_html('''
<style>
    .metric-card {
        background-color: #f0f2f6;
        padding: 1rem;
        border-radius: 0.5rem;
        text-align: center;
    }
    .metric-value {
        font-size: 1.5rem;
        font-weight: bold;
        color: #0e1117;
    }
    .metric-label {
        font-size: 0.875rem;
        color: #31333f;
    }
</style>
''', shared=True)

# Global state wrapper to handle UI events
class State:
    def __init__(self):
        self.map_name = "Map 1"
        self.method = "jerk"
        self.fnoise = 1.0

state = State()

@ui.page('/')
def main_page():
    ui.markdown('## HKUST ELEC5660 — Quadrotor Simulator').classes('text-2xl font-bold mb-4')

    with ui.row().classes('w-full items-start flex-nowrap gap-4'):
        # Sidebar-like control panel
        with ui.card().classes('w-80 shrink-0 p-4 gap-4'):
            ui.label('Simulation Settings').classes('text-lg font-bold')

            map_options = list(MAPS.keys()) + ["Map 3"]

            def on_map_change(e):
                state.map_name = e.value
                # Update the 3D plot with new map and path
                rng = np.random.default_rng()
                new_map_points = build_map(state.map_name, rng)
                new_path_points = build_path_from_map(new_map_points)

                new_fig = make_initial_3d_figure(new_map_points, new_path_points)
                new_fig.layout.uirevision = 'constant'
                plot3d.update_figure(new_fig)

            ui.select(
                map_options,
                value=state.map_name,
                label='Map',
                on_change=on_map_change,
            ).classes('w-full')

            default_method = next(label for label, key in METHODS.items() if key == state.method)
            ui.select(
                list(METHODS.keys()),
                value=default_method,
                label='Optimization',
                on_change=lambda e: setattr(state, 'method', METHODS[e.value]),
            ).classes('w-full mt-2')

            ui.label('Disturbance std (N)').classes('mt-2')

            # Using a label to show slider value
            slider_label = ui.label(f'{state.fnoise:.1f}')

            def update_noise(e):
                val = float(e.value)
                state.fnoise = val
                slider_label.set_text(f'{val:.1f}')

            ui.slider(min=0.0, max=3.0, step=0.1, value=state.fnoise,
                      on_change=update_noise).classes('w-full')

            run_btn = ui.button('Run simulation', on_click=lambda: start_simulation()).classes('w-full mt-4 bg-blue-600')

            # Metrics Area
            ui.label('Last Run Metrics').classes('text-md font-bold mt-6')
            metrics_container = ui.column().classes('w-full gap-2')

        # Main visualization area
        with ui.card().classes('grow min-w-[400px] p-4'):
            ui.label('3D Trajectory').classes('text-xl font-bold mb-2')

            # Initialize with map and path
            rng = np.random.default_rng()
            initial_map_points = build_map(state.map_name, rng)
            initial_path_points = build_path_from_map(initial_map_points)

            fig3d = make_initial_3d_figure(initial_map_points, initial_path_points)
            plot3d = ui.plotly(fig3d).classes('w-full h-[600px]')

    # Additional Plots Section
    ui.label('Analysis').classes('text-xl font-bold mt-8 mb-4')
    with ui.column().classes('w-full gap-4'):
        with ui.card().classes('w-full p-2'):
            ui.label('Attitude').classes('font-bold')
            plot_rpy = ui.plotly(go.Figure()).classes('w-full h-[400px]')

        with ui.card().classes('w-full p-2'):
             ui.label('Velocity').classes('font-bold')
             plot_vel = ui.plotly(go.Figure()).classes('w-full h-[400px]')

        with ui.card().classes('w-full p-2'):
             ui.label('Position').classes('font-bold')
             plot_pos = ui.plotly(go.Figure()).classes('w-full h-[400px]')

    async def start_simulation():
        run_btn.disable()
        run_btn.text = 'Running...'

        rng = np.random.default_rng()
        map_points = build_map(state.map_name, rng)
        path_points = build_path_from_map(map_points)
        generator = build_generator_from_map(map_points, state.method)
        trajectory_fn = trajectory_fn_from_generator(generator)
        engine = SimulationEngine(
            trajectory_fn=trajectory_fn,
            t_final=25.0,
            t_step=0.002,
            control_step=0.01,
            fnoise=float(state.fnoise),
            seed=None,
        )

        metrics_container.clear()

        render_steps = 100

        # Initialize figure for incremental updates with map and path
        current_fig, num_static_traces = make_incremental_3d_figure(map_points, path_points)

        while not engine.done:
            engine.step(controls=render_steps)
            result = engine.result(compute_metrics=False)

            if len(result.times) > 0:
                # Incremental update
                pos = result.states[:, 0:3]
                des = result.desired[:, 0:3]
                idx = len(result.times) - 1

                update_incremental_3d_figure(current_fig, num_static_traces, pos, des, idx)
                plot3d.update_figure(current_fig)

            # Allow UI to update
            await asyncio.sleep(0.001)

        # Final Render with all plots and metrics
        result = engine.result(compute_metrics=True)

        final_fig3d = make_3d_trajectory_plot(
            result,
            len(result.times) - 1,
            map_points=map_points,
            path_points=path_points,
        )
        final_fig3d.layout.uirevision = 'constant' # Keep camera even at end
        final_fig3d.layout.margin = dict(l=0, r=0, t=0, b=0)
        final_fig3d.layout.height = 600
        plot3d.update_figure(final_fig3d)

        plot_rpy.update_figure(make_rpy_plot(result))
        plot_vel.update_figure(make_velocity_plot(result))
        plot_pos.update_figure(make_position_plot(result))

        with metrics_container:
            # Helper to display metric
            def metric_display(label, value):
                with ui.row().classes('metric-card w-full justify-between items-center'):
                    ui.label(label).classes('metric-label')
                    ui.label(value).classes('metric-value')

            metric_display("RMSE Position", f"{result.rmse_pos:.4f} m")
            metric_display("RMSE Velocity", f"{result.rmse_vel:.4f} m/s")
            metric_display("RMSE Yaw", f"{result.rmse_yaw_deg:.3f} deg")
            metric_display("Smoothness", f"{result.smoothness:.3f}")

        run_btn.enable()
        run_btn.text = 'Run simulation'

ui.run(title='ELEC5660 Simulator', port=8080)
