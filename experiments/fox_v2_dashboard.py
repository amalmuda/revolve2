"""
Fox V2 Experiment Dashboard
============================
Streamlit dashboard for the Fox cluster v2 experiments.
- Robots: Spider, Gecko, Arachnid
- Bounds: [-1, 1]
- Lambda: 0 to 5 in 0.25 steps (21 values)
- Seeds: 25 per lambda
- Total: 525 runs per robot (1575 total)

Run with: streamlit run fox_v2_dashboard.py
"""

import streamlit as st
import plotly.graph_objects as go
import plotly.express as px
import pandas as pd
import numpy as np
import sqlite3
from pathlib import Path
import subprocess
import sys

# Page configuration
st.set_page_config(
    page_title="Fox V2 Experiment",
    page_icon="🦊",
    layout="wide",
    initial_sidebar_state="expanded"
)

# =============================================================================
# ROBOT CONFIGURATION - Fox V2 Experiments
# =============================================================================

ROBOT_CONFIG = {
    'spider': {
        'results_dir': 'spider_v2_bounds1_pop25_25seeds',
        'glob_pattern': 'spider_m1_power_bounds1_lambda*',
        'color': '#636EFA',
        'cpg_params': 12
    },
    'gecko': {
        'results_dir': 'gecko_v2_bounds1_pop25_25seeds',
        'glob_pattern': 'gecko_m1_power_bounds1_lambda*',
        'color': '#EF553B',
        'cpg_params': 9
    },
    'arachnid': {
        'results_dir': 'arachnid_v2_bounds1_pop25_25seeds',
        'glob_pattern': 'arachnid_m1_power_bounds1_lambda*',
        'color': '#00CC96',
        'cpg_params': 20
    },
    'tripod': {
        'results_dir': 'tripod_v1_bounds1_pop25_25seeds',
        'glob_pattern': 'tripod_m1_power_bounds1_lambda*',
        'color': '#AB63FA',
        'cpg_params': 9
    }
}

BASE_RESULTS_DIR = Path(__file__).parent / 'results'

# =============================================================================
# DATA LOADING
# =============================================================================

@st.cache_data
def load_lambda_data(robot_name: str):
    """Load experiment data from the lambda sweep experiment."""
    config = ROBOT_CONFIG.get(robot_name)
    if not config:
        return pd.DataFrame()

    results_dir = BASE_RESULTS_DIR / config['results_dir']

    if not results_dir.exists():
        return pd.DataFrame()

    all_data = []

    for exp_dir in sorted(results_dir.glob(config['glob_pattern'])):
        if not exp_dir.is_dir():
            continue

        # Parse lambda from directory name
        # Format: spider_m1_power_bounds1_lambda0_25 -> lambda=0.25
        name = exp_dir.name
        lambda_idx = name.find('lambda') + 6
        lambda_str = name[lambda_idx:].replace('_', '.')
        try:
            lam = float(lambda_str)
        except ValueError:
            continue

        for db_file in exp_dir.glob('run_*.sqlite'):
            try:
                run_num = int(db_file.stem.split('_')[1])
            except (IndexError, ValueError):
                continue

            try:
                conn = sqlite3.connect(db_file)
                c = conn.cursor()

                # Get best individual from final generation
                c.execute('''
                    SELECT i.fitness, i.distance, i.contact_m1, i.cost_of_transport,
                           i.straightness, i.stability, g.serialized_parameters
                    FROM individual i
                    JOIN genotype g ON i.genotype_id = g.id
                    JOIN population p ON i.population_id = p.id
                    JOIN generation gen ON gen.population_id = p.id
                    WHERE gen.generation_index = (SELECT MAX(generation_index) FROM generation)
                    ORDER BY i.fitness DESC
                    LIMIT 1
                ''')
                row = c.fetchone()
                conn.close()

                if row:
                    all_data.append({
                        'robot': robot_name,
                        'lambda': lam,
                        'run': run_num,
                        'fitness': row[0],
                        'distance': row[1],
                        'contact': row[2] * 100 if row[2] else 0,
                        'cot': row[3] if row[3] else 0,
                        'straightness': row[4] * 100 if row[4] else 0,
                        'stability': row[5] if row[5] else 0,
                        'params': row[6],
                        'db_path': str(db_file)
                    })
            except Exception as e:
                pass  # Skip problematic files

    return pd.DataFrame(all_data)


@st.cache_data
def load_all_robots_data():
    """Load data for all robots."""
    all_data = []
    for robot_name in ROBOT_CONFIG.keys():
        df = load_lambda_data(robot_name)
        if not df.empty:
            all_data.append(df)

    if all_data:
        return pd.concat(all_data, ignore_index=True)
    return pd.DataFrame()


@st.cache_data
def load_generation_data(db_path: str):
    """Load fitness evolution over generations."""
    try:
        conn = sqlite3.connect(db_path)
        c = conn.cursor()

        c.execute('''
            SELECT gen.generation_index,
                   MAX(i.fitness) as best_fitness,
                   AVG(i.fitness) as avg_fitness,
                   MAX(i.distance) as best_distance,
                   AVG(i.contact_m1) as avg_contact
            FROM individual i
            JOIN population p ON i.population_id = p.id
            JOIN generation gen ON gen.population_id = p.id
            GROUP BY gen.generation_index
            ORDER BY gen.generation_index
        ''')
        rows = c.fetchall()
        conn.close()

        return pd.DataFrame(rows, columns=['generation', 'best_fitness', 'avg_fitness',
                                           'best_distance', 'avg_contact'])
    except (sqlite3.Error, pd.errors.DatabaseError) as e:
        st.warning(f"Database error: {e}")
        return pd.DataFrame()


def get_lambda_stats(df, group_by_robot=False):
    """Calculate stats grouped by lambda (and optionally robot)."""
    metrics = ['fitness', 'distance', 'contact', 'cot', 'straightness', 'stability']

    agg_dict = {
        'n_runs': pd.NamedAgg(column='run', aggfunc='count')
    }
    for m in metrics:
        agg_dict[f'{m}_mean'] = pd.NamedAgg(column=m, aggfunc='mean')
        agg_dict[f'{m}_std'] = pd.NamedAgg(column=m, aggfunc='std')
        agg_dict[f'{m}_min'] = pd.NamedAgg(column=m, aggfunc='min')
        agg_dict[f'{m}_max'] = pd.NamedAgg(column=m, aggfunc='max')

    group_cols = ['robot', 'lambda'] if group_by_robot else ['lambda']
    stats = df.groupby(group_cols).agg(**agg_dict).reset_index()
    return stats


# =============================================================================
# PLOTTING
# =============================================================================

def create_lambda_plot(stats_df, raw_df, metric, show_std=True, show_points=False):
    """Create main lambda vs metric plot for single robot."""
    fig = go.Figure()

    x = stats_df['lambda']
    y_mean = stats_df[f'{metric}_mean']
    y_std = stats_df[f'{metric}_std']

    # Std shading
    if show_std:
        fig.add_trace(go.Scatter(
            x=pd.concat([x, x[::-1]]),
            y=pd.concat([y_mean + y_std, (y_mean - y_std)[::-1]]),
            fill='toself',
            fillcolor='rgba(99, 110, 250, 0.2)',
            line=dict(color='rgba(255,255,255,0)'),
            name='Std Dev',
            showlegend=False,
            hoverinfo='skip'
        ))

    # Mean line
    fig.add_trace(go.Scatter(
        x=x,
        y=y_mean,
        mode='lines+markers',
        name='Mean',
        line=dict(color='#636EFA', width=3),
        marker=dict(size=10),
        hovertemplate='lambda=%{x}<br>%{y:.3f}<extra></extra>'
    ))

    # Individual points
    if show_points:
        fig.add_trace(go.Scatter(
            x=raw_df['lambda'],
            y=raw_df[metric],
            mode='markers',
            name='Individual runs',
            marker=dict(color='#636EFA', size=5, opacity=0.4),
            hovertemplate='lambda=%{x}<br>Run %{customdata}<br>%{y:.3f}<extra></extra>',
            customdata=raw_df['run']
        ))

    fig.update_layout(
        xaxis_title='Lambda',
        yaxis_title=metric.replace('_', ' ').title(),
        hovermode='closest',
        legend=dict(yanchor="top", y=0.99, xanchor="right", x=0.99)
    )

    return fig


def create_multi_robot_plot(all_stats_df, metric, show_std=True):
    """Create lambda vs metric plot comparing all robots."""
    fig = go.Figure()

    for robot_name, config in ROBOT_CONFIG.items():
        robot_stats = all_stats_df[all_stats_df['robot'] == robot_name]
        if robot_stats.empty:
            continue

        x = robot_stats['lambda']
        y_mean = robot_stats[f'{metric}_mean']
        y_std = robot_stats[f'{metric}_std']
        color = config['color']

        # Std shading
        if show_std:
            # Convert color hex to rgba
            r, g, b = int(color[1:3], 16), int(color[3:5], 16), int(color[5:7], 16)
            fig.add_trace(go.Scatter(
                x=pd.concat([x, x[::-1]]),
                y=pd.concat([y_mean + y_std, (y_mean - y_std)[::-1]]),
                fill='toself',
                fillcolor=f'rgba({r},{g},{b},0.15)',
                line=dict(color='rgba(255,255,255,0)'),
                name=f'{robot_name.title()} Std',
                showlegend=False,
                hoverinfo='skip'
            ))

        # Mean line
        fig.add_trace(go.Scatter(
            x=x,
            y=y_mean,
            mode='lines+markers',
            name=robot_name.title(),
            line=dict(color=color, width=3),
            marker=dict(size=8),
            hovertemplate=f'{robot_name.title()}<br>lambda=%{{x}}<br>%{{y:.3f}}<extra></extra>'
        ))

    fig.update_layout(
        xaxis_title='Lambda',
        yaxis_title=metric.replace('_', ' ').title(),
        hovermode='x unified',
        legend=dict(yanchor="top", y=0.99, xanchor="right", x=0.99)
    )

    return fig


def create_box_plot(df, metric):
    """Create box plot showing distribution per lambda."""
    fig = px.box(
        df,
        x='lambda',
        y=metric,
        points='all',
        hover_data=['run']
    )

    fig.update_layout(
        xaxis_title='Lambda',
        yaxis_title=metric.replace('_', ' ').title()
    )

    return fig


def create_correlation_scatter(df, x_metric, y_metric, color_by='lambda'):
    """Create scatter plot of two metrics."""
    if color_by == 'lambda':
        fig = px.scatter(
            df,
            x=x_metric,
            y=y_metric,
            color='lambda',
            color_continuous_scale='Viridis',
            hover_data=['run', 'fitness', 'robot'] if 'robot' in df.columns else ['run', 'fitness']
        )
    elif color_by == 'robot':
        fig = px.scatter(
            df,
            x=x_metric,
            y=y_metric,
            color='robot',
            color_discrete_map={k: v['color'] for k, v in ROBOT_CONFIG.items()},
            hover_data=['lambda', 'run', 'fitness']
        )
    else:
        fig = px.scatter(
            df,
            x=x_metric,
            y=y_metric,
            hover_data=['lambda', 'run', 'fitness']
        )

    fig.update_layout(
        xaxis_title=x_metric.replace('_', ' ').title(),
        yaxis_title=y_metric.replace('_', ' ').title()
    )

    return fig


def create_multi_metric_plot(stats_df):
    """Create normalized multi-metric comparison."""
    fig = go.Figure()

    metrics = ['distance', 'contact', 'cot', 'straightness', 'stability']
    colors = px.colors.qualitative.Set2

    for i, metric in enumerate(metrics):
        y_mean = stats_df[f'{metric}_mean']

        # Normalize to 0-1 range for comparison
        y_norm = (y_mean - y_mean.min()) / (y_mean.max() - y_mean.min() + 1e-10)

        fig.add_trace(go.Scatter(
            x=stats_df['lambda'],
            y=y_norm,
            mode='lines+markers',
            name=metric.title(),
            line=dict(color=colors[i], width=2),
            marker=dict(size=6),
            hovertemplate=f'{metric}: %{{customdata:.3f}}<extra></extra>',
            customdata=y_mean
        ))

    fig.update_layout(
        xaxis_title='Lambda',
        yaxis_title='Normalized Value (0-1)',
        legend=dict(yanchor="top", y=0.99, xanchor="left", x=1.02)
    )

    return fig


def create_heatmap(all_df, metric):
    """Create heatmap of metric across robots and lambdas."""
    pivot = all_df.groupby(['robot', 'lambda'])[metric].mean().reset_index()
    pivot = pivot.pivot(index='robot', columns='lambda', values=metric)

    fig = px.imshow(
        pivot,
        labels=dict(x='Lambda', y='Robot', color=metric.title()),
        aspect='auto',
        color_continuous_scale='Viridis'
    )

    return fig


# =============================================================================
# VISUALIZATION
# =============================================================================

def visualize_robot(params_str: str, lam: float, run: int, robot_name: str):
    """Launch MuJoCo viewer."""
    params = [float(x) for x in params_str.split(';')]

    # Get absolute path to experiments directory
    experiments_dir = Path(__file__).resolve().parent

    viz_script = f'''
import sys
sys.path.insert(0, "{experiments_dir}")
from contact_detection import simulate_with_contact_detection

print("Visualizing: {robot_name}, bounds=[-1,1], lambda={lam}, run={run}")
simulate_with_contact_detection(
    robot_name="{robot_name}",
    simulation_time=30,
    verbose=False,
    cpg_params={params},
    headless=False,
    warmup_time=0.0,
    track_camera=True,
    cast_shadows=False,
)
'''

    temp_script = experiments_dir / '_temp_viz.py'
    temp_script.write_text(viz_script)

    subprocess.Popen([sys.executable, str(temp_script)], cwd=str(experiments_dir))
    return True


# =============================================================================
# MAIN DASHBOARD
# =============================================================================

def main():
    st.title("Fox V2 Experiment Dashboard")

    # Sidebar - View Mode Selection
    st.sidebar.header("View Mode")
    view_mode = st.sidebar.radio(
        "Analysis Mode",
        options=["Single Robot", "Compare Robots"],
        key='view_mode'
    )

    if view_mode == "Single Robot":
        single_robot_view()
    else:
        compare_robots_view()


def single_robot_view():
    """Single robot analysis view."""
    # Sidebar - Robot Selection
    st.sidebar.header("Robot Selection")
    available_robots = list(ROBOT_CONFIG.keys())
    selected_robot = st.sidebar.selectbox(
        "Robot",
        options=available_robots,
        format_func=lambda x: x.title(),
        key='robot_selector'
    )

    st.markdown(f"**Robot:** {selected_robot.title()} | **Bounds:** [-1, 1] | **Lambda:** 0 to 5 (0.25 steps) | **Seeds:** 25 per lambda")
    st.markdown("**Power Formula:** `fitness = distance * (1 - contact)^lambda`")

    # Load data for selected robot
    with st.spinner(f'Loading {selected_robot} experiment data...'):
        df = load_lambda_data(selected_robot)

    if df.empty:
        config = ROBOT_CONFIG[selected_robot]
        st.error(f"No data found in results/{config['results_dir']}/")
        st.info("Make sure you've downloaded the experiment results from Fox.")
        return

    # Calculate stats
    stats = get_lambda_stats(df)

    # Sidebar - Controls
    st.sidebar.markdown("---")
    st.sidebar.header("Controls")

    metric = st.sidebar.selectbox(
        "Primary Metric",
        options=['fitness', 'distance', 'contact', 'cot', 'straightness', 'stability'],
        format_func=lambda x: {
            'fitness': 'Fitness',
            'distance': 'Distance (m)',
            'contact': 'Contact (%)',
            'cot': 'Cost of Transport',
            'straightness': 'Straightness (%)',
            'stability': 'Stability'
        }[x]
    )

    show_std = st.sidebar.checkbox("Show std shading", value=True)
    show_points = st.sidebar.checkbox("Show individual runs", value=False)

    st.sidebar.markdown("---")
    st.sidebar.header("Quick Stats")
    st.sidebar.metric("Total Runs", len(df))
    st.sidebar.metric("Lambda Values", df['lambda'].nunique())
    st.sidebar.metric("Runs per Lambda", int(df.groupby('lambda').size().mean()))

    # Find optimal lambda
    best_idx = stats[f'{metric}_mean'].idxmax() if metric not in ['contact', 'cot', 'stability'] else stats[f'{metric}_mean'].idxmin()
    best_lambda = stats.loc[best_idx, 'lambda']
    best_value = stats.loc[best_idx, f'{metric}_mean']
    st.sidebar.metric(f"Best lambda for {metric}", f"{best_lambda} ({best_value:.2f})")

    # Main tabs
    tab1, tab2, tab3, tab4, tab5 = st.tabs([
        "Lambda Sweep",
        "Distributions",
        "Correlations",
        "All Metrics",
        "Robot Viewer"
    ])

    # TAB 1: Main Lambda Plot
    with tab1:
        st.header(f"{metric.title()} vs Lambda")

        fig = create_lambda_plot(stats, df, metric, show_std, show_points)
        fig.update_layout(height=500)
        st.plotly_chart(fig, use_container_width=True)

        # Key findings
        col1, col2, col3 = st.columns(3)

        with col1:
            st.markdown("**Best Lambda Values**")
            for m in ['fitness', 'distance']:
                best_idx = stats[f'{m}_mean'].idxmax()
                st.write(f"- {m.title()}: lambda={stats.loc[best_idx, 'lambda']} ({stats.loc[best_idx, f'{m}_mean']:.2f})")

        with col2:
            st.markdown("**Lowest Values (better)**")
            for m in ['contact', 'cot', 'stability']:
                best_idx = stats[f'{m}_mean'].idxmin()
                st.write(f"- {m.title()}: lambda={stats.loc[best_idx, 'lambda']} ({stats.loc[best_idx, f'{m}_mean']:.2f})")

        with col3:
            st.markdown("**Trend**")
            corr = df['lambda'].corr(df[metric])
            trend = "increases" if corr > 0.1 else "decreases" if corr < -0.1 else "stable"
            st.write(f"{metric.title()} {trend} with lambda")
            st.write(f"Correlation: {corr:.3f}")

        # Data table
        with st.expander("View Statistics Table"):
            display_cols = ['lambda', 'n_runs', f'{metric}_mean', f'{metric}_std', f'{metric}_min', f'{metric}_max']
            st.dataframe(stats[display_cols].round(3), use_container_width=True)

    # TAB 2: Box Plots
    with tab2:
        st.header("Distribution per Lambda")

        fig = create_box_plot(df, metric)
        fig.update_layout(height=500)
        st.plotly_chart(fig, use_container_width=True)

        # Compare specific lambdas
        st.subheader("Compare Two Lambdas")
        col1, col2 = st.columns(2)

        with col1:
            lambda1 = st.selectbox("Lambda 1", options=sorted(df['lambda'].unique()), index=0, key='cmp_l1')
        with col2:
            lambda2 = st.selectbox("Lambda 2", options=sorted(df['lambda'].unique()), index=len(df['lambda'].unique())-1, key='cmp_l2')

        df1 = df[df['lambda'] == lambda1]
        df2 = df[df['lambda'] == lambda2]

        col1, col2 = st.columns(2)
        with col1:
            st.markdown(f"**lambda = {lambda1}**")
            st.write(f"Mean {metric}: {df1[metric].mean():.3f}")
            st.write(f"Std: {df1[metric].std():.3f}")
            st.write(f"Best: {df1[metric].max() if metric not in ['contact', 'cot'] else df1[metric].min():.3f}")

        with col2:
            st.markdown(f"**lambda = {lambda2}**")
            st.write(f"Mean {metric}: {df2[metric].mean():.3f}")
            st.write(f"Std: {df2[metric].std():.3f}")
            st.write(f"Best: {df2[metric].max() if metric not in ['contact', 'cot'] else df2[metric].min():.3f}")

    # TAB 3: Correlations
    with tab3:
        st.header("Metric Correlations")

        col1, col2 = st.columns(2)
        with col1:
            x_metric = st.selectbox("X-axis", options=['distance', 'contact', 'cot', 'straightness', 'stability'], index=0)
        with col2:
            y_metric = st.selectbox("Y-axis", options=['distance', 'contact', 'cot', 'straightness', 'stability'], index=1)

        fig = create_correlation_scatter(df, x_metric, y_metric)
        fig.update_layout(height=500)
        st.plotly_chart(fig, use_container_width=True)

        # Correlation matrix
        st.subheader("Correlation Matrix")
        corr_metrics = ['lambda', 'fitness', 'distance', 'contact', 'cot', 'straightness', 'stability']
        corr_matrix = df[corr_metrics].corr()

        fig = px.imshow(
            corr_matrix,
            text_auto='.2f',
            color_continuous_scale='RdBu_r',
            aspect='auto'
        )
        fig.update_layout(height=400)
        st.plotly_chart(fig, use_container_width=True)

    # TAB 4: All Metrics
    with tab4:
        st.header("All Metrics vs Lambda")

        fig = create_multi_metric_plot(stats)
        fig.update_layout(height=500, title="Normalized Metrics (0-1 scale)")
        st.plotly_chart(fig, use_container_width=True)

        st.caption("All metrics normalized to 0-1 range for comparison. Hover for actual values.")

        # Individual metric plots
        st.subheader("Individual Metric Plots")

        metrics = ['distance', 'contact', 'cot', 'straightness', 'stability']
        cols = st.columns(len(metrics))

        for i, m in enumerate(metrics):
            with cols[i]:
                fig = go.Figure()
                fig.add_trace(go.Scatter(
                    x=stats['lambda'],
                    y=stats[f'{m}_mean'],
                    mode='lines+markers',
                    line=dict(width=2),
                    marker=dict(size=6)
                ))
                fig.update_layout(
                    title=m.title(),
                    height=250,
                    margin=dict(l=20, r=20, t=40, b=20),
                    showlegend=False
                )
                st.plotly_chart(fig, use_container_width=True)

    # TAB 5: Robot Viewer
    with tab5:
        robot_viewer_tab(df, selected_robot)

    # Footer
    st.markdown("---")
    st.markdown(
        f"**{selected_robot.title()} Fox V2 Experiment** | Bounds=[-1,1] | "
        f"{len(df)} runs | {df['lambda'].nunique()} lambda values | 25 seeds each"
    )


def compare_robots_view():
    """Compare all robots view."""
    st.markdown("**Comparing:** Spider, Gecko, Arachnid, Tripod | **Bounds:** [-1, 1] | **Lambda:** 0 to 5 (0.25 steps)")
    st.markdown("**Power Formula:** `fitness = distance * (1 - contact)^lambda`")

    # Load data for all robots
    with st.spinner('Loading all robot data...'):
        all_df = load_all_robots_data()

    if all_df.empty:
        st.error("No data found in results/")
        st.info("Make sure you've downloaded the experiment results from Fox.")
        return

    # Calculate stats grouped by robot
    all_stats = get_lambda_stats(all_df, group_by_robot=True)

    # Sidebar controls
    st.sidebar.markdown("---")
    st.sidebar.header("Controls")

    metric = st.sidebar.selectbox(
        "Primary Metric",
        options=['fitness', 'distance', 'contact', 'cot', 'straightness', 'stability'],
        format_func=lambda x: {
            'fitness': 'Fitness',
            'distance': 'Distance (m)',
            'contact': 'Contact (%)',
            'cot': 'Cost of Transport',
            'straightness': 'Straightness (%)',
            'stability': 'Stability'
        }[x],
        key='compare_metric'
    )

    show_std = st.sidebar.checkbox("Show std shading", value=True, key='compare_std')

    # Quick stats per robot
    st.sidebar.markdown("---")
    st.sidebar.header("Data Summary")
    for robot in ROBOT_CONFIG.keys():
        robot_df = all_df[all_df['robot'] == robot]
        st.sidebar.metric(f"{robot.title()}", f"{len(robot_df)} runs")

    # Main tabs
    tab1, tab2, tab3, tab4, tab5 = st.tabs([
        "Robot Comparison",
        "Heatmaps",
        "Cross-Robot Scatter",
        "Best per Lambda",
        "Robot Viewer"
    ])

    # TAB 1: Robot Comparison
    with tab1:
        st.header(f"{metric.title()} vs Lambda - All Robots")

        fig = create_multi_robot_plot(all_stats, metric, show_std)
        fig.update_layout(height=500)
        st.plotly_chart(fig, use_container_width=True)

        # Summary table
        st.subheader("Best Lambda per Robot")

        summary_data = []
        for robot in ROBOT_CONFIG.keys():
            robot_stats = all_stats[all_stats['robot'] == robot]
            if robot_stats.empty:
                continue

            if metric in ['contact', 'cot', 'stability']:
                best_idx = robot_stats[f'{metric}_mean'].idxmin()
            else:
                best_idx = robot_stats[f'{metric}_mean'].idxmax()

            summary_data.append({
                'Robot': robot.title(),
                'Best Lambda': robot_stats.loc[best_idx, 'lambda'],
                f'Best {metric.title()}': robot_stats.loc[best_idx, f'{metric}_mean'],
                f'{metric.title()} Std': robot_stats.loc[best_idx, f'{metric}_std']
            })

        st.dataframe(pd.DataFrame(summary_data), use_container_width=True, hide_index=True)

        # All metrics comparison
        st.subheader("All Metrics Comparison")

        metrics = ['fitness', 'distance', 'contact', 'cot', 'straightness', 'stability']
        cols = st.columns(3)

        for i, m in enumerate(metrics):
            with cols[i % 3]:
                fig = create_multi_robot_plot(all_stats, m, show_std=False)
                fig.update_layout(
                    title=m.title(),
                    height=300,
                    showlegend=(i == 0),
                    margin=dict(l=20, r=20, t=40, b=20)
                )
                st.plotly_chart(fig, use_container_width=True)

    # TAB 2: Heatmaps
    with tab2:
        st.header("Metric Heatmaps")

        heatmap_metric = st.selectbox(
            "Metric",
            options=['fitness', 'distance', 'contact', 'cot', 'straightness', 'stability'],
            key='heatmap_metric'
        )

        fig = create_heatmap(all_df, heatmap_metric)
        fig.update_layout(height=400)
        st.plotly_chart(fig, use_container_width=True)

        st.subheader("All Metrics Heatmaps")
        metrics = ['distance', 'contact', 'cot', 'fitness']
        cols = st.columns(2)

        for i, m in enumerate(metrics):
            with cols[i % 2]:
                fig = create_heatmap(all_df, m)
                fig.update_layout(title=m.title(), height=300)
                st.plotly_chart(fig, use_container_width=True)

    # TAB 3: Cross-Robot Scatter
    with tab3:
        st.header("Cross-Robot Correlation")

        col1, col2, col3 = st.columns(3)
        with col1:
            x_metric = st.selectbox("X-axis", options=['distance', 'contact', 'cot', 'straightness', 'stability', 'fitness'], index=0, key='cross_x')
        with col2:
            y_metric = st.selectbox("Y-axis", options=['distance', 'contact', 'cot', 'straightness', 'stability', 'fitness'], index=1, key='cross_y')
        with col3:
            color_by = st.selectbox("Color by", options=['robot', 'lambda'], index=0, key='cross_color')

        fig = create_correlation_scatter(all_df, x_metric, y_metric, color_by)
        fig.update_layout(height=500)
        st.plotly_chart(fig, use_container_width=True)

        # Correlation per robot
        st.subheader("Correlation per Robot")
        for robot in ROBOT_CONFIG.keys():
            robot_df = all_df[all_df['robot'] == robot]
            if robot_df.empty:
                continue
            corr = robot_df[x_metric].corr(robot_df[y_metric])
            st.write(f"**{robot.title()}**: {x_metric} vs {y_metric} correlation = {corr:.3f}")

    # TAB 4: Best per Lambda
    with tab4:
        st.header("Best Robot per Lambda")

        compare_metric = st.selectbox(
            "Compare by",
            options=['fitness', 'distance', 'contact', 'cot'],
            key='best_metric'
        )

        # Find best robot per lambda
        best_per_lambda = []
        for lam in sorted(all_df['lambda'].unique()):
            lam_df = all_df[all_df['lambda'] == lam]

            for robot in ROBOT_CONFIG.keys():
                robot_lam = lam_df[lam_df['robot'] == robot]
                if robot_lam.empty:
                    continue

                best_per_lambda.append({
                    'Lambda': lam,
                    'Robot': robot.title(),
                    'Mean': robot_lam[compare_metric].mean(),
                    'Best': robot_lam[compare_metric].max() if compare_metric not in ['contact', 'cot'] else robot_lam[compare_metric].min(),
                    'Std': robot_lam[compare_metric].std()
                })

        best_df = pd.DataFrame(best_per_lambda)

        # Bar chart
        fig = px.bar(
            best_df,
            x='Lambda',
            y='Mean',
            color='Robot',
            barmode='group',
            color_discrete_map={k.title(): v['color'] for k, v in ROBOT_CONFIG.items()}
        )
        fig.update_layout(height=400, title=f'Mean {compare_metric.title()} per Lambda')
        st.plotly_chart(fig, use_container_width=True)

        # Winner per lambda
        st.subheader("Winner per Lambda")

        winners = []
        for lam in sorted(all_df['lambda'].unique()):
            lam_data = best_df[best_df['Lambda'] == lam]
            if compare_metric in ['contact', 'cot']:
                winner = lam_data.loc[lam_data['Mean'].idxmin()]
            else:
                winner = lam_data.loc[lam_data['Mean'].idxmax()]
            winners.append({
                'Lambda': lam,
                'Winner': winner['Robot'],
                f'Best {compare_metric.title()}': winner['Mean']
            })

        winners_df = pd.DataFrame(winners)
        st.dataframe(winners_df, use_container_width=True, hide_index=True)

    # TAB 5: Robot Viewer
    with tab5:
        st.header("Robot Visualization")

        col1, col2 = st.columns(2)
        with col1:
            view_robot = st.selectbox(
                "Select Robot",
                options=list(ROBOT_CONFIG.keys()),
                format_func=lambda x: x.title(),
                key='view_robot'
            )

        robot_df = all_df[all_df['robot'] == view_robot]
        if not robot_df.empty:
            robot_viewer_tab(robot_df, view_robot)
        else:
            st.warning(f"No data available for {view_robot}")

    # Footer
    st.markdown("---")
    robot_counts = {r: len(all_df[all_df['robot'] == r]) for r in ROBOT_CONFIG.keys()}
    st.markdown(
        f"**Fox V2 Experiment** | " +
        " | ".join([f"{r.title()}: {c} runs" for r, c in robot_counts.items()])
    )


def robot_viewer_tab(df, selected_robot):
    """Robot viewer tab content."""
    # Selection mode
    selection_mode = st.radio(
        "Selection Mode",
        options=["Filter by Metrics", "Select by Lambda"],
        horizontal=True,
        key=f'viz_selection_mode_{selected_robot}'
    )

    if selection_mode == "Filter by Metrics":
        st.markdown("**Find robots matching your criteria:**")

        # Get data ranges for sliders
        distance_max_val = float(df['distance'].max()) if not pd.isna(df['distance'].max()) else 10.0
        cot_max_val = df['cot'].max()
        if pd.isna(cot_max_val) or cot_max_val <= 0:
            cot_max_val = 100.0
        else:
            cot_max_val = float(cot_max_val)

        # Range sliders for all metrics
        contact_min, contact_max = st.slider(
            "Contact (%)", 0.0, 100.0, (0.0, 100.0), 1.0, key=f'filter_contact_{selected_robot}'
        )
        distance_min, distance_max = st.slider(
            "Distance (m)", 0.0, distance_max_val, (0.0, distance_max_val), 0.1, key=f'filter_distance_{selected_robot}'
        )
        cot_min, cot_max = st.slider(
            "Cost of Transport", 0.0, cot_max_val, (0.0, cot_max_val), 0.5, key=f'filter_cot_{selected_robot}'
        )
        straightness_min, straightness_max = st.slider(
            "Straightness (%)", 0.0, 100.0, (0.0, 100.0), 1.0, key=f"filter_straightness_{selected_robot}"
        )
        stability_max_val = float(df["stability"].max()) if df["stability"].max() > 0 else 1.0
        stability_min, stability_max = st.slider(
            "Stability (lower=better)", 0.0, stability_max_val, (0.0, stability_max_val), 0.01, key=f"filter_stability_{selected_robot}"
        )

        # Optional: filter by lambda
        with st.expander("Additional Filters (Lambda)"):
            lambda_filter = st.multiselect(
                "Lambda",
                options=sorted(df['lambda'].unique()),
                default=sorted(df['lambda'].unique()),
                key=f'filter_lambda_{selected_robot}'
            )

        # Apply filters
        filtered_df = df[
            (df['contact'] >= contact_min) & (df['contact'] <= contact_max) &
            (df['distance'] >= distance_min) & (df['distance'] <= distance_max) &
            (df['cot'] >= cot_min) & (df['cot'] <= cot_max) &
            (df['straightness'] >= straightness_min) & (df['straightness'] <= straightness_max) &
            (df['stability'] >= stability_min) & (df['stability'] <= stability_max) &
            (df['lambda'].isin(lambda_filter))
        ].copy()

        # Sort options
        sort_col1, sort_col2 = st.columns(2)
        with sort_col1:
            sort_by = st.selectbox(
                "Sort by",
                options=['distance', 'contact', 'fitness', 'cot', 'straightness', 'stability'],
                key=f'filter_sort_by_{selected_robot}'
            )
        with sort_col2:
            sort_order = st.radio(
                "Order",
                options=['Best first', 'Worst first'],
                horizontal=True,
                key=f'filter_sort_order_{selected_robot}'
            )

        ascending = sort_order == 'Worst first'
        if sort_by in ['contact', 'cot', 'stability']:
            ascending = not ascending

        filtered_df = filtered_df.sort_values(sort_by, ascending=ascending)

        st.markdown(f"**Found {len(filtered_df)} robots matching criteria**")

        if filtered_df.empty:
            st.warning("No robots match the current filter criteria. Adjust the filters above.")
            return

        # Display results table
        display_df = filtered_df[['lambda', 'run', 'distance', 'contact', 'fitness', 'cot', 'straightness', 'stability']].copy()
        display_df.columns = ['Lambda', 'Run', 'Distance (m)', 'Contact (%)', 'Fitness', 'CoT', 'Straightness (%)', 'Stability']
        display_df['Distance (m)'] = display_df['Distance (m)'].round(2)
        display_df['Contact (%)'] = display_df['Contact (%)'].round(1)
        display_df['Fitness'] = display_df['Fitness'].round(2)
        display_df['CoT'] = display_df['CoT'].round(2)
        display_df['Straightness (%)'] = display_df['Straightness (%)'].round(1)

        # Show top results
        st.dataframe(display_df.head(20), use_container_width=True, hide_index=True)

        # Select robot from filtered results
        st.markdown("---")
        st.subheader("Select Robot to Visualize")

        selected_idx = st.selectbox(
            "Select from filtered results",
            options=range(min(20, len(filtered_df))),
            format_func=lambda i: f"#{i+1}: lambda={filtered_df.iloc[i]['lambda']} run={filtered_df.iloc[i]['run']} | {filtered_df.iloc[i]['distance']:.2f}m, {filtered_df.iloc[i]['contact']:.1f}%",
            key=f'filter_select_idx_{selected_robot}'
        )

        robot = filtered_df.iloc[selected_idx]
        viz_lambda = robot['lambda']
        viz_run = robot['run']

    else:  # Select by Lambda mode
        st.markdown("**Manual selection by lambda:**")

        col1, col2 = st.columns(2)

        with col1:
            viz_lambda = st.selectbox(
                "Lambda",
                options=sorted(df['lambda'].unique()),
                key=f'viz_lambda_{selected_robot}'
            )

        with col2:
            run_selection = st.radio(
                "Run Selection",
                options=["Best across runs", "Specific run"],
                horizontal=True,
                key=f'viz_run_selection_{selected_robot}'
            )

        config_df = df[df['lambda'] == viz_lambda]

        if config_df.empty:
            st.error(f"No data found for lambda={viz_lambda}")
            return

        if run_selection == "Best across runs":
            best_metric = st.selectbox(
                "Best by metric",
                options=['fitness', 'distance', 'contact (lowest)', 'cot (lowest)', 'stability (lowest)'],
                key=f'viz_best_metric_{selected_robot}'
            )

            if best_metric == 'contact (lowest)':
                robot = config_df.loc[config_df['contact'].idxmin()]
            elif best_metric == 'cot (lowest)':
                robot = config_df.loc[config_df['cot'].idxmin()]
            elif best_metric == 'stability (lowest)':
                robot = config_df.loc[config_df['stability'].idxmin()]
            else:
                metric_name = best_metric.split()[0]
                robot = config_df.loc[config_df[metric_name].idxmax()]

            viz_run = robot['run']
            st.info(f"Best robot is from **Run {int(viz_run)}**")

        else:
            available_runs = sorted(config_df['run'].unique())
            viz_run = st.selectbox(
                "Run",
                options=available_runs,
                key=f'viz_run_{selected_robot}'
            )
            robot = config_df[config_df['run'] == viz_run].iloc[0]

    # Display selected robot info
    st.markdown("---")
    st.subheader("Selected Robot")

    col1, col2, col3, col4, col5, col6 = st.columns(6)
    col1.metric("Distance", f"{robot['distance']:.2f} m")
    col2.metric("Contact", f"{robot['contact']:.1f} %")
    col3.metric("Fitness", f"{robot['fitness']:.2f}")
    col4.metric("CoT", f"{robot['cot']:.2f}")
    col5.metric("Straightness", f"{robot['straightness']:.1f} %")
    col6.metric("Stability", f"{robot['stability']:.3f}")

    st.caption(f"Config: {selected_robot.title()}, Bounds=[-1,1], lambda={viz_lambda}, Run={int(viz_run)}")

    st.markdown("---")

    # Visualize button
    if st.button("Launch Visualization", type="primary", use_container_width=True, key=f'viz_btn_{selected_robot}'):
        with st.spinner("Launching MuJoCo viewer..."):
            visualize_robot(robot['params'], viz_lambda, int(viz_run), selected_robot)
        st.success("Viewer launched! Check for the MuJoCo window.")

    # Evolution over generations
    st.markdown("---")
    st.subheader("Evolution Over Generations")

    gen_df = load_generation_data(robot['db_path'])
    if not gen_df.empty:
        gen_metric = st.selectbox(
            "Evolution Metric",
            options=['best_fitness', 'avg_fitness', 'best_distance', 'avg_contact'],
            key=f'gen_metric_{selected_robot}'
        )
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=gen_df['generation'],
            y=gen_df[gen_metric],
            mode='lines',
            name=gen_metric.replace('_', ' ').title(),
            hovertemplate='Gen %{x}<br>%{y:.4f}<extra></extra>'
        ))
        fig.update_layout(
            xaxis_title='Generation',
            yaxis_title=gen_metric.replace('_', ' ').title(),
            height=400,
            hovermode='x unified'
        )
        st.plotly_chart(fig, use_container_width=True)
    else:
        st.warning("Could not load generation data")

    # CPG Parameters
    with st.expander("CPG Parameters"):
        params = [float(x) for x in robot['params'].split(';')]
        st.code(f"params = {params}")


if __name__ == "__main__":
    main()
