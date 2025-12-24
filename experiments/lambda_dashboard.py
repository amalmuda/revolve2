"""
Lambda Experiment Dashboard (V1 - DEPRECATED)
==============================================
DEPRECATED: Use fox_v2_dashboard.py instead for the latest Fox V2 experiments.

This dashboard is for the OLD v1 experiments (spider, gecko only).
For the newer experiments with all robots, use:
    streamlit run fox_v2_dashboard.py

V1 Experiment Details:
- Bounds: [-1, 1] only
- Lambda: 0 to 5 in 0.25 steps (21 values)
- Seeds: 25 per lambda
- Robots: spider, gecko (old data)

Run with: streamlit run lambda_dashboard.py
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
    page_title="Lambda Experiment",
    page_icon="🔬",
    layout="wide",
    initial_sidebar_state="expanded"
)

# =============================================================================
# ROBOT CONFIGURATION
# =============================================================================

ROBOT_CONFIG = {
    'spider': {
        'results_dir': 'bounds1_pop25_25seeds',
        'glob_pattern': 'spider_m1_power_bounds1_lambda*'
    },
    'gecko': {
        'results_dir': 'gecko_bounds1_pop25_25seeds',
        'glob_pattern': 'gecko_m1_power_bounds1_lambda*'
    }
}

# =============================================================================
# DATA LOADING
# =============================================================================

@st.cache_data
def load_lambda_data(robot_name: str = 'spider'):
    """Load experiment data from the lambda sweep experiment."""
    config = ROBOT_CONFIG.get(robot_name, ROBOT_CONFIG['spider'])
    results_dir = Path(__file__).parent / 'results' / config['results_dir']

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
        lam = float(lambda_str)

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


def get_lambda_stats(df):
    """Calculate stats grouped by lambda."""
    metrics = ['fitness', 'distance', 'contact', 'cot', 'straightness', 'stability']

    agg_dict = {
        'n_runs': pd.NamedAgg(column='run', aggfunc='count')
    }
    for m in metrics:
        agg_dict[f'{m}_mean'] = pd.NamedAgg(column=m, aggfunc='mean')
        agg_dict[f'{m}_std'] = pd.NamedAgg(column=m, aggfunc='std')
        agg_dict[f'{m}_min'] = pd.NamedAgg(column=m, aggfunc='min')
        agg_dict[f'{m}_max'] = pd.NamedAgg(column=m, aggfunc='max')

    stats = df.groupby('lambda').agg(**agg_dict).reset_index()
    return stats


# =============================================================================
# PLOTTING
# =============================================================================

def create_lambda_plot(stats_df, raw_df, metric, show_std=True, show_points=False):
    """Create main lambda vs metric plot."""
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
        hovertemplate='λ=%{x}<br>%{y:.3f}<extra></extra>'
    ))

    # Individual points
    if show_points:
        fig.add_trace(go.Scatter(
            x=raw_df['lambda'],
            y=raw_df[metric],
            mode='markers',
            name='Individual runs',
            marker=dict(color='#636EFA', size=5, opacity=0.4),
            hovertemplate='λ=%{x}<br>Run %{customdata}<br>%{y:.3f}<extra></extra>',
            customdata=raw_df['run']
        ))

    fig.update_layout(
        xaxis_title='Lambda (λ)',
        yaxis_title=metric.replace('_', ' ').title(),
        hovermode='closest',
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
        xaxis_title='Lambda (λ)',
        yaxis_title=metric.replace('_', ' ').title()
    )

    return fig


def create_correlation_scatter(df, x_metric, y_metric, color_by_lambda=True):
    """Create scatter plot of two metrics."""
    if color_by_lambda:
        fig = px.scatter(
            df,
            x=x_metric,
            y=y_metric,
            color='lambda',
            color_continuous_scale='Viridis',
            hover_data=['run', 'fitness']
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
        xaxis_title='Lambda (λ)',
        yaxis_title='Normalized Value (0-1)',
        legend=dict(yanchor="top", y=0.99, xanchor="left", x=1.02)
    )

    return fig


# =============================================================================
# VISUALIZATION
# =============================================================================

def visualize_robot(params_str: str, lam: float, run: int, robot_name: str = 'spider'):
    """Launch MuJoCo viewer."""
    params = [float(x) for x in params_str.split(';')]

    viz_script = f'''
import sys
sys.path.insert(0, "{Path(__file__).parent}")
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

    temp_script = Path(__file__).parent / '_temp_viz.py'
    temp_script.write_text(viz_script)

    subprocess.Popen([sys.executable, str(temp_script)], cwd=str(Path(__file__).parent))
    return True


# =============================================================================
# MAIN DASHBOARD
# =============================================================================

def main():
    st.title("🔬 Lambda Experiment Dashboard")

    # Sidebar - Robot Selection (at the top)
    st.sidebar.header("Robot Selection")
    available_robots = list(ROBOT_CONFIG.keys())
    selected_robot = st.sidebar.selectbox(
        "Robot",
        options=available_robots,
        format_func=lambda x: x.title(),
        key='robot_selector'
    )

    st.markdown(f"**Robot:** {selected_robot.title()} | **Bounds:** [-1, 1] | **Lambda:** 0 to 5 (0.25 steps) | **Seeds:** 25 per λ")
    st.markdown("**Power Formula:** `fitness = distance × (1 - contact)^λ`")

    # Load data for selected robot
    with st.spinner(f'Loading {selected_robot} experiment data...'):
        df = load_lambda_data(selected_robot)

    if df.empty:
        config = ROBOT_CONFIG[selected_robot]
        st.error(f"No data found in results/{config['results_dir']}/")
        st.info("Make sure you've pulled the experiment results from Fox.")
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
    st.sidebar.metric(f"Best λ for {metric}", f"{best_lambda} ({best_value:.2f})")

    # Main tabs
    tab1, tab2, tab3, tab4, tab5 = st.tabs([
        "📈 Lambda Sweep",
        "📊 Distributions",
        "🔗 Correlations",
        "📉 All Metrics",
        "🤖 Robot Viewer"
    ])

    # ==========================================================================
    # TAB 1: Main Lambda Plot
    # ==========================================================================
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
                st.write(f"- {m.title()}: λ={stats.loc[best_idx, 'lambda']} ({stats.loc[best_idx, f'{m}_mean']:.2f})")

        with col2:
            st.markdown("**Lowest Values (better)**")
            for m in ['contact', 'cot', 'stability']:
                best_idx = stats[f'{m}_mean'].idxmin()
                st.write(f"- {m.title()}: λ={stats.loc[best_idx, 'lambda']} ({stats.loc[best_idx, f'{m}_mean']:.2f})")

        with col3:
            st.markdown("**Trend**")
            # Calculate correlation between lambda and metric
            corr = df['lambda'].corr(df[metric])
            trend = "increases" if corr > 0.1 else "decreases" if corr < -0.1 else "stable"
            st.write(f"{metric.title()} {trend} with λ")
            st.write(f"Correlation: {corr:.3f}")

        # Data table
        with st.expander("View Statistics Table"):
            display_cols = ['lambda', 'n_runs', f'{metric}_mean', f'{metric}_std', f'{metric}_min', f'{metric}_max']
            st.dataframe(stats[display_cols].round(3), use_container_width=True)

    # ==========================================================================
    # TAB 2: Box Plots
    # ==========================================================================
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
            st.markdown(f"**λ = {lambda1}**")
            st.write(f"Mean {metric}: {df1[metric].mean():.3f}")
            st.write(f"Std: {df1[metric].std():.3f}")
            st.write(f"Best: {df1[metric].max() if metric not in ['contact', 'cot'] else df1[metric].min():.3f}")

        with col2:
            st.markdown(f"**λ = {lambda2}**")
            st.write(f"Mean {metric}: {df2[metric].mean():.3f}")
            st.write(f"Std: {df2[metric].std():.3f}")
            st.write(f"Best: {df2[metric].max() if metric not in ['contact', 'cot'] else df2[metric].min():.3f}")

    # ==========================================================================
    # TAB 3: Correlations
    # ==========================================================================
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

    # ==========================================================================
    # TAB 4: All Metrics
    # ==========================================================================
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

    # ==========================================================================
    # TAB 5: Robot Viewer
    # ==========================================================================
    with tab5:
        st.header("Robot Visualization")

        # Selection mode
        selection_mode = st.radio(
            "Selection Mode",
            options=["Filter by Metrics", "Select by Lambda"],
            horizontal=True,
            key='viz_selection_mode'
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
                "Contact (%)", 0.0, 100.0, (0.0, 100.0), 1.0, key='filter_contact'
            )
            distance_min, distance_max = st.slider(
                "Distance (m)", 0.0, distance_max_val, (0.0, distance_max_val), 0.1, key='filter_distance'
            )
            cot_min, cot_max = st.slider(
                "Cost of Transport", 0.0, cot_max_val, (0.0, cot_max_val), 0.5, key='filter_cot'
            )
            straightness_min, straightness_max = st.slider(
                "Straightness (%)", 0.0, 100.0, (0.0, 100.0), 1.0, key="filter_straightness"
            )
            stability_max_val = float(df["stability"].max()) if df["stability"].max() > 0 else 1.0
            stability_min, stability_max = st.slider(
                "Stability (lower=better)", 0.0, stability_max_val, (0.0, stability_max_val), 0.01, key="filter_stability"
            )

            # Optional: filter by lambda
            with st.expander("Additional Filters (Lambda)"):
                lambda_filter = st.multiselect(
                    "Lambda",
                    options=sorted(df['lambda'].unique()),
                    default=sorted(df['lambda'].unique()),
                    key='filter_lambda'
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
                    key='filter_sort_by'
                )
            with sort_col2:
                sort_order = st.radio(
                    "Order",
                    options=['Best first', 'Worst first'],
                    horizontal=True,
                    key='filter_sort_order'
                )

            ascending = sort_order == 'Worst first'
            if sort_by in ['contact', 'cot', 'stability']:  # Lower is better for these
                ascending = not ascending

            filtered_df = filtered_df.sort_values(sort_by, ascending=ascending)

            st.markdown(f"**Found {len(filtered_df)} robots matching criteria**")

            if filtered_df.empty:
                st.warning("No robots match the current filter criteria. Adjust the filters above.")
                st.stop()

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
                format_func=lambda i: f"#{i+1}: λ={filtered_df.iloc[i]['lambda']} run={filtered_df.iloc[i]['run']} | {filtered_df.iloc[i]['distance']:.2f}m, {filtered_df.iloc[i]['contact']:.1f}%",
                key='filter_select_idx'
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
                    key='viz_lambda'
                )

            with col2:
                # Option to select best or specific run
                run_selection = st.radio(
                    "Run Selection",
                    options=["Best across runs", "Specific run"],
                    horizontal=True,
                    key='viz_run_selection'
                )

            config_df = df[df['lambda'] == viz_lambda]

            if config_df.empty:
                st.error(f"No data found for λ={viz_lambda}")
                st.stop()

            if run_selection == "Best across runs":
                # Let user choose what "best" means
                best_metric = st.selectbox(
                    "Best by metric",
                    options=['fitness', 'distance', 'contact (lowest)', 'cot (lowest)', 'stability (lowest)'],
                    key='viz_best_metric'
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
                    key='viz_run'
                )
                robot = config_df[config_df['run'] == viz_run].iloc[0]

        # Display selected robot info (common to both modes)
        st.markdown("---")
        st.subheader("Selected Robot")

        col1, col2, col3, col4, col5, col6 = st.columns(6)
        col1.metric("Distance", f"{robot['distance']:.2f} m")
        col2.metric("Contact", f"{robot['contact']:.1f} %")
        col3.metric("Fitness", f"{robot['fitness']:.2f}")
        col4.metric("CoT", f"{robot['cot']:.2f}")
        col5.metric("Straightness", f"{robot['straightness']:.1f} %")
        col6.metric("Stability", f"{robot['stability']:.3f}")

        st.caption(f"Config: {selected_robot.title()}, Bounds=[-1,1], λ={viz_lambda}, Run={int(viz_run)}")

        st.markdown("---")

        # Visualize button
        if st.button("Launch Visualization", type="primary", use_container_width=True):
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
                options=['best_fitness', 'avg_fitness', 'best_distance', 'avg_contact']
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

    # Footer
    st.markdown("---")
    st.markdown(
        f"**{selected_robot.title()} Lambda Experiment** | Bounds=[-1,1] | "
        f"{len(df)} runs | {df['lambda'].nunique()} lambda values | 25 seeds each"
    )


if __name__ == "__main__":
    main()
