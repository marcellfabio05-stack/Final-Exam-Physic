import streamlit as st
import numpy as np
import plotly.graph_objects as go

# ===============================
# THERMAL EXPANSION FUNCTION
# ===============================
def thermal_expansion(L0, alpha, T_start=20, T_end=200, steps=200):
    T = np.linspace(T_start, T_end, steps)
    L = L0 * (1 + alpha * (T - T_start))
    return T, L

# ===============================
# THERMAL CONDUCTION FUNCTION
# ===============================
def thermal_conduction(k, L=0.5, T_left=200, T_right=20, n_points=50):
    x = np.linspace(0, L, n_points)
    T = np.linspace(T_left, T_right, n_points)
    q = -k * (T_right - T_left) / L
    return x, T, q

# ===============================
# PID TEMPERATURE CONTROL FUNCTION
# ===============================
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.last_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

def pid_sim(Kp, Ki, Kd, T_set, T_initial=20, dt=0.1, total_time=80):
    controller = PID(Kp, Ki, Kd)
    times = np.arange(0, total_time, dt)
    T_vals = np.zeros_like(times)
    T_vals[0] = T_initial
    heat_capacity = 0.2

    for i in range(1, len(times)):
        error = T_set - T_vals[i-1]
        heater_power = controller.update(error, dt)
        heater_power = np.clip(heater_power, 0, 200)
        cooling = 0.02 * (T_vals[i-1] - 20)
        dTdt = heat_capacity * heater_power - cooling
        T_vals[i] = T_vals[i-1] + dTdt * dt

    return times, T_vals

# ===============================
# STREAMLIT UI
# ===============================
st.title("🔥 Thermal Simulation Dashboard")

# Sidebar sliders
st.sidebar.header("Thermal Expansion")
L0 = st.sidebar.slider("Initial Length L₀ (m)", 0.5, 2.5, 1.0)
alpha = st.sidebar.slider("Thermal Expansion α (1/°C)", 5e-6, 50e-6, 20e-6)

st.sidebar.header("Thermal Conduction")
k = st.sidebar.slider("Thermal Conductivity k (W/mK)", 10, 400, 205)
T_left = st.sidebar.slider("Left Temperature (°C)", 30, 300, 200)
T_right = st.sidebar.slider("Right Temperature (°C)", 0, 100, 30)

st.sidebar.header("PID Controller")
Kp = st.sidebar.slider("Kp", 0.1, 5.0, 1.2)
Ki = st.sidebar.slider("Ki", 0.0, 1.0, 0.1)
Kd = st.sidebar.slider("Kd", 0.0, 2.0, 0.4)
T_set = st.sidebar.slider("Set Temperature (°C)", 40, 200, 120)

# ===============================
# THERMAL EXPANSION PLOT
# ===============================
T_exp, L = thermal_expansion(L0, alpha)
fig_exp = go.Figure()
fig_exp.add_trace(go.Scatter(x=T_exp, y=L, mode='lines', name='Length'))
fig_exp.update_layout(
    title="Thermal Expansion",
    xaxis_title="Temperature (°C)",
    yaxis_title="Length (m)"
)
st.plotly_chart(fig_exp, use_container_width=True)

# ===============================
# THERMAL CONDUCTION PLOT
# ===============================
x_cond, T_cond, q = thermal_conduction(k, T_left=T_left, T_right=T_right)
fig_cond = go.Figure()
fig_cond.add_trace(go.Scatter(x=x_cond, y=T_cond, mode='lines', name='Temperature'))
fig_cond.update_layout(
    title="Thermal Conduction",
    xaxis_title="Position (m)",
    yaxis_title="Temperature (°C)"
)
st.plotly_chart(fig_cond, use_container_width=True)

# ===============================
# PID TEMPERATURE CONTROL PLOT
# ===============================
times, Tvals = pid_sim(Kp, Ki, Kd, T_set)
fig_pid = go.Figure()
fig_pid.add_trace(go.Scatter(x=times, y=Tvals, mode='lines', name='Temperature'))
fig_pid.add_trace(go.Scatter(x=times, y=[T_set]*len(times), mode='lines', 
                             name='Set Temperature', line=dict(dash='dash', color='red')))
fig_pid.update_layout(
    title="PID Temperature Control",
    xaxis_title="Time (s)",
    yaxis_title="Temperature (°C)"
)
st.plotly_chart(fig_pid, use_container_width=True)
