
import streamlit as st
import numpy as np
import plotly.graph_objects as go

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

st.sidebar.header("PID Controller")
Kp = st.sidebar.slider("Kp", 0.1, 100.0, 1.2)
Ki = st.sidebar.slider("Ki", 0.0, 50.0, 0.1)
Kd = st.sidebar.slider("Kd", 0.0, 100.0, 0.4)
T_set = st.sidebar.slider("Set Temperature (°C)", 40, 200, 120)

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
