import numpy as np
import streamlit as st
import plotly.graph_objects as go

# ==================================================
# PID CONTROLLER
# ==================================================
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

# ==================================================
# NTC THERMISTOR MODEL (≈ matches your resistance scale)
# ==================================================
def thermistor_resistance(T_celsius):
    """
    Returns resistance in kΩ
    Tuned to give ~4.5–9 kΩ over 30–45°C
    """
    R0 = 10.0        # kΩ at 25°C
    B  = 3500.0
    T0 = 298.15
    T  = T_celsius + 273.15
    return R0 * np.exp(B * (1/T - 1/T0))

# ==================================================
# CONTINUOUS SYSTEM + 5s RECORDING
# ==================================================
def simulate_system(
    Kp, Ki, Kd, setpoint,
    total_time=3000,
    dt_internal=0.5,
    dt_record=18.0
):
    pid = PID(Kp, Ki, Kd)

    temperature = 29.0      # initial temperature
    ambient = 29.0

    # Thermal parameters (matched to your plots)
    tau = 450.0             # thermal inertia
    heater_gain = 0.03
    heat_loss = 0.003

    # Sensor inertia
    alpha = 0.85
    measured_temp = temperature

    time_log = []
    resistance_log = []
    temperature_log = []

    t = 0.0
    next_record = 0.0

    while t <= total_time:
        error = setpoint - temperature
        control = pid.update(error, dt_internal)
        control = np.clip(control, 0, 100)

        # Thermal dynamics
        temperature += (
            -(temperature - ambient) / tau
            - heat_loss * (temperature - ambient)
            + heater_gain * control
        ) * dt_internal

        measured_temp = alpha * measured_temp + (1 - alpha) * temperature

        # Record every 5 seconds
        if t >= next_record:
            time_log.append(t)
            temperature_log.append(measured_temp)
            resistance_log.append(thermistor_resistance(measured_temp))
            next_record += dt_record

        t += dt_internal

    return time_log, resistance_log, temperature_log

# ==================================================
# STREAMLIT UI
# ==================================================
st.set_page_config(
    page_title="PID Thermistor Experiment Results",
    layout="wide"
)

st.title("PID-Controlled Thermistor Experiment Results")

st.sidebar.header("PID Parameters")

Kp = st.sidebar.slider("P", 0.0, 100.0, 10.0, 0.5)
Ki = st.sidebar.slider("I", 0.0, 100.0, 10.0, 0.5)
Kd = st.sidebar.slider("D", 0.0, 100.0, 10.0, 0.5)

setpoint = st.sidebar.slider("Temperature Setpoint (°C)", 35, 45, 40)

# ==================================================
# RUN SIMULATION
# ==================================================
time, resistance, temperature = simulate_system(
    Kp, Ki, Kd, setpoint
)

# ==================================================
# FIG 1 — RESISTANCE vs TIME
# ==================================================
fig1 = go.Figure()

fig1.add_trace(go.Scatter(
    x=time,
    y=resistance,
    mode="lines",
    name="Resistance (kΩ)"
))

fig1.update_layout(
    title="RTD Resistance (kΩ) vs Time (s)",
    xaxis_title="Time (s)",
    yaxis_title="Resistance (kΩ)",
    template="plotly_white"
)

st.plotly_chart(fig1, use_container_width=True)

# ==================================================
# FIG 2 — TEMPERATURE vs TIME
# ==================================================
fig2 = go.Figure()

fig2.add_trace(go.Scatter(
    x=time,
    y=temperature,
    mode="lines",
    name="Temperature (°C)"
))

fig2.add_hline(
    y=setpoint,
    line_dash="dash",
    line_color="red",
    annotation_text="Set Temperature"
)

fig2.update_layout(
    title="Temperature (°C) vs Time (s)",
    xaxis_title="Time (s)",
    yaxis_title="Temperature (°C)",
    template="plotly_white"
)

st.plotly_chart(fig2, use_container_width=True)


