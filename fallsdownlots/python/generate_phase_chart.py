import numpy as np

# Output should look like
'''
const uint32_t stepper_phase_chart[N_PHASES * 4] = {
    PWM_PERIOD_US, 0, PWM_PERIOD_US, 0,
    PWM_PERIOD_US / 2, PWM_PERIOD_US / 2, PWM_PERIOD_US, 0,
    0, PWM_PERIOD_US, PWM_PERIOD_US, 0,
    0, PWM_PERIOD_US, PWM_PERIOD_US / 2, PWM_PERIOD_US / 2,
    0, PWM_PERIOD_US, 0, PWM_PERIOD_US,
    PWM_PERIOD_US / 2, PWM_PERIOD_US / 2, 0, PWM_PERIOD_US,
    PWM_PERIOD_US, 0, 0, PWM_PERIOD_US,
    PWM_PERIOD_US, 0, PWM_PERIOD_US / 2, PWM_PERIOD_US / 2};
'''

val_string = "PWM_PERIOD_US"
N_PHASES = 16

t = np.linspace(0., 2.*np.pi, N_PHASES, endpoint=False)
vals = np.vstack([
    np.cos(t), -np.cos(t), np.sin(t), -np.sin(t)
]).T*0.5 + 0.5

chart = ""
for row in vals:
    chart += "\tPWM_PERIOD_US*{:.2f}, PWM_PERIOD_US*{:.2f}, PWM_PERIOD_US*{:.2f}, PWM_PERIOD_US*{:.2f},\n".format(
        *row)

out = """
const uint32_t N_PHASES = {N_PHASES}
const uint32_t stepper_phase_chart[N_PHASES * 4] = {{
    {chart}
}};
""".format(N_PHASES=N_PHASES, chart=chart)

print(out)
