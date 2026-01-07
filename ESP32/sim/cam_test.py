import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# Lane cam definition: along-lane positions (mm) and cam heights (mm).
# Updated fork cam: deeper pockets, steeper ramps, modest neutral. Tuned so total force crosses zero near pockets.
x_samples = np.array([ -80, -70, -64, -58, -52, -46, -40, -32, -24, -16, -8, 0, 8, 16, 24, 32, 40, 46, 52, 58, 64, 70, 80 ], dtype=float)
h_samples = np.array([
    0.0,  -0.4, -1.9, -2.6, -3.0, -2.6, -1.6,   # left pocket & ramps
    -0.8, -0.3, -0.15,                          # into neutral
    -0.9,  -1.5,                                # neutral pocket
    -0.9, -0.15, -0.3, -0.8,                    # out of neutral
    -1.6, -2.6, -3.0, -2.6, -1.9, -0.4, 0.0     # right pocket & ramps
], dtype=float)

# Fit a C1 spline (natural end conditions → zero curvature at ends; clamped can be used instead)
spline = CubicSpline(x_samples, h_samples, bc_type="natural")

# Evaluate on a dense grid
x = np.linspace(x_samples[0], x_samples[-1], 400)
h = spline(x)
dh_dx = spline(x, 1)  # first derivative (slope)

# Map slope to force: F = -spring_scale * dh/dx
spring_scale = 60.0  # N per unit height (detent stiffness)
force = -spring_scale * dh_dx

# Add a centering spring (pull to 0) to mimic fork return springs.
centering_k = 0.15  # N/mm
force_center = -centering_k * x
force_total = force + force_center

# Optional roller/ball follower: offset profile upward by radius and optionally smooth to mimic rounded contact
roller_radius = 5.0  # mm
smooth_sigma = 0.8   # mm (set to 0 to disable smoothing)

h_roller = h + roller_radius
if smooth_sigma > 0.0:
    from scipy.ndimage import gaussian_filter1d
    h_roller = gaussian_filter1d(h_roller, sigma=smooth_sigma / (x[1] - x[0]), mode="nearest")
dh_dx_roller = np.gradient(h_roller, x)
force_roller = -spring_scale * dh_dx_roller
force_total_roller = force_roller + force_center

# Plot elevation and force
fig, axes = plt.subplots(2, 1, sharex=True, figsize=(7, 6))
axes[0].plot(x, h, label="Cam height (spline)")
axes[0].plot(x, h_roller, "--", label=f"Roller path (r={roller_radius}mm)")
axes[0].plot(x_samples, h_samples, "o", label="Samples")
axes[0].set_ylabel("Height")
axes[0].legend()

axes[1].plot(x, force, color="tab:red", label="Force from slope")
axes[1].plot(x, force_center, color="tab:purple", linestyle=":", label="Center spring")
axes[1].plot(x, force_total, color="tab:blue", label="Slope + center")
axes[1].plot(x, force_roller, color="tab:green", linestyle="--", label="Force with roller")
axes[1].plot(x, force_total_roller, color="tab:orange", linestyle="-.", label="Roller + center")
axes[1].axhline(0, color="k", linewidth=0.8)
axes[1].set_xlabel("Along-lane position (mm)")
axes[1].set_ylabel("Force (N)")
axes[1].legend()

plt.tight_layout()
plt.show()
