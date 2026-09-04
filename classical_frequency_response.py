import numpy as np
import matplotlib.pyplot as plt
from control import (
    tf, feedback, step_response,
    frequency_response, nyquist_plot,
    margin, poles, zeros
)
from control.matlab import stepinfo
import warnings
import os
import sys

# Force UTF-8 output encoding for Windows
if sys.platform == 'win32':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except:
        pass

warnings.filterwarnings('ignore')

# Create output directory in current folder
output_dir = 'pid_figures'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)
    print(f"Created output directory: {output_dir}")

# Set publication-quality plot parameters
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['xtick.labelsize'] = 9
plt.rcParams['ytick.labelsize'] = 9
plt.rcParams['legend.fontsize'] = 9
plt.rcParams['figure.titlesize'] = 14
plt.rcParams['lines.linewidth'] = 2
plt.rcParams['axes.grid'] = True
plt.rcParams['grid.alpha'] = 0.3

# ========================= PARAMETERS =========================
# DC Motor Parameters
J = 0.01   # Moment of inertia (kg.m^2)
B = 0.1    # Viscous friction (N.m.s)
K = 0.01   # Motor constant (N.m/A)
R = 1.0    # Armature resistance (Ohm)
L = 0.5    # Armature inductance (H)

print("=" * 70)
print("DC MOTOR PID CONTROL SIMULATION")
print("=" * 70)

# Motor Transfer Function G(s) = K / [(L s + R)(J s + B) + K^2]
num_motor = [K]
den_motor = [L * J, L * B + R * J, R * B + K ** 2]
G_motor = tf(num_motor, den_motor)

print("\nMotor Transfer Function:")
print(G_motor)

# ========================= OPEN-LOOP ANALYSIS =========================
print("\n" + "=" * 70)
print("OPEN-LOOP ANALYSIS")
print("=" * 70)

t_open = np.linspace(0, 5, 1000)
t_out, y_out = step_response(G_motor, t_open)

# Calculate open-loop metrics
info_open = stepinfo(G_motor, t_open)
ss_value_open = y_out[-1]
ss_error_open = (1 - ss_value_open) * 100

print(f"\nRise Time: {info_open['RiseTime']:.3f} s")
print(f"Settling Time: {info_open['SettlingTime']:.3f} s")
print(f"Overshoot: {info_open['Overshoot']:.2f} %")
print(f"Steady-State Value: {ss_value_open:.4f} rad/s")
print(f"Steady-State Error: {ss_error_open:.2f} %")

# ========================= PID CONTROLLER DESIGN =========================
print("\n" + "=" * 70)
print("PID CONTROLLER DESIGN")
print("=" * 70)

# Manual tuning (optimized)
Kp = 100
Ki = 200
Kd = 10

print(f"\nPID Parameters:")
print(f"  Kp = {Kp}")
print(f"  Ki = {Ki}")
print(f"  Kd = {Kd}")

# PID Transfer Function C(s) = Kp + Ki/s + Kd*s  = (Kd s^2 + Kp s + Ki) / s
num_pid = [Kd, Kp, Ki]
den_pid = [1, 0]
C_pid = tf(num_pid, den_pid)

print("\nPID Controller Transfer Function:")
print(C_pid)

# Closed-Loop System T(s) = C(s)G(s) / [1 + C(s)G(s)]
T_closed = feedback(C_pid * G_motor, 1)

print("\nClosed-Loop Transfer Function:")
print(T_closed)

# ========================= CLOSED-LOOP ANALYSIS =========================
print("\n" + "=" * 70)
print("CLOSED-LOOP ANALYSIS")
print("=" * 70)

t_closed = np.linspace(0, 2, 1000)
t_cl, y_cl = step_response(T_closed, t_closed)

info_closed = stepinfo(T_closed, t_closed)
ss_value_closed = y_cl[-1]
ss_error_closed = (1 - ss_value_closed) * 100

print(f"\nRise Time: {info_closed['RiseTime']:.4f} s")
print(f"Settling Time: {info_closed['SettlingTime']:.4f} s")
print(f"Overshoot: {info_closed['Overshoot']:.2f} %")
print(f"Peak Value: {info_closed['Peak']:.4f} rad/s")
print(f"Steady-State Error: {ss_error_closed:.2f} %")

# Performance Improvement Calculation
rise_improvement = (1 - info_closed['RiseTime'] / info_open['RiseTime']) * 100
settling_improvement = (1 - info_closed['SettlingTime'] / info_open['SettlingTime']) * 100
error_improvement = (1 - ss_error_closed / ss_error_open) * 100

print(f"\nPerformance Improvements:")
print(f"  Rise Time: {rise_improvement:.1f}% reduction")
print(f"  Settling Time: {settling_improvement:.1f}% reduction")
print(f"  SS Error: {error_improvement:.1f}% reduction")

# ========================= FREQUENCY DOMAIN ANALYSIS =========================
print("\n" + "=" * 70)
print("FREQUENCY DOMAIN ANALYSIS")
print("=" * 70)

# Calculate margins for C(s)G(s)
gm, pm, wgm, wpm = margin(C_pid * G_motor)
gm_db = 20 * np.log10(gm) if gm and gm > 0 else float('inf')

print(f"\nGain Margin: {gm_db:.2f} dB (gain factor: {gm:.2f})")
print(f"Phase Margin: {pm:.2f} degrees")
print(f"Gain Crossover Frequency: {wpm:.2f} rad/s")
print(f"Phase Crossover Frequency: {wgm:.2f} rad/s")

# Poles and Zeros of closed loop
poles_cl = poles(T_closed)
zeros_cl = zeros(T_closed)

print(f"\nClosed-Loop Poles:")
for i, pole in enumerate(poles_cl):
    print(f"  p{i+1} = {pole:.4f}")
print(f"\nClosed-Loop Zeros:")
for i, z in enumerate(zeros_cl):
    print(f"  z{i+1} = {z:.4f}")

# ========================= VISUALIZATION =========================
# Create individual high-quality figures for LaTeX report

# Color scheme
color_open = '#2E86AB'
color_closed = '#A23B72'
color_ref = '#F18F01'
color_error = '#C73E1D'
color_grid = '#CCCCCC'

print("\n" + "=" * 70)
print("GENERATING FIGURES...")
print("=" * 70)

# FIGURE 1: Step Response Comparison
print("\n[1/9] Creating Step Response Comparison...")
fig1, ax1 = plt.subplots(figsize=(8, 5))
ax1.plot(t_out, y_out, color=color_open, linestyle='--', 
         linewidth=2.5, label='Open-Loop', alpha=0.8)
ax1.plot(t_cl, y_cl, color=color_closed, linestyle='-', 
         linewidth=2.5, label='Closed-Loop (PID)')
ax1.axhline(y=1.0, color='black', linestyle=':', linewidth=1.5, alpha=0.6)
ax1.grid(True, alpha=0.3, linestyle='--', color=color_grid)
ax1.set_xlabel('Time (s)', fontweight='bold')
ax1.set_ylabel('Angular Velocity (rad/s)', fontweight='bold')
ax1.set_title('Step Response Comparison: Open-Loop vs Closed-Loop', 
              fontweight='bold', pad=15)
ax1.legend(loc='lower right', framealpha=0.95, edgecolor='gray')
ax1.set_xlim([0, 5])
ax1.set_ylim([-0.05, 1.15])
plt.tight_layout()
plt.savefig(os.path.join(output_dir, 'fig1_step_response_comparison.png'), 
            dpi=300, bbox_inches='tight')
plt.savefig(os.path.join(output_dir, 'fig1_step_response_comparison.pdf'), 
            bbox_inches='tight')
plt.close()

# FIGURE 2: Closed-Loop Detailed Response with Settling Bands
print("[2/9] Creating Closed-Loop Detailed Response...")
fig2, ax2 = plt.subplots(figsize=(8, 5))
ax2.plot(t_cl, y_cl, color=color_closed, linewidth=2.5, label='System Response')
ax2.axhline(y=1.0, color='black', linestyle='-', linewidth=1.5, 
            alpha=0.6, label='Reference')
ax2.axhline(y=1.02, color='green', linestyle='--', linewidth=1.5, 
            alpha=0.5, label='±2% Settling Band')
ax2.axhline(y=0.98, color='green', linestyle='--', linewidth=1.5, alpha=0.5)
ax2.fill_between(t_cl, 0.98, 1.02, color='green', alpha=0.1)
ax2.grid(True, alpha=0.3, linestyle='--', color=color_grid)
ax2.set_xlabel('Time (s)', fontweight='bold')
ax2.set_ylabel('Angular Velocity (rad/s)', fontweight='bold')
ax2.set_title('Closed-Loop Step Response with Settling Characteristics', 
              fontweight='bold', pad=15)
ax2.legend(loc='lower right', framealpha=0.95, edgecolor='gray')
ax2.set_xlim([0, 2])
ax2.set_ylim([0, 1.15])
plt.tight_layout()
plt.savefig(os.path.join(output_dir, 'fig2_closed_loop_detailed.png'), 
            dpi=300, bbox_inches='tight')
plt.savefig(os.path.join(output_dir, 'fig2_closed_loop_detailed.pdf'), 
            bbox_inches='tight')
plt.close()

# FIGURE 3: Tracking Error
print("[3/9] Creating Tracking Error Plot...")
fig3, ax3 = plt.subplots(figsize=(8, 5))
error_signal = 1.0 - y_cl
ax3.plot(t_cl, error_signal, color=color_error, linewidth=2.5)
ax3.axhline(y=0, color='black', linestyle='-', linewidth=1.5, alpha=0.6)
ax3.fill_between(t_cl, 0, error_signal, color=color_error, alpha=0.2)
ax3.grid(True, alpha=0.3, linestyle='--', color=color_grid)
ax3.set_xlabel('Time (s)', fontweight='bold')
ax3.set_ylabel('Error (rad/s)', fontweight='bold')
ax3.set_title('Tracking Error vs Time', fontweight='bold', pad=15)
ax3.set_xlim([0, 2])
plt.tight_layout()
plt.savefig(os.path.join(output_dir, 'fig3_tracking_error.png'), 
            dpi=300, bbox_inches='tight')
plt.savefig(os.path.join(output_dir, 'fig3_tracking_error.pdf'), 
            bbox_inches='tight')
plt.close()

# FIGURE 4: Bode Plot (Combined)
print("[4/9] Creating Bode Plot...")
fig4, (ax4a, ax4b) = plt.subplots(2, 1, figsize=(8, 7))

# Use frequency_response instead of deprecated bode_plot
omega = np.logspace(-2, 3, 1000)
mag, phase, omega = frequency_response(C_pid * G_motor, omega)

# Magnitude plot
ax4a.semilogx(omega, 20 * np.log10(mag), color=color_closed, linewidth=2.5)
ax4a.axhline(y=0, color='red', linestyle='--', linewidth=1.5, alpha=0.7, 
             label='0 dB')
ax4a.grid(True, alpha=0.3, which='both', linestyle='--', color=color_grid)
ax4a.set_ylabel('Magnitude (dB)', fontweight='bold')
ax4a.set_title('Bode Diagram of Loop Transfer Function C(s)G(s)', 
               fontweight='bold', pad=15)
ax4a.legend(loc='upper right', framealpha=0.95, edgecolor='gray')

# Phase plot
ax4b.semilogx(omega, phase * 180 / np.pi, color=color_closed, linewidth=2.5)
ax4b.axhline(y=-180, color='red', linestyle='--', linewidth=1.5, alpha=0.7, 
             label='-180 deg')
ax4b.grid(True, alpha=0.3, which='both', linestyle='--', color=color_grid)
ax4b.set_xlabel('Frequency (rad/s)', fontweight='bold')
ax4b.set_ylabel('Phase (deg)', fontweight='bold')
ax4b.legend(loc='upper right', framealpha=0.95, edgecolor='gray')

plt.tight_layout()
plt.savefig(os.path.join(output_dir, 'fig4_bode_plot.png'), 
            dpi=300, bbox_inches='tight')
plt.savefig(os.path.join(output_dir, 'fig4_bode_plot.pdf'), 
            bbox_inches='tight')
plt.close()

# FIGURE 5: Nyquist Plot
print("[5/9] Creating Nyquist Plot...")
fig5, ax5 = plt.subplots(figsize=(7, 7))
count = nyquist_plot(C_pid * G_motor, omega=np.logspace(-2, 3, 2000), 
                     plot=True, ax=ax5)
ax5.plot(-1, 0, 'rx', markersize=15, markeredgewidth=3, 
         label='Critical Point (-1, 0j)')
ax5.grid(True, alpha=0.3, linestyle='--', color=color_grid)
ax5.set_xlabel('Real Axis', fontweight='bold')
ax5.set_ylabel('Imaginary Axis', fontweight='bold')
ax5.set_title('Nyquist Diagram', fontweight='bold', pad=15)
ax5.legend(loc='upper left', framealpha=0.95, edgecolor='gray')
ax5.axis('equal')
ax5.set_xlim([-2, 2])
ax5.set_ylim([-2, 2])
plt.tight_layout()
plt.savefig(os.path.join(output_dir, 'fig5_nyquist_plot.png'), 
            dpi=300, bbox_inches='tight')
plt.savefig(os.path.join(output_dir, 'fig5_nyquist_plot.pdf'), 
            bbox_inches='tight')
plt.close()

# FIGURE 6: Root Locus
print("[6/9] Creating Root Locus Plot...")
fig6, ax6 = plt.subplots(figsize=(8, 6))
from control import root_locus as rlocus
try:
    # Try new API
    rlist, klist = rlocus(C_pid * G_motor, plot=True, ax=ax6, 
                          grid=True, print_gain=False)
except:
    # Fallback to older API
    from control.matlab import rlocus
    rlist, klist = rlocus(C_pid * G_motor)

ax6.grid(True, alpha=0.3, linestyle='--', color=color_grid)
ax6.set_xlabel('Real Axis', fontweight='bold')
ax6.set_ylabel('Imaginary Axis', fontweight='bold')
ax6.set_title('Root Locus Diagram', fontweight='bold', pad=15)
ax6.axhline(y=0, color='black', linewidth=0.8)
ax6.axvline(x=0, color='black', linewidth=0.8)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, 'fig6_root_locus.png'), 
            dpi=300, bbox_inches='tight')
plt.savefig(os.path.join(output_dir, 'fig6_root_locus.pdf'), 
            bbox_inches='tight')
plt.close()

# FIGURE 7: Disturbance Rejection
print("[7/9] Creating Disturbance Rejection Plot...")
fig7, ax7 = plt.subplots(figsize=(8, 5))
t_dist = np.linspace(0, 10, 2000)
ref_input = np.ones_like(t_dist)

# Reference tracking
t_ref, y_ref = step_response(T_closed, t_dist)

# Disturbance path: G / (1 + CG)
G_dist = feedback(G_motor, C_pid)
t_d, y_d = step_response(G_dist, t_dist)
y_d = y_d * (-0.2)  # 20% load disturbance

# Combine reference and disturbance responses
y_total = np.copy(y_ref)
for i, t in enumerate(t_ref):
    if t >= 5:
        idx_offset = np.argmin(np.abs(t_d - (t - 5)))
        if idx_offset < len(y_d):
            y_total[i] += y_d[idx_offset]

ax7.plot(t_dist, ref_input, color=color_ref, linestyle='--', 
         linewidth=2, label='Reference Input')
ax7.plot(t_ref, y_total, color=color_closed, linestyle='-', 
         linewidth=2.5, label='Actual Output')
ax7.axvline(x=5, color='red', linestyle=':', linewidth=2, 
            alpha=0.6, label='Disturbance Applied')
ax7.grid(True, alpha=0.3, linestyle='--', color=color_grid)
ax7.set_xlabel('Time (s)', fontweight='bold')
ax7.set_ylabel('Angular Velocity (rad/s)', fontweight='bold')
ax7.set_title('Disturbance Rejection (20% Load Disturbance at t=5s)', 
              fontweight='bold', pad=15)
ax7.legend(loc='lower right', framealpha=0.95, edgecolor='gray')
ax7.set_xlim([0, 10])
ax7.set_ylim([0.6, 1.15])
plt.tight_layout()
plt.savefig(os.path.join(output_dir, 'fig7_disturbance_rejection.png'), 
            dpi=300, bbox_inches='tight')
plt.savefig(os.path.join(output_dir, 'fig7_disturbance_rejection.pdf'), 
            bbox_inches='tight')
plt.close()

# FIGURE 8: Performance Comparison Bar Chart
print("[8/9] Creating Performance Comparison Chart...")
fig8, ax8 = plt.subplots(figsize=(8, 5))
metrics = ['Rise Time\n(s)', 'Settling Time\n(s)', 'SS Error\n(%)']
open_vals = [info_open['RiseTime'], info_open['SettlingTime'], ss_error_open]
closed_vals = [info_closed['RiseTime'], info_closed['SettlingTime'], ss_error_closed]

x = np.arange(len(metrics))
width = 0.35

bars1 = ax8.bar(x - width/2, open_vals, width, label='Open-Loop', 
                color=color_open, alpha=0.8, edgecolor='black', linewidth=1.2)
bars2 = ax8.bar(x + width/2, closed_vals, width, label='Closed-Loop (PID)', 
                color=color_closed, alpha=0.8, edgecolor='black', linewidth=1.2)

# Add value labels on bars
for bars in [bars1, bars2]:
    for bar in bars:
        height = bar.get_height()
        ax8.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.3f}',
                ha='center', va='bottom', fontsize=9, fontweight='bold')

ax8.set_ylabel('Value', fontweight='bold')
ax8.set_title('Performance Metrics Comparison', fontweight='bold', pad=15)
ax8.set_xticks(x)
ax8.set_xticklabels(metrics)
ax8.legend(framealpha=0.95, edgecolor='gray')
ax8.grid(True, alpha=0.3, axis='y', linestyle='--', color=color_grid)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, 'fig8_performance_comparison.png'), 
            dpi=300, bbox_inches='tight')
plt.savefig(os.path.join(output_dir, 'fig8_performance_comparison.pdf'), 
            bbox_inches='tight')
plt.close()

# FIGURE 9: Combined Overview (3x3 subplot for comprehensive view)
print("[9/9] Creating Comprehensive Analysis Figure...")
fig9 = plt.figure(figsize=(16, 12))

# Subplot 1: Step Response Comparison
ax1 = plt.subplot(3, 3, 1)
ax1.plot(t_out, y_out, color=color_open, linestyle='--', linewidth=2, 
         label='Open-Loop', alpha=0.8)
ax1.plot(t_cl, y_cl, color=color_closed, linestyle='-', linewidth=2, 
         label='Closed-Loop')
ax1.axhline(y=1.0, color='black', linestyle=':', linewidth=1, alpha=0.5)
ax1.grid(True, alpha=0.3)
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angular Velocity (rad/s)')
ax1.set_title('Step Response Comparison')
ax1.legend(fontsize=8)
ax1.set_xlim([0, 5])

# Subplot 2: Closed-Loop Detailed
ax2 = plt.subplot(3, 3, 2)
ax2.plot(t_cl, y_cl, color=color_closed, linewidth=2)
ax2.axhline(y=1.0, color='black', linestyle=':', linewidth=1, alpha=0.5)
ax2.axhline(y=1.02, color='green', linestyle='--', linewidth=1, alpha=0.4)
ax2.axhline(y=0.98, color='green', linestyle='--', linewidth=1, alpha=0.4)
ax2.fill_between(t_cl, 0.98, 1.02, color='green', alpha=0.1)
ax2.grid(True, alpha=0.3)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angular Velocity (rad/s)')
ax2.set_title('Closed-Loop Response')
ax2.set_xlim([0, 2])

# Subplot 3: Error Signal
ax3 = plt.subplot(3, 3, 3)
error_signal = 1.0 - y_cl
ax3.plot(t_cl, error_signal, color=color_error, linewidth=2)
ax3.axhline(y=0, color='black', linestyle='-', linewidth=1, alpha=0.5)
ax3.fill_between(t_cl, 0, error_signal, color=color_error, alpha=0.2)
ax3.grid(True, alpha=0.3)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Error (rad/s)')
ax3.set_title('Tracking Error')
ax3.set_xlim([0, 2])

# Subplot 4: Bode Magnitude
ax4 = plt.subplot(3, 3, 4)
ax4.semilogx(omega, 20 * np.log10(mag), color=color_closed, linewidth=2)
ax4.axhline(y=0, color='red', linestyle='--', linewidth=1, alpha=0.6)
ax4.grid(True, alpha=0.3, which='both')
ax4.set_xlabel('Frequency (rad/s)')
ax4.set_ylabel('Magnitude (dB)')
ax4.set_title('Bode Magnitude')

# Subplot 5: Bode Phase
ax5 = plt.subplot(3, 3, 5)
ax5.semilogx(omega, phase * 180 / np.pi, color=color_closed, linewidth=2)
ax5.axhline(y=-180, color='red', linestyle='--', linewidth=1, alpha=0.6)
ax5.grid(True, alpha=0.3, which='both')
ax5.set_xlabel('Frequency (rad/s)')
ax5.set_ylabel('Phase (deg)')
ax5.set_title('Bode Phase')

# Subplot 6: Nyquist Plot
ax6 = plt.subplot(3, 3, 6)
nyquist_plot(C_pid * G_motor, omega=np.logspace(-2, 3, 1000), ax=ax6)
ax6.plot(-1, 0, 'rx', markersize=10, markeredgewidth=2.5)
ax6.grid(True, alpha=0.3)
ax6.set_xlabel('Real Axis')
ax6.set_ylabel('Imaginary Axis')
ax6.set_title('Nyquist Plot')
ax6.axis('equal')
ax6.set_xlim([-2, 2])
ax6.set_ylim([-2, 2])

# Subplot 7: Root Locus
ax7 = plt.subplot(3, 3, 7)
try:
    rlocus(C_pid * G_motor, ax=ax7, grid=True, print_gain=False)
except:
    pass
ax7.grid(True, alpha=0.3)
ax7.set_title('Root Locus')
ax7.axhline(y=0, color='black', linewidth=0.8)
ax7.axvline(x=0, color='black', linewidth=0.8)

# Subplot 8: Disturbance Rejection
ax8 = plt.subplot(3, 3, 8)
ax8.plot(t_dist, ref_input, color=color_ref, linestyle='--', 
         linewidth=1.5, label='Reference')
ax8.plot(t_ref, y_total, color=color_closed, linewidth=2, label='Actual')
ax8.axvline(x=5, color='red', linestyle=':', linewidth=1.5, alpha=0.5)
ax8.grid(True, alpha=0.3)
ax8.set_xlabel('Time (s)')
ax8.set_ylabel('Angular Velocity (rad/s)')
ax8.set_title('Disturbance Rejection')
ax8.legend(fontsize=8)
ax8.set_xlim([0, 10])

# Subplot 9: Performance Comparison
ax9 = plt.subplot(3, 3, 9)
x_pos = np.arange(len(metrics))
bars1 = ax9.bar(x_pos - width/2, open_vals, width, label='Open-Loop', 
                color=color_open, alpha=0.8)
bars2 = ax9.bar(x_pos + width/2, closed_vals, width, label='Closed-Loop', 
                color=color_closed, alpha=0.8)
ax9.set_ylabel('Value')
ax9.set_title('Performance Comparison')
ax9.set_xticks(x_pos)
ax9.set_xticklabels(['Rise\nTime', 'Settling\nTime', 'SS\nError'], fontsize=8)
ax9.legend(fontsize=8)
ax9.grid(True, alpha=0.3, axis='y')

plt.suptitle('DC Motor PID Control - Comprehensive Analysis', 
             fontsize=16, fontweight='bold', y=0.995)
plt.tight_layout(rect=[0, 0, 1, 0.99])
plt.savefig(os.path.join(output_dir, 'fig9_comprehensive_analysis.png'), 
            dpi=300, bbox_inches='tight')
plt.savefig(os.path.join(output_dir, 'fig9_comprehensive_analysis.pdf'), 
            bbox_inches='tight')
plt.close()

print("\n" + "=" * 70)
print("SIMULATION COMPLETE!")
print("=" * 70)