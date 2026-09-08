#!/usr/bin/env python3
"""
Flight Log Visualization
Plots the results from integration_test.c simulation
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Read CSV log
print("Loading flight_log.csv...")
df = pd.read_csv('flight_log.csv')

print(f"Loaded {len(df)} samples over {df['time'].max():.1f} seconds")

# Create figure with subplots
fig = plt.figure(figsize=(16, 12))

# ============================================================================
# 1. 3D Trajectory
# ============================================================================
ax1 = fig.add_subplot(3, 3, 1, projection='3d')
ax1.plot(df['pos_n'], df['pos_e'], -df['pos_d'], 'b-', linewidth=2)
ax1.scatter([df['pos_n'].iloc[0]], [df['pos_e'].iloc[0]], [-df['pos_d'].iloc[0]],
           c='g', s=100, marker='o', label='Start')
ax1.scatter([df['pos_n'].iloc[-1]], [df['pos_e'].iloc[-1]], [-df['pos_d'].iloc[-1]],
           c='r', s=100, marker='x', label='End')
ax1.set_xlabel('North (m)')
ax1.set_ylabel('East (m)')
ax1.set_zlabel('Altitude (m)')
ax1.set_title('3D Flight Path')
ax1.legend()
ax1.grid(True)

# ============================================================================
# 2. Altitude vs Time
# ============================================================================
ax2 = fig.add_subplot(3, 3, 2)
ax2.plot(df['time'], -df['pos_d'], 'b-', label='True Altitude', linewidth=2)
ax2.plot(df['time'], df['alt_est'], 'r--', label='Estimated Altitude', linewidth=1)
if 'alt_baro' in df.columns:
    ax2.plot(df['time'], df['alt_baro'], 'g:', label='Barometer', linewidth=1, alpha=0.7)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Altitude (m)')
ax2.set_title('Altitude Over Time')
ax2.legend()
ax2.grid(True)

# ============================================================================
# 3. Horizontal Position (Top-Down View)
# ============================================================================
ax3 = fig.add_subplot(3, 3, 3)
ax3.plot(df['pos_e'], df['pos_n'], 'b-', linewidth=2)
ax3.scatter([df['pos_e'].iloc[0]], [df['pos_n'].iloc[0]],
           c='g', s=100, marker='o', label='Start')
ax3.scatter([df['pos_e'].iloc[-1]], [df['pos_n'].iloc[-1]],
           c='r', s=100, marker='x', label='End')
ax3.set_xlabel('East (m)')
ax3.set_ylabel('North (m)')
ax3.set_title('Horizontal Path (Top View)')
ax3.legend()
ax3.grid(True)
ax3.axis('equal')

# ============================================================================
# 4. Attitude (Roll, Pitch, Yaw)
# ============================================================================
ax4 = fig.add_subplot(3, 3, 4)
ax4.plot(df['time'], df['roll'], 'r-', label='Roll', linewidth=1.5)
ax4.plot(df['time'], df['pitch'], 'g-', label='Pitch', linewidth=1.5)
ax4.plot(df['time'], df['yaw'], 'b-', label='Yaw', linewidth=1.5)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Angle (degrees)')
ax4.set_title('Attitude (Roll/Pitch/Yaw)')
ax4.legend()
ax4.grid(True)

# ============================================================================
# 5. Angular Rates (Gyro)
# ============================================================================
ax5 = fig.add_subplot(3, 3, 5)
ax5.plot(df['time'], np.degrees(df['gyro_x']), 'r-', label='Roll Rate', linewidth=1)
ax5.plot(df['time'], np.degrees(df['gyro_y']), 'g-', label='Pitch Rate', linewidth=1)
ax5.plot(df['time'], np.degrees(df['gyro_z']), 'b-', label='Yaw Rate', linewidth=1)
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Angular Rate (deg/s)')
ax5.set_title('Angular Rates')
ax5.legend()
ax5.grid(True)

# ============================================================================
# 6. Accelerometer
# ============================================================================
ax6 = fig.add_subplot(3, 3, 6)
ax6.plot(df['time'], df['accel_x'], 'r-', label='X', linewidth=1)
ax6.plot(df['time'], df['accel_y'], 'g-', label='Y', linewidth=1)
ax6.plot(df['time'], df['accel_z'], 'b-', label='Z', linewidth=1)
ax6.axhline(y=9.81, color='k', linestyle='--', alpha=0.3, label='Gravity')
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('Acceleration (m/s²)')
ax6.set_title('Accelerometer (Body Frame)')
ax6.legend()
ax6.grid(True)

# ============================================================================
# 7. Motor Commands
# ============================================================================
ax7 = fig.add_subplot(3, 3, 7)
ax7.plot(df['time'], df['m0'], 'r-', label='Motor 0 (FR)', linewidth=1)
ax7.plot(df['time'], df['m1'], 'g-', label='Motor 1 (RR)', linewidth=1)
ax7.plot(df['time'], df['m2'], 'b-', label='Motor 2 (RL)', linewidth=1)
ax7.plot(df['time'], df['m3'], 'm-', label='Motor 3 (FL)', linewidth=1)
ax7.set_xlabel('Time (s)')
ax7.set_ylabel('PWM (µs)')
ax7.set_title('Motor Commands')
ax7.legend()
ax7.grid(True)
ax7.set_ylim([900, 2100])

# ============================================================================
# 8. Velocity
# ============================================================================
ax8 = fig.add_subplot(3, 3, 8)
ax8.plot(df['time'], df['vel_n'], 'r-', label='North', linewidth=1.5)
ax8.plot(df['time'], df['vel_e'], 'g-', label='East', linewidth=1.5)
ax8.plot(df['time'], df['vel_d'], 'b-', label='Down', linewidth=1.5)
# Total speed
speed = np.sqrt(df['vel_n']**2 + df['vel_e']**2 + df['vel_d']**2)
ax8.plot(df['time'], speed, 'k--', label='Total Speed', linewidth=2)
ax8.set_xlabel('Time (s)')
ax8.set_ylabel('Velocity (m/s)')
ax8.set_title('Velocity (NED Frame)')
ax8.legend()
ax8.grid(True)

# ============================================================================
# 9. Flight Mode Timeline
# ============================================================================
ax9 = fig.add_subplot(3, 3, 9)

# Color-code by mode
modes = df['mode'].unique()
mode_colors = {'DISARMED': 'red', 'STABILIZE': 'blue', 'ALT_HOLD': 'green',
               'POS_HOLD': 'cyan', 'MISSION': 'magenta', 'RTL': 'orange'}

for mode in modes:
    mask = df['mode'] == mode
    ax9.scatter(df.loc[mask, 'time'], df.loc[mask, 'armed'],
               c=mode_colors.get(mode, 'gray'), label=mode, s=20, alpha=0.6)

ax9.set_xlabel('Time (s)')
ax9.set_ylabel('Armed (0=No, 1=Yes)')
ax9.set_title('Flight Mode & Armed State')
ax9.set_ylim([-0.1, 1.1])
ax9.legend()
ax9.grid(True)

# ============================================================================
# Final Layout
# ============================================================================
plt.tight_layout()
plt.savefig('flight_analysis.png', dpi=150)
print("\nPlot saved to: flight_analysis.png")

plt.show()

# ============================================================================
# Print Summary Statistics
# ============================================================================
print("\n" + "="*60)
print(" FLIGHT SUMMARY")
print("="*60)
print(f"Total flight time:        {df['time'].max():.1f} seconds")
print(f"Max altitude:             {-df['pos_d'].min():.2f} m")
print(f"Max horizontal distance:  {np.sqrt(df['pos_n']**2 + df['pos_e']**2).max():.2f} m")
print(f"Max speed:                {speed.max():.2f} m/s")
print(f"Max roll angle:           {df['roll'].abs().max():.1f} degrees")
print(f"Max pitch angle:          {df['pitch'].abs().max():.1f} degrees")
print(f"Armed duration:           {df[df['armed']==1]['time'].count() / 10:.1f} seconds")
print("\nModes used:")
for mode in df['mode'].unique():
    duration = df[df['mode']==mode]['time'].count() / 10
    print(f"  {mode:15s} {duration:6.1f}s")
print("="*60)
