"""
Generates high-resolution presentation graphics and animated GIFs for HexaPud:
1. d_loop_trajectory.png (High-res 300 DPI diagram of foot 3D trajectory)
2. d_loop_animation.gif (Smooth 50fps looping GIF of D-loop foot movement)
3. inverse_kinematics_diagram.png (High-res 300 DPI 3-DOF leg geometry diagram)
4. tripod_gait_animation.gif (Smooth looping GIF of 6-leg top-down walking)
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image

OUTPUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ─────────────────────────────────────────────────────────────
# 1. HIGH-RES D-LOOP TRAJECTORY DIAGRAM
# ─────────────────────────────────────────────────────────────
def generate_d_loop_diagram():
    fig, ax = plt.subplots(figsize=(12, 6.75), dpi=300)
    fig.patch.set_facecolor('#0d0f12')
    ax.set_facecolor('#13171f')

    # Data generation
    t_swing = np.linspace(0, 1, 200)
    # Raised cosine s-curve
    s_swing = 0.5 * (1.0 - np.cos(np.pi * t_swing))
    p_swing = -1.0 + 2.0 * s_swing  # -1 to +1
    h_swing = -140.0 + 34.0 * np.sin(np.pi * t_swing)
    disp_swing = p_swing * 23.0  # -23mm to +23mm

    t_stance = np.linspace(0, 1, 200)
    s_stance = 0.5 * (1.0 - np.cos(np.pi * t_stance))
    p_stance = 1.0 - 2.0 * s_stance  # +1 to -1
    h_stance = np.full_like(t_stance, -140.0)
    disp_stance = p_stance * 23.0

    # Ground line & boundary fills
    ax.axhline(-140, color='#38bdf8', linestyle='--', linewidth=1.5, alpha=0.5, label='Ground Plane (Z = -140 mm)')
    ax.fill_between([-35, 35], -140, -155, color='#1e293b', alpha=0.7)

    # Plot Stance (flat ground push)
    ax.plot(disp_stance, h_stance, color='#f59e0b', linewidth=4.5, label='STANCE Phase (Weight-bearing push on ground)')
    # Arrow on stance indicating backward push
    ax.annotate('', xy=(-8, -140), xytext=(8, -140),
                arrowprops=dict(arrowstyle="->", color='#f59e0b', lw=3.5, mutation_scale=20))

    # Plot Swing (arch)
    ax.plot(disp_swing, h_swing, color='#10b981', linewidth=4.5, label='SWING Phase (Aerial parabolic clearance)')
    # Arrow on swing indicating forward step
    ax.annotate('', xy=(8, -106), xytext=(-8, -106),
                arrowprops=dict(arrowstyle="->", color='#10b981', lw=3.5, mutation_scale=20))

    # Highlight Key Waypoints
    waypoints = [
        (-23.0, -140.0, "Point A: Liftoff\nP = -1.0 (-23 mm)\nZ = -140 mm", '#ef4444', (-60, -35)),
        (0.0, -106.0, "Point B: Apex\nP = 0.0 (0 mm)\nZ = -106 mm (+34 mm lift)", '#10b981', (-60, 20)),
        (23.0, -140.0, "Point C: Touchdown\nP = +1.0 (+23 mm)\nZ = -140 mm", '#3b82f6', (15, -35)),
        (0.0, -140.0, "Point D: Stance Center\nP = 0.0 (0 mm)\nZ = -140 mm", '#f59e0b', (-65, -35)),
    ]

    for x, y, label, col, offset in waypoints:
        ax.plot(x, y, marker='o', markersize=10, markerfacecolor=col, markeredgecolor='#ffffff', markeredgewidth=2, zorder=5)
        ax.annotate(label, xy=(x, y), xytext=(x + offset[0]*0.15, y + offset[1]*0.6),
                    fontsize=9, color='#f8fafc', fontweight='bold',
                    bbox=dict(boxstyle="round,pad=0.4", fc='#1e293b', ec=col, lw=1.5, alpha=0.9),
                    arrowprops=dict(arrowstyle="->", color=col, lw=1.5))

    # Stride dimension annotations
    ax.annotate('', xy=(23, -147), xytext=(-23, -147),
                arrowprops=dict(arrowstyle="<->", color='#94a3b8', lw=2))
    ax.text(0, -152, "Total Stride Length = 46.0 mm", color='#94a3b8', fontsize=11, fontweight='bold', ha='center')

    # Height lift dimension
    ax.annotate('', xy=(-28, -106), xytext=(-28, -140),
                arrowprops=dict(arrowstyle="<->", color='#10b981', lw=2))
    ax.text(-30, -123, "H_lift = 34.0 mm", color='#10b981', fontsize=10, fontweight='bold', ha='right', rotation=90)

    # Titles and grid
    ax.set_title("HexaPud Parametric Foot Trajectory: The 3D D-Loop", fontsize=16, fontweight='bold', color='#ffffff', pad=18)
    ax.set_xlabel("Horizontal Stride Displacement (mm) [P × Half-Stride]", fontsize=12, fontweight='bold', color='#cbd5e1', labelpad=10)
    ax.set_ylabel("Vertical Foot Height Z (mm)", fontsize=12, fontweight='bold', color='#cbd5e1', labelpad=10)
    ax.set_xlim(-36, 36)
    ax.set_ylim(-158, -98)
    ax.tick_params(colors='#94a3b8', labelsize=10)
    for spine in ax.spines.values(): spine.set_color('#334155')
    ax.grid(True, linestyle=':', color='#334155', alpha=0.6)
    ax.legend(loc='upper right', facecolor='#1e293b', edgecolor='#475569', labelcolor='#f8fafc', fontsize=10)

    plt.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, "d_loop_trajectory.png")
    plt.savefig(out_path, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.close()
    print("Saved:", out_path)

# ─────────────────────────────────────────────────────────────
# 2. SMOOTH ANIMATED D-LOOP GIF
# ─────────────────────────────────────────────────────────────
def generate_d_loop_gif():
    frames = []
    num_frames = 60

    # Curve points for background
    t_vals = np.linspace(0, 1, 100)
    s_vals = 0.5 * (1.0 - np.cos(np.pi * t_vals))
    sw_x = (-1.0 + 2.0 * s_vals) * 23.0
    sw_z = -140.0 + 34.0 * np.sin(np.pi * t_vals)
    st_x = (1.0 - 2.0 * s_vals) * 23.0
    st_z = np.full_like(st_x, -140.0)

    for i in range(num_frames):
        phi = i / num_frames  # master gait phase 0.0 to 1.0

        fig, ax = plt.subplots(figsize=(8, 5), dpi=100)
        fig.patch.set_facecolor('#0d0f12')
        ax.set_facecolor('#13171f')

        # Static background curves
        ax.plot(sw_x, sw_z, color='#10b981', lw=3, alpha=0.4)
        ax.plot(st_x, st_z, color='#f59e0b', lw=3, alpha=0.4)
        ax.axhline(-140, color='#38bdf8', linestyle='--', lw=1, alpha=0.3)
        ax.fill_between([-35, 35], -140, -155, color='#1e293b', alpha=0.7)

        # Compute current foot position
        if phi < 0.5:
            # Swing Phase
            t = phi / 0.5
            s = 0.5 * (1.0 - np.cos(np.pi * t))
            p = -1.0 + 2.0 * s
            curr_x = p * 23.0
            curr_z = -140.0 + 34.0 * np.sin(np.pi * t)
            phase_name = "SWING PHASE (In Air)"
            phase_color = "#10b981"
        else:
            # Stance Phase
            t = (phi - 0.5) / 0.5
            s = 0.5 * (1.0 - np.cos(np.pi * t))
            p = 1.0 - 2.0 * s
            curr_x = p * 23.0
            curr_z = -140.0
            phase_name = "STANCE PHASE (Ground Push)"
            phase_color = "#f59e0b"

        # Draw foot marker
        ax.plot(curr_x, curr_z, marker='o', markersize=14, markerfacecolor=phase_color, markeredgecolor='#ffffff', markeredgewidth=2.5, zorder=10)

        # Coordinate HUD readout
        hud_text = (f"Phase phi: {phi:.2f} | P: {p:+.2f}\n"
                    f"X_disp: {curr_x:+.1f} mm | Z: {curr_z:.1f} mm\n"
                    f"Status: {phase_name}")
        ax.text(0.03, 0.93, hud_text, transform=ax.transAxes, color='#ffffff', fontsize=10,
                fontfamily='monospace', fontweight='bold', va='top',
                bbox=dict(boxstyle="round,pad=0.5", fc='#1e293b', ec=phase_color, lw=2))

        ax.set_title("HexaPud Dynamic Foot Step Cycle (50Hz Gait Engine)", fontsize=13, fontweight='bold', color='#ffffff', pad=12)
        ax.set_xlabel("Stride Displacement (mm)", fontsize=10, fontweight='bold', color='#cbd5e1')
        ax.set_ylabel("Height Z (mm)", fontsize=10, fontweight='bold', color='#cbd5e1')
        ax.set_xlim(-32, 32)
        ax.set_ylim(-155, -98)
        ax.tick_params(colors='#94a3b8', labelsize=9)
        for spine in ax.spines.values(): spine.set_color('#334155')
        ax.grid(True, linestyle=':', color='#334155', alpha=0.5)

        plt.tight_layout()
        fig.canvas.draw()
        rgba = np.asarray(fig.canvas.buffer_rgba())
        frames.append(Image.fromarray(rgba))
        plt.close(fig)

    out_gif = os.path.join(OUTPUT_DIR, "d_loop_animation.gif")
    frames[0].save(out_gif, save_all=True, append_images=frames[1:], duration=35, loop=0)
    print("Saved:", out_gif)

# ─────────────────────────────────────────────────────────────
# 3. HIGH-RES INVERSE KINEMATICS 3-DOF DIAGRAM
# ─────────────────────────────────────────────────────────────
def generate_ik_diagram():
    fig, ax = plt.subplots(figsize=(12, 7.5), dpi=300)
    fig.patch.set_facecolor('#0d0f12')
    ax.set_facecolor('#13171f')

    # Joint positions in Sagittal 2D Frame
    # Origin = Hip (Femur pivot after Coxa horizontal translation)
    coxa_origin = np.array([-51.0, 0.0]) # Coxa shoulder
    hip = np.array([0.0, 0.0])           # Femur pivot
    
    # Example standing pose: d_horiz = 79mm (130 - 51), Z = -140mm
    d_horiz = 79.0
    z_foot = -140.0
    foot = np.array([d_horiz, z_foot])

    L1 = 51.0   # Coxa
    L2 = 80.0   # Femur
    L3 = 128.0  # Tibia

    # Solve IK for knee joint position
    D = math.sqrt(d_horiz**2 + z_foot**2)
    alpha = math.atan2(-z_foot, d_horiz)
    cos_b = (L2**2 + D**2 - L3**2) / (2.0 * L2 * D)
    beta = math.acos(cos_b)
    theta_femur = alpha - beta

    knee = np.array([L2 * math.cos(theta_femur), -L2 * math.sin(theta_femur)])

    # Draw Links
    # Coxa link (horizontal from shoulder to femur pivot)
    ax.plot([coxa_origin[0], hip[0]], [coxa_origin[1], hip[1]], color='#94a3b8', lw=7, solid_capstyle='round', label=f'Coxa Link L1 ({L1:.0f} mm)')
    # Femur link
    ax.plot([hip[0], knee[0]], [hip[1], knee[1]], color='#38bdf8', lw=8, solid_capstyle='round', label=f'Femur Link L2 ({L2:.0f} mm)')
    # Tibia link
    ax.plot([knee[0], foot[0]], [knee[1], foot[1]], color='#a855f7', lw=8, solid_capstyle='round', label=f'Tibia Link L3 ({L3:.0f} mm)')

    # Draw Reach Line D
    ax.plot([hip[0], foot[0]], [hip[1], foot[1]], color='#f59e0b', linestyle='--', lw=2.5, label='Direct Reach Vector D')

    # Draw Joints
    joints = [
        (coxa_origin, "Coxa Shoulder\n(Chassis Mount)", '#64748b'),
        (hip, "Femur Hip Pivot\n(Origin: d=0, Z=0)", '#38bdf8'),
        (knee, "Tibia Knee Joint\n(Law of Cosines)", '#a855f7'),
        (foot, f"Foot Contact Tip\n(X={130}, Z={z_foot})", '#10b981')
    ]
    for pos, label, col in joints:
        ax.plot(pos[0], pos[1], marker='o', markersize=14, markerfacecolor=col, markeredgecolor='#ffffff', markeredgewidth=2.5, zorder=10)
        offset_y = 12 if pos[1] >= -50 else -25
        offset_x = -35 if pos[0] > 50 else (15 if pos[0] >= 0 else -60)
        ax.annotate(label, xy=(pos[0], pos[1]), xytext=(pos[0] + offset_x, pos[1] + offset_y),
                    fontsize=9.5, color='#ffffff', fontweight='bold',
                    bbox=dict(boxstyle="round,pad=0.4", fc='#1e293b', ec=col, lw=1.5, alpha=0.9))

    # Ground line
    ax.axhline(z_foot, color='#10b981', linestyle=':', lw=2, alpha=0.6)
    ax.text(-40, z_foot - 7, "Floor Ground Plane (Z = -140 mm)", color='#10b981', fontsize=10, fontweight='bold')

    # Mathematical Formula Box
    formula_text = (
        "INVERSE KINEMATICS EQUATIONS:\n"
        "-------------------------------------\n"
        "1. d_horiz = sqrt(X^2 + Y^2) - L_coxa\n"
        "2. D = sqrt(d_horiz^2 + Z^2)\n"
        "3. cos(gamma) = (L2^2 + L3^2 - D^2) / (2 * L2 * L3)\n"
        "   --> theta_tibia = arccos(cos_gamma)\n"
        "4. alpha = atan2(-Z, d_horiz)\n"
        "   cos(beta) = (L2^2 + D^2 - L3^2) / (2 * L2 * D)\n"
        "   --> theta_femur = alpha - beta\n"
        "5. theta_coxa = atan2(Y, X)"
    )
    ax.text(0.03, 0.35, formula_text, transform=ax.transAxes, color='#f8fafc', fontsize=9.5,
            fontfamily='monospace', va='top',
            bbox=dict(boxstyle="round,pad=0.6", fc='#0f172a', ec='#38bdf8', lw=2, alpha=0.95))

    ax.set_title("HexaPud 3-DOF Geometric Inverse Kinematics (Sagittal Plane Derivation)", fontsize=15, fontweight='bold', color='#ffffff', pad=18)
    ax.set_xlabel("Horizontal Distance d_horiz (mm)", fontsize=11, fontweight='bold', color='#cbd5e1')
    ax.set_ylabel("Vertical Coordinate Z (mm)", fontsize=11, fontweight='bold', color='#cbd5e1')
    ax.set_xlim(-65, 110)
    ax.set_ylim(-165, 25)
    ax.tick_params(colors='#94a3b8', labelsize=10)
    for spine in ax.spines.values(): spine.set_color('#334155')
    ax.grid(True, linestyle=':', color='#334155', alpha=0.5)
    ax.legend(loc='upper right', facecolor='#1e293b', edgecolor='#475569', labelcolor='#f8fafc', fontsize=9.5)

    plt.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, "inverse_kinematics_diagram.png")
    plt.savefig(out_path, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.close()
    print("Saved:", out_path)

# ─────────────────────────────────────────────────────────────
# 4. TOP-DOWN HEXAPOD TRIPOD GAIT WALKING ANIMATION GIF
# ─────────────────────────────────────────────────────────────
def generate_tripod_gait_gif():
    frames = []
    num_frames = 60

    # Chassis center & 6 leg mount angles (deg)
    mount_angles = [30.0, 90.0, 150.0, -150.0, -90.0, -30.0]
    r_chassis = 75.0  # mm
    neutral_reach = 110.0 # mm

    for f_idx in range(num_frames):
        phi = f_idx / num_frames  # master phase 0.0 to 1.0

        fig, ax = plt.subplots(figsize=(7, 7), dpi=100)
        fig.patch.set_facecolor('#0d0f12')
        ax.set_facecolor('#13171f')

        # Chassis outline (hexagonal body)
        body_x = [r_chassis * math.cos(math.radians(a)) for a in mount_angles]
        body_y = [r_chassis * math.sin(math.radians(a)) for a in mount_angles]
        body_x.append(body_x[0])
        body_y.append(body_y[0])
        ax.plot(body_y, body_x, color='#475569', lw=3.5) # Y horizontal, X vertical (robot faces UP)
        ax.fill(body_y, body_x, color='#1e293b', alpha=0.8)

        # Center of Mass (CoM) marker
        ax.plot(0, 0, marker='+', markersize=14, color='#ffffff', markeredgewidth=2)
        ax.plot(0, 0, marker='o', markersize=6, markerfacecolor='#38bdf8', markeredgecolor='#ffffff', lw=1.5)

        # Forward Direction Arrow
        ax.annotate('', xy=(0, 110), xytext=(0, 60),
                    arrowprops=dict(arrowstyle="->", color='#38bdf8', lw=3, mutation_scale=18))
        ax.text(0, 120, "FORWARD", color='#38bdf8', fontsize=9, fontweight='bold', ha='center')

        # Calculate foot positions
        feet_pts = []
        is_grounded = []

        for l in range(6):
            is_tripod_a = (l in (0, 2, 4))
            leg_p = phi if is_tripod_a else ((phi + 0.5) % 1.0)
            in_swing = (leg_p < 0.5)
            is_grounded.append(not in_swing)

            # Progress factor P
            if in_swing:
                t = leg_p / 0.5
                s = 0.5 * (1.0 - math.cos(math.pi * t))
                p = -1.0 + 2.0 * s
            else:
                t = (leg_p - 0.5) / 0.5
                s = 0.5 * (1.0 - math.cos(math.pi * t))
                p = 1.0 - 2.0 * s

            m_rad = math.radians(mount_angles[l])
            # Coxa mount coordinate
            cx = r_chassis * math.cos(m_rad)
            cy = r_chassis * math.sin(m_rad)

            # Foot position: neutral + stride offset in world forward (+X)
            # Forward stride amplitude = 22mm
            fx = cx + neutral_reach * math.cos(m_rad) + (p * 22.0)
            fy = cy + neutral_reach * math.sin(m_rad)
            feet_pts.append((fy, fx)) # (plot_x, plot_y)

            # Draw leg link
            leg_color = '#10b981' if in_swing else '#f59e0b'
            leg_style = '--' if in_swing else '-'
            leg_lw = 2.5 if in_swing else 4.0
            ax.plot([cy, fy], [cx, fx], color=leg_color, linestyle=leg_style, lw=leg_lw)

            # Foot dot
            dot_color = '#10b981' if in_swing else '#f59e0b'
            dot_size = 9 if in_swing else 13
            ax.plot(fy, fx, marker='o', markersize=dot_size, markerfacecolor=dot_color, markeredgecolor='#ffffff', markeredgewidth=1.5)
            ax.text(fy * 1.15, fx * 1.05, f"L{l}", color='#94a3b8', fontsize=8, fontweight='bold', ha='center', va='center')

        # Draw Active Support Triangle between the 3 grounded feet
        grounded_indices = [idx for idx, g in enumerate(is_grounded) if g]
        if len(grounded_indices) == 3:
            tri_x = [feet_pts[idx][0] for idx in grounded_indices] + [feet_pts[grounded_indices[0]][0]]
            tri_y = [feet_pts[idx][1] for idx in grounded_indices] + [feet_pts[grounded_indices[0]][1]]
            ax.plot(tri_x, tri_y, color='#f59e0b', lw=2, linestyle=':')
            ax.fill(tri_x, tri_y, color='#f59e0b', alpha=0.15)

        tripod_active = "TRIPOD A GROUNDED (0, 2, 4)" if is_grounded[0] else "TRIPOD B GROUNDED (1, 3, 5)"
        ax.set_title(f"HexaPud Alternating Tripod Gait Simulation\n{tripod_active}", fontsize=11, fontweight='bold', color='#ffffff', pad=10)

        # Legend
        ax.text(0.04, 0.08, "● Yellow = Planted Stance (Pushing)\n● Green = Aerial Swing (Stepping)",
                transform=ax.transAxes, color='#f8fafc', fontsize=8.5, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.4", fc='#0f172a', ec='#475569', lw=1.5))

        ax.set_xlim(-220, 220)
        ax.set_ylim(-210, 230)
        ax.axis('off')

        plt.tight_layout()
        fig.canvas.draw()
        rgba = np.asarray(fig.canvas.buffer_rgba())
        frames.append(Image.fromarray(rgba))
        plt.close(fig)

    out_gif = os.path.join(OUTPUT_DIR, "tripod_gait_animation.gif")
    frames[0].save(out_gif, save_all=True, append_images=frames[1:], duration=40, loop=0)
    print("Saved:", out_gif)

if __name__ == "__main__":
    print("Generating presentation graphics & GIFs...")
    generate_d_loop_diagram()
    generate_d_loop_gif()
    generate_ik_diagram()
    generate_tripod_gait_gif()
    print("All assets successfully generated in:", OUTPUT_DIR)
