"""Workspace map + linkage animation for the five-bar (double SCARA) robot.
Geometry and collision model are taken from the repo's forward_kinematics.cpp
and collision_checker.py."""
import math, sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from shapely import affinity
from shapely.geometry import Point, Polygon

out = sys.argv[1]
L0, L1, L2 = 114.0, 60.0, 60.0          # mm, from forward_kinematics.cpp
M_L, M_R = np.array([0.0, 0.0]), np.array([L0, 0.0])
INK, MUTED, GRID = "#1f2933", "#7b8794", "#e4e7eb"
C_OK, C_BAD, C_ARM, C_FORE, C_TRACE = "#2f80c8", "#e0574f", "#e8833a", "#2f80c8", "#1f2933"

def fk(l, r):
    a = np.array([-L1*math.sin(l), L1*math.cos(l)]); b = np.array([L1*math.sin(r)+L0, L1*math.cos(r)])
    h = np.linalg.norm(b-a)
    if h > 2*L2 or h < 1e-9: return None
    phi = math.atan2(*(b-a)[::-1]); d = math.acos(h/(2*L2))
    c = a + L2*np.array([math.cos(phi+d), math.sin(phi+d)])
    return a, b, c

# collision_checker.py, shifted so the left servo sits at x=0 like the kinematics
def no_collision(l, r):
    xo1, xo2 = -57, 57
    p1 = Polygon([(xo1-12.5, 0), (xo1-12.5, 81), (xo1+12.5, 81), (xo1+12.5, 0)])
    c1 = Point(xo2, 60).buffer(8)
    p = affinity.rotate(p1, l*180/math.pi, Point(xo1, 0)); c = affinity.rotate(c1, -r*180/math.pi, Point(xo2, 0))
    return not p.intersects(c)

def style(ax):
    ax.set_aspect("equal"); ax.set_facecolor("white")
    for s in ax.spines.values(): s.set_visible(False)
    ax.tick_params(colors=MUTED, labelsize=8); ax.grid(color=GRID, lw=0.6); ax.set_axisbelow(True)
    ax.set_xlabel("x [mm]", color=MUTED, fontsize=9); ax.set_ylabel("y [mm]", color=MUTED, fontsize=9)

def draw_linkage(ax, a, b, c, alpha=1.0, lw=4):
    ax.plot([M_L[0], a[0]], [M_L[1], a[1]], color=C_ARM, lw=lw, solid_capstyle="round", alpha=alpha)
    ax.plot([M_R[0], b[0]], [M_R[1], b[1]], color=C_ARM, lw=lw, solid_capstyle="round", alpha=alpha)
    ax.plot([a[0], c[0], b[0]], [a[1], c[1], b[1]], color=C_FORE, lw=lw, solid_capstyle="round", alpha=alpha)
    ax.plot([M_L[0], M_R[0]], [0, 0], color=INK, lw=2.5, alpha=alpha)
    ax.scatter([M_L[0], M_R[0]], [0, 0], s=110, marker="s", color=INK, zorder=5, alpha=alpha)
    ax.scatter([a[0], b[0]], [a[1], b[1]], s=28, color="white", edgecolor=INK, zorder=6, alpha=alpha)
    ax.scatter([c[0]], [c[1]], s=60, color=C_BAD, edgecolor="white", zorder=7, alpha=alpha)

# ---------- workspace map ----------
ang = np.radians(np.arange(-180, 180, 1.5))
ok, bad = [], []
for l in ang:
    for r in ang:
        s = fk(l, r)
        if s is None: continue
        (ok if no_collision(l, r) else bad).append(s[2])
ok, bad = np.array(ok), np.array(bad)
fig, ax = plt.subplots(figsize=(7.5, 5.6), dpi=150)
style(ax)
ax.scatter(bad[:, 0], bad[:, 1], s=1.2, color=C_BAD, alpha=0.35, lw=0, label="reachable, rejected by collision checker")
ax.scatter(ok[:, 0], ok[:, 1], s=1.2, color=C_OK, alpha=0.35, lw=0, label="reachable and collision-free")
s = fk(-0.5, -0.2); draw_linkage(ax, *s)
ax.annotate("left servo", M_L, xytext=(12, -50), textcoords="offset points", color=INK, fontsize=8, bbox=dict(fc="white", ec="none", pad=1.5), arrowprops=dict(arrowstyle="-", color=MUTED))
ax.annotate("right servo", M_R, xytext=(-70, -50), textcoords="offset points", color=INK, fontsize=8, bbox=dict(fc="white", ec="none", pad=1.5), arrowprops=dict(arrowstyle="-", color=MUTED))
ax.annotate("end effector", s[2], xytext=(45, -5), textcoords="offset points", color=INK, fontsize=8, arrowprops=dict(arrowstyle="-", color=MUTED))
lg = ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.2), ncol=2, frameon=False, fontsize=8, markerscale=8)
ax.set_title("End-effector positions over a full sweep of both servo angles", color=INK, fontsize=10, pad=14)
fig.savefig(os.path.join(out, "workspace.png"), bbox_inches="tight", facecolor="white"); plt.close(fig)
print("workspace", len(ok), len(bad))

# ---------- animation: end effector follows a figure-eight via inverse kinematics ----------
def circ_int(p0, r0, p1, r1):
    d = np.linalg.norm(p1-p0); a = (r0*r0 - r1*r1 + d*d)/(2*d); h = math.sqrt(max(r0*r0 - a*a, 0))
    m = p0 + a*(p1-p0)/d; perp = np.array([-(p1-p0)[1], (p1-p0)[0]])/d
    return m + h*perp, m - h*perp
def ik(c):
    a = min(circ_int(M_L, L1, c, L2), key=lambda q: q[0]); b = max(circ_int(M_R, L1, c, L2), key=lambda q: q[0])
    return math.atan2(-a[0], a[1]), math.atan2(b[0]-L0, b[1])
N = 120
t = np.linspace(0, 2*np.pi, N, endpoint=False)
path = np.c_[57 + 30*np.sin(t), 72 + 14*np.sin(2*t)]
LR = [ik(c) for c in path]
frames = [fk(l, r) for l, r in LR]
assert all(f is not None and np.allclose(f[2], c, atol=1e-6) for f, c in zip(frames, path))
trace = path
fig, ax = plt.subplots(figsize=(5.6, 4.0), dpi=100)
def draw(i):
    ax.clear(); style(ax); ax.set_xlim(-45, 160); ax.set_ylim(-12, 112)
    ax.set_xlabel(""); ax.set_ylabel("")
    ax.plot(np.r_[trace[:, 0], trace[0, 0]], np.r_[trace[:, 1], trace[0, 1]], color=GRID, lw=2.5)
    ax.plot(trace[:i+1, 0], trace[:i+1, 1], color=C_TRACE, lw=1.6)
    draw_linkage(ax, *frames[i])
    ax.text(-42, 108, f"θ_left  = {math.degrees(LR[i][0]):6.1f}°\nθ_right = {math.degrees(LR[i][1]):6.1f}°", fontsize=8, color=INK, va="top", family="monospace")
    ax.text(157, 108, f"x = {trace[i,0]:5.1f} mm\ny = {trace[i,1]:5.1f} mm", fontsize=8, color=INK, va="top", ha="right", family="monospace")
    fig.subplots_adjust(0.07, 0.07, 0.98, 0.98)
anim = FuncAnimation(fig, draw, frames=N)
anim.save(os.path.join(out, "linkage.gif"), writer=PillowWriter(fps=30))
print("gif ok")
