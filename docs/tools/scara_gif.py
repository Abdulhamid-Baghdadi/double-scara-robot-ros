"""3D GIF of the double SCARA: the end effector traces a figure-eight (inverse kinematics), the pusher taps,
servos coloured as motors. Joint angles go into the URDF exactly as robot_publish.cpp maps them."""
import sys, os, io, math
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image
from stl import mesh as stlmesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_urdf import link_transforms

pkg, out = sys.argv[1], sys.argv[2]
elev, azim = float(sys.argv[3]), float(sys.argv[4])
urdf = os.path.join(pkg, "urdf", "robot_files.urdf")
L0, L1, L2 = 0.114, 0.06, 0.06

FRAME, SERVO, HORN, ARM, FORE, PUSH = "#c3c9d2", "#3a3f48", "#e8833a", "#e8833a", "#2f80c8", "#d94b4b"
def base_colors(tri):
    c = tri.mean(1)
    servo = (c[:, 2] > 0.004) & (np.minimum(abs(c[:, 0] + 0.057), abs(c[:, 0] - 0.057)) < 0.03)
    col = np.tile(matplotlib.colors.to_rgb(FRAME), (len(tri), 1))
    col[servo] = matplotlib.colors.to_rgb(SERVO)
    return col
link_col = {"Left_Wing": ARM, "Right_Wing": ARM, "Left_Arm": FORE, "Right_Arm": FORE, "Pusher": PUSH}
meshes = {}
for name in ["base_link", "Left_Wing", "Right_Wing", "Left_Arm", "Right_Arm", "Pusher"]:
    v = stlmesh.Mesh.from_file(os.path.join(pkg, "meshes", name + ".STL")).vectors
    meshes[name] = v

def circ_int(p0, r0, p1, r1):
    d = np.linalg.norm(p1-p0); a = (r0*r0 - r1*r1 + d*d)/(2*d); h = math.sqrt(max(r0*r0 - a*a, 0))
    m = p0 + a*(p1-p0)/d; perp = np.array([-(p1-p0)[1], (p1-p0)[0]])/d
    return m + h*perp, m - h*perp
def ik(c):
    a = min(circ_int(np.zeros(2), L1, c, L2), key=lambda q: q[0]); b = max(circ_int(np.array([L0, 0]), L1, c, L2), key=lambda q: q[0])
    return math.atan2(-a[0], a[1]), math.atan2(b[0]-L0, b[1])
def elbows(l, r):  # forward_kinematics.cpp: passive elbow angles theta3, theta4
    xa, ya = -L1*math.sin(l), L1*math.cos(l); xb, yb = L1*math.sin(r)+L0, L1*math.cos(r)
    h = math.hypot(xb-xa, yb-ya); phi = math.atan2(yb-ya, xb-xa); d = math.acos(h/(2*L2))
    xc, yc = xa+L2*math.cos(phi+d), ya+L2*math.sin(phi+d)
    l04, l14 = math.hypot(xc, yc), math.hypot(L0-xc, yc)
    t3 = math.pi - math.acos((L1*L1+L2*L2-l04*l04)/(2*L1*L2)); t4 = math.pi - math.acos((L1*L1+L2*L2-l14*l14)/(2*L1*L2))
    return t3, t4

light = np.array([0.35, -0.5, 0.8]); light /= np.linalg.norm(light)
def render(q, view=None, size=(5.6, 4.2)):
    _, tf = link_transforms(urdf, q)
    fig = plt.figure(figsize=size, dpi=100); ax = fig.add_subplot(111, projection="3d"); pts = []
    for name, v in meshes.items():
        M = tf[name]; tri = (M[:3,:3] @ v.reshape(-1,3).T).T + M[:3,3]; tri = tri.reshape(-1,3,3); pts.append(tri.reshape(-1,3))
        n = np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0]); n /= np.linalg.norm(n, axis=1, keepdims=True)+1e-12
        base = base_colors(tri) if name == "base_link" else np.tile(matplotlib.colors.to_rgb(link_col[name]), (len(tri), 1))
        ax.add_collection3d(Poly3DCollection(tri, facecolors=np.clip(base*(0.45+0.55*np.abs(n@light))[:,None],0,1), edgecolors="none"))
    if view is None:
        P = np.vstack(pts); lo, hi = P.min(0), P.max(0); view = ((lo+hi)/2, (hi-lo).max()/2)
    c, r = view
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.set_box_aspect((1,1,1)); ax.view_init(elev=elev, azim=azim); ax.set_axis_off()
    fig.patch.set_alpha(0); ax.patch.set_alpha(0); plt.subplots_adjust(-0.15,-0.15,1.15,1.15)
    buf = io.BytesIO(); fig.savefig(buf, transparent=True); plt.close(fig)
    return Image.open(buf).convert("RGBA"), view

N = int(sys.argv[5]) if len(sys.argv) > 5 else 36
frames, view = [], None
for i in range(N):
    t = 2*math.pi*i/N
    c = np.array([0.057 + 0.030*math.sin(t), 0.072 + 0.014*math.sin(2*t)])
    l, r = ik(c); t3, t4 = elbows(l, r)
    q = {"Left_Shoulder": l, "Right_Shoulder": r, "Left_Elbow": t3, "Right_Elbow": t4,
         "Pusher_Linear_Joint": 0.01*max(0.0, math.sin(4*t))}
    im, view = render(q, view); frames.append(im)
bb = None
for f in frames:
    b = f.getchannel("A").getbbox(); bb = b if bb is None else (min(bb[0],b[0]),min(bb[1],b[1]),max(bb[2],b[2]),max(bb[3],b[3]))
frames = [f.crop((bb[0]-6, bb[1]-6, bb[2]+6, bb[3]+6)) for f in frames]
if N == 1:
    bg = Image.new("RGBA", frames[0].size, (13,17,23,255)); bg.alpha_composite(frames[0]); bg.save(out); sys.exit()
gif = []
for f in frames:
    p = f.convert("RGB").quantize(colors=255, method=Image.Quantize.MEDIANCUT)
    p.paste(255, mask=f.getchannel("A").point(lambda v: 255 if v < 128 else 0)); gif.append(p)
gif[0].save(out, save_all=True, append_images=gif[1:], duration=70, loop=0, transparency=255, disposal=2)
print(out, gif[0].size, len(gif))
