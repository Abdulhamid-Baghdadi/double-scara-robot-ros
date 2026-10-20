"""Render a URDF (STL meshes) at given joint values with matplotlib."""
import sys, math, os, json
import xml.etree.ElementTree as ET
import numpy as np
from stl import mesh as stlmesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)))
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]); Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]); Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

def axis_angle(ax, a):
    ax = np.asarray(ax, float); ax /= np.linalg.norm(ax); x, y, z = ax; c, s = math.cos(a), math.sin(a); C = 1 - c
    return np.array([[c+x*x*C, x*y*C-z*s, x*z*C+y*s],[y*x*C+z*s, c+y*y*C, y*z*C-x*s],[z*x*C-y*s, z*y*C+x*s, c+z*z*C]])

def T(R=np.eye(3), t=(0,0,0)):
    M = np.eye(4); M[:3,:3] = R; M[:3,3] = t; return M

def link_transforms(urdf, q):
    root = ET.parse(urdf).getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin"); xyz = [float(v) for v in o.get("xyz").split()]; rpy = [float(v) for v in o.get("rpy").split()]
        ax = j.find("axis"); axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [0,0,1]
        joints[j.find("child").get("link")] = (j.get("name"), j.get("type"), j.find("parent").get("link"), xyz, rpy, axis)
    links = [l.get("name") for l in root.findall("link")]
    out = {}
    def get(l):
        if l in out: return out[l]
        if l not in joints: out[l] = np.eye(4); return out[l]
        name, typ, parent, xyz, rpy, axis = joints[l]
        M = get(parent) @ T(rpy_mat(*rpy), xyz)
        v = q.get(name, 0.0)
        if typ in ("revolute", "continuous"): M = M @ T(axis_angle(axis, v))
        elif typ == "prismatic": M = M @ T(t=np.asarray(axis) * v)
        out[l] = M; return M
    for l in links: get(l)
    return root, out

def render(urdf, meshdir, q, colors, outpath, elev=28, azim=-60, size=(8, 6)):
    root, tf = link_transforms(urdf, q)
    fig = plt.figure(figsize=size, dpi=150); ax = fig.add_subplot(111, projection="3d")
    light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
    allpts = []
    for l in root.findall("link"):
        name = l.get("name"); f = os.path.join(meshdir, name + ".STL")
        if not os.path.exists(f): continue
        m = stlmesh.Mesh.from_file(f); M = tf[name]
        tri = m.vectors.reshape(-1, 3); tri = (M[:3,:3] @ tri.T).T + M[:3,3]; tri = tri.reshape(-1, 3, 3)
        n = np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0]); n /= (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)
        shade = 0.45 + 0.55 * np.abs(n @ light)
        base = np.array(matplotlib.colors.to_rgb(colors.get(name, "#9aa4b1")))
        fc = np.clip(base[None,:] * shade[:,None], 0, 1)
        pc = Poly3DCollection(tri, facecolors=fc, edgecolors="none", linewidths=0); ax.add_collection3d(pc)
        allpts.append(tri.reshape(-1, 3))
    P = np.vstack(allpts); lo, hi = P.min(0), P.max(0); c = (lo+hi)/2; r = (hi-lo).max()/2
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.set_box_aspect((1,1,1)); ax.view_init(elev=elev, azim=azim); ax.set_axis_off()
    plt.subplots_adjust(0,0,1,1); fig.savefig(outpath, transparent=False, facecolor="white", bbox_inches="tight", pad_inches=0.05)
    print("saved", outpath, "bbox", lo, hi)

if __name__ == "__main__":
    urdf, meshdir, out = sys.argv[1], sys.argv[2], sys.argv[3]
    q = json.loads(sys.argv[4]) if len(sys.argv) > 4 else {}
    elev = float(sys.argv[5]) if len(sys.argv) > 5 else 28; azim = float(sys.argv[6]) if len(sys.argv) > 6 else -60
    colors = {"base_link": "#5b6575", "Left_Wing": "#e8833a", "Right_Wing": "#e8833a", "Left_Arm": "#2f80c8", "Right_Arm": "#2f80c8", "Pusher": "#d94b4b"}
    render(urdf, meshdir, q, colors, out, elev, azim)
