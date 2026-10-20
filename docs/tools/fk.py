import math, sys, json
L0, L1, L2 = 0.114, 0.06, 0.06
def fk(l, r):
    xa, ya = -L1*math.sin(l), L1*math.cos(l); xb, yb = L1*math.sin(r)+L0, L1*math.cos(r)
    h = math.hypot(xb-xa, yb-ya); phi = math.atan2(yb-ya, xb-xa); d = math.acos(h/(2*L2))
    xc, yc = xa+L2*math.cos(phi+d), ya+L2*math.sin(phi+d)
    l04, l14 = math.hypot(xc, yc), math.hypot(L0-xc, yc)
    t3 = math.pi - math.acos((L1*L1+L2*L2-l04*l04)/(2*L1*L2)); t4 = math.pi - math.acos((L1*L1+L2*L2-l14*l14)/(2*L1*L2))
    return xc, yc, t3, t4
l, r = float(sys.argv[1]), float(sys.argv[2]); xc, yc, t3, t4 = fk(l, r)
print(json.dumps({"Left_Shoulder": l, "Right_Shoulder": r, "Left_Elbow": t3, "Right_Elbow": t4, "Pusher_Linear_Joint": 0.005}), xc, yc, file=sys.stderr)
print(json.dumps({"Left_Shoulder": l, "Right_Shoulder": r, "Left_Elbow": t3, "Right_Elbow": t4, "Pusher_Linear_Joint": 0.005}))
