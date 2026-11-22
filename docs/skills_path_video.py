"""Renders the skills path as an MP4 (top-down field, robot, path trace, step list, timer)."""
import math, subprocess, sys
from PIL import Image, ImageDraw, ImageFont

OUT = sys.argv[1]
W, H, FPS = 1280, 720, 30
S = 4.8                      # px per inch
CX, CY = 30 + 71.4 * S, 360  # field center on screen
TILE = 23.8
WALL = 3 * TILE

F = "/usr/share/fonts/opentype/inter/Inter-%s.otf"
f_s = ImageFont.truetype(F % "Regular", 15)
f_m = ImageFont.truetype(F % "SemiBold", 17)
f_b = ImageFont.truetype(F % "Bold", 26)
f_t = ImageFont.truetype(F % "Bold", 40)
f_xs = ImageFont.truetype(F % "Medium", 12)

def P(x, y): return (CX + x * S, CY - y * S)

# ---- v6: curved, chained motion; 10 pts per owned yellow pin half showing; +8 park ----
T = TILE
LD = (-56.5, -59.2)
GOALS = {"LS": (-2 * T, T), "G2": (-2 * T, -T), "G3": (-T, -2 * T), "BR": (T, -2 * T), "TALL": (0, 0)}
# (x, y, heading, act, text, extra). act: None = drive through without stopping
REACH = 11.15  # robot center to the claw target (body 7.25 + goal radius 3.5 + 0.4)
def back(target, h):  # robot center for backing onto `target` with heading h (back faces the target)
    tx, ty = target
    return (round(tx + REACH * math.sin(math.radians(h)), 1), round(ty + REACH * math.cos(math.radians(h)), 1))
STACKS = {"FS": (-T, -T), "CS": (-2 * T, -2 * T), "A": (-67.6, 22.1), "D": (21.5, -66.8), "B": (-21.5, -66.8), "C": (-67.6, -22.1)}
# clusters: a cup in the middle, 4 bare pins around it (5" out)
CL = {"KL": (-T, T), "KB": (T, -T)}
CPINS = {}
for kname, (kx, ky) in CL.items():
    for side, (ox, oy) in {"W": (-5, 0), "E": (5, 0), "S": (0, -5), "N": (0, 5)}.items():
        CPINS[kname + side] = (kx + ox, ky + oy)
PINS = {}
def N(target, h, act, text, extra=None):
    tgt = GOALS.get(target) or STACKS.get(target) or CPINS.get(target)
    x_, y_ = back(tgt, h); return (x_, y_, h, act, text, extra if extra else target)
R = [
 (-60, 0, 180, "start", "Start at the wall toggle, preload (bare pin) in the claw", None),
 (-60, 0, 180, "arm", "Small lift: wall toggle flips", None),
 N("LS", 180, "score", "Curve back onto the left short goal: PRELOAD (1st)"),
 (-52.5, -0.5, 150, "stop", "Pull forward off the goal", None),
 N("A", 135, "grab", "Curve back onto WALL STACK A, grab"),
 (-52, 2, 170, "stop", "Forward off the wall", None),
 N("LS", 180, "score", "Back onto the left short goal: wall stack A (2nd)"),
 (-30, -2, 180, "stop", "Forward, under the cluster", None),
 N("KLS", 180, "pick", "Back onto the CLUSTER's bottom pin (bare)"),
 (-40, 2, 270, None, "Forward, curving left", None),
 (-47.6, 4, 0, "stop", "Above Red Goal 2", None),
 N("G2", 0, "score", "Back onto Red Goal 2: cluster pin (1st, bare)"),
 (-48, -4, 40, "stop", "Pull forward off the goal", None),
 N("C", 55, "grab", "Back onto WALL STACK C, grab"),
 (-48, -4, 40, "stop", "Forward off the wall", None),
 N("G2", 0, "score", "Back onto Red Goal 2: wall stack C (2nd)"),
 (-23.8, 0, 0, "stop", "Curve up and over, above the field stack", None),
 N("FS", 0, "grab", "Back SLOWLY onto the FIELD STACK, grab"),
 N("G2", 90, "score", "Curve back onto Red Goal 2: field stack (3rd)"),
 (-24, -30, 135, "stop", "Pull forward off the goal", None),
 N("CS", 45, "grab", "Curve back onto the CORNER STACK, grab"),
 N("G2", 180, "score", "Back onto Red Goal 2: corner stack (4th)"),
 (-47.6, -42, 180, "stop", "Pull forward off the goal", None),
 (LD[0], LD[1], 90, "load", "Back onto the loader: LOAD 1 (bare pin)", None),
 N("G3", 270, "score", "Turn, back onto Red Goal 3: load 1 (1st, bare)"),
]
NLOADS = 3
for n in range(2, 2 + NLOADS):
    R.append((-42, -47.6, 270, "stop", "Pull forward off the goal", None) if R[-1][5] == "G3" else (-47.6, -42, 180, "stop", "Pull forward off the goal", None))
    R.append((LD[0], LD[1], 90, "load", f"Back onto the loader: LOAD {n} (cup + pin)", None))
    if n % 2 == 0: R.append(N("G2", 180, "score", f"Turn, back up onto Red Goal 2: load {n}"))
    else: R.append(N("G3", 270, "score", f"Turn, back onto Red Goal 3: load {n}"))
R += [
 (-24, -31, 100, None, "Curve right, above Red Goal 3", None),
 (0, -57.5, 180, "arm", "Small lift near the wall: bottom toggle flips", None),
 N("KBW", 270, "pick", "Back onto the CLUSTER's left pin (bare)"),
 (3, -33, 200, "stop", "Forward out of the cluster", None),
 N("BR", 270, "score", "Back onto the bottom-right short goal: cluster pin (1st)"),
 (2, -45, 270, "stop", "Pull forward off the goal", None),
 N("D", 325, "grab", "Back onto WALL STACK D, grab"),
 N("BR", 270, "score", "Back onto the bottom-right short goal: wall stack D (2nd)"),
 (2, -40, 300, "stop", "Pull forward off the goal", None),
 N("KBN", 270, "pick", "Curve back onto the CLUSTER's top pin (bare)"),
 N("TALL", 215, "tall", "Curve back onto the TALL GOAL: cluster pin, and PARK"),
]
# a "pull forward off the goal" stop goes 5" straight ahead of where the robot scored
for _i in range(1, len(R)):
    if R[_i][3] == "stop" and R[_i][4] == "Pull forward off the goal" and R[_i - 1][3] in ("score", "tall"):
        _x, _y, _h = R[_i - 1][:3]
        R[_i] = (round(_x + 6.5 * math.sin(math.radians(_h)), 1), round(_y + 6.5 * math.cos(math.radians(_h)), 1), _h) + R[_i][3:]
GRABS = [i for i, r in enumerate(R) if r[3] == "grab"]
PICKS = [i for i, r in enumerate(R) if r[3] == "pick"]

def points_after(steps_done):
    goals = {}
    for k in sorted(steps_done):
        if R[k][3] in ("score", "tall"): goals.setdefault(R[k][5], []).append(1)
    return 10 * sum(len(v) for v in goals.values()), goals

VMAX, ACC, TURN = 62.0, 90.0, 360.0
def wrap(a): return (a + 180) % 360 - 180
def hv(h): return (math.sin(math.radians(h)), math.cos(math.radians(h)))
def ang(vx, vy): return math.degrees(math.atan2(vx, vy))

def act_time(i):
    a = R[i][3]
    if a == "score":
        on = sum(1 for k in range(i) if R[k][3] == "score" and R[k][5] == R[i][5])
        return 0.45 + 0.08 * on
    return {"tall": 0.6, "pick": 0.65, "grab": 0.85, "load": 0.5, "arm": 0.35, "stop": 0.05, "start": 0.4}.get(a, 0)

def bez(p0, p1, p2, p3, u):
    a = (1 - u) ** 3; b = 3 * (1 - u) ** 2 * u; c = 3 * (1 - u) * u * u; d = u ** 3
    return (a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0], a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1])

segs = []
x, y, h = R[0][0], R[0][1], R[0][2]
segs.append((act_time(0), lambda u, x=x, y=y, h=h: (x, y, h), 0, "act"))
i = 1
while i < len(R):
    # a run: consecutive nodes up to (and including) the next stopping node
    run = []
    j = i
    while True:
        run.append(j)
        if R[j][3] is not None or j == len(R) - 1: break
        j += 1
    curves = []  # (index, polyline [(x,y,h)])
    cx, cy, ch = x, y, h
    first = True
    for k in run:
        nx, ny, nh = R[k][:3]
        chord = (nx - cx, ny - cy)
        L = math.hypot(*chord)
        if L < 0.3:
            if abs(wrap(nh - ch)) > 1:
                segs.append((abs(wrap(nh - ch)) / TURN + 0.05, lambda u, x=cx, y=cy, h=ch, dh=wrap(nh - ch): (x, y, h + dh * u), k, "turn"))
            ch = nh
            continue
        ev = hv(nh)
        s = 1 if ev[0] * chord[0] + ev[1] * chord[1] >= 0 else -1
        sv = hv(ch)
        st = (s * sv[0], s * sv[1])
        off = math.degrees(math.acos(max(-1, min(1, (st[0] * chord[0] + st[1] * chord[1]) / L))))
        if off > 75:  # can't curve from here: turn in place toward the chord first
            want = ang(*chord) if s > 0 else ang(*chord) + 180
            dh = wrap(want - ch)
            segs.append((abs(dh) / TURN + 0.05, lambda u, x=cx, y=cy, h=ch, dh=dh: (x, y, h + dh * u), k, "turn"))
            ch = ch + dh; sv = hv(ch); st = (s * sv[0], s * sv[1])
        kk = 0.42 * L
        p0 = (cx, cy); p1 = (cx + st[0] * kk, cy + st[1] * kk)
        p3 = (nx, ny); p2 = (nx - s * ev[0] * kk, ny - s * ev[1] * kk)
        pts = []
        for m in range(81):
            u = m / 80
            px, py = bez(p0, p1, p2, p3, u)
            pts.append((px, py))
        poly = []
        for m, (px, py) in enumerate(pts):
            if m < len(pts) - 1: tx, ty = pts[m + 1][0] - px, pts[m + 1][1] - py
            else: tx, ty = px - pts[m - 1][0], py - pts[m - 1][1]
            hh = ang(tx, ty) if s > 0 else ang(tx, ty) + 180
            poly.append((px, py, hh))
        curves.append((k, poly))
        cx, cy, ch = nx, ny, nh
        first = False
    # time along the whole run (trapezoid), split per curve
    lens = []
    for k, poly in curves:
        lens.append(sum(math.hypot(poly[m + 1][0] - poly[m][0], poly[m + 1][1] - poly[m][1]) for m in range(len(poly) - 1)))
    Ltot = sum(lens)
    if Ltot > 0:
        da = VMAX * VMAX / ACC
        if Ltot >= da: Ttot = Ltot / VMAX + VMAX / ACC
        else: Ttot = 2 * math.sqrt(Ltot / ACC)
        vpk = min(VMAX, math.sqrt(Ltot * ACC))
        ta = vpk / ACC
        def s_at(t, Ltot=Ltot, Ttot=Ttot, vpk=vpk, ta=ta):
            if t <= ta: return 0.5 * ACC * t * t
            if t >= Ttot - ta: r = Ttot - t; return Ltot - 0.5 * ACC * r * r
            return 0.5 * ACC * ta * ta + vpk * (t - ta)
        def t_at(sv, Ttot=Ttot):
            lo, hi = 0.0, Ttot
            for _ in range(40):
                mid = (lo + hi) / 2
                if s_at(mid) < sv: lo = mid
                else: hi = mid
            return lo
        s0 = 0
        for (k, poly), Lc in zip(curves, lens):
            t0, t1 = t_at(s0), t_at(s0 + Lc)
            cum = [0]
            for m in range(len(poly) - 1): cum.append(cum[-1] + math.hypot(poly[m + 1][0] - poly[m][0], poly[m + 1][1] - poly[m][1]))
            def fn(u, poly=poly, cum=cum, t0=t0, t1=t1, s0=s0):
                sv = s_at(t0 + u * (t1 - t0)) - s0
                m = 0
                while m < len(cum) - 2 and cum[m + 1] < sv: m += 1
                f = 0 if cum[m + 1] == cum[m] else max(0, min(1, (sv - cum[m]) / (cum[m + 1] - cum[m])))
                a, b = poly[m], poly[m + 1]
                dh = wrap(b[2] - a[2])
                return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + dh * f)
            segs.append((max(0.01, t1 - t0), fn, k, "drive"))
            s0 += Lc
    x, y, h = R[run[-1]][:3]
    last = run[-1]
    if act_time(last) > 0:
        segs.append((act_time(last), lambda u, x=x, y=y, h=h: (x, y, h), last, "act"))
    i = run[-1] + 1
TOTAL = sum(sd[0] for sd in segs)

# ---- crash check: robot body (13 x 14.5) against goals, pins, stacks, clusters, walls ----
def crash_check():
    obst = [(gx, gy, 4.5 if g == "TALL" else 3.5, g) for g, (gx, gy) in GOALS.items()]
    obst += [(2 * T, T, 3.5, "blue goal"), (T, 2 * T, 3.5, "blue goal"), (-T, 2 * T, 3.5, "top short"), (2 * T, -T, 3.5, "right short")]
    for cx_, cy_ in [(-2 * T, 2 * T), (2 * T, -2 * T)]: obst.append((cx_, cy_, 6.8, "pin cluster"))
    for n_, (cx_, cy_) in CL.items(): obst.append((cx_, cy_, 2.8, "cluster cup"))
    for px, py in [(T, T), (2 * T, 2 * T), (67.6, 22.1), (67.6, -22.1), (-21.5, 66.8), (21.5, 66.8)]: obst.append((px, py, 1.8, "pin"))
    gone = set(); worst = []
    t = 0; problems = []
    for dur, fn, k, ph in segs:
        tgt = R[k][5] if R[k][3] in ("score", "tall", "pick", "grab") else None
        live = list(obst) + [(px, py, 1.8, n) for n, (px, py) in CPINS.items() if n not in gone] + [(px, py, 1.8, n) for n, (px, py) in PINS.items() if n not in gone] + [(px, py, 2.8, n) for n, (px, py) in STACKS.items() if n not in gone]
        for m in range(0, 21):
            xx, yy, hh = fn(m / 20)
            c, s_ = math.cos(math.radians(hh)), math.sin(math.radians(hh))
            for ox, oy, r, name in live:
                if name == tgt: continue
                # obstacle in robot frame (x right, y forward)
                dx, dy = ox - xx, oy - yy
                rx, ry = dx * c - dy * s_, dx * s_ + dy * c
                qx, qy = max(abs(rx) - 6.5, 0), max(abs(ry) - 7.25, 0)
                clear = math.hypot(qx, qy) - r
                if clear < 0: problems.append((round(t, 1), R[k][4], name, round(clear, 1)))
            for cx_ in (-6.5, 6.5):
                for cy_ in (-7.25, 7.25):
                    wx = xx + cx_ * c + cy_ * s_; wy = yy - cx_ * s_ + cy_ * c
                    if abs(wx) > WALL + 0.01 or abs(wy) > WALL + 0.01:
                        if R[k][3] != "load": problems.append((round(t, 1), R[k][4], "wall", round(WALL - max(abs(wx), abs(wy)), 1)))
        if ph == "act" and R[k][3] in ("grab", "pick"): gone.add(R[k][5])
        t += dur
    seen = set(); out = []
    for p in problems:
        if (p[1], p[2]) not in seen: seen.add((p[1], p[2])); out.append(p)
    return out

HOLD = 3.0

# ---- static field layer ----
def field_layer():
    im = Image.new("RGB", (W, H), (245, 246, 248))
    d = ImageDraw.Draw(im)
    x0, y0 = P(-WALL, WALL); x1, y1 = P(WALL, -WALL)
    d.rectangle([x0, y0, x1, y1], fill=(206, 210, 216))
    for k in range(1, 6):
        a = -WALL + k * TILE
        d.line([P(a, WALL), P(a, -WALL)], fill=(186, 191, 198), width=1)
        d.line([P(-WALL, a), P(WALL, a)], fill=(186, 191, 198), width=1)
    d.rectangle([x0, y0, x1, y1], outline=(40, 44, 52), width=5)
    # midfield diamond
    pts = [P(-TILE, 0), P(0, TILE), P(TILE, 0), P(0, -TILE), P(-TILE, 0)]
    for a, b in zip(pts, pts[1:]):
        d.line([a, b], fill=(120, 126, 136), width=2)
    def circ(px, py, r, fill, outline=(30, 30, 30), w=2):
        X, Y = P(px, py); d.ellipse([X - r * S, Y - r * S, X + r * S, Y + r * S], fill=fill, outline=outline, width=w)
    def label(px, py, t, dy=0):
        X, Y = P(px, py); d.text((X, Y + dy), t, font=f_xs, fill=(50, 54, 62), anchor="mm")
    circ(0, 0, 4.5, (90, 96, 108)); label(0, 0, "", 0)
    X, Y = P(0, 0); d.text((X, Y - 30), "TALL", font=f_xs, fill=(30, 30, 30), anchor="mm")
    for gx, gy in [(-TILE, 2 * TILE), (-2 * TILE, TILE), (2 * TILE, -TILE), (TILE, -2 * TILE)]:
        circ(gx, gy, 3.5, (150, 156, 166)); label(gx, gy, "short", 26)
    for gx, gy in [(-2 * TILE, -TILE), (-TILE, -2 * TILE)]:
        circ(gx, gy, 3.5, (214, 62, 62))
    for gx, gy in [(2 * TILE, TILE), (TILE, 2 * TILE)]:
        circ(gx, gy, 3.5, (52, 110, 214))
    label(-2 * TILE, -TILE, "G2", 26); label(-TILE, -2 * TILE, "G3", 26)
    for cx_, cy_ in [(-2 * TILE, 2 * TILE), (-TILE, TILE), (TILE, -TILE), (2 * TILE, -2 * TILE)]:
        for ox, oy in [(0, 0), (5, 0), (-5, 0), (0, 5), (0, -5)]:
            circ(cx_ + ox, cy_ + oy, 1.6, (240, 196, 40), w=1)
    for px, py in [(2 * TILE, 2 * TILE), (TILE, TILE)]:
        circ(px, py, 1.8, (240, 196, 40), w=1)
    # bottom-left match loader
    X0, Y0 = P(-WALL, -55); X1, Y1 = P(-WALL + 4.5, -64.5)
    d.rectangle([X0, Y0, X1, Y1], fill=(214, 62, 62), outline=(30, 30, 30), width=2)
    X, Y = P(-WALL + 2, -68); d.text((X + 4, Y + 4), "loader", font=f_xs, fill=(50, 54, 62), anchor="lm")
    # yellow wall pins
    for px, py in [(67.6, 22.1), (67.6, -22.1), (-21.5, 66.8), (21.5, 66.8)]:  # wall stacks (cup + pin)
        circ(px, py, 2.5, (240, 196, 40), w=2); circ(px, py, 1.0, (255, 255, 255), w=1)
    # (our two field stacks are drawn per frame: they disappear when grabbed)
    for px, py in [(T, T), (2 * T, 2 * T)]:
        pass
    # toggles
    for tx, ty, vert in [(-WALL, 0, True), (WALL, 0, True), (0, WALL, False), (0, -WALL, False)]:
        X, Y = P(tx, ty)
        if vert: d.rectangle([X - 4, Y - 18, X + 4, Y + 18], fill=(60, 60, 60))
        else: d.rectangle([X - 18, Y - 4, X + 18, Y + 4], fill=(60, 60, 60))
    # side panel
    d.rectangle([760, 0, W, H], fill=(255, 255, 255))
    d.line([760, 0, 760, H], fill=(210, 214, 220), width=2)
    d.text((790, 30), "19109M  ·  Skills path v8 (draft)", font=f_m, fill=(110, 116, 126))
    d.text((790, 56), "Middle scored, every goal stacked", font=f_b, fill=(20, 22, 26))
    d.text((1262, 30), "red side · tile 23.8\"", font=f_xs, fill=(110, 116, 126), anchor="ra")
    return im

if OUT == "check":
    for pr in crash_check(): print("CRASH", pr)
    print("total", round(TOTAL, 1), "s  pts", points_after(set(range(len(R))))[0] + 8)
    sys.exit()
BASE = field_layer()
CARRY = []
_c = False
for _r in R:
    if _r[3] in ("pick", "grab", "load"): _c = True
    if _r[3] in ("score", "tall"): _c = False
    CARRY.append(_c)

def robot(d, x, y, h, carrying):
    w, l = 13.0, 14.5
    c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
    def T(ox, oy):  # ox right, oy forward
        return P(x + ox * c + oy * s, y - ox * s + oy * c)
    body = [T(-w / 2, l / 2), T(w / 2, l / 2), T(w / 2, -l / 2), T(-w / 2, -l / 2)]
    d.polygon(body, fill=(30, 34, 42), outline=(10, 10, 10))
    d.polygon([T(0, l / 2 - 1), T(-4, l / 2 - 7), T(4, l / 2 - 7)], fill=(255, 255, 255))  # front arrow
    claw = [T(-3.5, -l / 2), T(3.5, -l / 2), T(3.5, -l / 2 - 3), T(-3.5, -l / 2 - 3)]
    d.polygon(claw, fill=(214, 62, 62))
    if carrying:
        X, Y = T(0, -l / 2 - 2.5)
        d.ellipse([X - 9, Y - 9, X + 9, Y + 9], fill=(240, 196, 40), outline=(30, 30, 30), width=2)

def frame(t, trail):
    im = BASE.copy(); d = ImageDraw.Draw(im)
    acc = 0; pose = None; idx = 0; phase = ""
    for dur, fn, i, ph in segs:
        if t <= acc + dur or (dur, fn, i, ph) == segs[-1]:
            u = 0 if dur == 0 else min(1, max(0, (t - acc) / dur))
            pose = fn(u); idx = i; phase = ph; break
        acc += dur
    x, y, h = pose
    trail.append(P(x, y))
    if len(trail) > 1: d.line(trail, fill=(214, 62, 62), width=3, joint="curve")
    if t >= TOTAL: carrying = False
    elif phase == "act" and R[idx][3] in ("pick", "grab", "load"): carrying = CARRY[idx]
    else: carrying = CARRY[idx - 1] if idx > 0 else False
    done_steps = set(range(1, idx)) | ({idx} if (phase == "act" or t >= TOTAL) else set())
    def stack(px, py):
        X, Y = P(px, py); d.ellipse([X - 12, Y - 12, X + 12, Y + 12], fill=(240, 196, 40), outline=(30, 30, 30), width=2)
        d.ellipse([X - 5, Y - 5, X + 5, Y + 5], fill=(255, 255, 255), outline=(30, 30, 30), width=1)
    def wpin(px, py):
        X, Y = P(px, py); d.ellipse([X - 9, Y - 9, X + 9, Y + 9], fill=(240, 196, 40), outline=(30, 30, 30), width=1)
    grabbed = {R[k][5] for k in done_steps if R[k][3] in ("grab", "pick")}
    for key, (px, py) in CPINS.items():
        X, Y = P(px, py)
        if key not in grabbed: d.ellipse([X - 8, Y - 8, X + 8, Y + 8], fill=(240, 196, 40), outline=(30, 30, 30), width=1)
    for kx, ky in CL.values():
        X, Y = P(kx, ky); d.ellipse([X - 12, Y - 12, X + 12, Y + 12], fill=(200, 205, 212), outline=(30, 30, 30), width=2)
    for key, (px, py) in STACKS.items():
        if key not in grabbed: stack(px, py)
    pts, goals = points_after(done_steps)
    if t >= TOTAL: pts += 8
    for g, (gx, gy) in {"G2": (-2 * T, -T), "G3": (-T, -2 * T), "TALL": (0, 0), "LS": (-2 * T, T), "BR": (T, -2 * T)}.items():
        if goals.get(g):
            X, Y = P(gx, gy); d.ellipse([X - 11, Y - 11, X + 11, Y + 11], fill=(240, 196, 40), outline=(30, 30, 30), width=3)
            d.text((X, Y), str(len(goals[g])), font=f_m, fill=(20, 22, 26), anchor="mm")
    robot(d, x, y, h, carrying)
    # panel: timer + steps
    d.text((790, 100), f"{min(t, TOTAL):5.1f} s", font=f_t, fill=(20, 22, 26))
    d.text((960, 118), f"/ 60 s", font=f_m, fill=(110, 116, 126))
    d.text((1060, 108), f"{pts} pts", font=f_b, fill=(20, 22, 26))
    top = max(1, min(idx - 6, len(R) - 18))
    yy = 168
    for j in range(top, min(top + 18, len(R))):
        cur = j == idx and t < TOTAL + 0.01
        past = j < idx or t >= TOTAL
        col = (20, 22, 26) if cur else ((150, 155, 165) if past else (90, 96, 106))
        if cur: d.rectangle([780, yy - 3, 1262, yy + 24], fill=(253, 236, 236))
        d.text((790, yy), f"{j:2d}", font=f_xs, fill=(150, 155, 165))
        d.text((818, yy), R[j][4], font=f_m if cur else f_s, fill=col)
        yy += 29
    d.text((790, 690), "wall pieces are cup + pin stacks · 10 pts per owned yellow half · +8 park", font=f_xs, fill=(110, 116, 126))
    return im

n = int((TOTAL + HOLD) * FPS)
p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
                      "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", "-movflags", "+faststart", OUT], stdin=subprocess.PIPE)
trail = []
for k in range(n):
    p.stdin.write(frame(k / FPS, trail).tobytes())
    if k == int(TOTAL * FPS * 0.62): frame(k / FPS, list(trail)).save(OUT.replace(".mp4", "_mid.png"))
p.stdin.close(); p.wait()
frame(TOTAL + 1, list(trail)).save(OUT.replace(".mp4", "_end.png"))
print("total", round(TOTAL, 1), "s")
