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

# ---- route: (x, y, heading, kind, text). kind: start/move/back/slow/toggle/pick/score/grab/end ----
T = TILE
LD = (-56.5, -59.7)  # robot center with its back on the bottom-left match loader
# 6th field on a scoring step: (goal, pin type on top: yy=20, yr=15, yb=10)
R = [
 (-62.9, 0, 0, "start", "Start at the wall toggle, preload (bare) in the claw"),
 (-62.9, 0, 0, "arm", "Small lift: toggle arm flips the wall toggle"),
 (-47.6, 6, 180, "move", "Line up below the left short goal"),
 (-47.6, 14.8, 180, "score", "PRELOAD on the left short goal (1st, bare)", ("LS", "yy")),
 (-48.7, 8.9, 125, "move", "Line up on yellow wall pin A"),
 (-60.2, 16.9, 125, "pick", "Back onto WALL PIN A, grab (bare)"),
 (-50, -4, 0, "move", "Line up above Red Goal 2"),
 (-47.6, -14.8, 0, "score", "Wall pin A on Red Goal 2 (1st, bare)", ("G2", "yy")),
 (-34, -6, 90, "move", "Forward off the goal, around the stack"),
 (-12.6, -12.6, 45, "move", "Line up, back toward the field stack"),
 (-17.4, -17.4, 45, "grab", "Back SLOWLY onto the FIELD STACK, grab"),
 (-30, -23.8, 270, "move", "Line up right of Red Goal 2, lift"),
 (-38.6, -23.8, 270, "score", "Field stack on Red Goal 2 (2nd)", ("G2", "yy")),
 (-34, -34, 45, "move", "Back toward the corner"),
 (-41.2, -41.2, 45, "grab", "Back SLOWLY onto the CORNER STACK, grab"),
 (-47.6, -40, 180, "move", "Line up below Red Goal 2, lift high"),
 (-47.6, -32.8, 180, "score", "Corner stack on Red Goal 2 (3rd)", ("G2", "yy")),
 (-14, -34, 90, "move", "Over the top of Red Goal 3"),
 (0, -59.5, 90, "move", "Down to the bottom toggle"),
 (0, -59.5, 90, "arm", "Small lift: toggle arm flips the bottom toggle"),
 (8.9, -48.7, 325, "move", "Line up on yellow wall pin D"),
 (16.3, -59.4, 325, "pick", "Back onto WALL PIN D, grab (bare)"),
 (6, -47.6, 270, "move", "Line up left of the short goal"),
 (14.8, -47.6, 270, "score", "Wall pin D on the bottom-right short goal (1st)", ("BR", "yy")),
 (-8.9, -48.7, 35, "move", "Line up on yellow wall pin B"),
 (-16.3, -59.4, 35, "pick", "Back onto WALL PIN B, grab (bare)"),
 (-6, -47.6, 90, "move", "Line up right of Red Goal 3"),
 (-14.8, -47.6, 90, "score", "Wall pin B on Red Goal 3 (1st, bare)", ("G3", "yy")),
 (-4, -60, 90, "move", "Down to the wall lane"),
 (LD[0], LD[1], 90, "load", "Back along the wall onto the loader: LOAD 1"),
 (-40, -47.6, 270, "move", "Turn, line up left of Red Goal 3"),
 (-32.8, -47.6, 270, "score", "Load 1 on Red Goal 3 (2nd)", ("G3", "yr")),
 (-44, -59.7, 90, "move", "Back to the loader"),
 (LD[0], LD[1], 90, "load", "LOAD 2"),
 (-40, -47.6, 270, "move", "Line up on Red Goal 3"),
 (-32.8, -47.6, 270, "score", "Load 2 on Red Goal 3 (3rd)", ("G3", "yb")),
 (-44, -59.7, 90, "move", "Back to the loader"),
 (LD[0], LD[1], 90, "load", "LOAD 3 (yellow + yellow)"),
 (-40, -47.6, 270, "move", "Line up on Red Goal 3, lift high"),
 (-32.8, -47.6, 270, "score", "Load 3 on Red Goal 3 (4th, top)", ("G3", "yy")),
 (-44, -59.7, 90, "move", "Back to the loader"),
 (LD[0], LD[1], 90, "load", "LOAD 4"),
 (-47.6, -40, 180, "move", "Line up below Red Goal 2, lift high"),
 (-47.6, -32.8, 180, "score", "Load 4 on Red Goal 2 (4th, top)", ("G2", "yr")),
 (-44, -59.7, 90, "move", "Back to the loader"),
 (LD[0], LD[1], 90, "load", "LOAD 5"),
 (-34, -40, 180, "move", "Out from the loader"),
 (-34, -2, 180, "move", "Up the lane past Red Goal 2"),
 (-47.6, 6, 180, "move", "Line up below the left short goal, lift"),
 (-47.6, 14.8, 180, "score", "Load 5 on the left short goal (2nd, top)", ("LS", "yr")),
 (-48.7, -8.9, 55, "move", "Line up on yellow wall pin C"),
 (-60.2, -16.9, 55, "pick", "Back onto WALL PIN C, grab (bare)"),
 (-48.7, -8.9, 55, "move", "Out the same way"),
 (-17.4, -17.4, 225, "move", "Across, back toward the tall goal"),
 (-6.4, -6.4, 225, "tall", "Wall pin C on the TALL GOAL (bare), PARK", ("TALL", "yy")),
]
TOPV = {"yy": 20, "yr": 15, "yb": 10}
def points_after(steps_done):
    goals = {}
    for k in sorted(steps_done):
        if len(R[k]) > 5 and R[k][5]: goals.setdefault(R[k][5][0], []).append(R[k][5][1])
    return sum(10 * (len(v) - 1) + TOPV[v[-1]] for v in goals.values()), goals
GRABS = [i for i, r in enumerate(R) if r[3] == "grab"]
PICKS = [i for i, r in enumerate(R) if r[3] == "pick"]
SPEED, SLOW, TURN = 53.0, 14.0, 330.0  # in/s, in/s (slow approach), deg/s
ACT = {"pick": 0.4, "arm": 0.5, "load": 0.5, "score": 0.8, "grab": 0.6, "tall": 1.1, "move": 0.0, "park": 0.3, "start": 0.6}

def wrap(a): return (a + 180) % 360 - 180

# ---- build a timeline of segments: each returns pose at local t ----
segs = []  # (duration, func(u)->(x,y,h), step index, phase)
x, y, h = R[0][0], R[0][1], R[0][2]
segs.append((ACT["start"], lambda u, x=x, y=y, h=h: (x, y, h), 0, "wait"))
for i in range(1, len(R)):
    nx, ny, nh, kind = R[i][:4]
    dx, dy = nx - x, ny - y
    dist = math.hypot(dx, dy)
    if dist > 0.3:
        travel = math.degrees(math.atan2(dx, dy))
        fwd_turn, back_turn = wrap(travel - h), wrap(travel + 180 - h)
        # drive the way that needs less turning; slow/grab and short moves keep the current heading
        if dist < 6 or kind in ("grab", "pick", "load"):
            dh1 = 0.0
        else:
            dh1 = fwd_turn if abs(fwd_turn) <= abs(back_turn) else back_turn
        if abs(dh1) > 1:
            d = abs(dh1) / TURN
            segs.append((d, lambda u, x=x, y=y, h=h, dh=dh1: (x, y, h + dh * u), i, "turn"))
            h = h + dh1
        sp = SLOW if kind in ("grab", "pick") else SPEED
        d = dist / sp + 0.15
        def drive(u, x=x, y=y, dx=dx, dy=dy, h=h):
            e = u * u * (3 - 2 * u)
            return (x + dx * e, y + dy * e, h)
        segs.append((d, drive, i, "drive"))
        x, y = nx, ny
    dh2 = wrap(nh - h)
    if abs(dh2) > 1:
        d = abs(dh2) / TURN
        segs.append((d, lambda u, x=x, y=y, h=h, dh=dh2: (x, y, h + dh * u), i, "turn"))
    h = nh
    if ACT.get(kind, 0) > 0:
        segs.append((ACT[kind], lambda u, x=x, y=y, h=h: (x, y, h), i, "act"))
TOTAL = sum(s[0] for s in segs)
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
    for px, py in [(67.6, 22.1), (67.6, -22.1), (-21.5, 66.8), (21.5, 66.8)]:
        circ(px, py, 1.8, (240, 196, 40), w=1)
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
    d.text((790, 30), "19109M  ·  Skills path v4 (draft)", font=f_m, fill=(110, 116, 126))
    d.text((790, 56), "3.25 in wheels: every second used", font=f_b, fill=(20, 22, 26))
    d.text((1262, 30), "red side · tile 23.8\"", font=f_xs, fill=(110, 116, 126), anchor="ra")
    return im

BASE = field_layer()
CARRY = []
_c = False
for _r in R:
    if _r[3] in ("pick", "grab", "load"): _c = True
    if _r[3] in ("score", "tall"): _c = False
    CARRY.append(_c)

def robot(d, x, y, h, carrying):
    w, l = 15.5, 17.8
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
    for k, (px, py) in zip(PICKS, [(-67.6, 22.1), (21.5, -66.8), (-21.5, -66.8), (-67.6, -22.1)]):
        if k not in done_steps: wpin(px, py)
    if GRABS[0] not in done_steps: stack(-T, -T)
    if GRABS[1] not in done_steps: stack(-2 * T, -2 * T)
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
    d.text((790, 690), "number on a goal = pins on it · points if we own every goal at the end (+8 park)", font=f_xs, fill=(110, 116, 126))
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
