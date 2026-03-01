import pygame, sys, time, random, math
from collections import deque

# =========================
# AYARLAR
# =========================
N = 8
HALF_W = 4
CELL = 80
MARGIN = 20
HUD_H = 100
W = MARGIN*2 + N*CELL
H = MARGIN*2 + N*CELL + HUD_H

BG = (15, 15, 15)
GRID = (35, 35, 35)
WALL = (235, 235, 235)
ROBOT = (60, 140, 255)
GOAL = (255, 80, 80)
START = (255, 215, 0)
RAY = (0, 255, 140)
TXT = (230, 230, 230)

# Yön: 0=N,1=E,2=S,3=W
DIRS = [(0,-1),(1,0),(0,1),(-1,0)]
DIRNAME = ["N","E","S","W"]

# Sensör çizimi: ön=0°, sol=-60°, sağ=+60°
SENSOR_ANGLES = [0, -60, +60]
SENSOR_MAX_CELLS = 8

STEP_EVERY = 0.22
TIME_LIMIT = 180.0

# Start: en alt 4 kare
BOTTOM4 = [(2,7), (3,7), (4,7), (5,7)]
# Goal havuzları: robotun tarafına göre
TOP4_LEFT  = [(0,0), (1,0), (2,0), (3,0)]
TOP4_RIGHT = [(4,0), (5,0), (6,0), (7,0)]

# =========================
# GERÇEK PİST
# =========================
class Maze:
    def __init__(self, n):
        self.n = n
        self.walls = [[[False]*4 for _ in range(n)] for _ in range(n)]

    def inb(self,x,y): return 0 <= x < self.n and 0 <= y < self.n

    def set_wall(self, x, y, d, v=True):
        if not self.inb(x,y): return
        self.walls[y][x][d] = v
        nx, ny = x + DIRS[d][0], y + DIRS[d][1]
        if self.inb(nx,ny):
            self.walls[ny][nx][(d+2)%4] = v

    def has_wall(self, x, y, d):
        return self.walls[y][x][d]

def add_outer_walls(mz: Maze):
    n = mz.n
    for x in range(n):
        mz.set_wall(x, 0, 0, True)
        mz.set_wall(x, n-1, 2, True)
    for y in range(n):
        mz.set_wall(0, y, 3, True)
        mz.set_wall(n-1, y, 1, True)

def carve_perfect_maze(width, height):
    walls = [[[True]*4 for _ in range(width)] for _ in range(height)]
    vis = [[False]*width for _ in range(height)]

    def inb(x,y): return 0 <= x < width and 0 <= y < height

    stack = [(random.randrange(width), random.randrange(height))]
    vis[stack[0][1]][stack[0][0]] = True

    while stack:
        x,y = stack[-1]
        nbrs = []
        for d,(dx,dy) in enumerate(DIRS):
            nx,ny = x+dx,y+dy
            if inb(nx,ny) and not vis[ny][nx]:
                nbrs.append((nx,ny,d))
        if not nbrs:
            stack.pop()
            continue
        nx,ny,d = random.choice(nbrs)
        walls[y][x][d] = False
        walls[ny][nx][(d+2)%4] = False
        vis[ny][nx] = True
        stack.append((nx,ny))

    return walls

def build_sym_maze():
    mz = Maze(N)
    add_outer_walls(mz)

    left = carve_perfect_maze(HALF_W, N)  # 4x8

    # sol yarı
    for y in range(N):
        for x in range(HALF_W):
            for d in range(4):
                mz.set_wall(x, y, d, left[y][x][d])

    # sağ yarı = ayna (E<->W)
    for y in range(N):
        for x in range(HALF_W):
            mx = (N-1) - x
            mz.set_wall(mx, y, 0, left[y][x][0])  # N
            mz.set_wall(mx, y, 2, left[y][x][2])  # S
            mz.set_wall(mx, y, 1, left[y][x][3])  # E = sol W
            mz.set_wall(mx, y, 3, left[y][x][1])  # W = sol E

    return mz

# =========================
# ROBOTUN BİLDİĞİ HARİTA
# UNKNOWN = bilinmiyor, OPEN = açık, WALLK = duvar
# =========================
UNKNOWN, OPEN, WALLK = 0, 1, 2

class KnownMap:
    def __init__(self, n):
        self.n = n
        self.k = [[[UNKNOWN]*4 for _ in range(n)] for _ in range(n)]

    def inb(self,x,y): return 0 <= x < self.n and 0 <= y < self.n

    def set_edge(self, x, y, d, state):
        if not self.inb(x,y): return
        self.k[y][x][d] = state
        nx, ny = x + DIRS[d][0], y + DIRS[d][1]
        if self.inb(nx,ny):
            self.k[ny][nx][(d+2)%4] = state

    def get_edge(self, x, y, d):
        return self.k[y][x][d]

def init_known_with_outer(n):
    km = KnownMap(n)
    for x in range(n):
        km.set_edge(x, 0, 0, WALLK)
        km.set_edge(x, n-1, 2, WALLK)
    for y in range(n):
        km.set_edge(0, y, 3, WALLK)
        km.set_edge(n-1, y, 1, WALLK)
    return km

# =========================
# SENSÖR: GERÇEK DUVARA BAKAR, BİLİNEN HARİTAYI GÜNCELLER
# =========================
def sense_walls_from_true(mz: Maze, x, y, heading):
    front = mz.has_wall(x,y,heading)
    left  = mz.has_wall(x,y,(heading+3)%4)
    right = mz.has_wall(x,y,(heading+1)%4)
    return left, front, right

def update_known_from_sensors(km: KnownMap, mz: Maze, x, y, heading):
    left, front, right = sense_walls_from_true(mz, x, y, heading)

    km.set_edge(x, y, heading,         WALLK if front else OPEN)
    km.set_edge(x, y, (heading+3)%4,   WALLK if left  else OPEN)
    km.set_edge(x, y, (heading+1)%4,   WALLK if right else OPEN)

# =========================
# ✅ ONLINE RECOMPUTED FLOOD FILL (BFS)
# Her adımda dist'i BAŞTAN hesaplar.
# UNKNOWN'ı geçilebilir sayıyoruz (keşif için).
# =========================
INF = 10**9

def bfs_dist_recompute(km: KnownMap, goal):
    gx, gy = goal
    dist = [[INF]*km.n for _ in range(km.n)]
    q = deque()
    dist[gy][gx] = 0
    q.append((gx,gy))

    while q:
        x,y = q.popleft()
        for d,(dx,dy) in enumerate(DIRS):
            # sadece WALLK engel; OPEN/UNKNOWN geçilebilir
            if km.get_edge(x,y,d) == WALLK:
                continue
            nx, ny = x+dx, y+dy
            if not km.inb(nx,ny):
                continue
            if dist[ny][nx] > dist[y][x] + 1:
                dist[ny][nx] = dist[y][x] + 1
                q.append((nx,ny))
    return dist

# =========================
# ÇİZİM
# =========================
def cell_center(x,y):
    return (MARGIN + x*CELL + CELL//2, MARGIN + y*CELL + CELL//2)

def draw_maze(screen, mz: Maze):
    for y in range(N):
        for x in range(N):
            rx = MARGIN + x*CELL
            ry = MARGIN + y*CELL
            pygame.draw.rect(screen, GRID, (rx, ry, CELL, CELL), 1)

    for y in range(N):
        for x in range(N):
            rx = MARGIN + x*CELL
            ry = MARGIN + y*CELL
            if mz.has_wall(x,y,0):
                pygame.draw.line(screen, WALL, (rx,ry), (rx+CELL,ry), 4)
            if mz.has_wall(x,y,1):
                pygame.draw.line(screen, WALL, (rx+CELL,ry), (rx+CELL,ry+CELL), 4)
            if mz.has_wall(x,y,2):
                pygame.draw.line(screen, WALL, (rx,ry+CELL), (rx+CELL,ry+CELL), 4)
            if mz.has_wall(x,y,3):
                pygame.draw.line(screen, WALL, (rx,ry), (rx,ry+CELL), 4)

def draw_text(screen, font, text, x, y, color=TXT):
    screen.blit(font.render(text, True, color), (x, y))

def angle_to_dir(heading, angle_offset_deg):
    # sensör çizimi 60°, ama duvara uzaklığı grid yönünde sayıyoruz:
    # açıyı en yakın kardinal yöne projekte ediyoruz (N/E/S/W)
    hd_deg = {0:-90, 1:0, 2:90, 3:180}[heading]
    a = hd_deg + angle_offset_deg
    while a <= -180: a += 360
    while a > 180: a -= 360
    cands = [(-90,0),(0,1),(90,2),(180,3),(-180,3)]
    best = None
    bestd = None
    for deg, d in cands:
        diff = abs(a - deg)
        if best is None or diff < best:
            best = diff
            bestd = d
    return bestd

def raycast_cells_to_wall_true(mz: Maze, x, y, d, max_cells=SENSOR_MAX_CELLS):
    steps = 0
    cx, cy = x, y
    while steps < max_cells:
        if mz.has_wall(cx, cy, d):
            break
        nx, ny = cx + DIRS[d][0], cy + DIRS[d][1]
        if not mz.inb(nx, ny):
            break
        steps += 1
        cx, cy = nx, ny
    return steps

def draw_sensor_rays(screen, mz: Maze, x, y, heading):
    cx, cy = cell_center(x,y)
    hd_deg = {0:-90, 1:0, 2:90, 3:180}[heading]
    for aoff in SENSOR_ANGLES:
        ang = (hd_deg + aoff) * math.pi / 180.0
        d = angle_to_dir(heading, aoff)
        steps = raycast_cells_to_wall_true(mz, x, y, d, SENSOR_MAX_CELLS)
        length = CELL * (steps + 0.5)
        ex = cx + length * math.cos(ang)
        ey = cy + length * math.sin(ang)
        pygame.draw.line(screen, RAY, (cx, cy), (ex, ey), 3)
        pygame.draw.circle(screen, RAY, (int(ex), int(ey)), 5)

# =========================
# START / GOAL
# =========================
def pick_start():
    return random.choice(BOTTOM4)

def pick_goal_for_start(start):
    if start[0] < 4:
        return random.choice(TOP4_LEFT)
    return random.choice(TOP4_RIGHT)

# =========================
# MAIN
# =========================
def main():
    pygame.init()
    screen = pygame.display.set_mode((W,H))
    pygame.display.set_caption("Online Recomputed Flood Fill (BFS) | unknown map + sensors | random start/goal")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 22)
    small = pygame.font.SysFont("consolas", 18)

    mz = build_sym_maze()

    start = pick_start()
    goal = pick_goal_for_start(start)
    rx, ry = start
    heading = 0

    km = init_known_with_outer(N)

    auto = True
    finished = False
    t0 = None
    elapsed = 0.0
    step_acc = 0.0

    # debug: BFS kaç hücre gezdiğini saymak istersen
    bfs_cells_last = 0

    def reset_positions_only():
        nonlocal start, goal, rx, ry, heading, km, finished, t0, elapsed, step_acc
        start = pick_start()
        goal = pick_goal_for_start(start)
        rx, ry = start
        heading = 0
        km = init_known_with_outer(N)
        finished = False
        t0 = None
        elapsed = 0.0
        step_acc = 0.0
        update_known_from_sensors(km, mz, rx, ry, heading)

    def reset_new_maze():
        nonlocal mz
        mz = build_sym_maze()
        reset_positions_only()

    def timer_start_if_needed():
        nonlocal t0
        if t0 is None:
            t0 = time.time()

    def timer_update():
        nonlocal elapsed, finished
        if t0 is not None and not finished:
            elapsed = time.time() - t0
            if elapsed >= TIME_LIMIT:
                finished = True

    def choose_next_dir_from_dist(dist):
        """
        dist haritasına göre komşu seç:
        - en küçük dist komşu
        - tie-break: düz > sağ/sol > geri
        """
        cur = dist[ry][rx]
        best = INF
        cands = []

        for d,(dx,dy) in enumerate(DIRS):
            if km.get_edge(rx, ry, d) == WALLK:
                continue
            nx, ny = rx+dx, ry+dy
            if not km.inb(nx,ny):
                continue
            v = dist[ny][nx]
            if v < best:
                best = v
                cands = [d]
            elif v == best:
                cands.append(d)

        if not cands:
            return None

        def rank(d):
            if d == heading: return 0
            if d == (heading+1)%4 or d == (heading+3)%4: return 1
            return 2

        cands.sort(key=rank)
        return cands[0]

    def step_robot():
        nonlocal rx, ry, heading, finished, bfs_cells_last

        if finished:
            return
        if (rx, ry) == goal:
            finished = True
            return

        # 1) sensörle öğren
        update_known_from_sensors(km, mz, rx, ry, heading)

        # 2) ✅ BFS ile dist'i BAŞTAN hesapla (recompute)
        dist = bfs_dist_recompute(km, goal)

        # (opsiyonel) debug sayaç: INF olmayan kaç hücre var
        bfs_cells_last = sum(1 for y in range(N) for x in range(N) if dist[y][x] < INF)

        # 3) yön seç
        nd = choose_next_dir_from_dist(dist)
        if nd is None or dist[ry][rx] >= INF:
            finished = True
            return

        heading = nd

        # döndükten sonra tekrar sensör (ön duvar)
        update_known_from_sensors(km, mz, rx, ry, heading)

        if mz.has_wall(rx, ry, heading):
            # duvarı bilinen haritaya yaz ve bu adım ilerleme olmasın
            km.set_edge(rx, ry, heading, WALLK)
            return

        # 4) ilerle
        rx += DIRS[heading][0]
        ry += DIRS[heading][1]

        # yeni hücrede öğren
        update_known_from_sensors(km, mz, rx, ry, heading)

        if (rx, ry) == goal:
            finished = True

    # başlangıç sensörü
    update_known_from_sensors(km, mz, rx, ry, heading)

    while True:
        dt = clock.tick(60) / 1000.0
        step_acc += dt

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit(); sys.exit(0)
                if event.key == pygame.K_n:
                    reset_new_maze()
                if event.key == pygame.K_r:
                    reset_positions_only()
                if event.key == pygame.K_a:
                    auto = not auto
                if event.key == pygame.K_SPACE:
                    timer_start_if_needed()
                    step_robot()

        if auto and not finished and step_acc >= STEP_EVERY:
            timer_start_if_needed()
            step_acc = 0.0
            step_robot()

        timer_update()

        # ÇİZİM
        screen.fill(BG)
        draw_maze(screen, mz)

        sx, sy = cell_center(*start)
        gx, gy = cell_center(*goal)
        pygame.draw.circle(screen, START, (sx, sy), 10)
        pygame.draw.circle(screen, GOAL, (gx, gy), 12)

        draw_sensor_rays(screen, mz, rx, ry, heading)

        cx, cy = cell_center(rx, ry)
        pygame.draw.circle(screen, ROBOT, (cx, cy), 14)
        ox, oy = cx, cy
        if heading == 0: oy -= 22
        if heading == 2: oy += 22
        if heading == 1: ox += 22
        if heading == 3: ox -= 22
        pygame.draw.line(screen, (255,255,255), (cx, cy), (ox, oy), 3)

        # HUD
        pygame.draw.rect(screen, (10,10,10), (0, H-HUD_H, W, HUD_H))
        mode = "AUTO" if auto else "MANUAL"
        status = "DONE" if finished else "RUN"
        draw_text(screen, font, f"Time: {elapsed:6.2f}s / {TIME_LIMIT:.0f}s", 15, H-HUD_H+10)
        draw_text(screen, font, f"Mode: {mode} | Status: {status}", 330, H-HUD_H+10)
        draw_text(screen, font, f"Robot: ({rx},{ry}) {DIRNAME[heading]} | Goal: {goal} | Start: {start}", 15, H-HUD_H+40)
        draw_text(screen, font, f"BFS reachable cells: {bfs_cells_last}", 15, H-HUD_H+70)

        draw_text(screen, small, "N: new maze+start/goal | R: same maze random start/goal | A: auto | SPACE: step",
                  330, H-HUD_H+72, (180,180,180))

        pygame.display.flip()

if __name__ == "__main__":
    main()
