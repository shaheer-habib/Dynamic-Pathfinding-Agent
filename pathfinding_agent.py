"""
Dynamic Pathfinding Agent
=========================
  Algorithms : A* Search  |  Greedy Best-First Search
  Heuristics : Manhattan  |  Euclidean
  GUI        : Pygame  (pip install pygame)
  Run        : python pathfinding_agent.py
"""

import pygame, random, heapq, math, time

# ════════════════════════════════════════════════════
#  WINDOW / LAYOUT
#  Total window = 1100 x 660
#  Left  : grid area  780 px wide
#  Right : panel      320 px wide
# ════════════════════════════════════════════════════
WIN_W  = 1100
WIN_H  = 660
GRID_W = 780          # grid area width
PNL_W  = WIN_W - GRID_W   # 320 px panel
ROWS   = 20
COLS   = 26
CELL   = GRID_W // COLS   # ≈ 30 px  (auto-sized)
FPS    = 30

# ────── colours ──────────────────────────────────
W   = (255,255,255)
BLK = (15, 15, 20)
DRK = (38, 38, 48)       # wall
GRY = (120,120,130)
LGY = (205,205,210)      # empty cell

G_GREEN  = (30, 195,  75)   # path
G_BLUE   = (55, 120, 215)   # visited
G_YELLOW = (235,205,  0)    # frontier
G_ORANGE = (240,130,  20)   # goal
G_TEAL   = (0,  185, 175)   # start
G_PURP   = (160, 45, 230)   # agent dot

P_BG     = (28,  28,  40)   # panel bg
P_BOX    = (18,  18,  32)   # info box bg
P_LINE   = (60,  60,  85)   # borders

# Button themes  (normal, hover, pressed/active)
T_BLUE  = ((52,108,195),(80,140,225),(32, 80,155))
T_GREEN = ((32,150, 62),(52,180, 82),(18,110, 42))
T_PURP  = ((118,42,168),(148,72,198),(88,22,138))

# ════════════════════════════════════════════════════
#  HEURISTICS
# ════════════════════════════════════════════════════
def h_manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def h_euclidean(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

# ════════════════════════════════════════════════════
#  SEARCH ALGORITHMS
# ════════════════════════════════════════════════════
def algo_astar(grid, start, goal, hfn):
    R,C = len(grid), len(grid[0])
    hp  = [(hfn(start,goal), 0, start)]
    came= {start:None}; gs={start:0}; vis=set(); log=[]
    while hp:
        _,g,cur = heapq.heappop(hp)
        if cur in vis: continue
        vis.add(cur); log.append(('V',cur))
        if cur==goal: return _trace(came,cur), vis, log
        r,c=cur
        for dr,dc in ((-1,0),(1,0),(0,-1),(0,1)):
            nb=(r+dr,c+dc); nr,nc=nb
            if 0<=nr<R and 0<=nc<C and grid[nr][nc]==0 and nb not in vis:
                ng=g+1
                if nb not in gs or ng<gs[nb]:
                    gs[nb]=ng; came[nb]=cur
                    heapq.heappush(hp,(ng+hfn(nb,goal),ng,nb))
                    log.append(('F',nb))
    return None,vis,log

def algo_gbfs(grid, start, goal, hfn):
    R,C = len(grid), len(grid[0])
    hp  = [(hfn(start,goal), start)]
    came= {start:None}; vis=set(); log=[]
    while hp:
        _,cur = heapq.heappop(hp)
        if cur in vis: continue
        vis.add(cur); log.append(('V',cur))
        if cur==goal: return _trace(came,cur), vis, log
        r,c=cur
        for dr,dc in ((-1,0),(1,0),(0,-1),(0,1)):
            nb=(r+dr,c+dc); nr,nc=nb
            if 0<=nr<R and 0<=nc<C and grid[nr][nc]==0 and nb not in vis and nb not in came:
                came[nb]=cur
                heapq.heappush(hp,(hfn(nb,goal),nb))
                log.append(('F',nb))
    return None,vis,log

def _trace(came, node):
    p=[]
    while node: p.append(node); node=came[node]
    return p[::-1]

# ════════════════════════════════════════════════════
#  BUTTON
# ════════════════════════════════════════════════════
class Btn:
    def __init__(self, x, y, w, h, text, theme=T_BLUE, font=None):
        self.r=pygame.Rect(x,y,w,h); self.text=text
        self.theme=theme; self.font=font; self.active=False
    def draw(self, surf):
        ho = self.r.collidepoint(pygame.mouse.get_pos())
        c  = self.theme[2] if self.active else (self.theme[1] if ho else self.theme[0])
        pygame.draw.rect(surf,c,self.r,border_radius=6)
        pygame.draw.rect(surf,P_LINE,self.r,1,border_radius=6)
        t=self.font.render(self.text,True,W)
        surf.blit(t,t.get_rect(center=self.r.center))
    def hit(self,pos): return self.r.collidepoint(pos)

# ════════════════════════════════════════════════════
#  APPLICATION
# ════════════════════════════════════════════════════
class App:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("Dynamic Pathfinding Agent")
        self.clock  = pygame.time.Clock()

        # fonts  (size: big, med, small)
        self.fb = pygame.font.SysFont("consolas",14,bold=True)
        self.fm = pygame.font.SysFont("consolas",13)
        self.fs = pygame.font.SysFont("consolas",11)

        # grid
        self.rows=ROWS; self.cols=COLS
        self.start=(0,0); self.goal=(ROWS-1,COLS-1)
        self.grid=[[0]*COLS for _ in range(ROWS)]
        self.density=30

        # settings
        self.algo  = 'astar'
        self.heur  = 'manhattan'
        self.mode  = 'wall'     # wall | start | goal
        self.dyn   = False
        self.speed = 4

        # visualisation
        self._clr()

        # dynamic agent
        self.ag_path=[]; self.ag_step=0; self.dtick=0

        # drag
        self.mdown=False; self.erase=False

        self._mkbtns()
        self._genmaze()

    # ── helpers ────────────────────────────────
    def _clr(self):
        self.vis=set(); self.fro=set()
        self.path=[]; self.log=[]; self.li=0
        self.going=False; self.done=False
        self.nv=0; self.pc=0; self.ms=0.0

    def _genmaze(self):
        self.grid=[[0]*self.cols for _ in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                if (r,c) in (self.start,self.goal): continue
                if random.random()<self.density/100: self.grid[r][c]=1
        self._clr()

    def _search(self, s=None, g=None):
        s=s or self.start; g=g or self.goal
        hfn = h_manhattan if self.heur=='manhattan' else h_euclidean
        t=time.time()
        if self.algo=='astar': path,vis,log=algo_astar(self.grid,s,g,hfn)
        else:                  path,vis,log=algo_gbfs (self.grid,s,g,hfn)
        self.ms=(time.time()-t)*1000; self.nv=len(vis)
        self.pc=len(path)-1 if path else 0
        return path,vis,log

    def _run(self):
        self._clr()
        path,_,log=self._search()
        self.log=log; self.path=path or []; self.going=True
        if self.dyn and path: self.ag_path=path; self.ag_step=0

    # ── make buttons ───────────────────────────
    def _mkbtns(self):
        x=GRID_W+10; w=PNL_W-20
        # two-column buttons (half width)
        hw=(w-6)//2

        def B(y,lbl,theme=T_BLUE,full=True):
            bw=w if full else hw
            return Btn(x,y,bw,28,lbl,theme,self.fm)
        def B2(y,lbl,ox=0,theme=T_BLUE):
            return Btn(x+ox,y,hw,28,lbl,theme,self.fm)

        y=10
        # ── row 1: Run + Clear ──
        self.bRun  =B2(y,"▶ Run",   0, T_GREEN)
        self.bClr  =B2(y,"Clear",  hw+6)
        y+=34
        # ── row 2: Reset + Maze ──
        self.bRst  =B2(y,"Reset Grid",   0)
        self.bMaze =B2(y,"Gen Maze",  hw+6)
        y+=42

        # ── Algorithm ──
        self._lAlgo=y; y+=14
        self.bAstar=B2(y,"A* Search",   0)
        self.bGbfs =B2(y,"Greedy BFS", hw+6)
        y+=42

        # ── Heuristic ──
        self._lHeur=y; y+=14
        self.bManh =B2(y,"Manhattan",  0)
        self.bEucl =B2(y,"Euclidean", hw+6)
        y+=42

        # ── Edit Mode ──
        self._lMode=y; y+=14
        self.bWall =B2(y,"Draw Wall",  0)
        self.bSS   =B2(y,"Set Start", hw+6)
        y+=34
        self.bSG   =B(y,"Set Goal",T_BLUE,True)
        y+=42

        # ── Dynamic ──
        self._lDyn=y; y+=14
        self.bDyn  =B(y,"Dynamic Mode: OFF",T_PURP,True)
        y+=42

        # ── Speed ──
        self._lSpd=y; y+=14
        self.bSpu  =B2(y,"Speed +",  0)
        self.bSpd  =B2(y,"Speed -", hw+6)
        y+=42

        # ── Density ──
        self._lDns=y; y+=14
        self.bDu   =B2(y,f"Density+",  0)
        self.bDd   =B2(y,f"Density-", hw+6)
        y+=42

        # ── Grid Size ──
        self._lGrd=y; y+=14
        self.bGu   =B2(y,"Grid +",  0)
        self.bGd   =B2(y,"Grid -", hw+6)
        y+=42

        self._metY = y+4    # metrics box starts here

        self.allB=[self.bRun,self.bClr,self.bRst,self.bMaze,
                   self.bAstar,self.bGbfs,self.bManh,self.bEucl,
                   self.bWall,self.bSS,self.bSG,self.bDyn,
                   self.bSpu,self.bSpd,self.bDu,self.bDd,
                   self.bGu,self.bGd]

        # defaults
        self.bAstar.active=True; self.bManh.active=True; self.bWall.active=True

    # ── animation step ─────────────────────────
    def _animstep(self):
        if not self.going: return
        for _ in range(self.speed):
            if self.li>=len(self.log):
                self.going=False; self.done=True; return
            k,cell=self.log[self.li]
            (self.vis if k=='V' else self.fro).add(cell)
            if k=='V': self.fro.discard(cell)
            self.li+=1

    # ── dynamic agent ──────────────────────────
    def _agentstep(self):
        if not self.ag_path or self.ag_step>=len(self.ag_path)-1: return
        self.ag_step+=1; cur=self.ag_path[self.ag_step]
        if random.random()<0.07:
            r=random.randint(0,self.rows-1); c=random.randint(0,self.cols-1)
            if (r,c) not in (self.start,self.goal,cur):
                self.grid[r][c]=1
                if (r,c) in self.ag_path[self.ag_step:]:
                    np,_,nl=self._search(s=cur)
                    if np: self.ag_path=self.ag_path[:self.ag_step]+np; self.path=self.ag_path; self.log+=nl
                    else:  self.ag_path=[]; self.path=[]

    # ── grid click ─────────────────────────────
    def _gclick(self, pos):
        c=pos[0]//CELL; r=pos[1]//CELL
        if not(0<=r<self.rows and 0<=c<self.cols): return
        if   self.mode=='start': self.start=(r,c); self.grid[r][c]=0; self._clr()
        elif self.mode=='goal':  self.goal =(r,c); self.grid[r][c]=0; self._clr()
        else:
            if (r,c) not in (self.start,self.goal):
                self.grid[r][c]=0 if self.erase else (0 if self.grid[r][c] else 1)
                self._clr()

    # ── button click ───────────────────────────
    def _bclick(self, pos):
        if   self.bRun.hit(pos):  self._run()
        elif self.bClr.hit(pos):  self._clr()
        elif self.bRst.hit(pos):  self.grid=[[0]*self.cols for _ in range(self.rows)]; self._clr()
        elif self.bMaze.hit(pos): self._genmaze()

        elif self.bAstar.hit(pos): self.algo='astar'; self.bAstar.active=True;  self.bGbfs.active=False
        elif self.bGbfs.hit(pos):  self.algo='gbfs';  self.bGbfs.active=True;   self.bAstar.active=False
        elif self.bManh.hit(pos):  self.heur='manhattan'; self.bManh.active=True; self.bEucl.active=False
        elif self.bEucl.hit(pos):  self.heur='euclidean'; self.bEucl.active=True; self.bManh.active=False

        elif self.bWall.hit(pos): self.mode='wall';  self.bWall.active=True;  self.bSS.active=False; self.bSG.active=False
        elif self.bSS.hit(pos):   self.mode='start'; self.bSS.active=True;    self.bWall.active=False; self.bSG.active=False
        elif self.bSG.hit(pos):   self.mode='goal';  self.bSG.active=True;    self.bWall.active=False; self.bSS.active=False

        elif self.bDyn.hit(pos):
            self.dyn=not self.dyn
            self.bDyn.text="Dynamic Mode: ON " if self.dyn else "Dynamic Mode: OFF"
            self.bDyn.active=self.dyn

        elif self.bSpu.hit(pos): self.speed=min(self.speed+1,30)
        elif self.bSpd.hit(pos): self.speed=max(self.speed-1,1)

        elif self.bDu.hit(pos): self.density=min(self.density+5,75); self._updlbls()
        elif self.bDd.hit(pos): self.density=max(self.density-5, 5); self._updlbls()

        elif self.bGu.hit(pos):
            self.rows=min(self.rows+2,30); self.cols=min(self.cols+2,38)
            self.start=(0,0); self.goal=(self.rows-1,self.cols-1)
            self._genmaze()
        elif self.bGd.hit(pos):
            self.rows=max(self.rows-2,8); self.cols=max(self.cols-2,10)
            self.start=(0,0); self.goal=(self.rows-1,self.cols-1)
            self._genmaze()

    def _updlbls(self):
        pass   # density shown in metrics box

    # ── draw ───────────────────────────────────
    def _draw(self):
        self.screen.fill(BLK)

        # ── grid cells ──
        cw = GRID_W // self.cols
        ch = WIN_H  // self.rows

        for r in range(self.rows):
            for c in range(self.cols):
                rx=c*cw; ry=r*ch
                rect=pygame.Rect(rx,ry,cw-1,ch-1)
                cell=(r,c)
                if   self.grid[r][c]:      col=DRK
                elif cell==self.start:     col=G_TEAL
                elif cell==self.goal:      col=G_ORANGE
                elif cell in self.path:    col=G_GREEN
                elif cell in self.vis:     col=G_BLUE
                elif cell in self.fro:     col=G_YELLOW
                else:                      col=LGY
                pygame.draw.rect(self.screen,col,rect,border_radius=2)
                if cell==self.start:
                    t=self.fs.render("S",True,W); self.screen.blit(t,t.get_rect(center=rect.center))
                elif cell==self.goal:
                    t=self.fs.render("G",True,W); self.screen.blit(t,t.get_rect(center=rect.center))

        # agent dot
        if self.dyn and self.ag_path and self.ag_step<len(self.ag_path):
            ar,ac=self.ag_path[self.ag_step]
            pygame.draw.circle(self.screen,G_PURP,(ac*cw+cw//2, ar*ch+ch//2),cw//2-2)

        # grid lines
        for r in range(self.rows+1):
            pygame.draw.line(self.screen,(60,60,72),(0,r*ch),(GRID_W,r*ch))
        for c in range(self.cols+1):
            pygame.draw.line(self.screen,(60,60,72),(c*cw,0),(c*cw,WIN_H))

        # ── panel bg ──
        pygame.draw.rect(self.screen,P_BG,(GRID_W,0,PNL_W,WIN_H))
        pygame.draw.line(self.screen,P_LINE,(GRID_W,0),(GRID_W,WIN_H),2)

        # panel title
        t=self.fb.render("Pathfinding Agent",True,W)
        self.screen.blit(t,(GRID_W+10,4))
        pygame.draw.line(self.screen,P_LINE,(GRID_W+8,20),(WIN_W-8,20),1)

        # section labels helper
        def sec(lbl, y):
            t=self.fs.render(lbl,True,G_YELLOW)
            self.screen.blit(t,(GRID_W+10,y))

        sec("[ Algorithm ]",  self._lAlgo)
        sec("[ Heuristic ]",  self._lHeur)
        sec("[ Edit Mode ]",  self._lMode)
        sec("[ Dynamic ]",    self._lDyn)
        sec("[ Speed ]",      self._lSpd)
        sec(f"[ Density: {self.density}% ]", self._lDns)
        sec("[ Grid Size ]",  self._lGrd)

        # all buttons
        for b in self.allB: b.draw(self.screen)

        # ── metrics box ──
        mx=GRID_W+8; my=self._metY
        bw=PNL_W-16; bh=WIN_H-my-8
        pygame.draw.rect(self.screen,P_BOX,(mx,my,bw,bh),border_radius=8)
        pygame.draw.rect(self.screen,P_LINE,(mx,my,bw,bh),1,border_radius=8)

        t=self.fm.render("── Metrics ──",True,G_YELLOW)
        self.screen.blit(t,(mx+8,my+5))

        stats=[
            ("Nodes Visited", str(self.nv),                  G_TEAL),
            ("Path Cost",     str(self.pc),                  G_GREEN),
            ("Time  (ms)",    f"{self.ms:.2f}",              G_ORANGE),
            ("Algorithm",     self.algo.upper(),             W),
            ("Heuristic",     self.heur[:4].capitalize(),    W),
            ("Anim Speed",    str(self.speed),               W),
            ("Grid",          f"{self.rows}x{self.cols}",    W),
        ]
        for i,(lbl,val,col) in enumerate(stats):
            ry=my+22+i*18
            self.screen.blit(self.fs.render(lbl+" :",True,GRY),(mx+8, ry))
            self.screen.blit(self.fs.render(val,     True,col),(mx+150,ry))

        # legend  (2 columns, bottom of metrics box)
        legend=[
            (G_YELLOW,"Frontier"),(G_BLUE,"Visited"),
            (G_GREEN, "Path"),    (G_TEAL,"Start"),
            (G_ORANGE,"Goal"),    (DRK,   "Wall"),
        ]
        ly=my+22+len(stats)*18+6
        pygame.draw.line(self.screen,P_LINE,(mx+4,ly-3),(mx+bw-4,ly-3),1)
        for i,(col,lbl) in enumerate(legend):
            ix=mx+8+(i%2)*135; iy=ly+(i//2)*16
            pygame.draw.rect(self.screen,col,(ix,iy+2,11,10),border_radius=2)
            self.screen.blit(self.fs.render(lbl,True,W),(ix+14,iy))

        pygame.display.flip()

    # ── main loop ──────────────────────────────
    def run(self):
        while True:
            self.clock.tick(FPS)
            self.dtick+=1

            for ev in pygame.event.get():
                if ev.type==pygame.QUIT: pygame.quit(); return

                elif ev.type==pygame.MOUSEBUTTONDOWN:
                    p=ev.pos
                    cw=GRID_W//self.cols
                    if p[0]<GRID_W:
                        self.mdown=True
                        r=p[1]//(WIN_H//self.rows); c=p[0]//cw
                        if 0<=r<self.rows and 0<=c<self.cols:
                            self.erase=self.grid[r][c]==1
                        self._gclick(p)
                    else: self._bclick(p)

                elif ev.type==pygame.MOUSEBUTTONUP:
                    self.mdown=False

                elif ev.type==pygame.MOUSEMOTION:
                    if self.mdown and ev.pos[0]<GRID_W:
                        self._gclick(ev.pos)

                elif ev.type==pygame.KEYDOWN:
                    if   ev.key==pygame.K_RETURN: self._run()
                    elif ev.key==pygame.K_c:      self._clr()
                    elif ev.key==pygame.K_r:
                        self.grid=[[0]*self.cols for _ in range(self.rows)]; self._clr()
                    elif ev.key==pygame.K_g:      self._genmaze()

            self._animstep()
            if self.dyn and self.done and self.ag_path and self.dtick%10==0:
                self._agentstep()
            self._draw()

# ════════════════════════════════════════════════════
if __name__=="__main__":
    App().run()