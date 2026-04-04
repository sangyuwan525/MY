import pygame
import sys
import heapq
import math

# ================= 配置与常量 =================
ROWS, COLS = 4, 3
GRID_NODES = 12
ENTRY_NODE = 12
EXIT_NODE = 13
TOTAL_NODES = 14

COST_MOVE = 10           
COST_SIDE_GRAB = 15      
COST_REMOVE_PENALTY = 5  

HEIGHT_MAP = [
    400, 200, 400,
    200, 400, 600,
    400, 600, 400,
    200, 400, 200,
    0, 0
]

KFS_NONE, KFS_R1, KFS_R2, KFS_FAKE = 0, 1, 2, 3
COLORS = {
    "BG": (30, 30, 35),
    "PANEL": (45, 45, 50),
    "STEP": (80, 80, 85),
    "TEXT": (255, 255, 255),
    KFS_NONE: (80, 80, 85),
    KFS_R1: (255, 140, 0),    
    KFS_R2: (0, 200, 255),    
    KFS_FAKE: (220, 20, 60),  
    "PATH": (50, 255, 50),
    "REMOVE": (255, 50, 255),
    "ENTRY_GRAB": (255, 215, 0),  
    "SIDE_GRAB": (0, 255, 150),   
    "BTN_NORMAL": (70, 130, 180),
    "BTN_HOVER": (100, 150, 200)
}

def draw_dashed_line(surface, color, start_pos, end_pos, width=3, dash_length=10):
    x1, y1 = start_pos
    x2, y2 = end_pos
    dl = math.hypot(x2 - x1, y2 - y1)
    if dl == 0: return
    dx = (x2 - x1) / dl
    dy = (y2 - y1) / dl
    for i in range(0, int(dl), dash_length * 2):
        start = (x1 + dx * i, y1 + dy * i)
        end = (x1 + dx * min(i + dash_length, dl), y1 + dy * min(i + dash_length, dl))
        pygame.draw.line(surface, color, start, end, width)

class Visualizer:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((1100, 750))
        pygame.display.set_caption("2D KFS Path Planner")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("arial", 18, bold=True)
        self.big_font = pygame.font.SysFont("arial", 22, bold=True)
        
        self.map_data = [KFS_NONE] * TOTAL_NODES
        self.result = None
        self.message = "Limits: R1(max 4), R2(max 4), Fake(max 1)."
        self.selected_tool = KFS_NONE
        
        self.ui_buttons = [
            [pygame.Rect(850, 100, 200, 50), KFS_NONE, "Eraser (None)"],
            [pygame.Rect(850, 170, 200, 50), KFS_R1, "Place R1 (Orange)"],
            [pygame.Rect(850, 240, 200, 50), KFS_R2, "Place R2 (Cyan)"],
            [pygame.Rect(850, 310, 200, 50), KFS_FAKE, "Place Fake (Red)"]
        ]
        self.btn_start = pygame.Rect(850, 450, 200, 60)
        
    def get_node_pos(self, node_idx):
        if node_idx == EXIT_NODE: return (400, 80)
        if node_idx == ENTRY_NODE: return (400, 660)
        r = node_idx // COLS
        c = node_idx % COLS
        return (200 + (2 - c) * 200, 200 + (3 - r) * 110)

    def is_adjacent(self, u, v):
        if u > v: u, v = v, u
        if u == 1 and v == ENTRY_NODE: return True
        if (u == 9 and v == EXIT_NODE) or (u == 11 and v == EXIT_NODE): return True
        if v >= ENTRY_NODE: return False
        r1, c1 = divmod(u, COLS)
        r2, c2 = divmod(v, COLS)
        return abs(r1 - r2) + abs(c1 - c2) == 1

    def is_adjacent_grid(self, u, v):
        if u >= 12 or v >= 12: return False
        r1, c1 = divmod(u, COLS)
        r2, c2 = divmod(v, COLS)
        return abs(r1 - r2) + abs(c1 - c2) == 1

    def run_logic(self):
        r2_nodes = [i for i, t in enumerate(self.map_data[:GRID_NODES]) if t == KFS_R2]
        if len(r2_nodes) < 2:
            self.message = "Error: Need at least 2 R2 nodes to plan!"
            return None

        entry_r2s = [node for node in [0, 1, 2] if self.map_data[node] == KFS_R2]
        best_plan = None
        min_cost = float('inf')

        for i in range(len(r2_nodes)):
            for j in range(i + 1, len(r2_nodes)):
                t1, t2 = r2_nodes[i], r2_nodes[j]
                
                if entry_r2s and (t1 not in entry_r2s and t2 not in entry_r2s):
                    continue
                
                start_node = ENTRY_NODE
                grab_choices = []
                
                # ===============================================
                # 修复：强制 Entry Grab 逻辑
                # ===============================================
                if self.map_data[1] == KFS_R2:
                    # 中间有，强制优先拿中间
                    if t1 == 1: grab_choices = [1]
                    elif t2 == 1: grab_choices = [1]
                    else: continue
                elif entry_r2s:
                    # 中间没有，但0或2有，也必须强制拿其中之一！
                    if t1 in [0, 2]: grab_choices.append(t1)
                    if t2 in [0, 2]: grab_choices.append(t2)
                
                pq = []
                visited = {}
                
                if not entry_r2s:
                    # 只有在0,1,2全都没有R2的时候，才能不进行Entry Grab走进去
                    heapq.heappush(pq, (0, start_node, 0, 0, 0, [start_node], [], []))
                else:
                    # 否则，必须支付代价执行强制的 Grab Choice
                    for g in grab_choices:
                        mask = 1 if g == t1 else 2
                        heapq.heappush(pq, (COST_SIDE_GRAB, start_node, 0, 0, mask, [start_node], [g], []))

                while pq:
                    cost, u, r1, r2_rem, mask, path, entry_grabbed, side_grabs = heapq.heappop(pq)
                    state = (u, r1, r2_rem, mask)
                    if visited.get(state, float('inf')) <= cost: continue
                    visited[state] = cost

                    if u == EXIT_NODE and mask == 3:
                        if cost < min_cost:
                            min_cost = cost
                            best_plan = {
                                "path": path, "cost": cost, "targets": (t1, t2),
                                "entry_grabbed": entry_grabbed, "side_grabs": side_grabs
                            }
                        break

                    neighs = []
                    if u == ENTRY_NODE: neighs = [1] 
                    elif u == EXIT_NODE: neighs = []
                    else:
                        r, c = divmod(u, COLS)
                        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                            nr, nc = r+dr, c+dc
                            if 0<=nr<ROWS and 0<=nc<COLS: neighs.append(nr*COLS+nc)
                        if u in [9, 11]: neighs.append(EXIT_NODE)

                    for v in neighs:
                        if abs(int(HEIGHT_MAP[v]) - int(HEIGHT_MAP[u])) != 200: continue
                        
                        nr1, nr2, nmask, ncost = r1, r2_rem, mask, cost + COST_MOVE
                        if v < GRID_NODES:
                            if self.map_data[v] == KFS_FAKE: continue
                            if self.map_data[v] == KFS_R1:
                                nr1 += 1
                                if nr1 > 2: continue
                            
                            if self.map_data[v] == KFS_R2:
                                if v == t1: nmask |= 1
                                elif v == t2: nmask |= 2
                                else:
                                    nr2 += 1
                                    ncost += COST_REMOVE_PENALTY
                                    if nr2 > 2: continue
                            
                            can_sg_t1 = (nmask & 1) == 0 and self.is_adjacent_grid(v, t1)
                            can_sg_t2 = (nmask & 2) == 0 and self.is_adjacent_grid(v, t2)
                            
                            sg_options = [(0, 0, [])] 
                            if can_sg_t1: sg_options.append((1, COST_SIDE_GRAB, [(v, t1)])) 
                            if can_sg_t2: sg_options.append((2, COST_SIDE_GRAB, [(v, t2)])) 
                            if can_sg_t1 and can_sg_t2: sg_options.append((3, COST_SIDE_GRAB * 2, [(v, t1), (v, t2)])) 

                            for m_add, c_add, sg_add in sg_options:
                                f_mask = nmask | m_add
                                f_cost = ncost + c_add
                                f_sgs = list(side_grabs) + sg_add
                                heapq.heappush(pq, (f_cost, v, nr1, nr2, f_mask, path + [v], entry_grabbed, f_sgs))
                        else:
                            heapq.heappush(pq, (ncost, v, nr1, nr2, nmask, path + [v], entry_grabbed, side_grabs))
        
        if best_plan:
            self.message = f"Success! Cost: {best_plan['cost']} (Move:{COST_MOVE}, SideGrab:{COST_SIDE_GRAB})"
        else:
            self.message = "No valid path found (check heights / constraints)!"
        return best_plan

    def draw(self):
        self.screen.fill(COLORS["BG"])
        for u in range(TOTAL_NODES):
            for v in range(TOTAL_NODES):
                if u < v and self.is_adjacent(u, v):
                    if abs(HEIGHT_MAP[u] - HEIGHT_MAP[v]) == 200:
                        pygame.draw.line(self.screen, (70, 70, 70), self.get_node_pos(u), self.get_node_pos(v), 3)

        for i in range(TOTAL_NODES):
            pos = self.get_node_pos(i)
            rect = pygame.Rect(0, 0, 100, 70)
            rect.center = pos
            bg_color = COLORS["STEP"]
            if i < GRID_NODES and self.map_data[i] != KFS_NONE:
                bg_color = COLORS[self.map_data[i]]
            pygame.draw.rect(self.screen, bg_color, rect, border_radius=8)
            pygame.draw.rect(self.screen, (200, 200, 200), rect, 2, border_radius=8)
            
            if i == ENTRY_NODE:
                txt1 = self.font.render("ENTRY 12", True, COLORS["TEXT"])
                self.screen.blit(txt1, txt1.get_rect(center=(pos[0], pos[1])))
            elif i == EXIT_NODE:
                txt1 = self.font.render("EXIT 13", True, COLORS["TEXT"])
                self.screen.blit(txt1, txt1.get_rect(center=(pos[0], pos[1])))
            else:
                txt_id = self.font.render(f"ID: {i}", True, COLORS["TEXT"])
                txt_h = self.font.render(f"H: {HEIGHT_MAP[i]}", True, COLORS["TEXT"])
                self.screen.blit(txt_id, txt_id.get_rect(center=(pos[0], pos[1]-12)))
                self.screen.blit(txt_h, txt_h.get_rect(center=(pos[0], pos[1]+12)))

        if self.result:
            path = self.result["path"]
            targets = self.result["targets"]
            entry_grabbed_nodes = self.result.get("entry_grabbed", [])
            side_grabbed_edges = self.result.get("side_grabs", [])
            
            for g_node in entry_grabbed_nodes:
                p1 = self.get_node_pos(ENTRY_NODE)
                p2 = self.get_node_pos(g_node)
                draw_dashed_line(self.screen, COLORS["ENTRY_GRAB"], p1, p2, width=4)
                px, py = p1[0] + (p2[0] - p1[0]) * 0.3, p1[1] + (p2[1] - p1[1]) * 0.3
                txt_surf = self.font.render(f"Entry Grab(+{COST_SIDE_GRAB})", True, COLORS["ENTRY_GRAB"])
                txt_rect = txt_surf.get_rect(center=(px, py - 15))
                pygame.draw.rect(self.screen, (20, 20, 20), txt_rect.inflate(6, 4))
                self.screen.blit(txt_surf, txt_rect)

            for sg_from, sg_target in side_grabbed_edges:
                p1 = self.get_node_pos(sg_from)
                p2 = self.get_node_pos(sg_target)
                draw_dashed_line(self.screen, COLORS["SIDE_GRAB"], p1, p2, width=4)
                px, py = p1[0] + (p2[0] - p1[0]) * 0.7, p1[1] + (p2[1] - p1[1]) * 0.7
                txt_surf = self.font.render(f"Side Grab(+{COST_SIDE_GRAB})", True, COLORS["SIDE_GRAB"])
                txt_rect = txt_surf.get_rect(center=(px, py - 15))
                pygame.draw.rect(self.screen, (20, 20, 20), txt_rect.inflate(6, 4))
                self.screen.blit(txt_surf, txt_rect)

            all_grabbed_targets = set(entry_grabbed_nodes + [t for _, t in side_grabbed_edges])

            for i in range(len(path)-1):
                p1 = self.get_node_pos(path[i])
                p2 = self.get_node_pos(path[i+1])
                target_node = path[i+1]
                
                action_text = "Move"
                line_color = COLORS["PATH"]
                
                if target_node < GRID_NODES:
                    if self.map_data[target_node] == KFS_R2:
                        if target_node in targets:
                            if target_node in all_grabbed_targets:
                                action_text = "Move (Grabbed)"
                                line_color = COLORS["PATH"]
                            else:
                                action_text = f"Take R2 (+{COST_MOVE})"
                                line_color = COLORS["PATH"]
                        else:
                            action_text = "Remove R2"
                            line_color = COLORS["REMOVE"]
                    elif self.map_data[target_node] == KFS_R1:
                        action_text = "Remove R1"
                        line_color = COLORS["REMOVE"]

                pygame.draw.line(self.screen, line_color, p1, p2, 6)
                pygame.draw.circle(self.screen, (255, 255, 255), (int(p2[0]), int(p2[1])), 5)
                px, py = p1[0] + (p2[0] - p1[0]) * 0.5, p1[1] + (p2[1] - p1[1]) * 0.5
                txt_surf = self.font.render(action_text, True, line_color)
                txt_rect = txt_surf.get_rect(center=(px, py - 25))
                pygame.draw.rect(self.screen, (20, 20, 20), txt_rect.inflate(6, 4))
                self.screen.blit(txt_surf, txt_rect)

        pygame.draw.rect(self.screen, COLORS["PANEL"], (800, 0, 300, 750))
        msg_surf = self.font.render(self.message, True, (255, 150, 150) if "Limit" in self.message or "Rule" in self.message else (255, 200, 0))
        self.screen.blit(msg_surf, (810, 20))
        
        for btn_rect, t_type, label in self.ui_buttons:
            color = COLORS[t_type] if t_type != KFS_NONE else (100, 100, 100)
            if self.selected_tool == t_type:
                pygame.draw.rect(self.screen, (255, 255, 255), btn_rect.inflate(8, 8))
            pygame.draw.rect(self.screen, color, btn_rect)
            lbl = self.font.render(label, True, (255, 255, 255))
            self.screen.blit(lbl, (btn_rect.x + 15, btn_rect.y + 15))

        mx, my = pygame.mouse.get_pos()
        btn_color = COLORS["BTN_HOVER"] if self.btn_start.collidepoint(mx, my) else COLORS["BTN_NORMAL"]
        pygame.draw.rect(self.screen, btn_color, self.btn_start, border_radius=10)
        start_lbl = self.big_font.render("START PLANNING", True, COLORS["TEXT"])
        self.screen.blit(start_lbl, start_lbl.get_rect(center=self.btn_start.center))
        pygame.display.flip()

    def main_loop(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit(); sys.exit()
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()
                    for btn_rect, t_type, _ in self.ui_buttons:
                        if btn_rect.collidepoint(mx, my): self.selected_tool = t_type
                    if self.btn_start.collidepoint(mx, my): self.result = self.run_logic()
                    if mx < 800:
                        for i in range(GRID_NODES):
                            if pygame.Rect(self.get_node_pos(i)[0]-50, self.get_node_pos(i)[1]-35, 100, 70).collidepoint(mx, my):
                                target_type, current_type = self.selected_tool, self.map_data[i]
                                if target_type != KFS_NONE and target_type != current_type:
                                    if target_type == KFS_R1 and sum(1 for x in self.map_data[:GRID_NODES] if x == KFS_R1) >= 4:
                                        self.message = "Limit Blocked: Max 4 R1 items!"; continue
                                    if target_type == KFS_R2 and sum(1 for x in self.map_data[:GRID_NODES] if x == KFS_R2) >= 4:
                                        self.message = "Limit Blocked: Max 4 R2 items!"; continue
                                    if target_type == KFS_FAKE and sum(1 for x in self.map_data[:GRID_NODES] if x == KFS_FAKE) >= 1:
                                        self.message = "Limit Blocked: Max 1 Fake item!"; continue
                                if target_type == KFS_FAKE and i in [0, 1, 2]:
                                    self.message = "Rule: No Fake on Entry (0,1,2)!"; continue
                                elif target_type == KFS_R1 and i in [4, 7]:
                                    self.message = "Rule: R1 only on outer nodes (Not 4,7)!"; continue
                                self.map_data[i] = target_type
                                self.result = None 
            self.draw()
            self.clock.tick(30)

if __name__ == "__main__":
    Visualizer().main_loop()