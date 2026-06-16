import os
import math

class RaceTrackEnv:
    # --- DIAGNOSTIC CONFIGURATION SWITCHES ---
    INVERT_Y = True          # True: Cartesian (Y=0 at bottom). False: Matrix (Y=0 at top row).
    V_FIRST_UPDATE = True    # True: v = v + a, x = x + v. False: x = x + v, v = v + a.
    GRID_CENTER_ROUND = True # True: round(x) tile matching. False: int(floor(x)) cell matching.
    
    # 
    MAX_SPEED = 4            # Set to an integer (e.g., 4) to limit top speed, or None for unlimited.

    def __init__(self, track_file):
        if not os.path.exists(track_file):
            raise FileNotFoundError(f"Track file not found: {track_file}")
            
        with open(track_file, 'r') as f:
            raw_lines = [line.replace('\n', '').replace('\r', '') for line in f]
        
        raw_lines = [line for line in raw_lines if len(line) > 0]
        
        self.height = len(raw_lines)
        self.width = len(raw_lines[0]) if self.height > 0 else 0
        
        self.obstacles = set()
        self.grass = set()
        self.tracks = set()
        self.finish_positions = set()
        self.start_positions = []
        
        has_explicit_track = any('T' in line for line in raw_lines)
        
        for r in range(self.height):
            y = (self.height - 1) - r if self.INVERT_Y else r
            for x in range(self.width):
                char = raw_lines[r][x]
                if char == 'G': 
                    self.grass.add((x, y))
                elif char == 'F': 
                    self.finish_positions.add((x, y))
                elif char == 'S': 
                    self.start_positions.append((x, y))
                    self.tracks.add((x, y))
                elif char == 'T': 
                    self.tracks.add((x, y))
                elif char in ['O', 'X', '#']:
                    self.obstacles.add((x, y))
                elif char in ['.', ' ']:
                    if has_explicit_track:
                        self.obstacles.add((x, y))
                    else:
                        self.tracks.add((x, y))
                else:
                    self.obstacles.add((x, y))
                    
        if not self.start_positions: raise ValueError("No start position 'S' found.")
        if not self.finish_positions: raise ValueError("No finish line 'F' found.")
        self.start_pos = self.start_positions[0]

    def get_valid_accelerations(self, x, y, vx, vy):
        valid_ax, valid_ay = [-1, 0, 1], [-1, 0, 1]
        
        # Handle grass speed constraints
        if (x, y) in self.grass:
            if vx >= 2: valid_ax = [-1]
            elif vx == 1: valid_ax = [-1, 0]
            elif vx <= -2: valid_ax = [1]
            elif vx == -1: valid_ax = [0, 1]
            if vy >= 2: valid_ay = [-1]
            elif vy == 1: valid_ay = [-1, 0]
            elif vy <= -2: valid_ay = [1]
            elif vy == -1: valid_ay = [0, 1]
            return valid_ax, valid_ay

        # Global Speed Limit Enforcer
        if self.MAX_SPEED is not None:
            valid_ax = [ax for ax in valid_ax if abs(vx + ax) <= self.MAX_SPEED]
            valid_ay = [ay for ay in valid_ay if abs(vy + ay) <= self.MAX_SPEED]
            
        return valid_ax, valid_ay

    def is_valid_move(self, x, y, vx, vy, ax, ay):
        v_ax, v_ay = self.get_valid_accelerations(x, y, vx, vy)
        if ax not in v_ax or ay not in v_ay:
            return x, y, vx, vy, False
            
        if self.V_FIRST_UPDATE:
            nvx, nvy = vx + ax, vy + ay
            nx, ny = x + nvx, y + nvy
        else:
            nx, ny = x + vx, y + vy
            nvx, nvy = vx + ax, vy + ay
        
        if not (0 <= nx < self.width and 0 <= ny < self.height):
            return x, y, vx, vy, False
            
        # Continuous Collision Detection
        x1, y1 = float(x), float(y)
        x2, y2 = float(nx), float(ny)
        dx, dy = x2 - x1, y2 - y1
        
        bx_start = max(0, min(int(x1), int(x2)) - 1)
        bx_end = min(self.width - 1, max(int(x1), int(x2)) + 1)
        by_start = max(0, min(int(y1), int(y2)) - 1)
        by_end = min(self.height - 1, max(int(y1), int(y2)) + 1)
        
        for cx in range(bx_start, bx_end + 1):
            for cy in range(by_start, by_end + 1):
                if (cx, cy) in self.obstacles:
                    if self.GRID_CENTER_ROUND:
                        xmin, xmax = cx - 0.5, cx + 0.5
                        ymin, ymax = cy - 0.5, cy + 0.5
                    else:
                        xmin, xmax = float(cx), float(cx + 1)
                        ymin, ymax = float(cy), float(cy + 1)
                        
                    t_min, t_max = 0.0, 1.0
                    possible_hit = True
                    
                    if abs(dx) < 1e-9:
                        if x1 < xmin or x1 > xmax: possible_hit = False
                    else:
                        t1 = (xmin - x1) / dx
                        t2 = (xmax - x1) / dx
                        t_min = max(t_min, min(t1, t2))
                        t_max = min(t_max, max(t1, t2))
                        
                    if possible_hit:
                        if abs(dy) < 1e-9:
                            if y1 < ymin or y1 > ymax: possible_hit = False
                        else:
                            t1 = (ymin - y1) / dy
                            t2 = (ymax - y1) / dy
                            t_min = max(t_min, min(t1, t2))
                            t_max = min(t_max, max(t1, t2))
                            
                    if possible_hit and (t_min <= t_max + 1e-9):
                        return x, y, vx, vy, False
                        
        return nx, ny, nvx, nvy, True

    def is_finish(self, x, y):
        return (x, y) in self.finish_positions