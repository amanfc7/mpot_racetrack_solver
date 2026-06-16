import heapq
import random
import math
from collections import deque

class SALNSSolver:
    def __init__(self, env):
        self.env = env
        self.distance_field = self._generate_distance_field()

    def _generate_distance_field(self):
        distance_field = {}
        queue = deque()
        for (fx, fy) in self.env.finish_positions:
            queue.append((fx, fy, 0))
            distance_field[(fx, fy)] = 0
        while queue:
            x, y, dist = queue.popleft()
            # 4-Connectivity prevents the heuristic leaking diagonally across wall corners
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.env.width and 0 <= ny < self.env.height:
                    if (nx, ny) not in self.env.obstacles and (nx, ny) not in distance_field:
                        distance_field[(nx, ny)] = dist + 1
                        queue.append((nx, ny, dist + 1))
        return distance_field

    def _heuristic(self, x, y):
        return self.distance_field.get((x, y), 999999)

    def solve(self, iterations=150, initial_temp=15.0, cooling_rate=0.995):
        print("  Generating robust global baseline trajectory...")
        best_path = self._get_initial_solution()
        
        if not best_path:
            return None
            
        print(f"  Baseline found: {len(best_path)} steps. Initializing Simulated Annealing loop...")
        current_path = list(best_path)
        T = initial_temp
        
        for i in range(iterations):
            if len(current_path) < 6: break
                
            if random.random() < 0.6 and len(current_path) > 10:
                candidates = []
                for _ in range(5):
                    s_idx = random.randint(0, len(current_path) - 6)
                    g = random.randint(3, min(12, len(current_path) - s_idx - 1))
                    window = current_path[s_idx : s_idx + g + 1]
                    avg_speed = sum(abs(v[2]) + abs(v[3]) for v in window) / len(window)
                    candidates.append((avg_speed, s_idx, g))
                candidates.sort(key=lambda item: item[0])
                _, start_idx, gap = candidates[0]
            else:
                start_idx = random.randint(0, len(current_path) - 4)
                gap = random.randint(2, min(8, len(current_path) - start_idx - 1))
            
            end_idx = start_idx + gap
            start_state = current_path[start_idx]
            target_state = current_path[end_idx]
            
            repaired_segment = self._repair(start_state, target_state, gap)
            
            if repaired_segment is not None:
                new_path = current_path[:start_idx + 1] + repaired_segment + current_path[end_idx + 1:]
                delta = len(new_path) - len(current_path)
                
                if delta < 0:
                    current_path = new_path
                    if len(current_path) < len(best_path):
                        best_path = current_path
                        print(f"    [Iteration {i+1}] SA Shortcut found! Path optimized to {len(best_path)} moves.")
                elif delta == 0:
                    current_path = new_path
                else:
                    prob = math.exp(-delta / T)
                    if random.random() < prob:
                        current_path = new_path
            
            T *= cooling_rate
            if T < 0.01: T = 0.01
                    
        return best_path

    def _get_initial_solution(self):
        start_state = (self.env.start_pos[0], self.env.start_pos[1], 0, 0)
        h = self._heuristic(start_state[0], start_state[1])
        count = 0
        queue = [(1.0 * h, 0, 0, count, start_state, [start_state])]
        visited = {start_state: 0}
        
        while queue:
            _, cost, _, _, current, path = heapq.heappop(queue)
            x, y, vx, vy = current
            if self.env.is_finish(x, y): return path
            if cost > visited.get(current, float('inf')): continue
                
            v_ax, v_ay = self.env.get_valid_accelerations(x, y, vx, vy)
            for ax in v_ax:
                for ay in v_ay:
                    nx, ny, nvx, nvy, valid = self.env.is_valid_move(x, y, vx, vy, ax, ay)
                    if valid:
                        next_state = (nx, ny, nvx, nvy)
                        next_cost = cost + 1
                        if next_cost < visited.get(next_state, float('inf')):
                            visited[next_state] = next_cost
                            next_h = self._heuristic(nx, ny)
                            speed_bonus = -(abs(nvx) + abs(nvy))
                            count += 1
                            heapq.heappush(queue, (next_cost + 1.0 * next_h, next_cost, speed_bonus, count, next_state, path + [next_state]))
        return None

    def _repair(self, start_state, target_state, max_steps):
        tx, ty, tvx, tvy = target_state
        init_h = abs(start_state[0] - tx) + abs(start_state[1] - ty) + abs(start_state[2] - tvx) + abs(start_state[3] - tvy)
        count = 0
        queue = [(init_h, 0, count, start_state, [])]
        visited = {start_state: 0}
        
        while queue:
            _, cost, _, current, segment = heapq.heappop(queue)
            if current == target_state: return segment
            if cost >= max_steps: continue
            
            x, y, vx, vy = current
            v_ax, v_ay = self.env.get_valid_accelerations(x, y, vx, vy)
            for ax in v_ax:
                for ay in v_ay:
                    nx, ny, nvx, nvy, valid = self.env.is_valid_move(x, y, vx, vy, ax, ay)
                    if valid:
                        next_state = (nx, ny, nvx, nvy)
                        next_cost = cost + 1
                        if next_cost < visited.get(next_state, float('inf')):
                            visited[next_state] = next_cost
                            next_h = abs(nx - tx) + abs(ny - ty) + abs(nvx - tvx) + abs(nvy - tvy)
                            count += 1
                            heapq.heappush(queue, (next_cost + next_h, next_cost, count, next_state, segment + [next_state]))
        return None