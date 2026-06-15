import math
import random
import time


class RaceTrackEnv:
    def __init__(self, track_string: str):
        """ Parse the character-based matrix text file."""
        self.grid = [list(line.strip()) for line in track_string.strip().split('\n') if line.strip()]
        self.height = len(self.grid)
        self.width = len(self.grid[0]) if self.height > 0 else 0

        self.starts = []
        self.finishes = []

        for y in range(self.height):
            for x in range(self.width):
                char = self.grid[y][x]
                if char == 'S':
                    self.starts.append((x, y))
                elif char == 'F':
                    self.finishes.append((x, y))

    def get_tile(self, x: int, y: int) -> str:
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[y][x]
        return 'O'

    def intersects_obstacle(self, x1: int, y1: int, x2: int, y2: int) -> bool:
        """ Line-segment intersection using grid traversal (Amanatides-Woo)."""
        if self.get_tile(x1, y1) == 'O' or self.get_tile(x2, y2) == 'O':
            return True
        if x1 == x2 and y1 == y2:
            return False

        x, y = x1, y1
        dx = x2 - x1
        dy = y2 - y1

        step_x = 1 if dx > 0 else -1 if dx < 0 else 0
        step_y = 1 if dy > 0 else -1 if dy < 0 else 0

        t_max_x = (x + 0.5 * step_x - x1) / dx if dx != 0 else float('inf')
        t_max_y = (y + 0.5 * step_y - y1) / dy if dy != 0 else float('inf')

        t_delta_x = abs(1 / dx) if dx != 0 else float('inf')
        t_delta_y = abs(1 / dy) if dy != 0 else float('inf')

        while (x != x2) or (y != y2):
            if t_max_x < t_max_y:
                t_max_x += t_delta_x
                x += step_x
            else:
                t_max_y += t_delta_y
                y += step_y

            if self.get_tile(x, y) == 'O':
                return True
        return False

    def get_valid_next_states(self, current_pos: tuple, current_vel: tuple) -> list:
        """Enforces acceleration boundaries (normal track vs mandatory grass slowdown)."""
        x_prime, y_prime = current_pos
        vx, vy = current_vel
        current_tile = self.get_tile(x_prime, y_prime)

        if current_tile == 'G':
            # Mandatory deceleration rules on grass (matches PDF Rule III)
            if vx >= 2:
                min_ax, max_ax = -1, -1
            elif vx <= -2:
                min_ax, max_ax = 1, 1
            elif vx == 1:
                min_ax, max_ax = -1, 0
            elif vx == -1:
                min_ax, max_ax = 0, 1
            else:  # vx == 0
                min_ax, max_ax = -1, 1

            if vy >= 2:
                min_ay, max_ay = -1, -1
            elif vy <= -2:
                min_ay, max_ay = 1, 1
            elif vy == 1:
                min_ay, max_ay = -1, 0
            elif vy == -1:
                min_ay, max_ay = 0, 1
            else:  # vy == 0
                min_ay, max_ay = -1, 1
        else:
            min_ax, max_ax = -1, 1
            min_ay, max_ay = -1, 1

        valid_moves = []
        for ax in range(min_ax, max_ax + 1):
            for ay in range(min_ay, max_ay + 1):
                next_vx = vx + ax
                next_vy = vy + ay
                next_x = x_prime + next_vx
                next_y = y_prime + next_vy

                if not self.intersects_obstacle(x_prime, y_prime, next_x, next_y):
                    valid_moves.append(((next_x, next_y), (next_vx, next_vy)))

        return valid_moves


class RaceTrackSolver:
    def __init__(self, env):
        self.env = env
        self.distance_map = self._build_bfs_distance_map()

    def _build_bfs_distance_map(self):
        """BFS distance from every tile to the finish line, over drivable tiles only."""
        dist_map = {}
        for x in range(self.env.width):
            for y in range(self.env.height):
                dist_map[(x, y)] = float('inf')

        queue = []
        for fx, fy in self.env.finishes:
            dist_map[(fx, fy)] = 0
            queue.append((fx, fy))

        while queue:
            curr_x, curr_y = queue.pop(0)
            curr_dist = dist_map[(curr_x, curr_y)]

            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = curr_x + dx, curr_y + dy
                    if self.env.get_tile(nx, ny) in ('T', 'S', 'G', 'F'):
                        if dist_map.get((nx, ny), float('inf')) > curr_dist + 1:
                            dist_map[(nx, ny)] = curr_dist + 1
                            queue.append((nx, ny))

        return dist_map

    def heuristic_distance(self, pos):
        return self.distance_map.get((int(pos[0]), int(pos[1])), float('inf'))

    def get_heuristic_score(self, pos, vel):
        track_dist = self.heuristic_distance(pos)
        speed = abs(vel[0]) + abs(vel[1])
        vel_penalty = 0.5 if speed == 0 else -0.05 * speed
        return track_dist + vel_penalty

    def can_brake_safely(self, pos, vel):
        """Check whether the car can decelerate to a full stop without crashing."""
        cx, cy = pos
        vx, vy = vel
        if (vx == 0 and vy == 0) or self.env.get_tile(cx, cy) == 'F':
            return True
        while vx != 0 or vy != 0:
            ax = -1 if vx > 0 else (1 if vx < 0 else 0)
            ay = -1 if vy > 0 else (1 if vy < 0 else 0)
            vx += ax
            vy += ay
            nx, ny = cx + vx, cy + vy
            if self.env.intersects_obstacle(cx, cy, nx, ny):
                return False
            cx, cy = nx, ny
            if self.env.get_tile(cx, cy) == 'F':
                return True
        return True

    def _choose_move(self, current_pos, current_vel, visited, top_k=3, explore_prob=0.15):
        """Shared move-selection logic: filters by safety + cycle-avoidance, then greedy/random pick."""
        valid_moves = self.env.get_valid_next_states(current_pos, current_vel)
        if not valid_moves:
            return None

        safe_moves = [m for m in valid_moves if self.can_brake_safely(m[0], m[1])]
        if safe_moves:
            valid_moves = safe_moves

        non_repeat = [m for m in valid_moves if (m[0], m[1]) not in visited]
        if non_repeat:
            valid_moves = non_repeat

        valid_moves.sort(key=lambda m: self.get_heuristic_score(m[0], m[1]))

        if len(valid_moves) > 1 and random.random() < explore_prob:
            return random.choice(valid_moves[:top_k])
        return valid_moves[0]

    def generate_randomized_greedy_path(self, max_steps=2000):
        """Construction heuristic: randomized greedy walk (85% greedy / 15% exploration),
        with cycle-avoidance via a visited-state set."""
        if not self.env.starts:
            return None

        current_pos = random.choice(self.env.starts)
        current_vel = (0, 0)
        path = [current_pos]
        visited = set()

        for _ in range(max_steps):
            if self.env.get_tile(current_pos[0], current_pos[1]) == 'F':
                return path

            visited.add((current_pos, current_vel))

            move = self._choose_move(current_pos, current_vel, visited)
            if move is None:
                return None

            current_pos, current_vel = move
            path.append(current_pos)

        return None

    def reconstruct_velocity(self, path):
        """Recompute velocity at the end of `path` from its last two positions."""
        if len(path) < 2:
            return (0, 0)
        return (path[-1][0] - path[-2][0], path[-1][1] - path[-2][1])

    def continue_greedy_from(self, path, max_steps):
        """Continue a randomized greedy walk from the end of `path` (used by VNS shaking)."""
        current_pos = path[-1]
        current_vel = self.reconstruct_velocity(path)
        visited = {(p, self.reconstruct_velocity(path[:i + 1])) for i, p in enumerate(path)}

        for _ in range(max_steps):
            if self.env.get_tile(current_pos[0], current_pos[1]) == 'F':
                return path

            visited.add((current_pos, current_vel))

            move = self._choose_move(current_pos, current_vel, visited, top_k=2)
            if move is None:
                return None

            current_pos, current_vel = move
            path.append(current_pos)

        return None

    def vns_improve(self, path, k_max=5, shake_tries=20):
        """Basic VNS: shake by truncating the path at increasing depth k, re-grow greedily,
        accept if the result is shorter (restart neighborhood index on improvement)."""
        best_path = path
        best_score = len(path) - 1

        k = 1
        while k <= k_max:
            improved = False
            for _ in range(shake_tries):
                cut_point = max(1, len(best_path) - k * 2)
                candidate = best_path[:cut_point]
                candidate = self.continue_greedy_from(candidate, max_steps=300)
                if candidate:
                    score = len(candidate) - 1
                    if score < best_score:
                        best_path, best_score = candidate, score
                        improved = True
                        break
            k = 1 if improved else k + 1

        return best_path, best_score

    def solve(self, iterations=2000, verbose=True):
        if verbose:
            print(f"Executing Metaheuristic optimization over {iterations} loops...")
        start_time = time.time()

        best_path = None
        min_moves = float('inf')
        successful_runs = []

        for i in range(iterations):
            path = self.generate_randomized_greedy_path()
            if path:
                score = len(path) - 1
                successful_runs.append(score)
                if score < min_moves:
                    min_moves = score
                    best_path = path
                    if verbose:
                        print(f" -> Iteration {i:4d}: Found faster path! Moves: {min_moves}")

        if best_path:
            best_path, min_moves = self.vns_improve(best_path)

        runtime = time.time() - start_time

        if successful_runs:
            avg_moves = sum(successful_runs) / len(successful_runs)
            variance = sum((x - avg_moves) ** 2 for x in successful_runs) / len(successful_runs)
            dev = math.sqrt(variance)
        else:
            min_moves, avg_moves, dev = "DNF", 0, 0

        return {
            "best_path": best_path,
            "best_score": min_moves,
            "avg_score": avg_moves,
            "std_dev": dev,
            "runtime": runtime,
        }


def save_path_to_csv(path, filename="tripFile.csv"):
    if not path:
        return
    with open(filename, 'w') as f:
        for (x, y) in path:
            f.write(f"{x},{y}\n")
    print(f"\n[Success] Optimized path exported to: '{filename}'")


if __name__ == "__main__":
    pass