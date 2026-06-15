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
        """ line-segment intersection using grid traversal."""
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
        x_prime, y_prime = current_pos
        vx, vy = current_vel
        current_tile = self.get_tile(x_prime, y_prime)
        
        if current_tile == 'G':
            min_ax = -1 if vx >= 1 else -1
            max_ax = -1 if vx >= 2 else (0 if vx == 1 else 1)
            min_ay = -1 if vy >= 1 else -1
            max_ay = -1 if vy >= 2 else (0 if vy == 1 else 1)
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
        avg_fx = sum(x for x, y in env.finishes) / len(env.finishes)
        avg_fy = sum(y for x, y in env.finishes) / len(env.finishes)
        self.target = (avg_fx, avg_fy)

    def heuristic_distance(self, pos):
        return math.sqrt((pos[0] - self.target[0])**2 + (pos[1] - self.target[1])**2)

    def generate_randomized_greedy_path(self, max_steps=150):
        current_pos = random.choice(self.env.starts)
        current_vel = (0, 0)
        path = [current_pos]
        
        for _ in range(max_steps):
            if self.env.get_tile(current_pos[0], current_pos[1]) == 'F':
                return path

            valid_moves = self.env.get_valid_next_states(current_pos, current_vel)
            if not valid_moves:
                return None 

            # Sort by distance to finish line
            valid_moves.sort(key=lambda m: self.heuristic_distance(m[0]))
            
            # GRASP Metaheuristic element: 85% greedy exploitation, 15% exploration
            if len(valid_moves) > 1 and random.random() < 0.15:
                chosen_move = random.choice(valid_moves[:3]) 
            else:
                chosen_move = valid_moves[0]

            current_pos = chosen_move[0]
            current_vel = chosen_move[1]
            path.append(current_pos)

        return None

    def solve(self, iterations=2000):
        print(f"Executing Metaheuristic optimization over {iterations} loops...")
        start_time = time.time()
        
        best_path = None
        min_moves = float('inf')
        successful_runs = []

        for i in range(iterations):
            path = self.generate_randomized_greedy_path()
            if path:
                # Number of moves is path length minus the starting position node
                score = len(path) - 1
                successful_runs.append(score)
                if score < min_moves:
                    min_moves = score
                    best_path = path
                    print(f" -> Iteration {i:4d}: Found faster path! Moves: {min_moves}")

        runtime = time.time() - start_time
        
        if successful_runs:
            avg_moves = sum(successful_runs) / len(successful_runs)
            variance = sum((x - avg_moves) ** 2 for x in successful_runs) / len(successful_runs)
            dev = math.sqrt(variance)
        else:
            avg_moves, dev = 0, 0

        return {
            "best_path": best_path,
            "best_score": min_moves,
            "avg_score": avg_moves,
            "std_dev": dev,
            "runtime": runtime
        }

def save_path_to_csv(path, filename="tripFile.csv"):
    if not path:
        return
    with open(filename, 'w') as f:
        for (x, y) in path:
            f.write(f"{x},{y}\n")
    print(f"\n[Success] Optimized path exported to: '{filename}'")

# Execution Entry Point 
    # Sample layout layout containing: Start (S), Track (T), Grass (G), Finish (F)
    sample_track = """
    OOOOOOOOOOOOOOOOOOOO
    OSSSSSSTTTTTTTTTTTFO
    OTTTTTTTTTGGGGGGTTFO
    OTTTTTTTTTGGGGGGTTFO
    OOOOOOOOOOOOOOOOOOOO
    """
    
    # Initialize environment map
    env = RaceTrackEnv(sample_track)
    
    # Run solver
    solver = RaceTrackSolver(env)
    results = solver.solve(iterations=3000)
    
    # Export metrics for submission & presentation
    if results["best_path"]:
        print("\n" + "="*45)
        print("   Benchmarks:  ")
        print("="*45)
        print(f"  Best Solution (Min Moves): {results['best_score']} moves")
        print(f"  Average Solution Score:    {results['avg_score']:.2f} moves")
        print(f"  Standard Deviation (Dev):  {results['std_dev']:.2f}")
        print(f"  Execution Runtime:         {results['runtime']:.4f} seconds")
        print("="*45)
        
        save_path_to_csv(results["best_path"], "tripFile.csv")
    else:
        print("\n[Error] The solver failed to reach the finish line. Adjust layout or iterations.")