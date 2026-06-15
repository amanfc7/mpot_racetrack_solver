import os
import time
from racetrack_solver import RaceTrackEnv, RaceTrackSolver

# class RaceTrackEnv:
#     def __init__(self, file_path: str):
#         """ Read the track layout from the text file."""
#         with open(file_path, 'r') as f:
#             lines = f.readlines()
            
#         self.grid = [list(line.strip()) for line in lines if line.strip()]
#         self.height = len(self.grid)
#         self.width = len(self.grid[0]) if self.height > 0 else 0
        
#         self.starts = []
#         self.finishes = []
        
#         for y in range(self.height):
#             for x in range(self.width):
#                 char = self.grid[y][x]
#                 if char == 'S':
#                     self.starts.append((x, y))
#                 elif char == 'F':
#                     self.finishes.append((x, y))

#     def get_tile(self, x: int, y: int) -> str:
#         if 0 <= x < self.width and 0 <= y < self.height:
#             return self.grid[y][x]
#         return 'O'

#     def intersects_obstacle(self, x1: int, y1: int, x2: int, y2: int) -> bool:
#         """ Rapid grid traversal (Amanatides-Woo) to flag collisions."""
#         if self.get_tile(x1, y1) == 'O' or self.get_tile(x2, y2) == 'O':
#             return True
#         if x1 == x2 and y1 == y2:
#             return False

#         x, y = x1, y1
#         dx = x2 - x1
#         dy = y2 - y1
        
#         step_x = 1 if dx > 0 else -1 if dx < 0 else 0
#         step_y = 1 if dy > 0 else -1 if dy < 0 else 0
        
#         t_max_x = (x + 0.5 * step_x - x1) / dx if dx != 0 else float('inf')
#         t_max_y = (y + 0.5 * step_y - y1) / dy if dy != 0 else float('inf')
        
#         t_delta_x = abs(1 / dx) if dx != 0 else float('inf')
#         t_delta_y = abs(1 / dy) if dy != 0 else float('inf')
        
#         while (x != x2) or (y != y2):
#             if t_max_x < t_max_y:
#                 t_max_x += t_delta_x
#                 x += step_x
#             else:
#                 t_max_y += t_delta_y
#                 y += step_y
                
#             if self.get_tile(x, y) == 'O':
#                 return True
#         return False

#     def get_valid_next_states(self, current_pos: tuple, current_vel: tuple) -> list:
#         """Enforces acceleration boundaries for normal tracks vs mandatory Grass slowdowns."""
#         x_prime, y_prime = current_pos
#         vx, vy = current_vel
#         current_tile = self.get_tile(x_prime, y_prime)
        
#         if current_tile == 'G':
#             min_ax = -1 if vx >= 1 else -1
#             max_ax = -1 if vx >= 2 else (0 if vx == 1 else 1)
#             min_ay = -1 if vy >= 1 else -1
#             max_ay = -1 if vy >= 2 else (0 if vy == 1 else 1)
#         else:
#             min_ax, max_ax = -1, 1
#             min_ay, max_ay = -1, 1

#         valid_moves = []
#         for ax in range(min_ax, max_ax + 1):
#             for ay in range(min_ay, max_ay + 1):
#                 next_vx = vx + ax
#                 next_vy = vy + ay
#                 next_x = x_prime + next_vx
#                 next_y = y_prime + next_vy
                
#                 if not self.intersects_obstacle(x_prime, y_prime, next_x, next_y):
#                     valid_moves.append(((next_x, next_y), (next_vx, next_vy)))
                    
#         return valid_moves


# class RaceTrackSolver:
#     def __init__(self, env):
#         self.env = env
#         self.distance_map = {}
#         self._generate_flow_field()

#     def _generate_flow_field(self):
#         """
#         Generates a wall-aware distance map starting from the finish line.
#         This provides a smart heuristic guiding our metaheuristic around track walls.
#         """
#         queue = deque()
#         for x, y in self.env.finishes:
#             self.distance_map[(x, y)] = 0
#             queue.append((x, y))

#         while queue:
#             cx, cy = queue.popleft()
#             current_dist = self.distance_map[(cx, cy)]

#             # Check 8-way directional movements on the track grid map
#             for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
#                 nx, ny = cx + dx, cy + dy
#                 if 0 <= nx < self.env.width and 0 <= ny < self.env.height:
#                     if self.env.get_tile(nx, ny) != 'O' and (nx, ny) not in self.distance_map:
#                         self.distance_map[(nx, ny)] = current_dist + 1
#                         queue.append((nx, ny))

#     def get_heuristic_score(self, pos, vel):
#         """Combines structural track distance with velocity alignment."""
#         track_dist = self.distance_map.get(pos, 99999)
        
#         # Penalize zero velocity states to encourage forward progression
#         vel_penalty = 0.5 if (vel[0] == 0 and vel[1] == 0) else 0.0
#         return track_dist + vel_penalty

#     def generate_randomized_greedy_path(self, max_steps=300):
#         if not self.env.starts:
#             return None
#         current_pos = random.choice(self.env.starts)
#         current_vel = (0, 0)
#         path = [current_pos]
        
#         for _ in range(max_steps):
#             if self.env.get_tile(current_pos[0], current_pos[1]) == 'F':
#                 return path

#             valid_moves = self.env.get_valid_next_states(current_pos, current_vel)
#             if not valid_moves:
#                 return None  # Crashed due to momentum rules

#             # Evaluate options using our custom wall-aware flow field
#             valid_moves.sort(key=lambda m: self.get_heuristic_score(m[0], m[1]))
            
#             # GRASP Metaheuristic: Exploit the best path alignment, explore alternates occasionally
#             if len(valid_moves) > 1 and random.random() < 0.15:
#                 chosen_move = random.choice(valid_moves[:2]) 
#             else:
#                 chosen_move = valid_moves[0]

#             current_pos = chosen_move[0]
#             current_vel = chosen_move[1]
#             path.append(current_pos)

#         return None

#     def solve(self, iterations=2500):
#         start_time = time.time()
#         best_path = None
#         min_moves = float('inf')
#         successful_runs = []

#         for _ in range(iterations):
#             path = self.generate_randomized_greedy_path()
#             if path:
#                 score = len(path) - 1
#                 successful_runs.append(score)
#                 if score < min_moves:
#                     min_moves = score
#                     best_path = path

#         runtime = time.time() - start_time
        
#         if successful_runs:
#             avg_moves = sum(successful_runs) / len(successful_runs)
#             variance = sum((x - avg_moves) ** 2 for x in successful_runs) / len(successful_runs)
#             dev = math.sqrt(variance)
#         else:
#             min_moves, avg_moves, dev = "DNF", 0, 0

#         return {
#             "best_path": best_path,
#             "best_score": min_moves,
#             "avg_score": avg_moves,
#             "std_dev": dev,
#             "runtime": runtime
#         }


def save_path_to_csv(path, filename):
    if not path:
        return
    with open(filename, 'w') as f:
        for (x, y) in path:
            f.write(f"{x},{y}\n")


# Main Batch Process Execution
if __name__ == "__main__":
    script_directory = os.path.dirname(os.path.abspath(__file__))
    tracks_directory = os.path.join(script_directory, "programmingExercise")
    
    track_files = []
    if os.path.exists(tracks_directory):
        for item in os.listdir(tracks_directory):
            full_path = os.path.join(tracks_directory, item)
            if os.path.isfile(full_path) and not item.endswith('.csv') and not item.startswith('.'):
                track_files.append(full_path)
    
    track_files = sorted(track_files)

    if not track_files:
        print(f"[Error] No valid track files discovered in absolute path: '{tracks_directory}'")
        exit()

    print(f"Targeting: {tracks_directory}")
    print(f"Found {len(track_files)} target maps to optimize. Running structural flow-field optimization...")
    
    summary_results = []

    for file_path in track_files:
        track_name = os.path.basename(file_path)
        print(f" -> Solving: {track_name} ... ", end="", flush=True)
        
        with open(file_path, 'r') as f:
            track_string = f.read()
        env = RaceTrackEnv(track_string)
        solver = RaceTrackSolver(env)
        res = solver.solve(iterations=2500) 
        
        base_name = os.path.splitext(track_name)[0]
        csv_name = f"{base_name}_trip.csv"
        csv_path = os.path.join(tracks_directory, csv_name)
        save_path_to_csv(res["best_path"], csv_path)
        
        summary_results.append({
            "name": track_name,
            "best": res["best_score"],
            "avg": f"{res['avg_score']:.2f}" if res["best_score"] != "DNF" else "N/A",
            "dev": f"{res['std_dev']:.2f}" if res["best_score"] != "DNF" else "N/A",
            "time": f"{res['runtime']:.3f}s"
        })
        print("Done.")

    # --- Print Final Presentation Table ---
    print("\n" + "="*70)
    print("              Benchmarks:        ")
    print("="*70)
    print(f"| {'Track File':<20} | {'Best (Moves)':<12} | {'Avg (Moves)':<12} | {'Dev':<8} | {'Runtime':<10} |")
    print("|" + "-"*22 + "|" + "-"*14 + "|" + "-"*14 + "|" + "-"*10 + "|" + "-"*12 + "|")
    for row in summary_results:
        print(f"| {row['name']:<20} | {row['best']:<12} | {row['avg']:<12} | {row['dev']:<8} | {row['time']:<10} |")
    print("="*70)
    print(f"\nAll solution files (*_trip.csv) have been saved.")