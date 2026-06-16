import os
import glob
import csv
import time
import platform
import psutil
import math
from racetrack_env import RaceTrackEnv
from racetrack_solver import SALNSSolver

def process_all_tracks():
    tracks_dir = "tracks"
    solutions_dir = "solutions"
    
    if not os.path.exists(solutions_dir): os.makedirs(solutions_dir)
    if not os.path.exists(tracks_dir): return

    track_files = glob.glob(os.path.join(tracks_dir, "*.t"))
    if not track_files: return
    
    print(f"Found {len(track_files)} map(s). Running Simulated Annealing LNS Suite...\n")

    # ================= SYSTEM SPECS =================
    print("\n===== SYSTEM SPECS =====")
    print(f"OS: {platform.system()} {platform.release()}")
    print(f"CPU: {platform.processor()}")
    print(f"RAM: {round(psutil.virtual_memory().total / (1024**3), 2)} GB")
    print("========================\n")
    
    runtimes = []
    solution_lengths = []

    for track_path in sorted(track_files):
        track_filename = os.path.basename(track_path)
        print("=" * 60)
        print(f"SOLVING TRACK: {track_filename}")
        print("=" * 60)
        
        try:
            env = RaceTrackEnv(track_path)
            solver = SALNSSolver(env)

            start_time = time.time()
            final_path = solver.solve(iterations=2000)
            runtime = time.time() - start_time

            runtimes.append(runtime)

            if final_path:
                solution_lengths.append(len(final_path))

                base_name = os.path.splitext(track_filename)[0]
                solution_path = os.path.join(solutions_dir, f"{base_name}_solution.csv")
                
                with open(solution_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    for (x, y, vx, vy) in final_path:
                        writer.writerow([x, y])

                print(f"-> Success! Path of {len(final_path)} steps exported to {solution_path}")
                print(f"-> Runtime: {runtime:.4f} sec")
            else:
                print("-> Error: No valid solution pathway could be mapped.")

        except Exception as e:
            print(f"-> Critical execution crash: {str(e)}")

        print("\n")

    # ================= FINAL STATISTICS =================
    if solution_lengths:
        best = min(solution_lengths)
        avg = sum(solution_lengths) / len(solution_lengths)
        dev = math.sqrt(sum((x - avg) ** 2 for x in solution_lengths) / len(solution_lengths))
        
        print("\n===== RESULT TABLES =====")
        print(f"Best solution length: {best}")
        print(f"Average solution length: {avg:.2f}")
        print(f"Std deviation: {dev:.2f}")
        print(f"Average runtime: {sum(runtimes)/len(runtimes):.4f} sec")
        print("=========================\n")


if __name__ == "__main__":
    process_all_tracks()