import os
import glob
import csv
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
    
    for track_path in sorted(track_files):
        track_filename = os.path.basename(track_path)
        print("=" * 60)
        print(f"SOLVING TRACK: {track_filename}")
        print("=" * 60)
        
        try:
            env = RaceTrackEnv(track_path)
            solver = SALNSSolver(env)
            final_path = solver.solve(iterations=2000)
            
            if final_path:
                base_name = os.path.splitext(track_filename)[0]
                solution_path = os.path.join(solutions_dir, f"{base_name}_solution.csv")
                
                with open(solution_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    for (x, y, vx, vy) in final_path:
                        writer.writerow([x, y])
                print(f"-> Success! Path of {len(final_path)} steps exported to {solution_path}")
            else:
                print("-> Error: No valid solution pathway could be mapped.")
        except Exception as e:
            print(f"-> Critical execution crash: {str(e)}")
        print("\n")

if __name__ == "__main__":
    process_all_tracks()