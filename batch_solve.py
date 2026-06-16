# main.py
import csv
from racetrack_env import RaceTrackEnv
from racetrack_solver import LNSSolver

def run():
    # Load
    env = RaceTrackEnv('track_05.t')
    solver = LNSSolver(env)
    
    # Solve
    # Assuming Start (S) and Finish (F) finding logic
    start_pos = (1, 1) # Find these from grid
    end_pos = (100, 50)
    
    path = solver.solve(start_pos, end_pos)
    
    # Output
    with open('my_solution.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        for step in path:
            writer.writerow(step)
    
    print("Optimization finished. File saved to my_solution.csv")

if __name__ == "__main__":
    run()