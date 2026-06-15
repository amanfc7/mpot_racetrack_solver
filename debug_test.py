with open("programmingExercise/track_05.t") as f:
    ts = f.read()
from racetrack_solver import RaceTrackEnv, RaceTrackSolver
env = RaceTrackEnv(ts)
print(env.height, env.width, env.starts, env.finishes)
solver = RaceTrackSolver(env)

path = solver.generate_randomized_greedy_path(max_steps=1000)
print("path len:", len(path) if path else None)

print("dist at (13,4):", solver.heuristic_distance((13,4)))
print("dist at start:", solver.heuristic_distance(env.starts[0]))