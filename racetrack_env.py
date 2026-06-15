import math

class RaceTrackEnv:
    def __init__(self, track_string: str):
        """
        Parse the character-based matrix text file.
        Coordinates: x = column index, y = row index (0,0 is top-left).
        """
        self.grid = [list(line.strip()) for line in track_string.strip().split('\n') if line.strip()]
        self.height = len(self.grid)
        self.width = len(self.grid[0]) if self.height > 0 else 0
        
        self.starts = []
        self.finishes = []
        
        # Pre-map coordinates for rapid lookup
        for y in range(self.height):
            for x in range(self.width):
                char = self.grid[y][x]
                if char == 'S':
                    self.starts.append((x, y))
                elif char == 'F':
                    self.finishes.append((x, y))

    def get_tile(self, x: int, y: int) -> str:
        """Returns the tile type at (x, y). Out of bounds counts as an Obstacle (crash)."""
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[y][x]
        return 'O'  # Out of bounds treated as Obstacle

    def intersects_obstacle(self, x1: int, y1: int, x2: int, y2: int) -> bool:
        """
        Line-segment intersection using grid traversal.
        Checks if the line from center (x1, y1) to center (x2, y2) crosses any 'O'.
        """
        # Quick check for endpoints
        if self.get_tile(x1, y1) == 'O' or self.get_tile(x2, y2) == 'O':
            return True
        if x1 == x2 and y1 == y2:
            return False

        # Ray-casting parameters matching the cell boundaries (shifted by 0.5)
        x, y = x1, y1
        dx = x2 - x1
        dy = y2 - y1
        
        step_x = 1 if dx > 0 else -1 if dx < 0 else 0
        step_y = 1 if dy > 0 else -1 if dy < 0 else 0
        
        # Distance to next cell boundary
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
        """
        Step 2: Physics Engine & Rule Validator.
        Returns a list of valid tuples: ((next_x, next_y), (next_vx, next_vy))
        """
        x_prime, y_prime = current_pos
        vx, vy = current_vel
        current_tile = self.get_tile(x_prime, y_prime)
        
        # Determine acceleration limits based on Track vs Grass
        if current_tile == 'G':
            # Grass deceleration logic
            min_ax = -1 if vx >= 1 else -1
            max_ax = -1 if vx >= 2 else (0 if vx == 1 else 1)
            
            min_ay = -1 if vy >= 1 else -1
            max_ay = -1 if vy >= 2 else (0 if vy == 1 else 1)
        else:
            # Normal Track/Start limits
            min_ax, max_ax = -1, 1
            min_ay, max_ay = -1, 1

        valid_moves = []
        
        # Evaluate all available acceleration vector changes
        for ax in range(min_ax, max_ax + 1):
            for ay in range(min_ay, max_ay + 1):
                next_vx = vx + ax
                next_vy = vy + ay
                next_x = x_prime + next_vx
                next_y = y_prime + next_vy
                
                # Verify that this movement doesn't cause a crash
                if not self.intersects_obstacle(x_prime, y_prime, next_x, next_y):
                    valid_moves.append(((next_x, next_y), (next_vx, next_vy)))
                    
        return valid_moves

# Verification Example 
if __name__ == "__main__":
    track_layout = """
    OOOOOOOOOO
    OSSSSSTTTO
    OTTTTTTTFO
    OTTTGGGTFO
    OOOOOOOOOO
    """
    env = RaceTrackEnv(track_layout)
    print(f"Track Loaded: Width={env.width}, Height={env.height}")
    print(f"Start coordinates: {env.starts}")
    
    # Simulate a car at (1, 1) [Start] with velocity (0,0)
    print("Valid next moves from Start (0,0 velocity):")
    moves = env.get_valid_next_states((1, 1), (0, 0))
    for pos, vel in moves:
        print(f"  -> Pos: {pos}, New Vel: {vel}")