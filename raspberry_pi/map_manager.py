import json
import heapq
from typing import Optional
from config import CELL_SIZE, DIRECTION_MAP


class MapManager:
    """Loads the map, finds a path with A*, generates waypoints"""

    def __init__(self):
        self.grid = []
        self.rows = 0
        self.cols = 0
        self.start = (0, 0)
        self.end = (0, 0)
        self.path = []
        self.waypoints = []

    def load_from_file(self, filepath):
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)

            self.grid = data["map"]
            self.rows = len(self.grid)
            self.cols = len(self.grid[0]) if self.rows > 0 else 0
            self.start = tuple(data["start"])
            self.end = tuple(data["end"])

            print(f"[MAP] loaded: {self.rows}x{self.cols}, "
                  f"start={self.start}, end={self.end}")
            return True
        except Exception as e:
            print(f"[MAP] error: {e}")
            return False

    def load_from_grid(self, grid, start, end):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.start = start
        self.end = end

    def find_path(self):
        """Find the shortest path using A*"""
        if not self.grid:
            print("[MAP] map not loaded")
            return []

        start, end = self.start, self.end

        if not self._is_valid(*start) or not self._is_valid(*end):
            print(f"[MAP] invalid start or end point")
            return []

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, end)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == end:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                self.path = path
                print(f"[MAP] path found, {len(path)} steps")
                return path

            for nb in self._get_neighbors(current):
                g = g_score[current] + 1
                if g < g_score.get(nb, float('inf')):
                    came_from[nb] = current
                    g_score[nb] = g
                    f_score[nb] = g + self._heuristic(nb, end)
                    heapq.heappush(open_set, (f_score[nb], nb))

        print("[MAP] no path found!")
        self.path = []
        return []

    def generate_waypoints(self):
        """Merge consecutive steps in the same direction into one waypoint"""
        if len(self.path) < 2:
            return []

        waypoints = []
        i = 0

        while i < len(self.path) - 1:
            current = self.path[i]
            dr = self.path[i + 1][0] - current[0]
            dc = self.path[i + 1][1] - current[1]
            direction = (dr, dc)

            if direction not in DIRECTION_MAP:
                i += 1
                continue

            heading = DIRECTION_MAP[direction]

            # count consecutive steps in this direction
            steps = 0
            j = i
            while j < len(self.path) - 1:
                ndr = self.path[j + 1][0] - self.path[j][0]
                ndc = self.path[j + 1][1] - self.path[j][1]
                if (ndr, ndc) == direction:
                    steps += 1
                    j += 1
                else:
                    break

            dist = steps * CELL_SIZE
            target = self.path[i + steps]
            waypoints.append({
                "from": current,
                "to": target,
                "heading": heading,
                "distance": dist,
                "steps": steps,
            })
            i += steps

        self.waypoints = waypoints
        print(f"[MAP] {len(waypoints)} waypoints")
        for idx, wp in enumerate(waypoints):
            print(f"  [{idx}] {wp['from']} -> {wp['to']}, "
                  f"{wp['heading']}°, {wp['distance']}m")
        return waypoints

    def print_map(self, path_overlay=None):
        path_set = set(path_overlay or self.path)
        print()
        for r in range(self.rows):
            row_str = ""
            for c in range(self.cols):
                pos = (r, c)
                if pos == self.start:
                    row_str += " S"
                elif pos == self.end:
                    row_str += " E"
                elif pos in path_set:
                    row_str += " *"
                elif self.grid[r][c] == 1:
                    row_str += " ."
                else:
                    row_str += " #"
            print(row_str)
        print()

    def _heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _get_neighbors(self, pos):
        result = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = pos[0] + dr, pos[1] + dc
            if self._is_valid(nr, nc):
                result.append((nr, nc))
        return result

    def _is_valid(self, row, col):
        return (0 <= row < self.rows and
                0 <= col < self.cols and
                self.grid[row][col] == 1)
