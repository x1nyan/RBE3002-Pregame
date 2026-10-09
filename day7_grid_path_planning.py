"""Day 7: Find a route through a small occupancy grid with A*.

Run: python day7_grid_path_planning.py

Rows increase downward; columns increase to the right. The robot can move
up, down, left, or right, but cannot move diagonally through obstacles.
"""
from heapq import heappop, heappush
from itertools import count

Cell = tuple[int, int]

GRID = [
    "..........",
    ".####.....",
    ".....#....",
    "...#.#.##.",
    "...#......",
    ".....###..",
    "..........",
]
START: Cell = (0, 0)
GOAL: Cell = (6, 9)


def astar(grid: list[str], start: Cell, goal: Cell) -> list[Cell]:
    """Return the shortest 4-connected path, including endpoints; [] if blocked."""
    if not grid or not grid[0] or any(len(row) != len(grid[0]) for row in grid):
        raise ValueError("grid must be a non-empty rectangle")
    height, width = len(grid), len(grid[0])

    def traversable(cell: Cell) -> bool:
        row, col = cell
        return (0 <= row < height and 0 <= col < width
                and grid[row][col] != "#")

    if not traversable(start) or not traversable(goal):
        return []

    def heuristic(cell: Cell) -> int:
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

    tie_breaker = count()
    frontier = [(heuristic(start), next(tie_breaker), start)]
    came_from: dict[Cell, Cell | None] = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        _, _, current = heappop(frontier)
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return list(reversed(path))

        row, col = current
        for neighbor in ((row - 1, col), (row + 1, col),
                         (row, col - 1), (row, col + 1)):
            if not traversable(neighbor):
                continue
            new_cost = cost_so_far[current] + 1
            if new_cost < cost_so_far.get(neighbor, float("inf")):
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + heuristic(neighbor)
                heappush(frontier, (priority, next(tie_breaker), neighbor))
    return []


def show_path(grid: list[str], path: list[Cell]) -> None:
    cells = set(path)
    for row, line in enumerate(grid):
        print("".join(
            "S" if (row, col) == START else
            "G" if (row, col) == GOAL else
            "*" if (row, col) in cells else char
            for col, char in enumerate(line)
        ))


def main() -> None:
    path = astar(GRID, START, GOAL)
    if not path:
        raise RuntimeError("No path found; check the grid and endpoints")
    assert path[0] == START and path[-1] == GOAL
    assert all(GRID[row][col] != "#" for row, col in path)
    assert all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
               for a, b in zip(path, path[1:]))
    show_path(GRID, path)
    print(f"Path length: {len(path) - 1} steps")


if __name__ == "__main__":
    main()

# Exercises:
# 1. Add a wall to GRID and check how the route changes.
# 2. Make one open cell costly to cross and update A* to use weighted costs.
# 3. Return [] for an unreachable goal and add an assertion for that case.
