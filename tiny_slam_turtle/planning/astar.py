import heapq
import numpy as np

def astar(binary_grid, start, goal, allow_diag=False):
    """A* on a binary occupancy grid.
    binary_grid: 2D np.array, 1 = obstacle, 0 = free
    start, goal: (y, x) indices in grid space
    allow_diag: whether to allow 8-connected moves
    Returns: list of (y, x) from start to goal (inclusive), or [] if none.
    """
    H, W = binary_grid.shape
    sy, sx = start; gy, gx = goal
    if not (0 <= sy < H and 0 <= sx < W and 0 <= gy < H and 0 <= gx < W):
        return []
    if binary_grid[sy, sx] == 1 or binary_grid[gy, gx] == 1:
        return []

    nbrs = [(-1,0),(1,0),(0,-1),(0,1)]
    if allow_diag:
        nbrs += [(-1,-1),(-1,1),(1,-1),(1,1)]

    def h(y, x):
        # Manhattan or Chebyshev if diag
        if allow_diag:
            return max(abs(y-gy), abs(x-gx))
        return abs(y-gy) + abs(x-gx)

    g = { (sy,sx): 0 }
    came = {}
    pq = [(h(sy,sx), 0, (sy,sx))]  # (f, g, node)

    while pq:
        f, gc, (y, x) = heapq.heappop(pq)
        if (y, x) == (gy, gx):
            # reconstruct
            path = [(y, x)]
            while (y, x) in came:
                y, x = came[(y, x)]
                path.append((y, x))
            path.reverse()
            return path

        for dy, dx in nbrs:
            ny, nx = y+dy, x+dx
            if not (0 <= ny < H and 0 <= nx < W): continue
            if binary_grid[ny, nx] == 1: continue
            ng = gc + (1.4142 if (dy!=0 and dx!=0) else 1.0)
            if (ny, nx) not in g or ng < g[(ny, nx)]:
                g[(ny, nx)] = ng
                came[(ny, nx)] = (y, x)
                nf = ng + h(ny, nx)
                heapq.heappush(pq, (nf, ng, (ny, nx)))
    return []
