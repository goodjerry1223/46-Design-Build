import heapq

def a_star_search(maze, start, goal):
    h, w = maze.shape
    heap = []
    heapq.heappush(heap, (0 + abs(start[0] - goal[0]) + abs(start[1] - goal[1]), 0, start))
    visited = set()
    parent = {}
    cost_so_far = {start: 0}
    while heap:
        f, g, cur = heapq.heappop(heap)
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return path
        if cur in visited:
            continue
        visited.add(cur)
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = cur[0]+dx, cur[1]+dy
            next = (nx, ny)
            if 0<=nx<h and 0<=ny<w and maze[nx, ny]==0:
                new_g = g + 1
                if next not in cost_so_far or new_g < cost_so_far[next]:
                    cost_so_far[next] = new_g
                    priority = new_g + abs(nx-goal[0]) + abs(ny-goal[1])
                    heapq.heappush(heap, (priority, new_g, next))
                    parent[next] = cur
    return None
