# Semantic Frontier Exploration Pseudocode

function processOccupancyGrid(occupancy_grid, semantic_value_map):
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    resolution = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin.position

    frontiers = []

    for y in range(1, height-1):
        for x in range(1, width-1):
            idx = y * width + x
            if occupancy_grid.data[idx] == FREE:
                if has_unknown_neighbor(x, y, occupancy_grid):
                    frontier_cluster = grow_frontier_from(x, y, occupancy_grid)
                    if frontier_cluster.length > min_frontier_length:
                        frontiers.append(frontier_cluster)

    scored_nodes = []
    for frontier in frontiers:
        centroid = compute_centroid(frontier)
        semantic_score = compute_semantic_score(frontier, semantic_value_map)
        exploration_score = compute_exploration_gain(frontier)
        total_score = semantic_weight * semantic_score + (1 - semantic_weight) * exploration_score
        
        node = create_graph_node(centroid, total_score)
        scored_nodes.append(node)

    publish_graph_nodes(scored_nodes)

function has_unknown_neighbor(x, y, grid):
    for dx, dy in 8-connected-neighborhood:
        nx = x + dx
        ny = y + dy
        if grid[nx, ny] == UNKNOWN:
            return true
    return false

function grow_frontier_from(x, y, grid):
    cluster = []
    queue = [(x, y)]
    while queue not empty:
        cx, cy = queue.pop()
        if is_frontier_point(cx, cy, grid):
            cluster.append((cx, cy))
            queue.extend(all_neighbors(cx, cy) not yet in cluster)
    return cluster

function compute_centroid(points):
    x_mean = mean of all x in points
    y_mean = mean of all y in points
    return (x_mean, y_mean)

function compute_semantic_score(frontier, semantic_map):
    return average(semantic_map[x, y] for (x, y) in frontier)

function create_graph_node(position, score):
    node = GraphNode()
    node.position = position
    node.score = score
    return node

