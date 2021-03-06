from heapq import heappush, heappop
from math import sqrt


def find_path(src, dest, mesh):
    path = []
    visited = []

    # src_box = dest_box = None
    #
    # for box in mesh['boxes']:
    #
    # if src[0] in range(box[0], box[1]):
    # if src[1] in range(box[2], box[3]):
    # src_box = box
    #             print 'Source in box ' + str(src_box)
    #     if dest[0] in range(box[0], box[1]):
    #         if dest[1] in range(box[2], box[3]):
    #             dest_box = box
    #             print 'Destination in box ' + str(dest_box)
    #
    #
    # """Straight line on print-out"""
    #
    # path.append((src, dest))
    #
    # if (src_box != None):
    #    visited.append(src_box)
    # if (dest_box != None):
    #    visited.append(dest_box)

    # path, visited = bfs(src, dest, mesh, adj_boxes)
    # path, visited = dijkstras_shortest_path(src, dest, mesh, adj_boxes)
    # path, visited = a_star(src, dest, mesh, adj_boxes, euclidean_distance)
    path, visited = bidirectional_a_star(src, dest, mesh, adj_boxes, euclidean_distance)

    if not path:
        print("No path")

    return path, visited


def bfs(source, destination, graph, adj):
    src_box = dest_box = None

    for box in graph['boxes']:
        if source[0] in range(box[0], box[1]):
            if source[1] in range(box[2], box[3]):
                src_box = box
        if destination[0] in range(box[0], box[1]):
            if destination[1] in range(box[2], box[3]):
                dest_box = box

    if src_box is None or dest_box is None:
        visited = []
        if src_box is not None:
            visited.append(src_box)
        if dest_box is not None:
            visited.append(dest_box)
        return [], visited

    prev = {}
    detail_points = {}

    queue = [src_box]
    prev[src_box] = None
    detail_points[src_box] = source

    while queue:
        node = heappop(queue)

        if node == dest_box:
            break

        neighbors = adj(graph, node)

        for next in neighbors:
            if next not in prev:
                prev[next] = node
                constrained_x = max(min(detail_points[node][0], next[1]), next[0])
                constrained_y = max(min(detail_points[node][1], next[3]), next[2])
                detail_points[next] = (constrained_x, constrained_y)
                heappush(queue, next)

    visited = []
    for n in prev:
        visited.append(n)

    if node == dest_box:
        path = []
        while prev[node]:
            # box_center = ((node[1] + node[0])//2, (node[3] + node[2])//2)
            # prev_center = ((prev[node][1] + prev[node][0])//2, (prev[node][3] + prev[node][2])//2)
            path.append((detail_points[node], detail_points[prev[node]]))
            node = prev[node]
        path.append((detail_points[dest_box], destination))
        path.reverse()

        return path, visited
    else:
        return [], visited


def dijkstras_shortest_path(source, destination, graph, adj):
    src_box = dest_box = None

    for box in graph['boxes']:
        if source[0] in range(box[0], box[1]):
            if source[1] in range(box[2], box[3]):
                src_box = box
        if destination[0] in range(box[0], box[1]):
            if destination[1] in range(box[2], box[3]):
                dest_box = box

    if src_box is None or dest_box is None:
        visited = []
        if src_box is not None:
            visited.append(src_box)
        if dest_box is not None:
            visited.append(dest_box)
        return [], visited

    dist = {}
    prev = {}
    detail_points = {}

    queue = [(0, src_box)]
    dist[src_box] = 0
    prev[src_box] = None
    detail_points[src_box] = source

    while queue:
        d, node = heappop(queue)

        if node == dest_box:
            break

        neighbors = adj(graph, node)

        for next in neighbors:
            constrained_x = max(min(detail_points[node][0], next[1]), next[0])
            constrained_y = max(min(detail_points[node][1], next[3]), next[2])
            next_dist = euclidean_distance((constrained_x, constrained_y), detail_points[node])
            new_cost = dist[node] + next_dist

            if next not in prev or new_cost < dist[next]:
                prev[next] = node
                dist[next] = new_cost
                detail_points[next] = (constrained_x, constrained_y)
                heappush(queue, (dist[next], next))

    visited = []
    for n in prev:
        visited.append(n)

    if node == dest_box:
        path = []
        while prev[node]:
            path.append((detail_points[node], detail_points[prev[node]]))
            node = prev[node]
        path.append((detail_points[dest_box], destination))
        path.reverse()
        return path, visited
    else:
        return [], visited


def a_star(source, destination, graph, adj, heuristic):
    src_box = dest_box = None

    for box in graph['boxes']:
        if source[0] in range(box[0], box[1]):
            if source[1] in range(box[2], box[3]):
                src_box = box
        if destination[0] in range(box[0], box[1]):
            if destination[1] in range(box[2], box[3]):
                dest_box = box

    if src_box is None or dest_box is None:
        visited = []
        if src_box is not None:
            visited.append(src_box)
        if dest_box is not None:
            visited.append(dest_box)
        return [], visited

    dist = {}
    prev = {}
    detail_points = {}

    queue = [(0, src_box)]
    dist[src_box] = 0
    prev[src_box] = None
    detail_points[src_box] = source

    while queue:
        d, node = heappop(queue)

        if node is dest_box:
            break

        neighbors = adj(graph, node)

        for next in neighbors:
            constrained_x = max(min(detail_points[node][0], next[1]), next[0])
            constrained_y = max(min(detail_points[node][1], next[3]), next[2])
            next_point = (constrained_x, constrained_y)
            next_dist = euclidean_distance(next_point, detail_points[node])
            remaining_estimate = heuristic(next_point, destination)
            new_cost = dist[node] + next_dist

            if next not in prev or new_cost < dist[next]:
                prev[next] = node
                dist[next] = new_cost
                detail_points[next] = next_point
                heappush(queue, (dist[next] + remaining_estimate, next))

    visited = []
    for n in prev:
        visited.append(n)

    if node == dest_box:
        path = []
        while prev[node]:
            path.append((detail_points[node], detail_points[prev[node]]))
            node = prev[node]
        path.append((detail_points[dest_box], destination))
        path.reverse()
        return path, visited
    else:
        return [], visited


def bidirectional_a_star(source, destination, graph, adj, heuristic):
    src_box = dest_box = None

    for box in graph['boxes']:
        if source[0] in range(box[0], box[1]):
            if source[1] in range(box[2], box[3]):
                src_box = box
        if destination[0] in range(box[0], box[1]):
            if destination[1] in range(box[2], box[3]):
                dest_box = box

    if src_box is None or dest_box is None:
        visited = []
        if src_box is not None:
            visited.append(src_box)
        if dest_box is not None:
            visited.append(dest_box)
        return [], visited

    forward_dist = {}
    backward_dist = {}
    forward_prev = {}
    backward_prev = {}
    detail_points = {}  # don't need to duplicate this one since each node will only be found by one search

    queue = [(0, src_box, 'dest'), (0, dest_box, 'src')]
    forward_dist[src_box] = 0
    backward_dist[dest_box] = 0
    forward_prev[src_box] = None
    backward_prev[dest_box] = None
    detail_points[src_box] = source
    detail_points[dest_box] = destination

    while queue:
        p, node, goal = heappop(queue)

        current_dest = dest_box
        current_dist = forward_dist
        current_prev = forward_prev
        other_prev = backward_prev
        if goal == 'src':
            current_dest = src_box
            current_dist = backward_dist
            current_prev = backward_prev
            other_prev = forward_prev

        if node in other_prev:
            break

        neighbors = adj(graph, node)

        for next in neighbors:
            constrained_x = max(min(detail_points[node][0], next[1]), next[0])
            constrained_y = max(min(detail_points[node][1], next[3]), next[2])
            next_point = (constrained_x, constrained_y)
            next_dist = euclidean_distance(next_point, detail_points[node])
            remaining_estimate = heuristic(next_point, current_dest)
            new_cost = current_dist[node] + next_dist

            if next not in current_prev or new_cost < current_dist[next]:
                current_prev[next] = node
                current_dist[next] = new_cost
                detail_points[next] = next_point
                heappush(queue, (current_dist[next] + remaining_estimate, next, goal))

    visited = []
    for n in forward_prev:
        visited.append(n)
    for n in backward_prev:  # will probably duplicate 1 box
        visited.append(n)

    if (node in forward_prev and node in backward_prev) or (node in forward_prev is node in backward_prev):
        path = []

        current_node = node
        while forward_prev[current_node]:
            path.append((detail_points[current_node], detail_points[forward_prev[current_node]]))
            current_node = forward_prev[current_node]

        current_node = node
        while backward_prev[current_node]:
            path.append((detail_points[current_node], detail_points[backward_prev[current_node]]))
            current_node = backward_prev[current_node]

        # path.append((detail_points[dest_box], destination))
        path.reverse()
        return path, visited
    else:
        return [], visited


def adj_boxes(mesh, box):
    adjacent_nodes = mesh['adj'].get(box, [])
    return adjacent_nodes


def euclidean_distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return sqrt(dx * dx + dy * dy)


__author__ = 'Alec Noble and Jesse Smith'
