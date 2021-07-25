import itertools
import numpy as np
from sklearn.cluster import KMeans
from scipy.sparse.csgraph import dijkstra
import matplotlib.pyplot as plt


def shortest_cycle(pos, max_distance):
    graph = np.sqrt(np.sum(np.square(np.expand_dims(pos, 0) - np.expand_dims(pos, 1)), axis=2))
    graph = np.triu(graph)
    path, min_cost = None, None
    for i in range(1, pos.shape[0]):
        weight = graph[0, i]
        limit = max(max_distance - weight, 0)
        graph[0, i] = 0
        # noinspection PyTupleAssignmentBalance
        dist_matrix, predecessors = dijkstra(graph, directed=False, indices=0, return_predecessors=True, limit=limit)
        graph[0, i] = weight
        cost = dist_matrix[i] + weight
        if not min_cost or cost < min_cost:
            min_cost = cost
            path = [i]
            while predecessors[path[-1]] != -9999:
                path.append(predecessors[path[-1]])
            path.reverse()
    if min_cost == np.inf:
        closest = np.argmin(graph[0, 1:]) + 1
        min_cost = 2 * graph[0, closest]
        if min_cost < max_distance:
            path = [0, closest]
        else:
            min_cost = np.inf
            path = None
    return path, min_cost


def best_route(pos, max_distance):
    # path, cost = shortest_cycle(pos, max_distance)
    # return pos[path], cost
    n = pos.shape[0]
    routes = []
    costs = []
    for perm in itertools.permutations(range(1, n), n - 1):
        pos_perm = np.concatenate([np.expand_dims(pos[0], 0), pos[list(perm)]], axis=0)
        route = []
        total_dist = 0
        final_dist_0 = 0
        for i in range(1, n):
            dist_0 = np.sqrt(np.sum(np.square(pos_perm[i])))
            dist = np.sqrt(np.sum(np.square(pos_perm[i] - pos_perm[i - 1])))
            if total_dist + dist + dist_0 > max_distance:
                break
            final_dist_0 = dist_0
            total_dist += dist
            route.append(perm[i - 1] - 1)
        total_dist += final_dist_0
        routes.append(route)
        costs.append(total_dist)
    costs = np.array(costs)
    lengths = np.array([len(route) for route in routes])
    max_len = np.max(lengths)
    costs = costs[lengths == max_len]
    routes = [routes[i] for i, take in enumerate(lengths == max_len) if take]
    best = np.argmin(costs)
    return routes[best], costs[best]


def main():
    pos = np.random.randint(low=-100, high=100, size=(20, 2))
    max_distance = 200
    n_clusters = 5
    start = np.zeros((1, 2))
    k_means = KMeans(n_clusters=n_clusters)
    clusters = k_means.fit_predict(pos)
    # plt.scatter(pos[:, 0], pos[:, 1], c=clusters)
    # for cluster in range(n_clusters):
    #     rel = pos[clusters == cluster]
    #     cycle = np.concatenate([start, rel], axis=0)
    #     path_pos, path_cost = best_route(cycle, max_distance=max_distance)
    #     route = np.concatenate([start, rel[np.array(path_pos)], start], axis=0)
    #     plt.plot(route[:, 0], route[:, 1], label=f'{path_cost:.2f}m')
    # plt.legend()
    # plt.show()
    while pos.shape[0]:
        path, cost, selected_cluster = None, None, None
        for cluster in range(n_clusters):
            rel = pos[clusters == cluster]
            if not rel.shape[0]:
                continue
            cycle = np.concatenate([start, rel], axis=0)
            path_pos, path_cost = best_route(cycle, max_distance=max_distance)
            if not path_pos:
                continue
            if not cost or len(path_pos) > path.shape[0] or (len(path_pos) == path.shape[0] and path_cost < cost):
                selected_cluster = cluster
                cost = path_cost
                path = np.array(path_pos)
        if selected_cluster is None:
            print(f'Could not reach positions {pos}')
            break
        route = pos[clusters == selected_cluster][path]
        route = np.concatenate([start, route, start], axis=0)
        plt.scatter(pos[:, 0], pos[:, 1], c=clusters)
        plt.plot(route[:, 0], route[:, 1], label=f'{cost:.2f}m')
        mask = np.ones(pos.shape[0], dtype=bool)
        mask[np.argwhere(clusters == selected_cluster)[path]] = False
        pos = pos[mask]
        clusters = clusters[mask]
        plt.legend()
        plt.show()


if __name__ == '__main__':
    np.random.seed(0)
    main()
