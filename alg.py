import itertools
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt


def best_route(pos, max_distance):
    # path, cost = shortest_cycle(pos, max_distance)
    # return pos[path], cost
    n = pos.shape[0]
    routes = []
    costs = []
    for perm in itertools.permutations(range(1, n), n - 1):  # Permutations over all paths
        pos_perm = np.concatenate([np.expand_dims(pos[0], 0), pos[list(perm)]], axis=0)
        route = []
        total_dist = 0
        final_dist_0 = 0
        for i in range(1, n):  # Attempt to add next node to path
            dist_0 = np.sqrt(np.sum(np.square(pos_perm[i])))  # Distance from starting node
            dist = np.sqrt(np.sum(np.square(pos_perm[i] - pos_perm[i - 1])))  # Distance from last node
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
    # Take paths of maximum length
    costs = costs[lengths == max_len]
    routes = [routes[i] for i, take in enumerate(lengths == max_len) if take]
    best = np.argmin(costs)  # Lowest cost path
    return routes[best], costs[best]


def main():
    pos = np.random.randint(low=-100, high=100, size=(50, 2))
    max_distance = 250  # Maximum cycle length
    n_clusters = 10
    start = np.zeros((1, 2))
    # Split locations into regions using k means
    k_means = KMeans(n_clusters=n_clusters)
    clusters = k_means.fit_predict(pos)
    plt.figure(figsize=(10, 7))
    plt.scatter(pos[:, 0], pos[:, 1], c=clusters)
    while pos.shape[0]:
        path, cost, selected_cluster = None, None, None
        # Find the cluster with the best route
        for cluster in range(n_clusters):
            rel = pos[clusters == cluster]  # Locations in cluster
            if not rel.shape[0]:
                continue
            cycle = np.concatenate([start, rel], axis=0)
            # Find best route in cluster
            path_pos, path_cost = best_route(cycle, max_distance=max_distance)
            if not path_pos:
                continue
            # Compare to other clusters
            if not cost or len(path_pos) > path.shape[0] or (len(path_pos) == path.shape[0] and path_cost < cost):
                selected_cluster = cluster
                cost = path_cost
                path = np.array(path_pos)
        if selected_cluster is None:
            print(f'Could not reach positions {pos}')
            break
        # Draw route
        route = pos[clusters == selected_cluster][path]
        route = np.concatenate([start, route, start], axis=0)
        plt.plot(route[:, 0], route[:, 1], label=f'{cost:.2f}m')
        # Remove route locations
        mask = np.ones(pos.shape[0], dtype=bool)
        mask[np.argwhere(clusters == selected_cluster)[path]] = False
        pos = pos[mask]
        clusters = clusters[mask]
    plt.legend()
    plt.show()


if __name__ == '__main__':
    np.random.seed(0)
    main()
