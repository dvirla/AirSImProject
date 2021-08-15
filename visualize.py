import pathlib
import pandas as pd
import matplotlib.pyplot as plt


def main():
    graphs = pathlib.Path('graphs/mcts')
    data = []
    for filename in graphs.iterdir():
        if not filename.is_file() or filename.suffix != '.png':
            continue
        data.append(list(map(int, filename.stem.split('_'))))
    df = pd.DataFrame(data=data, columns=['n_drones', 'n_packages', 'n_simulations', 'best_distance', 'time'])
    df['time'] = 1e-9 * df['time']

    df.pivot(index='n_simulations', columns=['n_drones', 'n_packages'], values='best_distance').plot(legend=False)
    plt.xlabel('n_simulations')
    plt.title('best_distance')
    plt.show()
    df.pivot(index='n_simulations', columns=['n_drones', 'n_packages'], values='time').plot(legend=False, logy=True)
    plt.xlabel('n_simulations')
    plt.title('time (logarithmic scale)')
    plt.show()
    df.pivot(index='n_packages', columns=['n_drones', 'n_simulations'], values='time').plot(legend=False, logy=True)
    plt.xlabel('n_packages')
    plt.title('time (logarithmic scale)')
    plt.show()


if __name__ == '__main__':
    main()
