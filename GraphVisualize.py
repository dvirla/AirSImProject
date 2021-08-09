# First networkx library is imported
# along with matplotlib
import networkx as nx
import matplotlib.pyplot as plt
import graphviz


# Defining a Class
class GraphVisualization:

    def __init__(self):
        # visual is a list which stores all
        # the set of edges that constitutes a
        # graph
        self.visual = []
        self.dot = graphviz.Digraph(comment="MCTS")

        # addEdge function inputs the vertices of an

    # edge and appends it to the visual list
    def addEdge(self, a, b):
        temp = [a, b]
        self.visual.append(temp)
        self.dot.edge(str(a), str(b))

        # In visualize function G is an object of

    def removeEdge(self, a, b):
        temp = [a, b]
        if temp in self.visual:
            self.visual.remove(temp)
    # class Graph given by networkx G.add_edges_from(visual)
    # creates a graph with a given list
    # nx.draw_networkx(G) - plots the graph
    # plt.show() - displays the graph
    def visualize(self):
        self.dot.format = 'png'
        self.dot.render()
        # G = nx.Graph()
        # G.add_edges_from(self.visual)
        # nx.draw_networkx(G)
        # plt.savefig('test.png')
        # plt.show()