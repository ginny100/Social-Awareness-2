# Python implementation of Dijkstra's algorithm
#https://gist.github.com/econchick/4666413

import collections
from numpy import sum

class Graph:
    def __init__(self):
        self.nodes = set()
        self.edges = collections.defaultdict(list)
        self.distances = {}

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.distances[(from_node, to_node)] = distance

def sum_lists(a, b):
    return list(map(sum, zip(a, b)))
                                       
def dijkstra(graph, initial):
    visited = {initial: [0,0]}
    path = {}
  
    nodes = set(graph.nodes)
  
    while nodes: 
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node

        if min_node is None:
            break

        nodes.remove(min_node)
        current_weight = visited[min_node]
        cw_radius = ((current_weight[1] - current_weight[0])/2)

        for edge in graph.edges[min_node]:
            # Need to find Fuzzy Membership Here somehow (Use def 3?)
            # D[z] is visited[edge]
            # weight is D[u] + w(u,z)
            # Fuzzy Path Membership Test
            temp = graph.distance[(min_node, edge)]
            temp_radius = ((temp[1] - temp[0])/2)
            if current_weight[1] < temp[0]:
                fuzzy = 1
            elif current_weight[0] <= temp[0] <= current_weight[1] < temp[1] and cw_radius > 0: 
                fuzzy = -1
            elif temp[0] <= current_weight[0] < current_weight[1] <= temp[1] and temp_radius > cw_radius:
                fuzzy = (temp[1] - current_weight[1])/(2(temp_radius - cw_radius))
            elif temp_radius == cw_radius and current_weight[1] == temp[1]:
                fuzzy = 0.5
            else:
                print("error")
                
            weight = sum_lists(current_weight, graph.distance[(min_node, edge)])
            if edge not in visited or weight < visited[edge]:
                visited[edge] = weight
                path[edge] = min_node

    return visited, path

def main():
    graph = Graph()
    ''' Picture of Graph
               B
            / /\ \
          / /  \ \
        A--C---E--F
         \ \  / /
          \ \/ /
            D
    '''
    graph.nodes = {'A','B','C','D','E','F'}
    graph.edges = {'A': ['B', 'C', 'D'], 'B':['A', 'C', 'E', 'F'], \
                'C':['A', 'B', 'D', 'E'], 'D':['A', 'C', 'E', 'F'],\
                'E': ['B', 'C', 'D', 'F'],\
                    'F': ['B', 'E', 'D']}
    graph.distance = {('A', 'B'):[6,8], ('A', 'C'):[2,4], ('A', 'D'):[7,9], \
                      ('B', 'C'):[2,4], ('B', 'E'):[3,5], ('B', 'F'):[9,11], \
                      ('C', 'E'):[1,3], ('C', 'D'):[5,7], \
                      ('E', 'D'):[4,6], ('E', 'F'):[3,5], \
                      ('F', 'D'):[8,10]}

    v, path = dijkstra(graph, 'A')
    print('Visited: ', v)
    print('Path :', path)

if __name__ == "__main__":
    main()
