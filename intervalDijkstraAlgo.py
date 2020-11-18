# Python implementation of Dijkstra's algorithm
#https://gist.github.com/econchick/4666413

import networkx as nx
                                       
'''def dijkstra(graph, initial):
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

    return visited, path'''

def main():
    graph = nx.Graph()
    ''' Picture of Graph
               B
            / /\ \
          / /  \ \
        A--C---E--F
         \ \  / /
          \ \/ /
            D
    '''
    
    graph.add_edge('A', 'B', weight=[6,8])
    graph.add_edge('A', 'C', weight=[2,4])
    graph.add_edge('A', 'D', weight=[7,9])
    graph.add_edge('B', 'C', weight=[2,4])
    graph.add_edge('B', 'E', weight=[3,5])
    graph.add_edge('B', 'F', weight=[9,11])
    graph.add_edge('C', 'E', weight=[1,3])
    graph.add_edge('C', 'D', weight=[5,7])
    graph.add_edge('E', 'D', weight=[4,6])
    graph.add_edge('E', 'F', weight=[3,5])
    graph.add_edge('F', 'D', weight=[8,10])
    
    for node1, node2, data in graph.edges(data=True):
        print(data['weight'])

if __name__ == "__main__":
    main()
