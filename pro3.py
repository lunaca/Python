#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 30 16:15:08 2017

@author: Zac Brooks, Allen Courtway, Chris Robertson
"""
#first we will import a few tools to help us accomplish our goals
import heapq
import networkx as nx
import matplotlib.pyplot as plt
from collections import defaultdict
import time
from itertools import groupby

#This is the unweighted, undirected graph that we will be using for BFS
#And DFS

fig = {
'A': set(['B', 'E', 'F']),
 'B': set(['A', 'C', 'F']),
 'C': set(['B', 'D', 'G']),
 'D': set(['C', 'G']),
 'E': set(['A', 'F', 'I']),
 'F': set(['A', 'B', 'E', 'I']),
 'G': set(['C', 'D', 'J']),
 'H': set(['K', 'L']),
 'I': set(['E', 'F', 'J', 'M']),
 'J': set(['G', 'I']),
 'K': set(['H', 'O', 'L']),
 'L': set(['K', 'H', 'P']),
 'M': set(['I', 'N']),
 'N': set(['M']),
 'O': set(['K']),
 'P': set(['L'])
} 

# this is the directed graph for the Decomposition into SCC problem
diGraph1 = {
         '1': set(['3']),
         '2': set(['1']),
         '3': set(['2', '5']),
         '4': set(['1','2']),
         '5': set(['6', '8']),
         '6': set(['7','8','10']),
         '7': set(['10']),
         '8': set(['9','10']),
         '9': set(['5','11']),
         '10': set(['9','11']),
         '11': set(['12']),
         '12': set()
            }


# this is the weighted graph that we will use for Dijkstra's and the minimal spanning tree
weightedGraph = { 
   'A': {'B': 22, 'C': 9, 'D': 12}, 
   'B': {'A': 22, 'C': 35,'F' : 36, 'H' : 34}, 
   'C': {'A': 9, 'B': 35, 'D': 4, 'E' : 65, 'F' : 42}, 
   'D': {'A': 12, 'C': 4, 'E': 33, 'I' : 30}, 
   'E': {'C': 65, 'D': 33, 'F': 18, 'G': 23}, 
   'F': {'B': 36, 'C': 42, 'E': 18, 'G' : 39, 'H' : 24}, 
   'G': {'E': 23, 'F': 39, 'H': 25, 'I' : 21},
   'H': {'B' : 34, 'F' : 24, 'G' : 25, 'I' : 19}, 
   'I': {'D' : 30, 'G' : 21, 'H': 19}
}




#This class will be our data structure for handling the majority of our graph functions

class pathGraph:
   # this is the initializer that initializes a total vertice count, 
    # a dictionary representation of a graph, and a networkx representation
    def __init__(self):
       
        self.V= len(fig.keys())#No. of vertices
        self.graph = fig
        self.Z = nx.Graph()
      

   
    #This function will take the dictionary form of the graph and draw it with
    #networkx
    def drawGraph(self,dictionary):
        i =0
        for key, value in sorted(dictionary.items()):
            self.Z.add_node(key)
           
           
            adjacency = sorted(value)
            for value in adjacency:
                
                self.Z.add_edge(key,value)
        plt.figure(2,figsize=(8,8))           
        nx.draw(self.Z, with_labels = True)
        plt.show()
    
    #This function will take the dictionary form of the graph and draw it with
    #networkx, with weights added on and displayed.        
    def drawWeightedGraph(self,dictionary):
         for keys,values in dictionary.items():
             self.Z.add_node(keys)
             for keys1,values1 in values.items():
                     self.Z.add_edge(keys,keys1, weight = values1)
                
              
           
         pos=nx.spring_layout(self.Z)
         plt.figure(2,figsize=(8,8)) 
         nx.draw(self.Z,pos, with_labels = True)
         labels = nx.get_edge_attributes(self.Z,'weight')
         nx.draw_networkx_edge_labels(self.Z,pos,edge_labels=labels)
        
         plt.show()
  
         
#This class is a representation of drawing the 
class diGraph:
    # this is the initializer that initializes a total vertice count, 
    # a dictionary representation of a graph, and a networkx representation
     def __init__(self):
       
        self.V= len(fig.keys())#No. of vertices
        self.graph = diGraph1
        self.Z = nx.DiGraph()
    #this will take every value in a dictionary, add it to the directional graph,
    #and display with networkx
     def drawGraph(self,dictionary):
        
        for key, value in sorted(dictionary.items()):
            self.Z.add_node(key)
           
           
            adjacency = sorted(value)
            for value in adjacency:
                
                self.Z.add_edge(key,value)
        plt.figure(2,figsize=(8,8))           
        nx.draw(self.Z, with_labels = True)
        plt.show()
            


#A simple DFS search to find the components connected to node
def connectedDFS(graph, start):
    visited, stack = set(), [start]
    while stack:
        vertex = stack.pop()
        if vertex not in visited:
            visited.add(vertex)
            stack.extend(graph[vertex] - visited)
    return visited
    
#This runs dfs on every vertex in the graph to print the connected
#components of the whole graph from each vertex    
def allConnected(graph):
    components = []
    count = 0
    for node in graph:
       comp = connectedDFS(graph, node)
       flag = 0
       count += 1
       if count == 1: 
           
           components.append(comp)
       for value in components:
           
           if comp == value :
               flag = 1
       if flag == 0:
           components.append(comp)
    print(components)          
               
               
               
        
#This shows paths from bfs from start to end node
def bfs_paths(graph, start, goal):
    queue = [(start, [start])]
    while queue:
        (vertex, path) = queue.pop(0)
        for next in graph[vertex] - set(path):
            if next == goal:
                yield path + [next]
            else:
                queue.append((next, path + [next]))

# Use BFS to check path between s and d
def dfs_paths(graph, start, goal, nx):
    stack = [(start, [start])]
    while stack:
        (vertex, path) = stack.pop()
        for next in graph[vertex] - set(path):
            if next == goal:
                yield path + [next]
            else:
                stack.append((next, path + [next]))
 #https://github.com/ChuntaoLu/Algorithms-Design-and-
#Analysis/blob/master/week4%20Graph%20search%20and%20SCC/scc.py#L107

#The original script was written in Python 2. It computes the strong
# connected components(SCC) of a given graph.   
#This class keeps track of the current time, current source, component leader,
    #finish time of each node and the explored nodes.
class Tracker(object):
  
    
    #'self.leader' is informs of {node: leader, ...}."

    def __init__(self):
        self.current_time = 0
        self.current_source = None
        self.leader = {}
        self.finish_time = {}
        self.explored = set()

 #THIS IS THE DFS THAT IS USED BY THE DECOMPOSITION FUNCTION
#Inner loop explores all nodes in a SCC. Graph represented as a dict,
   # {tail: [head_list], ...}. Depth first search runs recursively and keeps
    #track of the parameters
def dfs(graph_dict, node, Tracker):
    

    Tracker.explored.add(node)
    Tracker.leader[node] = Tracker.current_source
    for head in graph_dict[node]:
        if head not in Tracker.explored:
            dfs(graph_dict, head, Tracker)
    Tracker.current_time += 1
    Tracker.finish_time[node] = Tracker.current_time

 ##Outer loop checks out all SCCs. Current source node changes when one
 ##SCC inner loop finishes."""
def dfs_loop(graph_dict, nodes, Tracker):
   

    for node in nodes:
        if node not in Tracker.explored:
            Tracker.current_source = node
            dfs(graph_dict, node, Tracker)

# Given a directed graph in forms of {tail:[head_list], ...}, compute
 # a reversed directed graph, in which every edge changes direction.
def graph_reverse(graph):
  

    reversed_graph = defaultdict(list)
    for tail, head_list in graph.items():
        for head in head_list:
            reversed_graph[head].append(tail)
    return reversed_graph

### Using Dr. Hu's reference code to complete my graph to be decomposed into it's
### strongly connected components.
###
# """First runs dfs_loop on reversed graph with nodes in decreasing order,
   # then runs dfs_loop on original graph with nodes in decreasing finish
   # time order(obtained from first run). Return a dict of {leader: SCC}."""    
def scc(graph):
   

    out = defaultdict(list)
    Tracker1 = Tracker()
    Tracker2 = Tracker()
    nodes = set()
    reversed_graph = graph_reverse(graph)
    for tail, head_list in graph.items():
        nodes |= set(head_list)
        nodes.add(tail)
    nodes = sorted(list(nodes), reverse=True)
    dfs_loop(reversed_graph, nodes, Tracker1)
    sorted_nodes = sorted(Tracker1.finish_time,
                          key=Tracker1.finish_time.get, reverse=True)
    dfs_loop(graph, sorted_nodes, Tracker2)
    for lead, vertex in groupby(sorted(Tracker2.leader, key=Tracker2.leader.get),
                                key=Tracker2.leader.get):
        out[lead] = list(vertex)
    return out


#This is our implementation of Dijkstra's, using a heap, and being passed a weighted Graph
def dijkstrasPath(weightedGraph, start, end):
    queue,seen = [(0, start, [])], set()
    while True:
        (cost, v, path) = heapq.heappop(queue)
        if v not in seen:
            path = path + [v]
            seen.add(v)
            if v == end:
                return cost, path
            for (next, c) in weightedGraph[v].items():
                heapq.heappush(queue, (cost + c, next, path))
	#This is our implementation of Prims Algorithm to find a minimal spanning tree		
def primsTree(graph):
    queue = [(0, '#', list(graph.keys())[0])]
    tree = []
    seen = set()
    while queue:
        _, v0, v1 = heapq.heappop(queue)
        seen.add(v0)
        if v1 not in seen:
            tree.append((v0,v1))
            for (v2, c) in graph[v1].items():
                heapq.heappush(queue, (c, v1, v2))
    tree.pop(0)
    return tree

def decompose(diGraph):
    
    groups = scc(diGraph) 
    top_5 = heapq.nlargest(5, groups, key=lambda x: len(groups[x]))
   
    result = []
    for i in range(5):
        try:
            result.append(len(groups[top_5[i]]))
    
        except:
            result.append(0)
    return result, groups

# This function wil first call decompose to get the strongly connected components,
#Then it will draw them in a readable format
def drawSCC (diGraph1):     
    count, components = decompose(diGraph1)
    print('Strongly connected components are:')
    nodenames = []
    
    for key in components:
        noden = ""
        for value in components[key]:
            noden = noden + str(value) + "," 
        nodenames.append(noden)
    print(nodenames)
    D = nx.Graph()
    plt.figure(2,figsize=(8,8))
    D.add_nodes_from(nodenames)
    i = 0 
    for value in nodenames:
        if i != (len(nodenames)-1):
            D.add_edge(nodenames[i], nodenames[i+1])
            i += 1
        
    pos = nx.spring_layout(D)
    
    nx.draw(D, node_size = 8000, with_labels = True)
    
    plt.show()
def main():
   
   while True:
        G = pathGraph()
        H = pathGraph()
        I = diGraph()
       
       
        
        print(" From this menu, you may :")
        print(" 0: display all of the connected components in a graph")
        print(" 1: run BFS on plain graph to find paths between two nodes")
        print(" 2: run DFS on plain graph to find paths between two nodes")
        print(" 3: decompose a DiGraph into a DAG of Strongly Connected Components")
        print(" 4: run Dijkstra's on weighted Graph.")
        print(" 5: Find the minimal spanning tree of a weighted graph")
        
        program = input("Please enter the number for what you would like to do.     ")
        if program == '0':
            G.drawGraph(fig)
            print('The connected components of this graph are:')
            allConnected(fig)
        if program == '1':
            G.drawGraph(fig)
            print('Okay, lets run a BFS on this graph, what two nodes would you like to see paths for?')
            node1 = input("node 1 : ")
            node2 = input("node 2 : ")
            print("The paths that exist between " + node1 + " and " + node2 + " are:")
        
            bfspaths = list(bfs_paths(G.graph, node1, node2))
            
            if len(bfspaths) == 0:
               
                print("THERE IS NO PATH FROM " + node1 + " TO " + node2) 
            for key in bfspaths: 
                print(key)
          
        if program == '2':
            G.drawGraph(fig)
            print('Okay, lets run a DFS on this graph, what two nodes would you like to see paths for?')
            node1 = input("node 1 : ")
            node2 = input("node 2 : ")
            print("The paths that exist between " + node1 + " and " + node2 + " are:")
            dfspaths = list(dfs_paths(G.graph, node1, node2, G.Z))
            if len(dfspaths) == 0:
               
                print("THERE IS NO PATH FROM " + node1 + " TO " + node2) 
            for key in dfspaths: 
                print(key)
        if program == '3': 
            I.drawGraph(diGraph1)
            print("This is the DiGraph in it's original form, but it's strongly connected components are : ")
            
            time.sleep(3)
            drawSCC(diGraph1)
        if program == '4':
            H.drawWeightedGraph(weightedGraph)
            print('Okay,  on what nodes would you like dijkstras to find a shortest pathy?')
            node1 = input("node 1 : ")
            node2 = input("node 2 : ")
            cost, path = dijkstrasPath(weightedGraph, node1, node2)
            print('the shortest path has a weight of and exists through:')
            print(cost,path)
        if program == '5':
            H.drawWeightedGraph(weightedGraph)
            print('The spanning tree for the weighted Graph is as follows:')
            print(primsTree(weightedGraph))
        repeat = input('Would you like to repeat the program? (y/n)          ')
        if repeat == 'n':
            break
if __name__ == '__main__':
    main()      