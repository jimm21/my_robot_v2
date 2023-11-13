#!/usr/bin/env python
import time
import numpy as np
import matplotlib.pyplot as plt
import rospy
from tf.transformations import euler_from_quaternion
from robot_arm_simulate.srv import sp, spResponse

def callback(request):

    ##################### READING MESH POINTS FILE #######################
    nodes_list = []
    with open('/home/user/robot_ws/src/robot_arm_simulate/src/mesh.txt', 'r') as filehandle:
        filecontents = filehandle.readlines()

        for line in filecontents:
            if(line[-1] == "\n"): x, y, z = line[:-1].split("\t")
            else: x, y, z = line[:].split("\t")
            nodes_list.append([float(x), float(y), float(z)])

    nodes = np.array(nodes_list, dtype=np.float64) 
    nt = len(nodes)

    ##################### APPLYING GREEDY ALGORITHM #######################

    # Drilling points are represented as complex numbers in 2D
    Point = complex
    Node  = Point

    def Y(point): 
        "The y coordinate of a point."
        return point.real

    def Z(point): 
        "The z coordinate of a point."
        return point.imag

    def distance(A, B): 
        "The distance between two points."
        return abs(A - B)

    def tour_length(tour):
        "The total of distances between each pair of consecutive points in the tour."
        return sum(distance(tour[i], tour[i-1]) 
                for i in range(len(tour)))

    def Nodes(nodes, nt):
        return set(Node(nodes[c,1], nodes[c,2]) for c in range(nt))

    def plot_tour(tour): 
        "Plot the points as circles and the tour as lines between them."
        plot_lines(list(tour) + [tour[0]])
        
    def plot_lines(points, style='bo-'):
        "Plot lines to connect a series of points."
        plt.plot(list(map(Y, points)), list(map(Z, points)), style)
        plt.axis('scaled'); plt.axis('off')
        
    def plot_tsp(algorithm, points):
        "Apply a TSP algorithm to points, plot the resulting tour, and print information."
        # Find the solution and time how long it takes
        t0 = time.process_time()
        tour = algorithm(points)
        t1 = time.process_time()
        assert valid_tour(tour, points)
        plot_tour(tour); plt.show()
        print("{} city tour with length {:.1f} in {:.3f} secs for {}"
            .format(len(tour), tour_length(tour), t1 - t0, algorithm.__name__))
        
    def valid_tour(tour, points):
        "Is tour a valid tour for these points?"
        return set(tour) == set(points) and len(tour) == len(points)

    def greedy_tsp(points):
        """Go through edges, shortest first. Use edge to join segments if possible."""
        edges = shortest_edges_first(points) # A list of (A, B) pairs
        endpoints = {c: [c] for c in points} # A dict of {endpoint: segment}
        for (A, B) in edges:
            if A in endpoints and B in endpoints and endpoints[A] != endpoints[B]:
                new_segment = join_endpoints(endpoints, A, B)
                if len(new_segment) == len(points):
                    return new_segment
                
    def shortest_edges_first(points):
        "Return all edges between distinct points, sorted shortest first."
        edges = [(A, B) for A in points for B in points 
                        if id(A) < id(B)]
        return sorted(edges, key=lambda edge: distance(*edge))

    def join_endpoints(endpoints, A, B):
        "Join B's segment onto the end of A's and return the segment. Maintain endpoints dict."
        Asegment, Bsegment = endpoints[A], endpoints[B]
        if Asegment[-1] is not A: Asegment.reverse()
        if Bsegment[0] is not B: Bsegment.reverse()
        Asegment.extend(Bsegment)
        del endpoints[A], endpoints[B]
        endpoints[Asegment[0]] = endpoints[Asegment[-1]] = Asegment
        return Asegment

    ##################### APPLYING 2-OPT ALGORITHM #######################

    # Calculate the euclidian distance in n-space of the route r traversing cities c, ending at the path start.
    path_distance = lambda r,c: np.sum([np.linalg.norm(c[r[p]]-c[r[p-1]]) for p in range(len(r))])
    # Reverse the order of all elements from element i to element k in array r.
    two_opt_swap = lambda r,i,j: np.concatenate((r[0:i+1],r[j:i:-1],r[j+1:len(r)]))

    def two_opt(points,improvement_threshold): # 2-opt Algorithm adapted from https://en.wikipedia.org/wiki/2-opt
        route = np.arange(points.shape[0]) # Make an array of row numbers corresponding to mesh points.
        improvement_factor = 1 # Initialize the improvement factor.
        best_distance = path_distance(route,points) # Calculate the distance of the initial path.
        while improvement_factor > improvement_threshold: # If the route is still improving, keep going!
            distance_to_beat = best_distance # Record the distance at the beginning of the loop.
            for swap_first in range(1,len(route)-1): # From each city except the first and last,
                for swap_last in range(swap_first + 1,len(route)): # to each of the points following,
                    new_route = two_opt_swap(route,swap_first,swap_last) # try reversing the order of these points
                    new_distance = path_distance(new_route,points) # and check the total distance with this modification.
                    if new_distance < best_distance: # If the path distance is an improvement,
                        route = new_route # make this the accepted best route
                        best_distance = new_distance # and update the distance corresponding to this route.
            improvement_factor = 1 - best_distance/distance_to_beat # Calculate how much the route has improved.
        return route # When the route is no longer improving substantially, stop searching and return the route.

    # 2-opt algorithm implementation
    # https://stackoverflow.com/questions/25585401/travelling-salesman-in-scipy

    # Create a matrix of points, with each row being a location in 2-space (function works in n-dimensions).
    aux = greedy_tsp(Nodes(nodes, nt))
    i = 0
    for val in aux:
        
        if i == 0: tour_0 = np.array([[val.real, val.imag]])
        else: tour_0 = np.append(tour_0, [[val.real, val.imag]], axis=0)
        i = i + 1

    # Find a good route with 2-opt ("route" gives the order in which to travel to each point by row number.)
    route = two_opt(tour_0, 0.001)

    # Reorder the points matrix by route order in a new matrix
    tour_final = np.array([tour_0[route[i]] for i in range(len(route))])
    distancia = path_distance(route,tour_0)
    # print("Distance: " + str(distance))

    # Plot the points.
    tour_plot = np.concatenate((np.array([tour_0[route[i]] for i in range(len(route))]), np.array([tour_0[0]])))
    plt.scatter(tour_0[:,0],tour_0[:,1])
    # Plot the path.
    plt.plot(tour_plot[:,0],tour_plot[:,1])
    plt.show()

    ##################### WRITING PATH TO A FILE #######################

    with open('/home/user/robot_ws/src/robot_arm_simulate/src/mesh_planned.txt', 'w') as my_file:
        for fila in tour_final:
            my_file.write(str(x) + "\t" + str(fila[0]) + "\t" + str(fila[1]) + "\n")
    return distancia


if __name__ == '__main__':

    rospy.init_node('sequence_planning')
    service = rospy.Service('sp_algorithm', sp, callback)
    rospy.spin()