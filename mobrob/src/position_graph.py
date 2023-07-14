#!/usr/bin/env python3

import rospy
import numpy as np
from mobrob_util.msg import ME439WaypointXY
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, Float32

min_sample_distance = rospy.get_param("/min_distance_between_graph_samples")
max_sample_distance = rospy.get_param("/max_distance_between_graph_samples")
min_node_distance = rospy.get_param("/min_distance_between_graph_nodes")
max_node_distance = rospy.get_param("/max_distance_between_graph_nodes")

wall_left_distance = 0.0
wall_right_distance = 0.0
wall_front_distance = 0.0
left_dist_ready = False
right_dist_ready = False
front_dist_ready = False

current_x = 0
current_y = 0
current_theta = 0
previous_x = 0
previous_y = 0
robot_points = []


class Graph():
    maxWeight = 2.1
    allVertex = []  # array of vertex objects
    # allEdge = [] #array of edge objects
    vertexCount = 0
    # base class vertex
    # contains x,y coordinates and connected edges
    # "point" is based on number and not name
    # starting location will be the first point (point==0)

    class vertex():
        def __init__(self, x, y):
            self.point = Graph.vertexCount  # int
            self.x = x  # float
            self.y = y  # float
            self.edges = []  # array
    # base class vertex
    # contains the connected vertex and weight

    class edge():
        def __init__(self, vertex1, weight):
            self.vertex = vertex1  # connected vertices
            self.weight = weight  # float

    # function getEdgesFromVertex
    # inputs: vertex : the vertex you want to get the edges from
    # outputs: all edges from this vertex in the form: [[vertexPoint,weight],[vertexPoint,weight]]
    def getEdgesFromVertex(vertex):
        allEdgesFromVertex = []
        for i in range(len(Graph.allVertex)):
            for testEdge in Graph.allVertex[i].edges:
                if vertex == testEdge.vertex:
                    allEdgesFromVertex.append(
                        [Graph.allVertex[i].point, testEdge.weight])
        return (allEdgesFromVertex)

    # function getWeight
    # inputs: vertex1 and vertex2 : the vertices you want to get the distance between
    # output: a float number based on pythagorean theorem
    def getWeight(vertex1, vertex2):
        x1 = vertex1.x
        y1 = vertex1.y
        x2 = vertex2.x
        y2 = vertex2.y
        return (((x2-x1)**2+(y2-y1)**2)**.5)

    # function addVertex
    # inputs: x and y coordinates : the location of the vertex
    # outputs: None, but will link all other vertices based on maxWeight
    def addVertex(self, x, y):
        newVertex = self.vertex(x, y)
        # allVertex.append(newVertex)
        for i in range(len(Graph.allVertex)):
            # print(Graph.allVertex)
            Graph.addEdges(self, Graph.allVertex[i], newVertex)
        # print(newVertex)
        Graph.allVertex.append(newVertex)
        Graph.vertexCount = Graph.vertexCount+1
        # print(Graph.allVertex)

    # function addEdges
    # inputs: vertex and newVertex : the vertices that might be connected
    # outputs: None, but will link the two vertices based on maxWeight
    def addEdges(self, vertex, newVertex):
        weight = Graph.getWeight(vertex, newVertex)
        if weight < Graph.maxWeight:
            newEdge = Graph.edge(newVertex, weight)
            vertex.edges.append(newEdge)
            newEdge = Graph.edge(vertex, weight)
            newVertex.edges.append(newEdge)

    # function printGraph
    #inputs: None
    # outputs: a string of each vertex and the connected vertices
    def printGraph(self):
        responseString = ""
        for i in range(len(Graph.allVertex)):
            edgesFromVertex = Graph.getEdgesFromVertex(Graph.allVertex[i])
            responseString = responseString + \
                str(Graph.allVertex[i].point) + \
                ": " + str(edgesFromVertex) + "\n"
        return (responseString)

    # function printGraph
    #inputs: None
    # outputs: a string of each vertex and the connected vertices along with the weights
    def printVertices(self):
        responseString = ""
        edgeString = ""
        for i in range(len(Graph.allVertex)):
            edgeString = "["
            edgesFromVertex = Graph.getEdgesFromVertex(Graph.allVertex[i])
            for edge in ((Graph.allVertex[i].edges)):
                edgeString = edgeString + \
                    str(edge.vertex.point) + ", " + str(edge.weight) + "\t"
            responseString = responseString + \
                str(Graph.allVertex[i].point) + ": " + str(edgeString) + "]\n"
        return (responseString)

    # function printGraph
    #inputs: None
    # outputs: an array matrix containing the distance to each point (inf=not connected, 0=same point)
    def printWeightArray(self):
        weightArray = np.zeros((len(Graph.allVertex), len(Graph.allVertex)))
        for i in range(len(Graph.allVertex)):
            vertexArray = np.ones(len(Graph.allVertex))
            vertexArray = vertexArray*np.inf
            vertexArray[i] = 0
            edgesFromVertex = Graph.getEdgesFromVertex(Graph.allVertex[i])
            for edge in ((Graph.allVertex[i].edges)):
                vertexArray[edge.vertex.point] = edge.weight
            weightArray[i] = vertexArray
        return (weightArray)

    # function shortestPath
    # will always calculate to start point (0)
    # inputs : point : the point from which we are at now to calculate the way back to the start point
    # outputs : the vertices needed and total weight
    # in the event of two weights with equal ties, it goes to the first accessed node

    def shortestPath(self, point):
        shortestRoute = []
        for i in range(len(Graph.allVertex)):
            shortestRoute.append(str(i))
        # print(shortestRoute)
        shortestWeight = np.ones(len(Graph.allVertex))*np.inf
        shortestWeight[0] = 0
        nextRoute = [0]
        testedRoutes = []
        while len(nextRoute) != 0:
            testRoute = nextRoute[0]
            nextRoute.remove(testRoute)
            testedRoutes.append(testRoute)
            edgesFromRoute = Graph.getEdgesFromVertex(
                Graph.allVertex[testRoute])
            for edge in edgesFromRoute:
                compareWeight = shortestWeight[testRoute]+edge[1]
                if (compareWeight < shortestWeight[edge[0]]):
                    shortestWeight[edge[0]] = compareWeight
                    shortestRoute[edge[0]] = str(
                        edge[0]) + "->" + shortestRoute[testRoute]
                if (edge[0] not in testedRoutes and edge[0] not in nextRoute):
                    nextRoute.append(edge[0])
        return (shortestRoute[point], shortestWeight[point])


"""
Example Calls:

testGraph = Graph()
testGraph.addVertex(0,0)
testGraph.addVertex(0,1)
testGraph.addVertex(0,2)
testGraph.addVertex(2,2)
testGraph.addVertex(0,3)
testGraph.addVertex(0,4)
testGraph.addVertex(0,5)
#print(testGraph.printVertices())
#print(testGraph.printGraph())
print(testGraph.printWeightArray())
print(testGraph.shortestPath(3))

"""


def talker():
    rospy.init_node('position_graph', anonymous=True)

    sub_position = rospy.Subscriber(
        '/robot_pose_graph', Pose2D, update_position)
    sub_create_graph = rospy.Subscriber('/solve_maze_graph', Bool, solve_graph)

    sub_wall_left_distance = rospy.Subscriber(
        '/wall_distance_left_graph', Float32, store_left_dist)
    sub_wall_right_distance = rospy.Subscriber(
        '/wall_distance_right_graph', Float32, store_right_dist)
    sub_wall_front_distance = rospy.Subscriber(
        '/wall_distance_front_graph', Float32, store_front_dist)

    rospy.spin()


def update_position(pose_msg_in):
    global current_x, current_y, current_theta
    current_x = pose_msg_in.x
    current_y = pose_msg_in.y
    current_theta = pose_msg_in.theta


def store_left_dist(float_msg_in):
    global wall_left_distance, left_dist_ready
    wall_left_distance = float_msg_in
    left_dist_ready = True
    check_dist_ready()


def store_right_dist(float_msg_in):
    global wall_right_distance, right_dist_ready
    wall_right_distance = float_msg_in
    right_dist_ready = True
    check_dist_ready()


def store_front_dist(float_msg_in):
    global wall_front_distance, front_dist_ready
    wall_front_distance = float_msg_in
    front_dist_ready = True
    check_dist_ready()


def check_dist_ready():
    global left_dist_ready, right_dist_ready, front_dist_ready, current_x, current_y, previous_x, previous_y, min_sample_distance
    if right_dist_ready is True:
        if left_dist_ready is True:
            if front_dist_ready is True:
                # Should also add something that splits something greater than the max_sample_distance into multiple segments
                if (get_distance(current_x, current_y, previous_x, previous_y) >= min_sample_distance):
                    record_points()
                    previous_x = current_x
                    previous_y = current_y
                left_dist_ready = False
                right_dist_ready = False
                front_dist_ready = False


def record_points():
    global current_x, current_y, current_theta, wall_left_distance, wall_right_distance, wall_front_distance, robot_points, min_node_distance, max_node_distance
    heading_left = current_theta - np.pi / 2
    heading_right = current_theta + np.pi / 2
    next_wall_dist_left = Float32()
    next_wall_dist_left.data = min_node_distance
    next_wall_dist_right = Float32()
    next_wall_dist_right.data = min_node_distance
    next_wall_dist_front = Float32()
    next_wall_dist_front.data = 0.0  # Front is used to put in the current robot position
    while next_wall_dist_front.data < wall_front_distance.data:
        node_x = current_x + next_wall_dist_front.data * np.sin(current_theta)
        node_y = current_x + next_wall_dist_front.data * np.cos(current_theta)
        robot_points.append([node_x, node_y])
        next_wall_dist_front.data += min_node_distance

    while next_wall_dist_left.data < wall_left_distance.data:
        node_x = current_x + next_wall_dist_left.data * np.sin(heading_left)
        node_y = current_x + next_wall_dist_left.data * np.cos(heading_left)
        robot_points.append([node_x, node_y])
        next_wall_dist_left.data += min_node_distance

    while next_wall_dist_right.data < wall_right_distance.data:
        node_x = current_x + next_wall_dist_right.data * np.sin(heading_right)
        node_y = current_x + next_wall_dist_right.data * np.cos(heading_right)
        robot_points.append([node_x, node_y])
        next_wall_dist_right.data += min_node_distance


def get_distance(x1, y1, x2, y2):
    return (((x2-x1)**2+(y2-y1)**2)**.5)


def solve_graph(bool_msg_in):
    global robot_points
    robot_graph = Graph()
    for x in range(len(robot_points)):
        robot_graph.addVertex(robot_points[x][0], robot_points[x][1])
    robot_graph.shortestPath()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
