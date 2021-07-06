import math
import random
import numpy as np
from itertools import islice


class Node:
    """
        A class used to represent a Node

        ...

        Attributes
        ----------
        puzzle : array
            a multidimensional array that represents a 3x3 puzzle
        level : int
            depth of the current node within the tree
        fval : str
            Value by which the Algorithm chooses the next node to traverse. Sum of heuristic value and number of nodes traversed by depth
    """
    def __init__(self, puzzle, level, fval):
        """ Initialize the node with the data, level of the node and the calculated fvalue """
        self.puzzle = puzzle
        self.level = level
        self.fval = fval

    def generate_child(self):
        """ Generate child nodes from the given node by moving the blank space
            either in the four directions {up,down,left,right} """
        x, y = self.find(self.puzzle, 0)
        val_list = [[x, y - 1], [x, y + 1], [x - 1, y], [x + 1, y]]
        children = []
        for i in val_list:
            child = self.shuffle(self.puzzle, x, y, i[0], i[1])
            if child is not None:
                child_node = Node(child, self.level + 1, 0)
                children.append(child_node)
        return children

    def shuffle(self, puz, x1, y1, x2, y2):
        """ Move the blank space in the given direction and if the position value are out
            of limits the return None """
        if x2 >= 0 and x2 < len(self.puzzle) and y2 >= 0 and y2 < len(self.puzzle):
            temp_puz = []
            temp_puz = self.copy(puz)
            temp = temp_puz[x2][y2]
            temp_puz[x2][y2] = temp_puz[x1][y1]
            temp_puz[x1][y1] = temp
            return temp_puz
        else:
            return None

    def copy(self, root):
        """ Copy function to create a similar matrix of the given node"""
        temp = []
        for i in root:
            t = []
            for j in i:
                t.append(j)
            temp.append(t)
        return temp

    def find(self, puz, x):
        """ Specifically used to find the position of the blank space """
        for i in range(0, len(self.puzzle)):
            for j in range(0, len(self.puzzle)):
                if puz[i][j] == x:
                    return i, j


class Puzzle:
    """
        A class used to find a path to solve a 3x3 puzzle.
        The A* algorithm is used for pathfinding
        For optimal solution a heuristic value is used (hemmington, manhattan)

        ...

        Attributes
        ----------
        puzzleSize : int
            size of the puzzle
        maxIterations : int
            the maximum number of iterations the puzzle solver makes before throwing away the initial puzzle and generate a new one
        heuristic : string
            the heuristic algorithm used to calculate the distance to the goal state
        open: array
            list of nodes that havent been traversed yet
        closed: array
            list of nodes that have already been traversed
        startPuzzle: array
            multidimensianal array that represents the start state of the puzzle
        goalPuzzle: array
            multidimensianal array that represents the goal state of the puzzle
        searchCost: int
            number of iterations done by the A* algorithm. Represents the search cost
    """
    def __init__(self, puzzleSize, maxIterations, heuristic):
        """ Initialize the puzzle size by the specified size,open and closed lists to empty """
        self.puzzleSize = puzzleSize
        self.maxIterations = maxIterations
        self.useHeuristic = heuristic
        if heuristic == "manhattan":
            self.heuristic = self.h_manhattan
        elif heuristic == 'hemmington':
            self.heuristic = self.h_hemmington
        else:
            self.heuristic = self.h_hemmington
        self.open = []
        self.closed = []
        self.startPuzzle = ''
        self.goalPuzzle = self.goalPuzzle()
        self.searchCost = 0

    """
        Check if a given
        instance of 8 puzzle is solvable or not
        by counting the inversions in given puzzle 'arr[]'
    """
    def getInvCount2(self, arr):

        inv_count = 0
        for i in range(0, 2):
            for j in range(i + 1, 3):

                # Value 0 is used for empty space
                if (arr[j][i] > 0 and arr[j][i] > arr[i][j]):
                    inv_count += 1
        return inv_count



    def isSolvable(self, puzzle):
        """
           Check if a given
           instance of 8 puzzle is solvable or not
           by counting the inversions in given puzzle 'arr[]'
        """
        # Count inversions in given 8 puzzle
        invCount = self.getInvCount2(puzzle)

        # return true if inversion count is even.
        return (invCount % 2 == 0)

    def printPuzzle(self, node):
        """
            Gets and prints the given node

            Parameters
            ----------
            node : node
                current node
        """
        print(f'---d: {node.level} searchCost: {self.searchCost} fval: {node.fval} heuristic val: {self.heuristic(node.puzzle, self.goalPuzzle)}---')
        for i in range(self.puzzleSize):
            for j in range(self.puzzleSize):
                print(node.puzzle[i][j], end=' ')
            print('')

    def randomStartPuzzle(self):
        """
             Generate random Puzzle matrice

             Parameters
             ----------
             node : node
                 current node

            Returns
            -------
            array
                a multidimensional array based on puzzleSize
         """

        start = list(range(0, self.puzzleSize ** 2))
        random.shuffle(start)
        start = [start[i:i + self.puzzleSize] for i in range(0, len(start), self.puzzleSize)]
        return start

    def goalPuzzle(self):
        """
            Generate Puzzle matrice that represents the goal state

            Returns
            -------
            array
                a multidimensional array based on puzzleSize
         """
        goal = list(range(0, self.puzzleSize ** 2))
        goal = [goal[i:i + self.puzzleSize] for i in range(0, len(goal), self.puzzleSize)]
        return goal

    def f(self, node, goal):
        """
             Heuristic Function to calculate hueristic value f(x) = h(x) + g(x)

             Parameters
             ----------
             node : node
                 current node
            goal : node
                goal matrice

            Returns
            -------
            int
                a value that represents how "close" the algorithm is to reach the goal state
         """
        return self.heuristic(node.puzzle, goal) + node.level

    def h_hemmington(self, start, goal):
        """
             Calculates the different between the given puzzles

             Parameters
             ----------
             start : node
                 start matrice
            goal : node
                goal matrice

            Returns
            -------
            int
                heuristic value
         """
        temp = 0
        for i in range(0, self.puzzleSize):
            for j in range(0, self.puzzleSize):
                startVal = start[i][j]
                goalVal = goal[i][j]
                if startVal != goalVal:
                    temp += 1
        return temp

    def h_manhattan(self, start, goal):
        """
             Calculates the steps per tile to get into correct position

             Parameters
             ----------
             start : node
                 start matrice
            goal : node
                goal matrice

            Returns
            -------
            int
                heuristic value
         """
        distance = 0
        # iterate through all numbers in start puzzle
        for y, row in enumerate(start):
            for x, curr_num in enumerate(row):
                # find current num in goal puzzle
                for y2, row in enumerate(goal):
                    for x2, goal_num in enumerate(row):
                        if goal_num == curr_num:
                            # calculate distance
                            distance += (abs(x2 - x) + abs(y2 - y))

        return distance

    def estimate_effective_branching_factor(self, searchCost, depth):
        """
             Calculates the effective branching factor. The average number of successors per state

             Parameters
             ----------
             searchCost : int
                 actual traversed nodes
            depth : int
                actual depth of the tree

            Returns
            -------
            int
                effective branching factor
         """
        return searchCost ** (1 / depth)

    def main(self):
        """ Generate a solvable starting puzzle """
        while True:
            self.startPuzzle = self.randomStartPuzzle()
            if self.isSolvable(self.startPuzzle):
                print('puzzle is solvable')
                break
            print('puzzle not solvable, calculate new one')

        """ set the start/goal puzzle states """
        startNode = Node(self.startPuzzle, 0, 0)
        goal = self.goalPuzzle
        startNode.fval = self.f(startNode, goal)
        """ Put the start node in the open list"""
        self.open.append(startNode)
        iterations = 0
        """ start the A* algorithm """
        while True:
            iterations = iterations + 1
            currNode = self.open[0]
            self.printPuzzle(currNode)
            """ End condition. Maximum iteriations reached. Restart Start Puzzle generation"""
            if(iterations > self.maxIterations):
                self.open = []
                self.closed = []
                self.searchCost = 0
                self.main()
            """ End condition. If heuristics equal 0 the goal is reached"""
            if self.heuristic(currNode.puzzle, self.goalPuzzle) == 0:
                print(f'Algorithm A* with heuristic {self.useHeuristic}')
                print(
                    f"effective branching factor: {self.estimate_effective_branching_factor(self.searchCost, currNode.level)}")
                print("result:")
                self.printPuzzle(currNode)
                break

            """ Generate childs for current node and calculate its parameters"""
            childs = currNode.generate_child()
            for i in childs:
                new = True
                i.fval = self.f(i, self.goalPuzzle)
                """ Check if this puzzle already existed"""
                for x in self.closed + self.open:
                    if x.puzzle == i.puzzle:
                        new = False
                        break
                if new:
                    self.open.append(i)
                else:
                    new = True
            self.closed.append(currNode)
            del self.open[0]

            """ sort the open list based on f value """
            self.open.sort(key=lambda x: x.fval, reverse=False)
            self.searchCost = self.searchCost + 1


def user_interface():
    """
        Provides the user with an user interface

        user provided values
        ------
        heuristic : string
            the heuristic algorithm that should be used
        iterations : int
            the amount of maximum iterations per solvable puzzle until a new one is generated

    """
    while True:
        print("Enter heuristic ('manhattan' or 'hemmington'): ")
        try:
            heuristic = str(input())
            if heuristic == "manhattan" or heuristic == "hemmington":
                break
            print("The input is not valid 1")
            continue
        except ValueError:
            print("The input is not valid 2")

    while True:
        print("Enter maximum number of iterations: ")
        try:
            maxIterations = int(input())
            if maxIterations > 0:
                break
            print("The input must be positive")
            continue
        except ValueError:
            print("The input is not a valid number")

    return heuristic, maxIterations


""" Generate new puzzle instance based on user input and start the program """
heuristic, maxIterations = user_interface()
puz = Puzzle(3, maxIterations, heuristic)
puz.main()
