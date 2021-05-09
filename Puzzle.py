import random
import numpy as np
from itertools import islice


class Node:
    def __init__(self, puzzle, level, fval):
        """ Initialize the node with the data, level of the node and the calculated fvalue """
        self.puzzle = puzzle
        self.level = level
        self.fval = fval

    def generate_child(self):
        """ Generate child nodes from the given node by moving the blank space
            either in the four directions {up,down,left,right} """
        x, y = self.find(self.puzzle, 0)
        """ val_list contains position values for moving the blank space in either of
            the 4 directions [up,down,left,right] respectively. """
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
    def __init__(self, puzzleSize, iterations, heuristic):
        """ Initialize the puzzle size by the specified size,open and closed lists to empty """
        self.puzzleSize = puzzleSize
        self.iterations = iterations
        self.useHeuristic = heuristic
        if heuristic == "manhattan":
            self.heuristic = self.h_manhattan
        elif heuristic == 'hemmington':
            self.heuristic = self.h_hemmington
        else:
            self.heuristic = self.h_hemmington
        self.open = []
        self.closed = []
        #self.startPuzzle = [[7,2,4],[5,0,6],[8,3,1]]
        self.startPuzzle = self.randomStartPuzzle()
        #self.goalPuzzle=[[1,2,3],[4,5,6],[7,8,0]]
        self.goalPuzzle = self.goalPuzzle()
        self.searchCost = 0

    def printPuzzle(self, node):
        print(f'---d: {node.level} searchCost: {self.searchCost} fval: {node.fval}---')
        for i in range(self.puzzleSize):
            for j in range(self.puzzleSize):
                print(node.puzzle[i][j], end=' ')
            print('')

    def randomStartPuzzle(self):
        """ Generate Puzzle matrice"""
        start = list(range(0, self.puzzleSize ** 2))
        random.shuffle(start)
        start = [start[i:i + self.puzzleSize] for i in range(0, len(start), self.puzzleSize)]
        return start

    def goalPuzzle(self):
        """ Generate Puzzle matrice"""
        goal = list(range(0, self.puzzleSize ** 2))
        goal = [goal[i:i + self.puzzleSize] for i in range(0, len(goal), self.puzzleSize)]
        return goal

    def f(self, node, goal):
        """ Heuristic Function to calculate hueristic value f(x) = h(x) + g(x) """
        return self.heuristic(node.puzzle, goal) + node.level

    def h_hemmington(self, start, goal):
        """ Calculates the different between the given puzzles """
        temp = 0
        for i in range(0, self.puzzleSize):
            for j in range(0, self.puzzleSize):
                startVal = start[i][j]
                goalVal = goal[i][j]
                if startVal != goalVal:
                    temp += 1
        return temp

    def h_manhattan(self, start, goal):
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
        return searchCost**(1/depth)

    def main(self):
        startNode = Node(self.startPuzzle, 0, 0)
        goal = self.goalPuzzle
        startNode.fval = self.f(startNode, goal)
        """ Put the start node in the open list"""
        self.open.append(startNode)
        while True:
            currNode = self.open[0]
            self.printPuzzle(currNode)

            """ End condition. If heuristics equal 0 the goal is reached"""
            if self.heuristic(currNode.puzzle, self.goalPuzzle) == 0:
                print(f'Algorithm A* with heuristic {self.useHeuristic}')
                print(f"effective branching factor: {self.estimate_effective_branching_factor(self.searchCost, currNode.level)}")
                print("result:")
                self.printPuzzle(currNode)
                break
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
            self.searchCost = self.searchCost+1



puz = Puzzle(3, 1, 'manhattan')
puz.main()
