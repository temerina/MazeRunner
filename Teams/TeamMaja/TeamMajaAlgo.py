
"""
This class is the template class for the Maze solver
"""

import sys
from math import sqrt
import queue
import numpy
import os.path


class TeamMajaAlgo:

    EMPTY = 0       # empty cell
    OBSTACLE = 1    # cell with obstacle / blocked cell
    START = 2       # the start position of the maze (red color)
    TARGET = 3      # the target/end position of the maze (green color)

    def __init__(self):
        # TODO: this is you job now :-)
        self.master = 0
        self.dimCols = 0
        self.dimRows = 0
        self.startCol = 0
        self.startRow = 0
        self.endCol = 0
        self.endRow = 0
        self.grid = [[]]
        self.came_from = []
        print("\n[TeamMajaAlgo]: Constructor TeamMajaAlgo successfully executed.")

    # Setter method for the maze dimension of the rows
    def setDimRows(self, rows):
        # TODO: this is you job now :-)
        pass

    # Setter method for the maze dimension of the columns
    def setDimCols(self, cols):
        # TODO: this is you job now :-)
        pass

    # Setter method for the column of the start position
    def setStartCol(self, col):
        # TODO: this is you job now :-)
        pass

    # Setter method for the row of the start position
    def setStartRow(self, row):
        # TODO: this is you job now :-)
        pass

    # Setter method for the column of the end position
    def setEndCol(self, col):
        # TODO: this is you job now :-)
        pass

    # Setter method for the row of the end position
    def setEndRow(self, row):
        # TODO: this is you job now :-)
        pass

    # Setter method for blocked grid elements
    def setBlocked(self, row, col):
        # TODO: this is you job now :-)
        pass

    # Start to build up a new maze
    # HINT: don't forget to initialize all member variables of this class (grid, start position, end position, dimension,...)
    def startMaze(self, columns=0, rows=0):
        # TODO: this is you job now :-)
        pass

    # Start to build up a new maze
    # HINT: don't forget to initialize all member variables of this class (grid, start position, end position, dimension,...)

    # Define what shall happen after the full information of a maze has been received
    def endMaze(self):
        # TODO: this is you job now :-)
        # HINT: did you set start position and end position correctly?
        pass

    # just prints a maze on the command line
    def printMaze(self):
        # TODO: this is you job now :-)
        pass

    # loads a maze from a file pathToConfigFile
    def loadMaze(self, pathToConfigFile):
        # check whether a function numpy.loadtxt() could be useful
        # https://numpy.org/doc/1.20/reference/generated/numpy.loadtxt.html
        # TODO: this is you job now :-)
        exists = os.path.exists(pathToConfigFile)

        if exists:
            print("[TeamMajaAlgo]: SUCCESS loading file: ", pathToConfigFile)
        else:
            print("[TeamMajaAlgo]: ERROR loading file ", pathToConfigFile)

        self.grid = numpy.loadtxt(pathToConfigFile, delimiter=',', dtype=int)
        self.startMaze(self.grid.shape[1], self.grid.shape[0])
        self.grid = numpy.loadtxt(pathToConfigFile, delimiter=',', dtype=int)
        start_arr = numpy.where(self.grid == 2)
        self.startRow = int(start_arr[0][0])
        self.startCol = int(start_arr[1][0])

        end_arr = numpy.where(self.grid == 3)
        self.endRow = int(end_arr[0][0])
        self.endCol = int(end_arr[1][0])
        return True

        return True

    # clears the complete maze
    def clearMaze(self):
        # TODO: this is you job now :-)
        self.startMaze()

    # Decides whether a certain row,column grid element is inside the maze or outside
    def isInGrid(self, row, column):
        # TODO: this is you job now :-)
        if row < 0:
            return False

        if column < 0:
            return False

        if row >= self.grid.shape[0]:
            return False

        if column >= self.grid.shape[1]:
            return False

        return True

    # Returns a list of all grid elements neighboured to the grid element row,column
    def getNeighbours(self, row, column):
        # TODO: this is you job now :-)
        # TODO: Add a Unit Test Case --> Very good example for boundary tests and condition coverage
        neighbours = []

        # no neighbours for out-of-grid elements
        if self.isInGrid(row, column) is False:
            return neighbours

        # no neighbours for blocked grid elements
        if self.grid[row, column] == self.OBSTACLE:
            return neighbours

        nextRow = row + 1
        if (self.isInGrid(nextRow, column) is True and self.grid[nextRow][column] != self.OBSTACLE):
            neighbours.append([nextRow, column])

        previousRow = row - 1
        if (self.isInGrid(previousRow, column) is True and self.grid[previousRow][column] != self.OBSTACLE):
            neighbours.append([previousRow, column])

        nextColumn = column + 1
        if (self.isInGrid(row, nextColumn) is True and self.grid[row][nextColumn] != self.OBSTACLE):
            neighbours.append([row, nextColumn])

        previousColumn = column - 1
        if (self.isInGrid(row, previousColumn) is True and self.grid[row][previousColumn] != self.OBSTACLE):
            neighbours.append([row, previousColumn])

        return neighbours

    # Gives a grid element as string, the result should be a string row,column
    def gridElementToString(self, row, col):
        # TODO: this is you job now :-)
        # HINT: this method is used as primary key in a lookup table
        result = ""
        result += str(row)
        result += ","
        result += str(col)
        return result

    # check whether two different grid elements are identical
    # aGrid and bGrid are both elements [row,column]
    def isSameGridElement(self, aGrid, bGrid):
        # TODO: this is you job now :-)
        if (aGrid[0] == bGrid[0] and aGrid[1] == bGrid[1]):
            return True

        return False

    # Defines a heuristic method used for A* algorithm
    # aGrid and bGrid are both elements [row,column]

    def heuristic(self, aGrid, bGrid):
        # TODO: this is you job now :-)
        # HINT: a good heuristic could be the distance between to grid elements aGrid and bGrid
        return abs(aGrid[0] - bGrid[0]) + abs(aGrid[1] - bGrid[1])

    # Generates the resulting path as string from the came_from list
    def generateResultPath(self, came_from):
        # TODO: this is you job now :-)
        # HINT: this method is a bit tricky as you have to invert the came_from list (follow the path from end to start)
        result_path = []

        #############################
        # Here Creation of Path starts
        #############################
        startKey = self.gridElementToString(self.startRow, self.startCol)
        currentKey = self.gridElementToString(self.endRow, self.endCol)
        path = []
        while currentKey != startKey:
            path.append(currentKey)
            current = came_from[currentKey]
            currentKey = self.gridElementToString(current[0], current[1])

        path.append(startKey)
        path.reverse()
        #############################
        # Here Creation of Path ends
        #############################

        for next_element in path:
            nextPath = next_element.split(",")
            result_path.append([int(nextPath[0]), int(nextPath[1])])
        self.resultpath = result_path
        return result_path


    def getResultPath(self):
        # TODO: this is you job now :-)
       return self.resultpath


    #############################
    # Definition of Maze solver algorithm
    #
    # implementation taken from https://www.redblobgames.com/pathfinding/a-star/introduction.html
    #############################
    def aStar(self):
        result_path = []
        print("[MazeSolverAlgoAStar]: Start of A* Solver...")

        print("[MazeSolverAlgoAStar]: Start = ", self.startRow, self.startCol)
        print("[MazeSolverAlgoAStar]: End = ", self.endRow, self.endCol)
        print("[MazeSolverAlgoAStar]: Maze = \n", self.grid)

#        print("Neighbours [0,4] : " , self.getNeighbours(0,4))

        #############################
        # Here A* starts
        #############################
        start = [self.startRow, self.startCol]
        frontier = queue.PriorityQueue()
        frontier.put((0, start))

        startKey = self.gridElementToString(self.startRow, self.startCol)
        came_from = {}
        came_from[startKey] = None

        cost_so_far = {}
        cost_so_far[startKey] = 0

        goal = [self.endRow, self.endCol]

        while not frontier.empty():
            current = frontier.get()[1]
            currentKey = self.gridElementToString(current[0], current[1])
            # print("First Queue Element = " , currentKey)

            if self.isSameGridElement(current, goal):
                break

            for next_neighbour in self.getNeighbours(current[0], current[1]):
                # + 1 is extremely important, otherwise you would not punish additional moves!!!
                new_cost = cost_so_far[currentKey] + 1
                # + 1 = graph costs

                nextKey = self.gridElementToString(
                    next_neighbour[0], next_neighbour[1])
                if nextKey not in cost_so_far or new_cost < cost_so_far[nextKey]:
                    cost_so_far[nextKey] = new_cost
                    priority = new_cost + self.heuristic(goal, next_neighbour)
                    # print("Next = " , nextKey , " - priority = " , priority)
                    frontier.put((priority, next_neighbour))
                    came_from[nextKey] = current

        #############################
        # Here A* ends
        #############################

        self.came_from = came_from
        result_path = self.generateResultPath(came_from)

        print("[MazeSolverAlgoAStar]: Resulting length A* Solution: ",
              len(result_path))
        print("[MazeSolverAlgoAStar]: Resulting A* Solution Path = ", result_path)

        print("[MazeSolverAlgoAStar]: Finished A* Solver....")
        self.resultpath = result_path
        return result_path

    def solveMaze(self):
        return self.aStar()

if __name__ == '__main__':
    mg = TeamMajaAlgo()

    # HINT: in case you want to develop the solver without MQTT messages and without always
    #       loading new different mazes --> just load any maze you would like from a file

    mg.loadMaze("../../MazeExamples/maze2.txt")
    print("[TeamMajaAlgo]: loaded maze", mg.grid)

    # solve the maze
    # HINT: this command shall be received from MQTT client in run_all mode
    solutionString = mg.solveMaze()
    print("[TeamMajaAlgo]: Result of solving maze: ", solutionString)
