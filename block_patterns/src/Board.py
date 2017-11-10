#!/usr/bin/env python
import Tkinter as tk
import BlockInfo
import Block

LAYER_NUM = 0

class Board(object):
    def __init__(self, size):
        # Square grid for block placement
        self.size = size
        self.rows = self.cols = self.size

        # Use self.grid for internal calculations
        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]  
        
        # Use self.tiles for the GUI
        self.tiles = [[None for _ in range(self.cols)] for _ in range(self.rows)]

    def updateBoard(self, block):
        # Function: updateBoard
        # Description: places block on grid starting from outer to inner layer
        # Input: block (with a given size and color)
        # Output: places in internal grid (self.grid)
        
        # TODO:
        #   1) LAYER_NUM update
        #   2) for loop for ea. side of square
        #   3) when "newBlock" called, need to do it in a loop until good block
        #   shows up
        if (LAYER_NUM == 0):
            if old == 5:
                rotateblock(block)
            else if (old + new <= self.size):
                placeblock(block)
            else:
                newBlock(self, blockInfoList)
        else if (LAYER_NUM == 1):
            if old == 3:
                rotateblock(block)
            else if (old + new <= self.size - 2):
                placeblock(block)
            else:
                newBlock(self, blockInfoList)
        else:
            if old == 1:
                rotateblock(block)
            else if (old + new <= self.size - 4):
                placeblock(block)
            else:
                newBlock(self, blockInfoList)

    def showGrid(self):
        # Function: showGrid
        # Description: shows internal grid using Tkinter 
        # Input: none
        # Output: visualizes internal grid
    
    def placeBlock(self, block):
        # Function: placeBlock
        # Description: adds block to internal occupancy grid
        # Input: block
        # Output: updates grid, returns nothing

    def rotateBlock(self, block):
        # Function: rotateBlock
        # Description: adds block to internal occupancy grid after rotating it
        # Input: block
        # Output: updates grid, returns nothing

        placeblock(block) # should be called after rotation
