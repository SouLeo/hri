#!/usr/bin/env python
import random
import BlockInfo

class Block(objects):
    def __init__(self, size, color):
        self.size = size
        self.color = color

    def pickBlockColor(self, blockType):
        # Function: pickBlockColor
        # Description: selects random color for a given block size
        # Input: predetermined block size
        # Output: color choice
        color = random.choice(blockType.colorList)
        index = blockType.colorList.index(color)
        blockType.qtyList[index]--
        if (blockType.qtyList[index] == 0):
            blockType.qtyList.remove(0)
            blockType.colorList.remove(color)
        return color

    def newBlock(self, blockInfoList):
        # Function: newBlock
        # Description: generates new block to place on board
        # Input: none
        # Output: block size and color to try placing on board
        blockInfoSelection = random.choice(blockInfoList)
        
        blockSize = blockInfoSelection.size
        color = pickBlockColor(blockInfoSelection)
        
        self.size = blockSize
        self.color = color
        return self
