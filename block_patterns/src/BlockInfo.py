#!/usr/bin/env python

class BlockInfo(object):
    def __init__(self, size, qtyList, colorList):
        # Describes different block types by sizes, colors, and qtys 
        self.size = size # size: (4 x 1) = 0; size: (3 x 1) = 1; size: (2 x 1) = 2;
        self.qtyList = qtyList          # integer list      
        self.colorList = colorList      # string list
