#!/usr/bin/env python

# Maintainer: Selma Wanna, slwanna@utexas.edu

def createPatterns():
    # Function: createPatterns
    # Description: fills out all possible pattern solutions
    # Input: none
    # Output: all possible solutions
    possiblePatterns = []
    possiblePatterns.append("rrgygy")
    possiblePatterns.append("rryygb")
    possiblePatterns.append("rybygg")
    possiblePatterns.append("rybyrb")
    possiblePatterns.append("yyyyrb")
    possiblePatterns.append("yyyygg")
    possiblePatterns.append("gbrrby")
    possiblePatterns.append("ggrrby")
    possiblePatterns.append("bbggry")
    possiblePatterns.append("bbggrb")
    possiblePatterns.append("bgbgby")
    possiblePatterns.append("bgbrby")
    return possiblePatterns

def comparePattern(currentPattern, possiblePatterns):
    # Function: comparePattern
    # Description: compares currently viewed pattern against possible solutions
    # Input: currentPattern (string), possiblePatterns (list of strings)
    # Output: possible solutions  (list of strings)
    if (len(possiblePatterns) <= 1):
        rospy.loginfo('Pattern Known from comparePattern')
        return possiblePatterns 
    for i, pattern in enumerate(possiblePatterns):
        if currentPattern not in possiblePatterns[i]:
            possiblePatterns.pop(i)
    return possiblePatterns

def nextBlock(possiblePatterns):
    # Function: nextBlock
    # Description: Chooses which block should be tried next
    # Input: currentPattern (string)
    # Output: block which should be tried next
    firstpossiblePattern = possiblePatterns[0]
    numBlocks = len(firstpossiblePattern)
    if (numBlocks + 1 <= 6):
        return firstpossiblePattern[numBlocks + 1] # return next character
    else:
        rospy.logerr('Indexed out of bound in the nextBlock() function!!!')
        return '0' # end of sequence. Something went wrong

 def main():

 __name__ == '__main__':
    main()
