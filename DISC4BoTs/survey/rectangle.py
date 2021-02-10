import random

STRIP_WIDTH = 5

class Rectangle:
    """
    A class representing a rectangle with areas to survey

    Attributes:
        start (tuple) - the start point of the rectangle
        end (tuple) - the end point of the rectangle
        path (list) - list of points that represent the path which surveys this rectangle
        distance (int) - the length of this rectangle's survey path
    
    Methods:
        divide_in_two():
        randomly divides the current rectangle into two smaller rectangles
        divide(n: int):
        divides the current rectangle into n  smaller rectangles
        get_path():
        creates and returns the current rectangle's survey path
        get_distance():
        returns the length of the survey path
        get_area():
        returns the area of the current rectangle

    """
    def __init__(self, start: tuple, end: tuple):
        self.start = start
        self.end = end
        self.path = None
        self.distance = None

    
    """ Divide the current rectangle into two smaller rectangles
    By returning a smaller rectangle and removing from the current rectangle the new one

    Args:
        None

    Returns:
        Rectangle which is the new rectangle created.
    """
    def divide_in_two(self):
        # Check which sides of the rectangle is longer. This side would be divided
        if abs(self.start[1] - self.end[1]) > abs(self.start[0] - self.end[0]):
            # Randomly decide where to divide the side
            new_y = random.randint(self.start[1] + 5, self.end[1] - 5)
            # Update the current rectangle
            old_end = self.end
            self.end = (self.end[0], new_y)
            # Return the new rectangle
            return Rectangle((self.start[0], new_y), old_end)
        else:
            # Randomly decide where to divide the side
            new_x = random.randint(self.start[0], self.end[0])
            # Update the current rectangle
            old_end = self.end
            self.end = (new_x, self.end[1])
            # Return the new rectangle
            return Rectangle((new_x, self.start[1]), old_end)
    

    """ Divide the current rectangle into n smaller rectangles

    Args:
        n (int) - number of desired rectangles

    Returns:
        new_rectangles(list) - list of the created rectangles
    """
    def divide(self, n: int):
        new_rectangles = [self]
        for i in range(n - 1):
            # Each time, find the biggest rectangle and divide it
            biggest = get_biggest_rect(new_rectangles)
            # Add the newly created rectangle to the list of final rectangles
            rect = new_rectangles[biggest].divide_in_two()
            new_rectangles.append(rect)
        return new_rectangles


    """ Returns the path (list of points) which scans the current rectangle
        And measuring the length of this path 
    """
    def get_path(self):
        # If the path was not set yet - start calculating it
        if self.path == None:
            x = self.start[0]
            path = [self.start]
            self.distance = 0
            # Until the end of the path was reached
            while x < self.end[0]:
                # creating the path by adding points from one side of the rectangle to another
                # after moving from side to side, the drone will move STRIP WIDTH meters
                # and then continue to move towards the other side of the rectangle
                self.distance += self.end[1] - self.start[1]
                path.append((x, self.end[1]))
                x += STRIP_WIDTH
                self.distance += STRIP_WIDTH
                path.append((x, self.end[1]))
                self.distance += self.end[1] - self.start[1]
                path.append((x, self.start[1]))
                x += STRIP_WIDTH
                self.distance += STRIP_WIDTH
                path.append((x, self.start[1]))
            self.path = path
        return path
    

    """ Return the length of the survey path (self.path)
    """ 
    def get_distance(self):
        if self.distance == None:
            self.get_path()
        return self.distance
    

    """ Return the area (int) of the current rectangle
    """
    def get_area(self):
        return abs(self.end[0] - self.start[0]) * abs(self.end[1] - self.start[1])


""" Find and return the biggest rectangle among the given rectangles

    Args:
        rectangles (list): finds the biggest rectangle in this list of rectangles

    Returns:
        biggest_rect (Rectangle): the biggest rectangle
"""
def get_biggest_rect(rectangles: list):
    biggest_area = 0
    biggest_rect = 0
    index = 0
    for r in rectangles:
        # Find the biggest rectangle according to its area
        if r.get_area() > biggest_area:
            biggest_area = r.get_area()
            # If the current rectangle's area is bigger than the max, it is the max
            biggest_rect = index
        index += 1
    return biggest_rect