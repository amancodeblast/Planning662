import sys
import cv2
import math
import numpy as np
from queue import PriorityQueue
import time

class Node:
    
    def __init__(self, pos, cost, parent):
        self.parent = parent
        self.pos = pos
        self.cost = cost
        self.x = pos[0]
        self.y = pos[1]        



def djiktras_algo(strt,goal,grid_rows,grid_columns):
    # strt = [8, 195]  

    # goal = [500, 8]


    # grid_rows = 250  
    # grid_columns = 600

    image_grid = np.zeros((grid_rows, grid_columns), np.uint8)
    points_up_rect = np.array(
        [[100, 0], [150, 0], [150, 100], [100, 100]])  
    points_bot_rect = np.array(
        [[100, 150], [150, 150], [150, 250], [100, 250]])  
    points_trngl = np.array([[460, 25], [510, 125], [460, 225]])  

    points_hexa = np.array([[300, 45], [370, 88], [370, 162], [300, 205], [
                        230, 162], [230, 88]])  


    pad1 = np.array([[0, 0], [600, 0], [600, 5], [0, 5]])
    pad2 = np.array([[595, 0], [600, 0], [600, 250], [595, 250]])
    pad3 = np.array([[0, 0], [5, 0], [5, 250], [0, 250]])
    pad4 = np.array([[0, 250], [0, 245], [600, 245], [600, 250]])

    cv2.fillPoly(image_grid, pts=[points_up_rect], color=(255, 0, 0))
    cv2.fillPoly(image_grid, pts=[points_bot_rect], color=(255, 0, 0))
    cv2.fillPoly(image_grid, pts=[points_trngl], color=(255, 0, 0))
    cv2.fillPoly(image_grid, pts=[points_hexa], color=(255, 0, 0))
    cv2.fillPoly(image_grid, pts=[pad4], color=(255, 0, 0))
    cv2.fillPoly(image_grid, pts=[pad3], color=(255, 0, 0))
    cv2.fillPoly(image_grid, pts=[pad2], color=(255, 0, 0))
    cv2.fillPoly(image_grid, pts=[pad1], color=(255, 0, 0))

    start_time = time.time()  

    possible = True
    if image_grid[grid_rows - strt[1], grid_columns - strt[0]] == 1 or image_grid[grid_rows - goal[1], grid_columns - goal[0]] == 1:
        print("The Coordinates entered for either start or goal node falls upon an obstacle. Change the input accordingly")
        possible = False
        exit()
    
    ## Drawing start and end using circle
    radius = 5

    color_goal = (0, 255, 0)
    color_start = (255, 0, 0)

    thickness = -1



    cv2.imshow("Image",image_grid)
    cv2.waitKey(0)
    grid_image_colored = np.dstack([image_grid.copy(), image_grid.copy(), image_grid.copy()])
    cv2.circle(grid_image_colored, (goal[0], goal[1]), radius, color_goal, thickness)
    cv2.circle(grid_image_colored, (strt[0], strt[1]),
            radius, color_start, thickness)
    cv2.imshow("Image_show",grid_image_colored)
    cv2.waitKey(0)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter("C:/Users/amank/all_files/1.aman/out.mp4", fourcc, 5000, (grid_columns, grid_rows))





def next_step(image_grid, node):  
    i = node.x
    j = node.y

    diff_steps = [(i, j + 1), (i + 1, j), (i - 1, j), (i, j - 1), (i + 1, j + 1), (i - 1, j - 1), (i - 1, j + 1),
             (i + 1, j - 1)]  
    allowed_steps = []
    for pos, step in enumerate(diff_steps):
        
        
        if not (step[0] >= grid_columns or step[0] < 0 or step[1] >= grid_rows or step[1] < 0):

            if image_grid[step[1]][step[0]] == 0:  
                
                cost = math.sqrt(2) if pos > 3 else 1
                allowed_steps.append([step, cost])
    return allowed_steps  

def org2opencv_conversion(org_coord_start,org_cord_goal, row, col):
    strt= [org_coord_start[0],row-org_coord_start[1]]
    goal= [org_cord_goal[0],row-org_cord_goal[1]]
    print("Start",strt)
    print("Goal",goal)
    return strt,goal


    
def main():
    if len(sys.argv) != 5:
        print("Usage: python program_name.py x1 y1 x2 y2")
        sys.exit(1)

    try:
        x1 = int(sys.argv[1])
        y1 = int(sys.argv[2])
        x2 = int(sys.argv[3])
        y2 = int(sys.argv[4])
    except ValueError:
        print("Coordinates must be integers")
        sys.exit(1)

    org_coord_start = [x1, y1]
    org_cord_goal = [x2, y2]

    # print("Start coordiante: {}".format(org_coord_start))
    # print("Goal Coordinate: {}".format(org_cord_goal))
    grid_row = 250
    grid_column = 600
    strt_img, goal_img = org2opencv_conversion(org_coord_start,org_cord_goal, grid_row, grid_column)
    djiktras_algo(strt_img,goal_img,grid_row,grid_column)

if __name__ == "__main__":
    main()

