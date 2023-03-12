import sys
import cv2

def org2opencv_conversion(org_coord_start,org_cord_goal, row, col):
    strt= [org_coord_start[0],row-org_coord_start[1]]
    goal= [org_cord_goal[0],row-org_cord_goal[1]]
    print("Start",strt)
    print("Goal",goal)


    
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
    org2opencv_conversion(org_coord_start,org_cord_goal, grid_row, grid_column)

if __name__ == "__main__":
    main()

