import sys
import cv2


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

    coord1 = [x1, y1]
    coord2 = [x2, y2]

    print("Start coordiante: {}".format(coord1))
    print("Goal Coordinate: {}".format(coord2))

if __name__ == "__main__":
    main()

