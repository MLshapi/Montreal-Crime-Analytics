import os
import time

import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import shapefile


# variables
my_path = os.getcwd()
inputShpFile = my_path + "\crime_dt.shp"
gdf = gpd.read_file(inputShpFile)
sf = shapefile.Reader(inputShpFile, encoding='ISO-8859-1')
sfRecords = sf.shapeRecords()
cornerPoints = sf.bbox
stats = {'x': [], 'y': [], 'total': []}
grid = {'x': [], 'y': [], 'total': []}
pathTracer = []
path_x_coords = []
path_y_coords = []
dangerousPoints = set()
categorizedRecords = []
numberOfDecimals = 4
totalGScore = 0
Threshold = 0
EXCU_PathFinder_IS_DONE = False


# set all possible moves
def possibleMovesCreator(space):
    return [(0, space), (0, -space), (space, 0), (-space, 0),
            (space, space), (space, -space), (-space, space), (-space, -space)]


# set x-axis ranges
def setXRanges(space):
    return np.arange(cornerPoints[2], cornerPoints[0], space)


# set y-axis ranges
def setYRanges(space):
    return np.arange(cornerPoints[3], cornerPoints[1], space)


# categorize all the records in grids
def recordsOrganizer(records):
    OrganizedRecords = {}
    for i in range(len(records)):
        x = records[i].shape.__geo_interface__["coordinates"][0]
        for j in xRangeOriginalMap:
            if x > j:
                A = j
                break
        y = records[i].shape.__geo_interface__["coordinates"][1]
        for j in yRangeOriginalMap:
            if y > j:
                B = j
                break
        if (A, B) in OrganizedRecords:
            OrganizedRecords[(A, B)].append(records[i])
        else:
            OrganizedRecords[(A, B)] = [records[i]]
    return OrganizedRecords


# create a grid to trace the road
def gridCreator():
    for x in xRangeOriginalMap[::-1]:
        for y in yRangeOriginalMap:
            if (x, y) in categorizedRecords:
                if len(categorizedRecords[(x, y)]) >= Threshold:
                    grid['x'].append(x)
                    grid['y'].append(y)
                    grid['total'].append(len(categorizedRecords[(x, y)]))
                    dangerousPoints.add((x, y))
                    dangerousPoints.add((x - interval, y))
                    dangerousPoints.add((x, y - interval))
                    dangerousPoints.add((x - interval, y - interval))
                    dangerousPoints.add((x - interval / 2, y - interval / 2))
                else:
                    grid['x'].append(x)
                    grid['y'].append(y)
                    grid['total'].append(0)
            else:
                grid['x'].append(x)
                grid['y'].append(y)
                grid['total'].append(0)


# statsGrid creator
def statsGridCreator():
    for x in xRangeOriginalMap[::-1]:
        for y in yRangeOriginalMap:
            if (x, y) in categorizedRecords:
                stats['x'].append(x)
                stats['y'].append(y)
                stats['total'].append(len(categorizedRecords[(x, y)]))


# Heuristic Function
def twoPointDistance(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


# A* algorithm
def pathFinder(sPoint, gPoint):
    # Straight Moves
    UP, DOWN, RIGHT, LEFT = moves[0], moves[1], moves[2], moves[3]
    # Diagonal Moves
    UP_RIGHT, DOWN_RIGHT, UP_LEFT, DOWN_LEFT = moves[4], moves[5], moves[6], moves[7]
    startPoint = sPoint[0], sPoint[1]
    goalPoint = gPoint[0], gPoint[1]
    came_from = {}
    visited_points = set()
    gScore = {startPoint: 0}
    fScore = {startPoint: twoPointDistance(startPoint, goalPoint)}
    adjacentPointsPQ = []
    adjacentPointsPQ.insert(0, (startPoint, fScore[startPoint]))
    while len(adjacentPointsPQ) != 0:
        # pick the best choice in the array
        adjacentPointsPQ.sort(key=lambda l: l[1])
        currentPoint = adjacentPointsPQ.pop(0)[0]
        isCurrentPointRisky = isDangeriousPoint(currentPoint)
        # if the point is the goal then we return the route from start to end
        if currentPoint == goalPoint:
            while currentPoint in came_from:
                pathTracer.append(currentPoint)
                currentPoint = came_from[currentPoint]
                path_x_coords.append(currentPoint[0])
                path_y_coords.append(currentPoint[1])
            pathTracer.append(startPoint)
            path_x_coords.append(startPoint[0])
            path_y_coords.append(startPoint[1])
            pathTracer.reverse()
            EXCU_PathFinder_IS_DONE = True
            return

        # mark the point as visited
        visited_points.add(currentPoint)

        # adding the surrounding point to the heap
        for i, j in moves:
            # UP , DOWN , RIGHT , LEFT
            # UP-RIGHT ,DOWN-RIGHT , UP-LEFT, DOWN-LEFT
            nextAdjacentPoint = currentPoint[0] + i, currentPoint[1] + j

            isNextAdjacentPointRisky = isDangeriousPoint(nextAdjacentPoint)

            # ignore it if was visited
            if nextAdjacentPoint in visited_points:
                continue

            # avoiding risky path
            if isCurrentPointRisky:
                if isNextAdjacentPointRisky:
                    # diagonal risky path avoid
                    if isDiagonal(nextAdjacentPoint, currentPoint):
                        # handling a case where
                        # i need to go from risky risky point in diagonal path
                        # .|__    or   .__
                        #    .|__      |__|.
                        # the first example is allowed; whereas, the second is not
                        if (i, j) == DOWN_RIGHT:
                            if isDangeriousPoint((currentPoint[0] + DOWN[0],currentPoint[1] + DOWN[1])) :
                                if isDangeriousPoint((currentPoint[0] + RIGHT[0],currentPoint[1] + RIGHT[1])):
                                    if isDangeriousPoint((currentPoint[0] + DOWN_RIGHT[0] / 2,currentPoint[1] + DOWN_RIGHT[1] / 2)):
                                        continue
                        elif (i, j) == DOWN_LEFT:
                            if isDangeriousPoint((currentPoint[0] + DOWN[0],currentPoint[1] + DOWN[1])):
                                if isDangeriousPoint((currentPoint[0] + LEFT[0],currentPoint[1] + LEFT[1])):
                                    if isDangeriousPoint((currentPoint[0] + DOWN_LEFT[0] / 2,currentPoint[1] + DOWN_LEFT[1] / 2)):
                                        continue
                        elif (i, j) == UP_LEFT:
                            if isDangeriousPoint((currentPoint[0] + UP[0],currentPoint[1] + UP[1])):
                                if isDangeriousPoint((currentPoint[0] + LEFT[0],currentPoint[1] + LEFT[1])):
                                    if isDangeriousPoint((currentPoint[0] + UP_LEFT[0] / 2,currentPoint[1] + UP_LEFT[1] / 2)):
                                        continue
                        else:
                            if isDangeriousPoint((currentPoint[0] + UP[0],currentPoint[1] + UP[1])):
                                if isDangeriousPoint((currentPoint[0] + RIGHT[0],currentPoint[1] + RIGHT[1])):
                                    if isDangeriousPoint((currentPoint[0] + UP_RIGHT[0] / 2,currentPoint[1] + UP_RIGHT[1] / 2)):
                                        continue

                    # straight risky path avoid between two block
                    # also handle the case where a safe grid between two risky grids
                    elif isDangerousMove(currentPoint, moves.index((i, j))):
                        continue

            # All the boundaries edges of the map are considered as inaccessible.
            # no walk on the boundaries
            if (currentPoint[0] == cornerPoints[0] and nextAdjacentPoint[0] == cornerPoints[0]) or (
                    currentPoint[1] == cornerPoints[1] and nextAdjacentPoint[1] == cornerPoints[1]):
                continue
            if (currentPoint[0] == cornerPoints[2] and nextAdjacentPoint[0] == cornerPoints[2]) or (
                    currentPoint[1] == cornerPoints[3] and nextAdjacentPoint[1] == cornerPoints[3]):
                continue

            # avoid going out of boundaries
            if cornerPoints[0] > nextAdjacentPoint[0] or nextAdjacentPoint[0] > cornerPoints[2]:
                # array bound x walls
                continue

            if yRangeOriginalMap[-1] > nextAdjacentPoint[1] or nextAdjacentPoint[1] > yRangeOriginalMap[0]:
                # array bound y walls
                continue
            # calculate the G score according to the came_from for diagonal came_from
            if i != 0 and j != 0:
                tentative_g_score = gScore[currentPoint] + 1.5
            # calculate the G score according to the came_from for straight came_from
            else:
                if isCurrentPointRisky and isNextAdjacentPointRisky:
                    tentative_g_score = gScore[currentPoint] + 1.3
                else:
                    tentative_g_score = gScore[currentPoint] + 1

            if nextAdjacentPoint not in [i[0] for i in adjacentPointsPQ]:
                came_from[nextAdjacentPoint] = currentPoint
                gScore[nextAdjacentPoint] = tentative_g_score
                cumulative_g_score = tentative_g_score + gScore[currentPoint]
                if currentPoint != startPoint:
                    predecessor = came_from[currentPoint]
                    while predecessor != startPoint:
                        cumulative_g_score += gScore[predecessor]
                        predecessor = came_from[predecessor]
                fScore[nextAdjacentPoint] = cumulative_g_score + twoPointDistance(nextAdjacentPoint, goalPoint)
                adjacentPointsPQ.insert(0, (nextAdjacentPoint, fScore[nextAdjacentPoint]))
    print("Due to blocks, no came_from is found.Please change the map and try again")


# check if the point is going to pass through two block in order to avoid risk
def isDangerousMove(p, direction):
    # UP , DOWN , RIGHT , LEFT
    # UP-RIGHT ,DOWN-RIGHT , UP-LEFT, DOWN-LEFT
    if direction == 0:
        if isDangeriousPoint((p[0] + interval / 2,p[1] + interval / 2)):
            if isDangeriousPoint((p[0] - interval / 2,p[1] + interval / 2)):
                return True
    elif direction == 1:
        if isDangeriousPoint((p[0] + interval / 2,p[1] - interval / 2)):
            if isDangeriousPoint((p[0] - interval / 2,p[1] - interval / 2)):
                return True
    elif direction == 2:
        if isDangeriousPoint((p[0] + interval / 2,p[1] + interval / 2)):
            if isDangeriousPoint((p[0] + interval / 2,p[1] - interval / 2)):
                return True
    else:
        if isDangeriousPoint((p[0] - interval / 2,p[1] + interval / 2)):
            if isDangeriousPoint((p[0] - interval / 2,p[1] - interval / 2)):
                return True
    return False


# centralize the point to the right top corner of its the grid
def pointCentralizer(p):
    A = xRangeOriginalMap[-1]
    B = yRangeOriginalMap[-1]
    for v in xRangeOriginalMap[::-1]:
        if p[0] <= v:
            A = v
            break
    for w in yRangeOriginalMap[::-1]:
        if p[1] <= w:
            B = w
            break
    return A, B


# To check if the path is diagonal
def isDiagonal(p1, p2):
    if (p1[0] - p2[0]) != 0:
        if (p1[1] - p2[1]) != 0:
            return True


def chooseStartGoalPoints():
    points = [((-73.58604919099992, 45.5219124127), (-73.55604919099999, 45.49391241269996)),
              ((-73.58604919099992, 45.5219124127), (-73.57605, 45.51191), (-73.57605, 45.51191)),
              ((-73.57805, 45.49191), (-73.58605, 45.49191)), ((-73.55425, 45.51791), (-73.58605, 45.49191)),
              ((-73.57185, 45.51591), (-73.58605, 45.49191)), ((-73.5717, 45.49425), (-73.58605, 45.49191)),
              ((-73.57185, 45.51591), (-73.5717, 45.49425))]
    print("choose start end point:")
    count = 1
    for t in points:
        print(count, " : ", t)
        count += 1
    print("press 0 to Enter another point")
    userInput = int(input())
    if userInput == 0:
        xStart = int(input("Enter x of start Point"))
        yStart = int(input("Enter y of start Point"))
        xGoal = int(input("Enter x of goal Point"))
        yGoal = int(input("Enter y of goal Point"))
        return (xStart, yStart), (xGoal, yGoal)
    else:
        return points[userInput - 1]


def calculateThreshold(num, df_stats):
    result = 0
    if int(num) == 50:
        result = int(len(sfRecords)/len(categorizedRecords))
    else:
        df = df_stats[df_stats.average > num]
        print(df)
        result = df['average'].min()
    return result


def isDangeriousPoint(point):
    tolerance = interval / 6
    for p in dangerousPoints:
        if abs(point[0] - p[0]) <= tolerance:
            if abs(point[1] - p[1]) <= tolerance:
                return True

if __name__ == "__main__":


    start_time = time.time()
    # taking Grids size
    interval = float(input("Enter grid size : "))

    # find the x and y ranges of the original map ordered increasingly
    xRangeOriginalMap = setXRanges(-interval).tolist()
    yRangeOriginalMap = setYRanges(-interval).tolist()

    # categorizing the map records
    categorizedRecords = recordsOrganizer(sfRecords)


    statsGridCreator()



    # displaying statistics
    df_stats = pd.DataFrame.from_dict(stats)
    df_stats['average'] = df_stats.mean(numeric_only=True, axis=1)
    print("description:")
    print(df_stats.describe())
    print('Median is :', df_stats['total'].median())
    print('total records :', len(sfRecords))
    # Setting the Threshold
    Threshold = calculateThreshold(int(input("Please Enter the Threshold?")), df_stats)
    print("Max Number of cases in a grid is: ", Threshold)

    # choosing start and end goal
    choosenPoints = chooseStartGoalPoints()

    gridCreator()

    # setting possilbe moves
    # UP , DOWN , RIGHT , LEFT
    # UP-RIGHT ,DOWN-RIGHT , UP-LEFT, DOWN-LEFT
    moves = possibleMovesCreator(interval)

    df_grid = pd.DataFrame.from_dict(grid)

    # centralizing start and goal points and find is there is a path
    start = pointCentralizer(choosenPoints[0])
    goal = pointCentralizer(choosenPoints[1])
    path_x_coords.append(goal[0])
    path_y_coords.append(goal[1])

    # creating a head map and save the result as png picture
    plt.figure(figsize=(9, 9))
    pivot_table = df_grid.pivot(index='y', columns='x', values='total')
    cMap = sns.cm.rocket_r
    ax = sns.heatmap(pivot_table, cbar=False, xticklabels=False, yticklabels=False, annot=True, fmt='g', linewidths=2,
                     annot_kws={"size": 9}, linecolor='yellow', square=True, cmap=cMap)
    ax.invert_yaxis()
    ax.set_ylabel('')
    ax.set_xlabel('')
    my_file = 'graph.png'
    plt.savefig(os.path.join(my_path, my_file), bbox_inches='tight', pad_inches=0, dpi=300)
    plt.close()

    # Timing the Path Finder function up to 10 sec
    # Creating the main thread that executes the function
    pathFinder(start, goal)
    xRangeOriginalMap = np.linspace(cornerPoints[2], cornerPoints[0], len(xRangeOriginalMap), endpoint=False)
    yRangeOriginalMap = np.linspace(cornerPoints[3], cornerPoints[1], len(yRangeOriginalMap), endpoint=False)
    path_x_coords = []
    path_y_coords = []
    for p in pathTracer:
        newp = pointCentralizer(p)
        path_x_coords.append(newp[0])
        path_y_coords.append(newp[1])
    start = pointCentralizer(start)
    goal = pointCentralizer(goal)
    # show the graph with path if found
    img = plt.imread(os.path.join(my_path, my_file))
    fig, ax = plt.subplots()
    ax.set(ylim=(cornerPoints[1], cornerPoints[3]))
    ax.set(xlim=(cornerPoints[0], cornerPoints[2]))
    plt.xticks(xRangeOriginalMap, rotation='vertical')
    plt.yticks(yRangeOriginalMap)
    ln = plt.plot(path_x_coords, path_y_coords)
    plt.scatter(start[0] , start[1] , marker="o", color="yellow", s=200)
    plt.scatter(goal[0] , goal[1] , marker="*", color="red", s=200)
    ax.imshow(img, extent=[cornerPoints[0], cornerPoints[2], cornerPoints[1], cornerPoints[3]])
    if (time.time() - start_time > 10):
        print("time out")
        print("--- %s seconds ---" % (time.time() - start_time))
    else:
        print("perfect")
    plt.show()

