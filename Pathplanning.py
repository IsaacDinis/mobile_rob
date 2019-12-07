import numpy as np
import matplotlib.pyplot as plt
from extremitypathfinder.extremitypathfinder import PolygonEnvironment as Environment

import vision
import cv2
class Obstacle:
    def __init__(self, vertexList, margeObs):

        self.vertex=vertexList
        self.vertexExpanded=[]
        #centerOfMass
        tot=[0,0]
        for point in self.vertex:
            tot[0]+= point[0]
            tot[1]+= point[1]
        N= len(vertexList)
        centerOfMass= [0,0]
        centerOfMass= np.array([ tot[0],  tot[1]] )/N

        for point in self.vertex:
            v=np.array([point[0], point[1]])-centerOfMass
            v=v/float(np.linalg.norm(v))*margeObs
            pointArr= np.array([point[0], point[1]])
            pointArr= pointArr + v
            self.vertexExpanded.append( (pointArr[0], pointArr[1]))


def take_picture_to_init(margeObs=9, plotFlag=False, cam_capture=2):
    """will return path, thymio pos and thymio theta"""

    MAP_MAX_X_AXIS= 81
    MAP_MAX_Y_AXIS= 114

    # img = cv2.imread("map_test\\map_test_more_complicated.png")
    # img = cv2.flip(img, 0)
    img = vision.capture_image_from_webcam(cam_capture)
    cv2.imshow("proj", img)
    pix_to_unit_x = 27*3/img.shape[1]
    pix_to_unit_y = 38 * 3 / img.shape[0]
    thymioPos = vision.detect_thymio(img)
    if thymioPos:
        start = [thymioPos.pos.x*pix_to_unit_x, thymioPos.pos.y*pix_to_unit_y]

    goal = vision.detect_goal(img)
    goal = (goal[0]*pix_to_unit_x, goal[1]*pix_to_unit_y)
    obstacles_vision = vision.detect_obstacles(img)

    obsList = []
    for obst_vis in obstacles_vision:
        obst_vis = obst_vis.astype('float64')
        obst_vis[:, :, 0] *= pix_to_unit_x
        obst_vis[:, :, 1] *= pix_to_unit_y
        vertex_vision = np.ndarray.tolist(obst_vis.squeeze())

        obsList.append(Obstacle(vertex_vision, margeObs))
    if plotFlag:
        for obs in obsList:
            unzippedList = list(zip(*obs.vertex))
            unzippedList = [list(elem) for elem in unzippedList]
            unzippedList[0].append(unzippedList[0][0])
            unzippedList[1].append(unzippedList[1][0])
            plt.plot(unzippedList[0], unzippedList[1])

            unzippedList=list(zip(*obs.vertexExpanded))
            unzippedList = [list(elem) for elem in unzippedList]
            unzippedList[0].append(unzippedList[0][0])
            unzippedList[1].append(unzippedList[1][0])
            plt.plot(unzippedList[0], unzippedList[1], '--')

        plt.plot(start[0], start[1], 'o', color='green')
        plt.plot(goal[0], goal[1], 'o', color='red')


    map = Environment()
    # anti clockwise vertex numbering!
    boundary_coordinates = [(0.0, 0.0), (MAP_MAX_X_AXIS, 0.0), (MAP_MAX_X_AXIS, MAP_MAX_Y_AXIS), (0.0, MAP_MAX_Y_AXIS)]

    # clockwise numbering!
    list_of_obstacle = []
    for obs in obsList:
        list_of_obstacle.append(obs.vertexExpanded)

    map.store(boundary_coordinates, list_of_obstacle, validate=False)
    map.prepare()
    path, length = map.find_shortest_path(start, goal)
    if plotFlag:
        unzippedPath = list(zip(*path))
        plt.plot(unzippedPath[0], unzippedPath[1], '--', color='black')
        plt.xlim(0, MAP_MAX_X_AXIS)
        plt.ylim(0, MAP_MAX_Y_AXIS)
        plt.show()
    return path, thymioPos.pos.x*pix_to_unit_x, thymioPos.pos.y*pix_to_unit_y, thymioPos.theta


# if __name__ == "__main__":
#    [path, pos, th] = get_map_info(9, True)