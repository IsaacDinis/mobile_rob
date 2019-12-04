import numpy as np
import matplotlib.pyplot as plt
from extremitypathfinder.extremitypathfinder import PolygonEnvironment as Environment

from vision import *
import cv2


class Obstacle:
    def __init__(self, vertexList):
        borderDistance = 1
        self.vertex=vertexList
        self.vertexExpanded=[]
        #centerOfMass
        tot=[0,0]
        for point in  self.vertex:
            tot[0]+= point[0]
            tot[1]+= point[1]
        N= len(vertexList)
        centerOfMass= [0,0]
        centerOfMass= np.array([ tot[0],  tot[1]] )/N

        i=0
        for point in self.vertex:
            v=np.array([point[0], point[1]])-centerOfMass
            v=v/float(np.linalg.norm(v)) *float(borderDistance)
            pointArr= np.array([point[0], point[1]])
            pointArr= pointArr + v
            self.vertexExpanded.append( (pointArr[0], pointArr[1]))
            i+=1

#obstacle definition
# clockwise numbering!
# obs1= Obstacle([(2,2), (3, 3), (4,2), (3, 1)])
# obs2=  Obstacle([(6,10), (11, 11), (4+8+5,2+8), (3+8, 1+8)])
# obs3=  Obstacle([(2+15,2+15), (3+15, 3+15), (4+15,2+15), (3+15, 1+15)])
# obs4 = Obstacle([(2+15+7,2+15), (3+15+7, 3+15), (4+15+7,2+15), (3+15+7, 1+15)])
# obsListOLD= [obs1,obs2, obs3, obs4]
#
# start = (0.5, 0.5)
# goal = (27,25-4)


img = cv2.imread("map_test_more_complicated.png")
img= cv2.flip(img, 0)

# cv2.imshow("Display window", img)
warped = map_projection(img)
scale_percent = 150  # percent of original size
resized = resize_img(warped, 1.5)
thymioPos = detect_thymio(resized)
if thymioPos:
    start=[thymioPos.pos.x, thymioPos.pos.y]

goal = detect_goal(resized)
obstaclesListIsaac = detect_obstacles(resized)

obsList=[]
for obstacleIsaac in obstaclesListIsaac:
    vertexIsaac=np.ndarray.tolist(obstacleIsaac.squeeze())
    vertexIsaac.reverse()
    obstacleMax= Obstacle(vertexIsaac)
    obsList.append(obstacleMax)


for obs in obsList:
    unzippedList=list(zip(*obs.vertex))
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
plt.plot(goal[0], goal[1], 'o',color='red')



print("")
#%%
map = Environment()
# anti clockwise vertex numbering!
boundary_coordinates = [(0.0, 0.0), (2000.0, 0.0), (2000.0, 2000.0), (0.0, 2000.0)]

# clockwise numbering!
# list_of_obstacle = [[(3.0, 7.0), (5.0, 9.0), (5.0, 4.0)],
#                     [(3.0 + 30, 7.0 + 30), (5.0 + 30, 9.0 + 30), (5.0 + 30, 4.0 + 30)]]
list_of_obstacle = []
for obs in obsList:
    list_of_obstacle.append(obs.vertexExpanded)


# environment.store(boundary_coordinates, list_of_holes, validate=True, export_plots=True)
map.store(boundary_coordinates, list_of_obstacle, validate=False)

map.prepare()


path, length = map.find_shortest_path(start, goal)
print(path, length)
unzippedPath=list(zip(*path))
plt.plot(unzippedPath[0], unzippedPath[1], '--',color='black')

plt.show()
