import numpy as np
import matplotlib.pyplot as plt

from extremitypathfinder.extremitypathfinder import PolygonEnvironment as Environment


#%% asdf


issactest=np.array([[1,2,3],[1,2,4]])
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
obs1= Obstacle([(2,2), (3, 3), (4,2), (3, 1)])
obs2=  Obstacle([(6,10), (11, 11), (4+8+5,2+8), (3+8, 1+8)])
obs3=  Obstacle([(2+15,2+15), (3+15, 3+15), (4+15,2+15), (3+15, 1+15)])
obs4 = Obstacle([(2+15+7,2+15), (3+15+7, 3+15), (4+15+7,2+15), (3+15+7, 1+15)])
obsList= [obs1,obs2, obs3, obs4]
start = (0.5, 0.5)
goal = (27,25-4)

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
boundary_coordinates = [(0.0, 0.0), (50.0, 0.0), (50.0, 50.0), (0.0, 50.0)]

# clockwise numbering!
# list_of_obstacle = [[(3.0, 7.0), (5.0, 9.0), (5.0, 4.0)],
#                     [(3.0 + 30, 7.0 + 30), (5.0 + 30, 9.0 + 30), (5.0 + 30, 4.0 + 30)]]
list_of_obstacle = []
for obs in obsList:
    list_of_obstacle.append(obs.vertexExpanded)


# environment.store(boundary_coordinates, list_of_holes, validate=True, export_plots=True)
map.store(boundary_coordinates, list_of_obstacle, validate=False)

map.prepare()

start_coordinates = (0.0, 0.0 )
goal_coordinates = (5.0 + 34, 4.0 + 35)
path, length = map.find_shortest_path(start, goal)
print(path, length)
unzippedPath=list(zip(*path))
plt.plot(unzippedPath[0], unzippedPath[1], '--',color='black')

plt.show()
print("")


#%%
plt.plot(np.array([0.5,1.2,1.9]), np.array([1,2,3]), 'o')
plt.show()
