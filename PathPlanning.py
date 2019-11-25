import numpy as np
import matplotlib.pyplot as plt
from extremitypathfinder.plotting import PlottingEnvironment as Environment


#%% asdf
class Obstacle:
    def __init__(self, N, vertexList):
        self.N=N
        self.vertex=vertexList


#obstacle definition
obs1= Obstacle(4, np.array([(2,2), (3, 3), (4,2), (3, 1)]))
obs2=  Obstacle(4, np.array([(2+8,2+8), (3+8, 3+8), (4+8,2+8), (3+8, 1+8)]))
obs3=  Obstacle(4, np.array([(2+15,2+15), (3+15, 3+15), (4+15,2+15), (3+15, 1+15)]))

obsList= [obs1,obs2, obs3]
for obs in obsList:

    plt.scatter(obs.vertex[:,0 ], obs.vertex[:,1 ])

start = np.array([1,1])
goal = np.array([25,25])
plt.scatter(start[0], start[1])
plt.scatter(goal[0], goal[1])

plt.show()
print("")
#%%
boundary_coordinates = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]







#
# NodeList=np.array( [start[0], start[1], -1])
# NodeList=np.vstack([NodeList, [goal[0], goal[1], -2]])
#
# numObs=0
# for obs in obsList:
#     for point in obs.vertex:
#         NodeList = np.vstack([NodeList, [point[0], point[1], numObs]])
#
#     numObs+=1
#
# obs0= NodeList[np.nonzero(NodeList[:,2]==0)[0], :]
#
# print("")




