# from extremitypathfinder.extremitypathfinder import PolygonEnvironment as Environment

# enable for plotting:
from extremitypathfinder.plotting import PlottingEnvironment as Environment


if __name__ == "__main__":
    map = Environment()
    # anti clockwise vertex numbering!
    boundary_coordinates = [(0.0, 0.0), (50.0, 0.0), (50.0, 50.0), (0.0, 50.0)]

    # clockwise numbering!
    list_of_obstacle = [[(3.0, 7.0), (5.0, 9.0), (5.0, 4.0)], [(3.0+30, 7.0+30), (5.0+30, 9.0+30), (5.0+30, 4.0+30)] ]
    # environment.store(boundary_coordinates, list_of_holes, validate=True, export_plots=True)
    map.store(boundary_coordinates, list_of_obstacle, validate=False)

    map.prepare()


    start_coordinates = (4.5-2, 1.0+4)
    goal_coordinates =  (5.0+34, 4.0+35)
    path, length = map.find_shortest_path(start_coordinates, goal_coordinates)
    print(path, length)
