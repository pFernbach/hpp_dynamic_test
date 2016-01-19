from hpp.corbaserver.box import Robot
from hpp.corbaserver import ProblemSolver
from hpp.gepetto import Viewer
from hpp.gepetto import PathPlayer
white=[1.0,1.0,1.0,1.0]
green=[0.23,0.75,0.2,0.5]
yellow=[0.85,0.75,0.15,1]
pink=[1,0.6,1,1]
orange=[1,0.42,0,1]

robot = Robot ('box')
robot.setJointBounds ("base_joint_xyz", [0,5,0,2,0,2])
robot.client.robot.setDimensionExtraConfigSpace(robot.getNumberDof()) # extraDof for velocitiy

ps = ProblemSolver (robot)
ps.selectPathPlanner("dyn")
v = Viewer (ps)

#v.loadObstacleModel ("iai_maps", "tunnel", "tunnel")
v.loadObstacleModel ("iai_maps", "abstract", "abstract")


q_init = [0,1,1,1,0,0,0]
q_goal = [5,1,1,0.9239,0,-0.3827,0]
v (q_init)

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.addPathOptimizer ("RandomShortcut")

ps.solve ()

v.displayRoadmap("rmB",white,0.01,1,green)
pp = PathPlayer (robot.client, v)

pp (0)
pp (1)
