import numpy as np
np.random.seed(42)
nbplants=5
l = 10*np.random.random(size=(nbplants,2))-5*np.ones((nbplants,2))
l2 = 0.13*np.random.random((nbplants))+0.02*np.ones((nbplants))
print(l)
plants_poses = ["{} {} 0.5 0 0 0".format(x,y) for (x,y) in l]
plants_sizes = [i for i in l2]
print(plants_poses)
print(plants_sizes)
