import random
import numpy as np

def create_world(file, name):
    f = open(file, "w")
    f.write('<?xml version="1.0"?>\n\
    <sdf version="1.4">\n\
      <world name="{}">\n\n'.format(name))

    # pose = "0 0 0.5 0 0 0"
    # radius = 1
    # length = 1
    # color = "0 1 0 1"
    # create_plant(f, "plant1", pose, radius, length, color)
    #
    # size = "10 0.1 0.5"
    # create_wall(f, "wall1", pose, size)

    create_walls(f)
    create_floor(f, "floor", "0 0 -0.01 0 0 0", "10 10 0.02", 0.1, 0.1, "0 0 1", "240 240 240 1")
    create_random_plants(f, nbplants=10)

    f.write('\n\n  </world>\n\
    </sdf>')
    f.close()

def create_plant(f, name, pose, radius, length, color):
    f.write('<model name="{}">\n\
  <pose>{}</pose>\n\
  <static>true</static>\n\
  <link name="body">\n\
    <visual name="visual">\n\
      <geometry>\n\
        <cylinder>\n\
          <radius>{}</radius>\n\
          <length>{}</length>\n\
        </cylinder>\n\
      </geometry>\n\
      <material>\n\
    <ambient>{}</ambient>\n\
    <diffuse>{}</diffuse>\n\
  </material>\n\
    </visual>\n\
  </link>\n\
</model>\n\n'.format(name, pose, radius, length, color, color))

def create_random_plants(f, nbplants):
    np.random.seed(42)
    l = 10*np.random.random(size=(nbplants,2))-5*np.ones((nbplants,2))
    l2 = 0.13*np.random.random((nbplants))+0.02*np.ones((nbplants))
    plants_poses = ["{} {} 0.05 0 0 0".format(x,y) for (x,y) in l]
    plants_radiuses = [i for i in l2]
    length = 0.1
    color = "0 1 0 1"
    for i in range(nbplants):
        create_plant(f, "plant"+str(i), plants_poses[i], plants_radiuses[i], length, color)


def create_wall(f, name, pose, size, color):
    f.write('<model name="{}">\n\
      <pose>{}</pose>\n\
      <static>true</static>\n\
      <link name="body">\n\
        <visual name="visual">\n\
          <geometry>\n\
            <box>\n\
              <size>{}</size>\n\
            </box>\n\
          </geometry>\n\
          <material>\n\
        <ambient>{}</ambient>\n\
        <diffuse>{}</diffuse>\n\
      </material>\n\
        </visual>\n\
        <collision name="collision">\n\
          <geometry>\n\
            <box>\n\
              <size>{}</size>\n\
            </box>\n\
          </geometry>\n\
        </collision>\n\
      </link>\n\
    </model>\n\n'.format(name, pose, size, color, color, size))

def create_floor(f, name, pose, size, mu, mu2, fdir1, color):
    f.write('<model name="{}">\n\
      <pose>{}</pose>\n\
      <static>true</static>\n\
      <link name="body">\n\
        <visual name="visual">\n\
          <geometry>\n\
            <box>\n\
              <size>{}</size>\n\
            </box>\n\
          </geometry>\n\
          <material>\n\
        <ambient>{}</ambient>\n\
        <diffuse>{}</diffuse>\n\
      </material>\n\
        </visual>\n\
        <collision name="collision">\n\
          <geometry>\n\
            <box>\n\
              <size>{}</size>\n\
            </box>\n\
          </geometry>\n\
          <surface>\n\
                <friction>\n\
                  <ode>\n\
                    <mu>{}</mu>\n\
                    <mu2>{}</mu2>\n\
                    <fdir1>{}</fdir1>\n\
                  </ode>\n\
                </friction>\n\
              </surface>\n\
        </collision>\n\
      </link>\n\
    </model>\n\n'.format(name, pose, size, color, color, size, mu, mu2, fdir1))

def create_walls(f):
    poses = ["-5 0 0.25 0 0 1.57079", "5 0 0.25 0 0 1.57079", "0 -5 0.25 0 0 0", "0 5 0.25 0 0 0"]
    size = "10 0.1 0.5"
    color = "100 100 100 1"
    for i in range(4):
        name = "wall"+str(i)
        create_wall(f,name,poses[i], size, color)




if __name__ == "__main__":
    create_world("test_world.world", "default")
