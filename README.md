# Robot désherbant 

https://github.com/arslanan/Robot_dufur

Dans le cadre de l'uv 5.8, nous travaillons sur un robot desherbant pour appliquer la méthode de gestion de projet agile : Scrum. 

## Initialisation

Cloner ou forquer la branche master qui fonctionne toujours. 
La branche dev est parfois en revanche plus avancée mais non testée.

## Prerequis et install

Pour pouvoir lancer la modélisation il faut : 
	
- Rospy : sudo apt-get install python-rospy
- Topic_tools : sudo apt-get install topic_tools
- OpencvPython : pip3 install opencv-python
- Simple_pid : pip3 install simple-pid
- Joint state Controller : sudo apt install ros-melodic-hardware-interface ros-melodic-effort-controllers ros-melodic-joint-state-controller

## Initialisation du workspace

Dans un terminal :
 
```
cd workspaceRos/devel
source setup.sh
source setup.bash
cd ..
catkin_make
```

## Lancement de la modélisation

Dans le même terminal : 

```
roslaunch robot_global_result global_result.launch
```

## Auteurs

Antony ARSLANYAN		https://github.com/arslanan
Juliette BRUGIER		https://github.com/brugieju
Maria Luiza COSTA VIANNA
Sarah DELMAS
Olivier LAURENDIN
Louis VALERY







