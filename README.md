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
- OpencvPython : pip install opencv-python
- Simple_pid : pip install simple-pid
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
roslaunch robot_global_result global_result.launch gui:=true
```
ou 
```
roslaunch robot_global_result global_result.launch gui:=false

```

## Auteurs

Antony ARSLANYAN		https://github.com/arslanan
<br/>Juliette BRUGIER		https://github.com/brugieju
<br/>Maria Luiza COSTA VIANNA   https://github.com/marialuizacvianna
<br/>Sarah DELMAS
<br/>Olivier LAURENDIN
<br/>Louis VALERY               https://github.com/LouisValery







