# projet_M1_GUERRIER_LAGALLE

A* et détecteur du volant violet (+ suivi du volant)




# Utilisation du code




## Détection du volant et suivi du volant violet




Ce code se compose de 2 parties.

Une première qui permet de publier le flux vidéo de la caméra dans un topic ROS.

Et une deuxième qui traite l'image, détecte le blob et donne les indications au robot pour se deplacer.

### Pour le TurtleBot2 :

Connecter sur l'ordinateur la base Kobuki du robot (et les autre capteurs si vous voulez les utiliser)
A savoir que pour le hokuyo, il doit surement manquer un package à installer (via un sudo apt install) pour le faire fonctionner, à verifier
Verifier que les EXPORT pour ROS sont en local (pour le roscore)

##### Lancer le robot (bringup, voir .bashrc)
$ start_robot
ou
$ roslaunch turtlebot_bringup start.launch

##### Lancer la publication du flux video
$ cam
ou
$ roslaunch ball_detection usb_cam.launch

##### Lancer le code de detection du volant violet
$ rosrun ball_detection detector_follow
(voir .bashrc pour l'alias qui doit etre 'follow' ou 'detect_follow')



### Pour turtlebot 3 (donatello_rp)

Le package devrait etre present en local sur le robot et fonctionnel. Il est donc possible de lancer le code sur le robot en ssh

##### Lancer le robot (bringup, voir .bashrc)
$ start_robot
(voir .bashrc si la comande est differente)

##### Lancer la publication du flux video
$ cam

ou

$ roslaunch ball_detection usb_cam.launch

##### Lancer le code de detection du volant violet
$ rosrun ball_detection detector_follow

(voir .bashrc pour l'alias qui doit etre 'follow' ou 'detect_follow')




## A*



Le programme pour A*

##### Lancer un roscore
$ roscore

##### Lancer rviz
$ rviz

##### Charger une carte sur rviz
$rosrun map_server map_server {map path}

##### Lancer le code de A*
$ rosrun ball_detection a_etoile



Pour utiliser l'algorithme, placer une première flèche de départ (verte) puis une flèche d'arrivée (violette) ; un chemin en vert devrait apparaitre sur la carte.

Si vous replacez ensuite une flèche d'arrivée, le point de départ sera l'ancien point d'arrivée.

Vous pouvez également placer une nouvelle flèche de départ à la place.


