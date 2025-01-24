#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include <map>
#include <list>
#include <tuple>
#include <cmath>
#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>

////
////// Commande pour lancer le script : rosrun a_etoile a_etoile_code
////



char* map_data;
int width;
int height;
ros::Publisher path_pub;
nav_msgs::Path path_test;
geometry_msgs::PoseStamped start;
geometry_msgs::PoseStamped end;
float resolution = 0.01;
float origin_x = -12.2;
float origin_y = -15.4;
int x_end, y_end;
std::vector<std::pair<int,int>> directions = {{-1,0}, {1,0}, {0,-1}, {0,1}};
// std::vector<std::pair<int,int>> directions = {{-1,0}, {1,0}, {0,-1}, {0,1},
//                                               {-1,-1}, {1,-1}, {-1,1}, {1,1}};


//struct Node decrivant chaque point, chaque noeud à parcourir
struct Node{
  int x,y;
  int g,h;
  Node* parent;//Node precedente dans le chemin

  Node(int x, int y, int g, int h, Node* parent = nullptr) : x(x), y(y), g(g), h(h), parent(parent) {
  
  }

  int getF() const {return g+h;} //Score du Node utiliser dans le NodeComparator

  //Operator de comparaison pour savoir si un Node est au meme endroit qu'un autre (si c'est le meme)
  bool operator==(const Node& other) const {
    return x == other.x && y == other.y;
  } 
};

//operator permettant de comparer un node à un autre. Operator utilise pour la priority_queue
struct NodeComparator {
  bool operator()(const Node* lhs, const Node* rhs) const {
    return lhs->getF() > rhs->getF();
  }
};


//Distance de manhattan entre 2 points
int manhattanDist(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}


//Fonction permettant de savoir si le point {x,y} est atteignable (si mur ou vide ou autre)
bool isValid(int x, int y){
  bool inMap = x>=0 && x<=width && y>=0 && y<=height;
  bool isWallOrUnkown = map_data[y*width + x] == 100 || map_data[y*width + x] == -1;
  return inMap && !isWallOrUnkown;
}


//Fonction retournant les Nodes voisinne à la Node current
std::vector<Node> getVoisins(Node* current) {
  std::vector<Node> voisins;

  //Parcours des direction pour avoir les nodes voisinnes accessibles 
  for(auto dir : directions) {
    int newX,newY;

    newX = current->x + dir.first;
    newY = current->y + dir.second;
    if(isValid(newX, newY)) {//Si la node voisinne est accessible, on l'ajoute à la liste
      voisins.push_back(Node(newX, newY, current->g + 1, manhattanDist(newX, x_end, newY, y_end), current));
    }
  }
  return voisins;
}


//Fonction permettant de remplir le Path ROS
nav_msgs::Path constructPath(Node* node) {
  nav_msgs::Path new_path;

  //Header du path
  new_path.header.frame_id = "map";
  int seq = 0;
  new_path.header.seq = seq;

  //Bouclage sur les node tant qu'on en à une qui existe
  while(node != nullptr) {

    geometry_msgs::PoseStamped pose_stamped;

    //Header du PoseStamped pose_stamped
    pose_stamped.header.seq = seq++;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map";

    //Creation du point dans le path avec les coordonnées
    pose_stamped.pose.position.x = (node->x * resolution) + origin_x;
    pose_stamped.pose.position.y = (node->y * resolution) + origin_y;
    pose_stamped.pose.orientation.w = 1.0; //Quaternion sans direction

    //ROS_INFO("Pose : x = %f, y = %f", pose_stamped.pose.position.x, pose_stamped.pose.position.y); //Info des point du path

    new_path.poses.push_back(pose_stamped); //Ajout du point à la fin du path
    
    node = node->parent; //On passe au node suivant pour remonter le chemin
  }
  // ROS_INFO("Path size : %ld", new_path.poses.size()); //Info de la taill du chemin
  new_path.header.stamp = ros::Time::now(); //Time header du path

  return new_path;
}



//Algo A*
nav_msgs::Path a_star(geometry_msgs::Pose debut, geometry_msgs::Pose fin){
  
  std::priority_queue<Node*, std::vector<Node*>, NodeComparator> openList; //Priority queue des nodes base sur leur f_score via le NodeComparator
  std::unordered_map<int, Node*> openMap;                                  //unordered_map permettant de retrouver facilement les elements de la queue

  std::vector<std::vector<bool>> closedList(height, std::vector<bool>(width, false)); //Carte de false permettant de savoir si le Noeud à été traité ou non
  
  nav_msgs::Path final_path; //Creation d'un path
  
  
  //Coordonnée en pixel du point de départ
  int x_start = ((debut.position.x - origin_x)/resolution);
  int y_start = ((debut.position.y - origin_y)/resolution);
  ROS_INFO("Origine en pixel du point start : x = %d, y = %d", x_start, y_start); //Retour d'info

  //Coordonnée en pixel du point de fin
  x_end = ((fin.position.x - origin_x)/resolution);
  y_end = ((fin.position.y - origin_y)/resolution);
  ROS_INFO("Origine en pixel du point de fin : x = %d, y = %d", x_end, y_end); //Retour d'info

  //Creation de la premiere Node avec le point de début
  Node* startNode = new Node(x_start, y_start, 0, manhattanDist(x_start, y_start, x_end, y_end));

  //On l'ajoute aux 2 liste pour commencer le calcul
  openList.push(startNode);
  openMap[startNode->x * startNode->y] = startNode;

  while(!openList.empty()) {
    //On prend la Node avec la plus haute priorité
    Node* current = openList.top();
    openList.pop();
    openMap.erase(current->x * current->y);

    //Si le point que l'on traite et le point d'arriver, on fini la boucle et retrace le chemin
    if(current->x == x_end && current->y == y_end) {
      return constructPath(current);
    }

    //On declare que le point à été traité 
    closedList[current->y][current->x] = true;

    //On obtient la liste des voisin de la Node en cours
    std::vector<Node> voisins = getVoisins(current);
    for(auto& voisin : voisins) {

      //Si la node à déjà été traité, on passe un tour de boucle
      if(closedList[voisin.y][voisin.x]) {
        continue;
      }


      if(openMap.find(voisin.x*voisin.y) == openMap.end()) {
        //Si voisin pas dans la liste ouverte on le rajoute

        Node* voisinNode = new Node(voisin.x, voisin.y, voisin.g, voisin.h, current);
        openList.push(voisinNode);
        openMap[voisinNode->x * voisinNode->y] = voisinNode;
      }
    }
  }
  //Si le code arrive ici, aucun chemain n'a été trouvé
  nav_msgs::Path empty_path; //On retourne un Path vide
  ROS_INFO("No path found"); //Retour d'info
  return(empty_path);
}











//Callback pose point de départ
void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
  ROS_INFO("I heard start position: x =[%lf] y = [%lf]", msg.pose.pose.position.x,msg.pose.pose.position.y);
  start.pose = msg.pose.pose; //Mise à jour du point de départ
  
}


//Callback pose point à atteindre
void endPoseCallback(const geometry_msgs::PoseStamped& msg){

  ROS_INFO("I heard final position: x =[%lf] y = [%lf]", msg.pose.position.x,msg.pose.position.y);
  // path_test.header.stamp = ros::Time::now();
  // path_test.header.frame_id = "map";
  end = msg; //Mise à jour du point

  //A*   Calcul du chemin via A*
  nav_msgs::Path a_star_path = a_star(start.pose,end.pose);
  //

  x_end = ((end.pose.position.x - origin_x)/resolution);
  y_end = ((end.pose.position.y - origin_y)/resolution);

  // path_test.poses.push_back(msg);
  path_pub.publish(a_star_path); //Envoi du Path sur le topic /path

  start.pose = end.pose; //Mise à jour de point de départ comme le point normalement atteint via le Path
}

//Callback du topic /map lors de la réception de la map
void mapCallback(const nav_msgs::OccupancyGrid& msg){
  ROS_INFO("Recieved map : %d * %d Res : %f", msg.info.width,msg.info.height,msg.info.resolution);

  //Copie de la map dans map_data
  map_data = (char*)malloc(msg.info.width*msg.info.height*sizeof(char));
  for(int i = 0; i<msg.info.width*msg.info.height; i++){
      map_data[i] = msg.data[i];

  }
  //Récuperation de la taille de la map
  width = msg.info.width;
  height = msg.info.height;
}










int main(int argc, char **argv){

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  //Subscriber pour les callback
  ros::Subscriber sub_init = n.subscribe("/initialpose", 1000, initPoseCallback);
  ros::Subscriber sub_end = n.subscribe("/move_base_simple/goal", 1000, endPoseCallback);
  ros::Subscriber sub_map = n.subscribe("/map", 2, mapCallback);

  //Publisher pour le Path
  path_pub = n.advertise<nav_msgs::Path>("/path", 1000);

  ros::spin();

  return 0;
}