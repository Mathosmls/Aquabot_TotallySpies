import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap  # Importer le service GetMap
from geometry_msgs.msg import PoseArray
import numpy as np

class MapModifierNode(Node):
    def __init__(self):
        super().__init__('map_modifier_node')
        self.get_logger().info('map modifier node has started !')

        # Client de service pour obtenir la carte
        self.map_client = self.create_client(GetMap, '/map_server/map')

        # Faire l'appel au service pour récupérer la carte
        request = GetMap.Request()
        future = self.map_client.call_async(request)
        future.add_done_callback(self.map_callback_response)
        # Souscription au topic /map
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10  # Taille du queue (10 messages en file d'attente)
        )

                # Souscription au topic /map
        self.subscription_pos_wt = self.create_subscription(
            PoseArray,
            '/local_wind_turbine_positions',
            self.pos_wt_callback,
            10  # Taille du queue (10 messages en file d'attente)
        )
        
        # Publication sur un topic /modified_map
        self.publisher = self.create_publisher(OccupancyGrid, '/modified_map', 10)

        # Variable pour stocker la carte modifiée
        self.modified_map = None
        self.published_modified_map= False

        self.received_pos_wt=False
        self.pos_wt_array=np.zeros([3, 2])

        # Définir un timer pour republier la carte modifiée toutes les 2 secondes
        self.timer = self.create_timer(0.5, self.publish_modified_map)  # 2 secondes d'intervalle


    def pos_wt_callback(self,msg) :
        for i in range(len(msg.poses)) :
            self.pos_wt_array[i] = np.array([msg.poses[i].position.x,msg.poses[i].position.y])
        self.received_pos_wt =True

    def map_callback(self, msg: OccupancyGrid):
        # Modifier la carte reçue
        self.modified_map = msg

    def map_callback_response(self, future):
        try:
            response = future.result()  # Récupérer la réponse du service
            self.modified_map = response.map  # Carte reçue et stockée
            self.get_logger().info('Carte reçue et prête à être modifiée.')
        except Exception as e:
            self.get_logger().error(f"Échec de l'appel au service map_server/map: {e}")
        

    def modify_map(self, original_map: OccupancyGrid) -> OccupancyGrid:
        # Créer une copie de la carte originale
        modified_map = original_map
       # Extraire les dimensions et la résolution de la carte
        width = modified_map.info.width
        height = modified_map.info.height
        resolution = modified_map.info.resolution  # Résolution de la carte (mètres par pixel)
        origin_x = modified_map.info.origin.position.x  # Origine de la carte en mètres (coin inférieur gauche)
        origin_y = modified_map.info.origin.position.y

        # Convertir les données en liste mutable
        data = modified_map.data

        # Rayon du cercle (en mètres)
        radius_meters = 2.0  # Exemple : rayon de 10 mètres
        radius_pixels = int(radius_meters / resolution)  # Convertir le rayon en cellules

        # Ajouter des cercles pour chaque position dans self.pos_wt_array
        for pos in self.pos_wt_array:
            # Récupérer les coordonnées (en mètres)
            world_x, world_y = pos

            # Convertir en indices de grille
            grid_x = int((world_x - origin_x) / resolution)
            grid_y = int((world_y - origin_y) / resolution)

            # Dessiner le cercle sur la grille
            for dx in range(-radius_pixels, radius_pixels + 1):
                for dy in range(-radius_pixels, radius_pixels + 1):
                    if dx**2 + dy**2 <= radius_pixels**2:  # Vérifier si dans le cercle
                        cell_x = grid_x + dx
                        cell_y = grid_y + dy

                        # Vérifier que la cellule est dans les limites de la grille
                        if 0 <= cell_x < width and 0 <= cell_y < height:
                            index = cell_y * width + cell_x  # Calculer l'indice linéaire
                            data[index] = 100  # Valeur d'occupation (100 = occupé)

        # Réassigner les données modifiées
        modified_map.data = data  # Retour à un tuple immuable si requis
        
        return modified_map

    def publish_modified_map(self):
        # Vérifier si nous avons une carte modifiée à publier
        if self.modified_map is not None and not self.published_modified_map:
            if self.received_pos_wt :
                self.modify_map(self.modified_map)
                # Publier la carte modifiée
                self.publisher.publish(self.modified_map)
                self.get_logger().info('Carte modifiée publiée sur /modified_map.')
                self.published_modified_map=True
            else :
                self.publisher.publish(self.modified_map)
                self.get_logger().info('Carte NON modifiée publiée sur /modified_map.')

                
                



def main(args=None):
    rclpy.init(args=args)
    node = MapModifierNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
