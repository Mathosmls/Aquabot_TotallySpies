import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class MapModifierNode(Node):
    def __init__(self):
        super().__init__('map_modifier_node')
        
        # Souscription au topic /map
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10  # Taille du queue (10 messages en file d'attente)
        )
        
        # Publication sur un topic /modified_map
        self.publisher = self.create_publisher(OccupancyGrid, '/modified_map', 10)

        # Variable pour stocker la carte modifiée
        self.modified_map = None

        # Définir un timer pour republier la carte modifiée toutes les 2 secondes
        self.timer = self.create_timer(5.0, self.publish_modified_map)  # 2 secondes d'intervalle

    def map_callback(self, msg: OccupancyGrid):
        # Modifier la carte reçue
        self.modified_map = self.modify_map(msg)
        
        # Exporter les données de la carte dans un fichier texte
        self.export_map_to_text(self.modified_map)

    def modify_map(self, original_map: OccupancyGrid) -> OccupancyGrid:
        # Créer une copie de la carte originale
        modified_map = original_map
        
        # Exemple de modification : changer la valeur de certaines cellules (par exemple, une petite zone au centre de la carte)
        width = modified_map.info.width
        height = modified_map.info.height
        data = modified_map.data
        
        # Exemple : modifier une zone spécifique de la carte pour la rendre occupée (valeur 100)
        center_x = width // 2
        center_y = height // 2
        radius = 100  # rayon autour du centre à modifier
        
        for dx in range(-radius, radius+1):
            for dy in range(-radius, radius+1):
                if 0 <= center_x + dx < width and 0 <= center_y + dy < height:
                    index = (center_y + dy) * width + (center_x + dx)
                    data[index] = 100  # Valeur d'occupation (100 = occupé)
        
        # Créer un nouvel en-tête pour la carte modifiée (avec le même timestamp et frame_id que l'original)
        # modified_map.header = Header()
        # modified_map.header.stamp = self.get_clock().now().to_msg()
        # modified_map.header.frame_id = original_map.header.frame_id
        
        return modified_map

    def publish_modified_map(self):
        # Vérifier si nous avons une carte modifiée à publier
        if self.modified_map is not None:
            # Publier la carte modifiée
            self.publisher.publish(self.modified_map)
            self.get_logger().info('Carte modifiée publiée sur /modified_map.')

    def export_map_to_text(self, map: OccupancyGrid):
        # Accéder aux dimensions de la carte et aux données
        width = map.info.width
        height = map.info.height
        data = map.data

        # Ouvrir un fichier texte en mode écriture
        file_path = '/tmp/map_data.txt'  # Tu peux choisir un chemin spécifique
        with open(file_path, 'w') as file:
            # Écrire les dimensions de la carte
            file.write(f"Map dimensions: {width}x{height}\n")
            file.write(f"Resolution: {map.info.resolution} m/pixel\n\n")
            
            # Écrire les données de la carte sous forme de grille
            for y in range(height):
                row = data[y * width:(y + 1) * width]  # Extraire la ligne correspondante
                row_str = ' '.join([str(value) for value in row])  # Convertir chaque valeur en chaîne
                file.write(row_str + '\n')
            
            self.get_logger().info(f"Carte exportée dans le fichier {file_path}.")

def main(args=None):
    rclpy.init(args=args)
    node = MapModifierNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
