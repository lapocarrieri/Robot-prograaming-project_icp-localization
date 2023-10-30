bool collisionDetected(const Eigen::Vector2f& current_laser_point, const Eigen::Vector2f& actual_point, const Eigen::Isometry2f& _laser_in_world, std::vector<Eigen::Vector2f>& prediction) {
    // Verifica la presenza di ostacoli tra i punti
    bool collision = false;

    // Calcola la distanza tra il punto corrente del laser e il punto previsto
    float distance_to_actual = (actual_point - current_laser_point).norm();

    // Calcola la distanza tra il punto corrente del laser e il punto "_laser_in_world"
    float distance_to_laser_in_world = (current_laser_point - _laser_in_world.translation()).norm();

    // Calcola la distanza tra il punto previsto e il punto "_laser_in_world"
    float distance_to_predicted = (actual_point - _laser_in_world.translation()).norm();

    // Verifica se i tre punti sono allineati
    if (isAligned(current_laser_point, actual_point, _laser_in_world.translation())) {
        // Se i punti sono allineati, elimina il punto più lontano da "prediction"
        float max_distance = std::max({distance_to_actual, distance_to_laser_in_world, distance_to_predicted});
        if (max_distance == distance_to_actual) {
            // Elimina "actual_point" da "prediction" se è il punto più lontano
            prediction.erase(std::remove(prediction.begin(), prediction.end(), actual_point), prediction.end());
        } else if (max_distance == distance_to_laser_in_world) {
            // Elimina "current_laser_point" da "prediction" se è il punto più lontano
            prediction.erase(std::remove(prediction.begin(), prediction.end(), current_laser_point), prediction.end());
        } else {
            // Elimina "_laser_in_world.translation()" da "prediction" se è il punto più lontano
            prediction.erase(std::remove(prediction.begin(), prediction.end(), _laser_in_world.translation()), prediction.end());
        }
    }

    // Qui dovresti eseguire un controllo accurato sulla mappa per verificare la presenza di ostacoli tra i punti.
    // Se vengono rilevati ostacoli, imposta "collision" su "true".

    return collision;
}

bool isAligned(const Eigen::Vector2f& point1, const Eigen::Vector2f& point2, const Eigen::Vector2f& point3) {
    // Verifica se i tre punti sono allineati
    return (point1 - point2).cross(point1 - point3) == 0.0;
}
