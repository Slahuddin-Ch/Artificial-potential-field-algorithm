


# Code Explaination


### Path Function: 
pathFunction is a function that defines a cubic polynomial path for the robot to follow. It takes an x-coordinate and a vector of coefficients as input and returns the corresponding y-coordinate on the path.

```
// Define the polynomial function for the path (cubic polynomial)
double pathFunction(double x, const std::vector<double>& coefficients) {
    return coefficients[0] * std::pow(x, 3) + coefficients[1] * std::pow(x, 2) +
           coefficients[2] * x + coefficients[3];
}
```

### Attractive Force Function: 
attractiveForce calculates an attractive force towards the path. It aims to pull the robot towards a target point on the path.

```
// Calculate the attractive force towards the path
std::vector<double> attractiveForce(double x, double y, double pathX, const std::vector<double>& coefficients) {
    double pathY = pathFunction(pathX, coefficients);
    double forceX = -1.0 * (x - pathX); // Attractive force towards the path in x
    double forceY = -1.0 * (y - pathY); // Attractive force towards the path in y
    return {forceX, forceY};
}
```

### Repulsive Force Function: 
repulsiveForce calculates a force that repels the robot away from obstacles. This force increases as the robot gets closer to an obstacle.


```
// Calculate the repulsive force away from obstacles
std::vector<double> repulsiveForce(double x, double y, const std::vector<std::vector<double>>& obstacle_positions, double obstacle_radius) {
    double forceX = 0;
    double forceY = 0;

    for (const auto& obstacle : obstacle_positions) {
        double distance = std::hypot(x - obstacle[0], y - obstacle[1]);
        if (distance < obstacle_radius && distance > 0) { // Check to avoid division by zero
            double repulsionStrength = 1.0 / (distance * distance); // Repulsion decreases with distance squared
            forceX += repulsionStrength * (x - obstacle[0]);
            forceY += repulsionStrength * (y - obstacle[1]);
        }
    }

    return {forceX, forceY};
}
```

### Main Function followPath:

1. Initializes the robot's position.
2. Runs a loop that continues as long as ROS is running (ros::ok()).
3. Within the loop, it calculates the attractive force towards the path and the repulsive force from obstacles.
4. Combines these forces to compute a resultant force that dictates the robot's movement.
5. Normalizes this force to control the robot's speed.
6. Updates the robot's position in the Gazebo simulation using a ROS service call.
7. Checks if the robot has reached its goal and exits the loop if so.
