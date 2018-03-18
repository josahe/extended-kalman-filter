# Extended Kalman Filter
> A sensor data processing pipeline to estimate the state of a moving object

## INTRODUCTION
The goal of this project is to write a Kalman Filter to estimate the state of a moving object using noisy lidar and radar measurements. The state of the object is parameterised by its x,y position and its velocity.

This project was undertaken as part of the [Udacity Self-Driving Car NanoDegree](https://eu.udacity.com/course/self-driving-car-engineer-nanodegree--nd013).

### Pipeline summary
The Kalman Filter is a continuous loop of measurement updates and predictions. The pipeline is triggered when new measurement data is captured, and causes two updates to the object state: first based on a prediction and then based on the measurement data provided by either radar or lidar sensors. The first invocation of the pipeline takes care of state parameter initialisation and further invocations runs the prediction/measurement algorithms.

The pipeline forks depending on which sensor the measurement has come from. The Kalman Filter equations are similar between the two sensor types - they are essentially extracting the same information. The main differences are:
1. Radar produces data with Polar coordinates, which must be converted to the Cartesian coordinates used to track the object.
2. The equations to map radar data to position/velocity state parameters are non-linear, which produces a non-Gaussian distribution that can then not be used to make reliable predictions with our otherwise Gaussian data. To maintain the desired distribution of our data, the mapping functions are linearised.

## HOW TO USE
### Project dependencies
You can follow the guide in the README of the original project repo.
* https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

## RELEVANT FILES
* [FusionEKF.cpp](src/FusionEKF.cpp)
  * Initialises the state parameters, then calls the relevant Kalman Filter equations based on the sensor
* [kalman_filter.cpp](src/kalman_filter.cpp)
  * Implements the prediction and measurement update algorithms
* [tools.cpp](src/tools.cpp)
  * Implements a Jacobian matrix calculation function and a RMSE function
