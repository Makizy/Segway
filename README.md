# Segway
Building a segway with 6v DC motor and MPU6050 


---

# Segway Project README

## Project Overview

This repository contains all the code and documentation for our microelectronics course's Segway project. Our goal was to design and build a functioning Segway prototype with limited hardware resources and a tight budget. The project was structured in two main versions, each addressing specific challenges and improvements.

## Key Components

- Arduino Uno
- MPU6050 
- 6V DC Motor

## Challenges and Solutions

Throughout the project, we encountered and overcame several significant challenges:

### Synchronized Motor Start-Up

**Challenge**: Our initial design could not start the motors simultaneously, affecting the Segway's balance and functionality.

**Solution**: We identified the issue as an inadequate power source and resolved it by upgrading our battery and using adapters to optimize power efficiency. This adjustment ensured that both motors could start and run simultaneously.

### Data Integrity with the IMU

**Challenge**: We experienced data loss from the MPU6050, a critical sensor for maintaining the Segway's balance.

**Solution**: To mitigate this, we isolated the power supply to the motors from the IMU and Arduino to reduce electrical noise interference. Additionally, we developed a custom shield to minimize wire length and ensure a secure, direct connection, effectively eliminating data loss.

### Addressing Gyro-Induced Drift

**Challenge**: Gyro-induced drift was affecting the accuracy and precision of the Segway's movements.

**Solution**: We implemented both complementary and Kalman filters to process the sensor data. This approach significantly improved the Segway's response time and precision.

## Version History

### Version 1: Component Testing and Initial Integration

- Focused on testing the compatibility and functionality of individual components.
- Attempted to synchronize motor operations, highlighting the voltage issue with the motors.



https://github.com/Makizy/Segway/assets/53753128/82f3954d-89e7-4e05-802c-d488cf1644bd


### Version 2: Refinement and Optimization

- Addressed the data loss issue by creating a shield for the MPU6050, ensuring cleaner data transmission.
- Experimented with 9V and 6V motors, finding the 6V option provided better control and stability due to its lower RPM, making it the preferred choice for our final design.



https://github.com/Makizy/Segway/assets/53753128/015e0f77-2a18-4b61-94c8-302de6b02a53



## Conclusion

The Segway project not only tested our engineering and problem-solving skills but also taught us the importance of iteration and flexibility in design. The second version of our project was notably more successful, incorporating lessons learned from the initial phase and further optimizations.

We invite you to explore our project files, code, and documentation to learn more about our journey from concept to completion. Your feedback and contributions are welcome as we continue to refine and improve our Segway prototype.


