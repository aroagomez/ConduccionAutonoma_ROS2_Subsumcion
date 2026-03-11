# Conduccion Autonoma con ROS2 y Arquitectura de Subsumcion

Este proyecto implementa un sistema de conducción autónoma para un robot móvil (Yahboom) capaz de navegar en un entorno controlado. El sistema integra percepción visual para el seguimiento de carreteras y reconocimiento de señales de tráfico, coordinado mediante una arquitectura de control reactiva.

## Tecnologias y Entorno
* **Framework:** ROS2 (Robot Operating System).
* **Simulacion:** Gazebo (escenario yahboom_track_signs).
* **Hardware:** Robot real Yahboom con cámara integrada.
* **Lenguaje:** Python / C++.

## Arquitectura de Control: Subsumcion
Para la toma de decisiones en tiempo real, se ha implementado una **Arquitectura de Subsumción**. Este enfoque jerárquico permite que comportamientos de alto nivel (como reaccionar a una señal) tomen el control sobre los comportamientos base:

1. **Road Following (Nivel Base):** Control de bajo nivel que mantiene al robot dentro de los límites del carril mediante procesamiento de imagen.
2. **Autonomous Driving (Nivel Superior):** Lógica de decisión que identifica señales de tráfico (STOP, giros, etc.) y "subsume" o anula el control del nivel base para ejecutar maniobras específicas.



## Estructura del Repositorio
El proyecto está dividido en paquetes de ROS2 especializados:
* **/road_following:** Paquete encargado de la segmentación de carril, detección de líneas y cálculo del error de trayectoria para el seguimiento de carretera.
* **/autonomus_driving:** Nodo principal que gestiona la detección de señales y la máquina de estados de la arquitectura de subsunción.
* **Conduccion_autonoma_INFORME.pdf:** Memoria técnica detallada con el análisis de las pruebas en simulación y entorno real.

## Analisis y Resultados
El informe técnico incluido analiza la transición del entorno de simulación al real, documentando los desafíos en la calibración de hiperparámetros de visión y las estrategias probadas para optimizar la respuesta del robot ante señales consecutivas.
