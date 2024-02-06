[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/79j9H7gG)
[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-718a45dd9cf7e7f842a935f5ebbe5719a5e09af4491e668f4dbf3b35d5cca122.svg)](https://classroom.github.com/online_ide?assignment_repo_id=13690311&assignment_repo_type=AssignmentRepo)

# Creacion del paquete
El paquete se compone de una carpeta [include](https://github.com/Docencia-fmrico/2024-p2-goforward-jmartinm2021/tree/main/node_forward/include/node_forward) que contiene el archivo [ForwardNode.hpp](https://github.com/Docencia-fmrico/2024-p2-goforward-jmartinm2021/blob/main/node_forward/include/node_forward/ForwardNode.hpp), una carpeta [src](https://github.com/Docencia-fmrico/2024-p2-goforward-jmartinm2021/tree/main/node_forward/src) que contiene el archivo [forward_class.cpp](https://github.com/Docencia-fmrico/2024-p2-goforward-jmartinm2021/blob/main/node_forward/src/forward_class.cpp) y [ForwardNode.cpp](https://github.com/Docencia-fmrico/2024-p2-goforward-jmartinm2021/blob/main/node_forward/src/node_forward/ForwardNode.cpp), y dos archivos, el [CMakeList.txt](https://github.com/Docencia-fmrico/2024-p2-goforward-jmartinm2021/blob/main/node_forward/CMakeLists.txt) y el [package.xml](https://github.com/Docencia-fmrico/2024-p2-goforward-jmartinm2021/blob/main/node_forward/package.xml).

# Creacion del nodo publicador suscriptor
En [ForwardNode.hpp](https://github.com/Docencia-fmrico/2024-p2-goforward-jmartinm2021/blob/main/node_forward/include/node_forward/ForwardNode.hpp) declaramos el nodo:
´´ć

# 2024-P2-GoForward

En esta práctica debes crear un nodo de ROS 2 que cuando se inicie haga avanzar el robot Kobuki a una velocidad moderada (0.2-0.4) hasta que:

1. el bumper del robot detecte la colisión con un obstáculo, ó
2. hayan pasado 5 segundos.

Entonces el robot se parará. 

[5seg.webm](https://github.com/Docencia-fmrico/2024-p2-goforward-jmartinm2021/assets/92941332/9d4b5ac2-482b-4740-a524-784a7a51fc10)

[bumper_stop.webm](https://github.com/Docencia-fmrico/2024-p2-goforward-jmartinm2021/assets/92941332/7484c2a6-fdc6-4307-9715-4ff8d8ac1d47)
