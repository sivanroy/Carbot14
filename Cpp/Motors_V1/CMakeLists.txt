# Nous voulons un cmake "récent" pour utiliser les dernières fonctionnalités
cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON)

# Notre projet est étiqueté hello
project(motors)

# locate lib
find_package(WiringPi REQUIRED)

# Include headers
include_directories(${WIRIGNPI_INCLUDE_DIRS})

#link against libs
targe_link_libraries(motors ${WIRINGPI_LIBRARIES})

# Crée des variables avec les fichiers à compiler
set(SRCS
    main.cpp
    DualMC33926RPi.cpp
    )
    
set(HEADERS
    DualMC33926RPi.h
    )

# On indique que l'on veut un exécutable "hello" compilé à partir des fichiers décrits par les variables SRCS et HEADERS
add_executable(motors ${SRCS} ${HEADERS})