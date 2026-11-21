# Stabilisation QuadPlane enregistrée

`quadplane_hold_settling.csv` contient les 14 premières secondes de télémétrie
`GLOBAL_POSITION_INT` après la consigne `DO_REPOSITION` de l'essai SITL/Gazebo
du 14 novembre 2026 à 19:39:47 (America/Toronto).

Les positions sont exprimées en mètres par rapport à la consigne de maintien ;
la vitesse verticale est positive vers le bas. Aucune coordonnée géographique
absolue n'est nécessaire au test. La position arrive à environ 4 Hz, tandis que
le superviseur tourne à 10 Hz.

La vitesse et la position respectent leurs seuils suffisamment longtemps juste
après dix secondes. L'ancien superviseur échouait au tick intermédiaire avec
`hold_stabilization_timeout`, puis restait au neutre même après stabilisation.
Le test rejoue aussi les ticks sans nouveau message afin de conserver ce cas.
