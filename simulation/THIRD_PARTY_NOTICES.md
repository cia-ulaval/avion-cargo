# Origine des fichiers

Ce scénario rassemble les fichiers utilisés localement avec Autolander.

- `worlds/iris_runway_aruco.sdf` adapte le monde `worlds/iris_runway.sdf` du [plugin ArduPilot pour Gazebo](https://github.com/ArduPilot/ardupilot_gazebo) en ajoutant la cible d'atterrissage. Le chemin de sa texture est rendu relatif dans cette copie.
- `models/gimbal_small_3d/` provient du même dépôt, avec les réglages locaux de la caméra conservés dans `model.sdf`. Les autres fichiers du modèle sont copiés sans modification.
- `config/gazebo-iris-gimbal.parm` provient du même dépôt et est conservé sans modification.
- La licence du dépôt source est reproduite dans [LICENSE.ardupilot_gazebo.md](LICENSE.ardupilot_gazebo.md). Sa révision figure dans [versions.txt](versions.txt).
- Le fichier `models/gimbal_small_3d/model.config` conserve les attributions d'origine : modèle Gazebo de Rhys Mainwaring et géométrie MotorPixie, avec les références du modèle 3D.
- La texture ArUco, la configuration Autolander et la calibration proviennent de l'installation locale utilisée pour l'essai. Le chemin de calibration dans le JSON est adapté à ce dossier.
