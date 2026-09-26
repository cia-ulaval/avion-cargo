# Simulation complète d'Autolander

Ce dossier fournit le monde et les réglages utilisés pour l'essai du 25 septembre 2026. Il rassemble la cible ArUco, la caméra modifiée, les paramètres SITL et la configuration d'Autolander. Vous pouvez ainsi reprendre une simulation complète depuis le dépôt.

La procédure suit les quatre terminaux de l'essai. Elle démarre le contrôleur et le monde, relaie les images vers ROS 2, puis lance Autolander lorsque les deux flux sont disponibles.

## Ce qu'il vous faut

Installez Autolander, ArduPilot SITL, ROS 2 Jazzy, Gazebo Harmonic et le plugin ArduPilot en suivant le wiki. Pour le pont d'images utilisé ici, installez également :

```bash
sudo apt install ros-jazzy-ros-gz-image
```

La commande `image_bridge` est décrite dans la [documentation de ros_gz_image](https://docs.ros.org/en/jazzy/p/ros_gz_image/). Les révisions présentes lors de la copie sont conservées dans [versions.txt](versions.txt). Utilisez ces révisions d'ArduPilot et du plugin pour retrouver la même base de simulation.

Les modèles standards `runway`, `iris_with_gimbal` et `iris_with_standoffs` restent fournis par le plugin ArduPilot. Le dossier `models/gimbal_small_3d` contient la caméra modifiée et doit être recherché avant les modèles standards.

## Les fichiers de l'essai

| Fichier | Utilisation |
| --- | --- |
| [worlds/iris_runway_aruco.sdf](worlds/iris_runway_aruco.sdf) | Monde Iris avec une cible au sol ; son nom interne reste `iris_runway`. |
| [worlds/markers/aruco.png](worlds/markers/aruco.png) | Texture de la cible, référencée par un chemin relatif. |
| [models/gimbal_small_3d/model.sdf](models/gimbal_small_3d/model.sdf) | Caméra de 640 × 480 pixels, cadence demandée de 30 Hz et réglages optiques de l'essai. |
| [config/gazebo-iris-gimbal.parm](config/gazebo-iris-gimbal.parm) | Paramètres du véhicule et de la nacelle pour SITL. |
| [config/autolanding_config.json](config/autolanding_config.json) | Configuration d'Autolander : topic caméra, cible ID 29 du dictionnaire 16, côté de 0,896 m et port MAVLink 14550. |
| [calibration/pi_camera_calibration.yaml](calibration/pi_camera_calibration.yaml) | Calibration référencée lors de l'essai, conservée avec ses valeurs d'origine. |

Les sources et attributions des fichiers sont indiquées dans [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## Démarrer avec quatre terminaux

Ouvrez chaque terminal à la racine du dépôt Autolander, là où se trouve `pyproject.toml`. Les exemples supposent qu'ArduPilot est installé dans `~/ardupilot` et son environnement Python dans `~/venv-ardupilot` ; adaptez ces chemins à votre installation.

### Terminal 1 : ArduPilot SITL

```bash
AUTOLANDER_ROOT="$PWD"
source "$HOME/venv-ardupilot/bin/activate"
cd "$HOME/ardupilot"
sim_vehicle.py -D -v ArduCopter -f JSON \
  --add-param-file="$AUTOLANDER_ROOT/simulation/config/gazebo-iris-gimbal.parm"
```

Cette commande reprend celle de l'essai. Le message `no config for frame (JSON)` peut apparaître avec cette révision ; le fichier de paramètres est fourni explicitement. SITL attend les échanges avec Gazebo. Attendez le heartbeat dans MAVProxy et laissez ce terminal ouvert. MAVProxy transmet sa sortie à `127.0.0.1:14550`, comme attendu dans le JSON. Réservez ce port à Autolander si vous ajoutez une station sol.

### Terminal 2 : Gazebo

Chargez ROS 2 avec `source /opt/ros/jazzy/setup.zsh` sous Zsh, ou `source /opt/ros/jazzy/setup.bash` sous Bash. Depuis la racine d'Autolander :

```bash
ARDUPILOT_GAZEBO_DIR="$HOME/gz_ws/src/ardupilot_gazebo"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$PWD/simulation/models:$PWD/simulation/worlds:$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -v4 -r simulation/worlds/iris_runway_aruco.sdf
```

Si vous avez suivi l'installation du wiki dans `~/ardupilot_gazebo`, utilisez `ARDUPILOT_GAZEBO_DIR="$HOME/ardupilot_gazebo"`. Gardez les deux dossiers du scénario en tête de `GZ_SIM_RESOURCE_PATH` : Gazebo doit charger la caméra fournie ici. La simulation doit rester en lecture pour publier les images. Le topic attendu est `/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image`.

### Terminal 3 : pont d'images ROS 2

Chargez ROS 2 dans ce terminal avec le fichier `setup.zsh` ou `setup.bash` correspondant à votre shell, puis lancez :

```bash
ros2 run ros_gz_image image_bridge \
  /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

Dans un autre terminal où ROS 2 est chargé, vérifiez que le pont publie bien un message `sensor_msgs/msg/Image` :

```bash
ros2 topic list -t
ros2 topic info /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image --verbose
ros2 topic hz /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

Arrêtez `ros2 topic hz` avec `Ctrl+C`, mais laissez `image_bridge` fonctionner.

### Terminal 4 : Autolander

Si l'environnement ArduPilot est actif dans ce terminal, quittez-le avec `deactivate` avant d'utiliser Poetry. Chargez ROS 2 avec le fichier `setup.zsh` ou `setup.bash`, puis, depuis la racine du dépôt :

```bash
poetry run precision_landing simulation/config/autolanding_config.json --gz-simulation
```

Le JSON trouve la calibration à partir de son propre dossier. Le journal doit confirmer la connexion MAVLink, `ROS camera ready`, le démarrage du suivi et le serveur WebRTC sur `http://127.0.0.1:8085`. Ouvrez cette adresse et cliquez sur **Start stream**. Vérifiez le renouvellement de l'image, la présence de la cible et les valeurs de pose.

Si l'état reste `NOT_FOUND`, vérifiez d'abord que la cible est visible, puis le dictionnaire `16`, l'ID `29`, la taille `0.896` et la texture chargée par Gazebo.

Pour terminer, arrêtez Autolander avec `Ctrl+C`, puis le pont d'images, SITL et Gazebo dans leurs terminaux.

## Tester l'atterrissage dans SITL

Le lancement d'Autolander ne déclenche pas l'atterrissage. Après avoir vérifié l'image, la détection et MAVLink, configurez ArduCopter dans la console MAVProxy du terminal 1 :

```text
param set PLND_ENABLED 1
param set PLND_TYPE 1
```

Redémarrez SITL si ArduPilot le demande, puis reproduisez la procédure de vol simulé prévue par votre configuration. Par exemple, seulement lorsque SITL est prêt :

```text
mode guided
arm throttle
takeoff 5
mode land
```

MAVProxy ou la station sol restent responsables de l'armement, du décollage, du mode `LAND` et de l'arrêt d'urgence. Autolander transmet la position de la cible ; il ne commande pas ces actions.

## État du scénario conservé

L'essai fourni montrait une connexion MAVLink et la réception des images, avec l'état de détection `NOT_FOUND`. Il ne valide donc pas encore un atterrissage ni la précision des distances.

La calibration copiée est celle utilisée lors de cet essai. Ses paramètres ne correspondent pas à ceux du modèle de caméra : par exemple, `fx` vaut environ 622,94 dans le YAML et 599,86 dans le SDF. De plus, Gazebo signalait que le modèle de distorsion Brown demandé n'était pas pris en charge par le moteur Ogre2. Avant d'utiliser les distances, établissez une calibration correspondant aux images réellement produites par Gazebo. Le guide de simulation du wiki décrit les formats attendus.

Les paramètres SITL locaux sauvegardés dans `~/ardupilot` ne sont pas inclus ; seul le fichier `.parm` passé à la commande est conservé. Si vous avez changé des paramètres dans MAVProxy, conservez aussi ces valeurs pour reproduire cet état.
