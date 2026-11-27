# Simulation complète d'Autolander

Ce dossier fournit les mondes Iris et tricopter, la caméra modifiée, les paramètres SITL
et la configuration d'Autolander. La simulation utilise une cible FArUco `FRACTAL_5L_6`
de **8 pieds de côté (2,4384 m)**, mesurés sur le marqueur extérieur hors marge blanche.

La procédure suit les quatre terminaux de l'essai. Elle démarre le contrôleur et le monde, relaie les images vers ROS 2, puis lance Autolander lorsque les deux flux sont disponibles.

## Ce qu'il vous faut

Installez Autolander, ArduPilot SITL, ROS 2 Jazzy, Gazebo Harmonic et le plugin ArduPilot en suivant le wiki. Pour le pont d'images utilisé ici, installez également :

```bash
sudo apt install ros-jazzy-ros-gz-image
poetry install --with faruco
```

La commande `image_bridge` est décrite dans la [documentation de ros_gz_image](https://docs.ros.org/en/jazzy/p/ros_gz_image/). Les révisions présentes lors de la copie sont conservées dans les fichiers `versions/*.version` et `versions/*.distribution`. Utilisez ces révisions d'ArduPilot et du plugin pour retrouver la même base de simulation. Docker clone et compile le [dépôt ArduPilot patché](https://github.com/bertrand-awz/ardupilot), avec son API de lacet QLAND, et utilise la révision fixée du plugin ArduPilot Gazebo.

Les modèles standards `runway`, `iris_with_gimbal` et `iris_with_standoffs` restent fournis par le plugin ArduPilot. Le dossier `models/gimbal_small_3d` contient la caméra modifiée et doit être recherché avant les modèles standards.

## Les fichiers de l'essai

| Fichier | Utilisation |
| --- | --- |
| [worlds/runway_iris_faruco.sdf](worlds/runway_iris_faruco.sdf) | Monde Iris avec une cible FArUco au sol ; son nom interne reste `iris_runway`. |
| [worlds/markers/faruco.png](worlds/markers/faruco.png) | Texture fractale à cinq niveaux, référencée par un chemin relatif. |
| [models/gimbal_small_3d/model.sdf](models/gimbal_small_3d/model.sdf) | Caméra de 640 × 480 pixels, cadence demandée de 30 Hz et réglages optiques de l'essai. |
| [config/gazebo_iris.parm](config/gazebo_iris.parm) | Paramètres du véhicule et de la nacelle pour SITL. |
| [config/autolanding_iris_config.json](config/autolanding_iris_config.json) | Configuration d'Autolander pour le iris. |
| [config/autolanding_vtol_config.json](config/autolanding_vtol_config.json) | Configuration d'Autolander pour le vtol. |
| [calibration/pi_camera_calibration.yaml](calibration/pi_camera_calibration.yaml) | Calibration référencée lors de l'essai, conservée avec ses valeurs d'origine. |

Les sources et attributions des fichiers sont indiquées dans [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

La texture de 1440 pixels contient un marqueur extérieur de 1248 pixels et une marge
blanche de 96 pixels de chaque côté. Les surfaces Gazebo mesurent donc 2,8135384615 m
de côté, afin que le marqueur lui-même mesure exactement 2,4384 m, comme `targeted_marker.length`.
Le niveau fractal est choisi automatiquement par le détecteur selon la portion visible.
L'ancien monde `runway_iris_aruco.sdf` reste disponible pour des essais ArUco avec une configuration adaptée.

## Essai manuel du payload avec trilatération

Cet essai utilise une application indépendante de `payload_manual`, avec son propre
point d’entrée Python, sa propre interface Tauri/Vue et son monde Gazebo à trois ancres.
Pour lancer le scénario depuis la racine du dépôt :

```bash
docker compose -f compose.yaml -f compose-payload-manual-trilateration.yaml up --build --force-recreate -d dev
COMPOSE_FILE=compose.yaml:compose-payload-manual-trilateration.yaml bash scripts/launch-simulation.sh
```

La configuration utilisée est `simulation/config/manual_payload_trilateration_config.json`.
Le scénario legacy reste accessible par `compose-payload-manual.yaml` et utilise toujours
le paquet `payload_manual` et son interface d’origine.

## Tricopter QuadPlane avec Docker

Docker clone `https://github.com/bertrand-awz/ardupilot.git` à la révision
`ardupilot=` de [versions/ardupilot.version](versions/ardupilot.version), qui contient l'API de lacet et
l'applet `qland_yaw_alignment.lua`. Aucune copie locale d'ArduPilot n'est requise.
Depuis la racine d'Autolander, activez le scénario tricopter puis lancez la simulation :

```bash
docker compose -f compose.yaml -f compose-full-sim.yaml up --build --force-recreate -d dev
COMPOSE_FILE=compose.yaml:compose-full-sim.yaml bash scripts/launch-simulation.sh
```

Ajoutez vos fichiers Compose habituels pour l'affichage ou le GPU aux deux commandes.
La préparation Docker installe le groupe optionnel `faruco`. Pour un conteneur existant
dont les dépendances ont déjà été préparées, exécutez `docker compose exec dev poetry install --with dev,faruco`.
Après un changement de cible ou de dimensions, relancez Gazebo et Autolander pour charger
à la fois le monde et les paramètres de détection.
Les lanceurs du conteneur sont maintenus dans [docker/launchers](../docker/launchers).
Le script [docker/install-launchers.sh](../docker/install-launchers.sh) crée leurs liens dans `/usr/local/bin` lors de la construction de l’image.
Les lanceurs sont exécutés depuis le dépôt monté sous `/workspace/avion-cargo`;
leurs modifications sont donc prises en compte sans reconstruire l'image. Les paramètres
de simulation sont lus depuis ce même dépôt.

Après une modification du Dockerfile, arrêtez la simulation puis reconstruisez et
recréez le conteneur avec la commande ci-dessus. Si seules les sources des lanceurs,
les variables d'environnement ou les publications de ports de Compose ont changé,
utilisez `docker compose -f compose.yaml -f compose-full-sim.yaml up --force-recreate -d dev`,
avec les mêmes fichiers Compose X11/NVIDIA éventuels. Un simple `restart` n'applique
pas ces changements. Voir les [options de `docker compose up`](https://docs.docker.com/reference/cli/docker/compose/up/).

Avant chaque démarrage d'ArduPlane, le lanceur vérifie et installe
`scripts/plane_precland.lua` et `scripts/qland_yaw_alignment.lua` dans le dossier ArduPilot utilisé comme répertoire de travail
de SITL. Les deux applets proviennent de `libraries/AP_Scripting/applets/`
dans le dépôt patché cloné dans l'image. Le lanceur répare un lien
absent ou cassé et refuse de démarrer si la source manque. Un script personnalisé déjà
présent est préservé et signalé pour inspection s'il diffère de l'applet fournie.

Pour démarrer uniquement SITL dans le conteneur configuré pour le scénario tricopter :

```bash
docker compose exec dev /usr/local/bin/avion-sitl
```

Si `avion-sitl` affiche directement `SIM_VEHICLE: Start` sans le message
`QuadPlane precision landing script ready`, l'ancien lanceur est encore installé dans
le conteneur. Reconstruisez l'image et recréez le conteneur avec `up --build --force-recreate -d dev`,
en conservant vos fichiers Compose. Une recréation seule conserve le lanceur de
l'image existante ; une modification du dépôt n'actualise pas cette copie.

Le terminal doit afficher `QuadPlane precision landing script ready`, puis MAVProxy
doit recevoir `PLND: Loaded`. Le fichier [tricopter-vtol.parm](config/tricopter-vtol.parm)
active `SCR_ENABLE=1`, `PLND_ENABLED=1`, `PLND_TYPE=1` et affecte `RC7_OPTION=39`.
En `QLOITER`, activez RC7 avec `rc 7 1900` et vérifiez `PLND: PrecLoiter enabled`.
En `QLAND`, l'applet applique automatiquement les corrections lorsque la cible est acquise.

Le script de lacet doit également annoncer `QYAW: loaded; QLAND yaw controller`.
Après ce message, activez-le dans MAVProxy avec `param set QYAW_ENABLE 1`.
Ce paramètre Lua n'est disponible qu'après chargement du script et n'est pas
initialisé par le fichier de valeurs par défaut de SITL sur cette révision.
`QYAW_MIN_HGT=4` l'arrête sous quatre mètres au-dessus de la cible.
Le Dockerfile compile le firmware et inclut les applets depuis le dépôt patché.
Pour intégrer une modification du Lua ou des sources ArduPilot, la pousser dans
ce dépôt, actualiser le commit `ardupilot=` de [versions/ardupilot.version](versions/ardupilot.version),
puis exécuter `up --build --force-recreate -d dev`.
Les paramètres, le protocole et l'installation sur un FC sont décrits dans le
[guide d'alignement Lua](https://github.com/cia-ulaval/avion-cargo/wiki/Alignement-du-lacet-QLAND).

Le profil tricopter vise une descente nominale de 20 m en moins de 45 secondes
jusqu'au contact : 1,5 m/s en hauteur, ralentissement entre 14 et 8 m, puis
0,3 m/s sous 8 m. Le calcul idéal donne 38,7 s, dont 26,7 s à basse vitesse.
Le centrage reste prioritaire et peut prolonger la descente au-delà de 45 s.
Voir le [profil détaillé](https://github.com/cia-ulaval/avion-cargo/wiki/Alignement-du-lacet-QLAND#descente-de-20-m-avec-un-objectif-de-45-secondes).

`PrecLand: Init Complete` confirme seulement l'initialisation de l'estimateur : ce message
ne prouve pas que l'applet de commande est chargée. Sans elle, la vision peut détecter le
marqueur et transmettre `LANDING_TARGET` sans que le QuadPlane se recentre.
Si `PLND: Loaded` manque malgré l'installation, vérifiez les paramètres effectivement
chargés par SITL, notamment `SCR_ENABLE` et `PLND_ENABLED`, ainsi que les erreurs Lua.
Redémarrez SITL après la restauration du script; un processus déjà lancé ne constitue
pas une validation de son chargement au démarrage.

### QGroundControl sur l'hôte avec Docker

Le service `dev` publie une sortie MAVLink TCP de MAVProxy sur
`127.0.0.1:5761` de l'hôte. Le lanceur lit `AVION_SITL_GCS_OUT`, défini par Compose
à `tcpin:0.0.0.0:5761`, et ajoute cette sortie au démarrage. La liaison TCP `5760`
reste utilisée par MAVProxy pour communiquer avec ArduPlane ; la sortie UDP
`127.0.0.1:14550` reste destinée à Autolander dans le conteneur.

Après cette mise à jour, arrêtez la simulation avec `Ctrl+C`, puis appliquez les
ports et l'environnement au conteneur depuis la racine du dépôt sur l'hôte :

```bash
export COMPOSE_FILE=compose.yaml:compose-full-sim.yaml
docker compose up --force-recreate -d dev
bash scripts/launch-simulation.sh
```

Conservez vos fichiers Compose X11/NVIDIA éventuels dans `COMPOSE_FILE` ; pour Iris,
omettez `compose-full-sim.yaml`. Si le lanceur embarqué dans l'image a aussi changé,
ajoutez `--build`. `restart` seul n'applique pas les nouveaux ports ni les nouvelles
variables d'environnement.

Dans QGroundControl sur le même hôte, ouvrez **Settings → Comm Links → Add New Link** :

- Nom : `SITL Docker`.
- Type : **TCP**.
- Adresse du serveur : `127.0.0.1`.
- Port : `5761`.

Enregistrez, puis sélectionnez la liaison et cliquez sur **Connect**. Le journal
de lancement de MAVProxy doit contenir `--out tcpin:0.0.0.0:5761`, en plus de sa sortie
UDP habituelle. Dans la console MAVProxy, `output` permet de vérifier les sorties.
Depuis l'hôte, `docker compose port dev 5761` doit afficher `127.0.0.1:5761`.

L'adresse `0.0.0.0` fait écouter MAVProxy sur les interfaces du conteneur, ce qui
permet la publication Docker. Côté hôte, le port est limité à la boucle locale.
La sortie est ajoutée automatiquement à chaque lancement ; il n'est pas nécessaire
de saisir `output add` à la main. Cette sortie reste optionnelle : le lanceur ne
l'ajoute que si `AVION_SITL_GCS_OUT` est défini et non vide dans le conteneur.
Voir la [gestion des liaisons QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/comm_links.html).

## Démarrer avec quatre terminaux

Ouvrez chaque terminal à la racine du dépôt Autolander, là où se trouve `pyproject.toml`. Les exemples supposent qu'ArduPilot est installé dans `~/ardupilot` et son environnement Python dans `~/venv-ardupilot` ; adaptez ces chemins à votre installation.

### Terminal 1 : ArduPilot SITL

```bash
AUTOLANDER_ROOT="$PWD"
source "$HOME/venv-ardupilot/bin/activate"
cd "$HOME/ardupilot"
sim_vehicle.py -D -v ArduCopter -f JSON \
  --add-param-file="$AUTOLANDER_ROOT/simulation/config/gazebo_iris.parm"
```

Cette commande reprend celle de l'essai. Le message `no config for frame (JSON)` peut apparaître avec cette révision ; le fichier de paramètres est fourni explicitement. SITL attend les échanges avec Gazebo. Attendez le heartbeat dans MAVProxy et laissez ce terminal ouvert. MAVProxy transmet sa sortie à `127.0.0.1:14550`, comme attendu dans le JSON. Réservez ce port à Autolander si vous ajoutez une station sol.

### Terminal 2 : Gazebo

Chargez ROS 2 avec `source /opt/ros/jazzy/setup.zsh` sous Zsh, ou `source /opt/ros/jazzy/setup.bash` sous Bash. Depuis la racine d'Autolander :

```bash
ARDUPILOT_GAZEBO_DIR="$HOME/gz_ws/src/ardupilot_gazebo"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$PWD/simulation/models:$PWD/simulation/worlds:$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -v4 -r simulation/worlds/runway_iris_faruco.sdf
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

Si l'état reste `NOT_FOUND`, vérifiez que la cible est visible, que `faruco-bridge` est installé,
puis le type `faruco`, le preset `FRACTAL_5L_6`, la taille `2.4384` et la texture chargée par Gazebo.

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
