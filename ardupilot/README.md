# Alignement du lacet pendant QLAND

Autolander publie une mesure MAVLink `LANDING_TARGET` contenant la position 3D,
l’orientation du marqueur et l’instant de capture. Le contrôleur Lua s’exécute
sur le FC et commande le lacet pendant la descente native en `QLAND`.

Le [script `qland_yaw_alignment.lua`](https://github.com/bertrand-awz/ardupilot/blob/master/libraries/AP_Scripting/applets/qland_yaw_alignment.lua),
sa [documentation](https://github.com/bertrand-awz/ardupilot/blob/master/libraries/AP_Scripting/applets/qland_yaw_alignment.md)
et ses tests unitaires sont maintenus dans le [dépôt ArduPilot patché](https://github.com/bertrand-awz/ardupilot),
avec les bindings `quadplane:set_landing_yaw_rate(deg_s)` et
`quadplane:clear_landing_yaw_rate()` dont il dépend. Le script se trouve dans
`libraries/AP_Scripting/applets/`, à côté de `plane_precland.lua`.

## Firmware et installation

Docker clone `https://github.com/bertrand-awz/ardupilot.git` à la révision
`ardupilot=` fixée dans [`simulation/versions.txt`](../simulation/versions.txt),
puis initialise les sous-modules et compile le firmware. Cette révision contient
l'API de lacet et les deux applets. Les scripts et le lanceur sont inclus dans
l'image ; aucune copie locale d'ArduPilot n'est nécessaire pour la construction.
Depuis la racine du dépôt Autolander :

```bash
docker compose -f compose.yaml -f compose-full-sim.yaml up --build --force-recreate -d dev
COMPOSE_FILE=compose.yaml:compose-full-sim.yaml bash scripts/launch-simulation.sh
```

Pour une construction Docker sans Compose :

```bash
docker build -t avion-cargo-dev:ubuntu24.04 .
```

Pour intégrer de nouvelles modifications du firmware ou du Lua, les pousser dans
le dépôt patché, puis mettre à jour `ardupilot=` avec le nouveau commit avant
de reconstruire l'image.

Pour un FC physique, compiler ArduPlane depuis une copie de ce même dépôt patché
à la révision indiquée dans `simulation/versions.txt`
pour la carte avec le scripting activé, puis installer le firmware et copier
`libraries/AP_Scripting/applets/qland_yaw_alignment.lua` dans `APM/scripts/`
sur la carte SD. Suivre la [documentation de l’applet](https://github.com/bertrand-awz/ardupilot/blob/master/libraries/AP_Scripting/applets/qland_yaw_alignment.md)
pour les paramètres `QYAW_*`, les prérequis et les limites du contrôleur.
Conserver également `plane_precland.lua` et la configuration `PLND_*` pour le centrage XY.

Le contrôleur est désactivé par défaut. Activer `SCR_ENABLE=1`, redémarrer le FC,
puis attendre `QYAW: loaded` avant d’activer le contrôleur :

```text
param set QYAW_ENABLE 1
```

Sur cette révision, les paramètres créés par Lua ne sont pas initialisés par
`--add-param-file`/`--defaults` de SITL : les régler par MAVLink après chargement.

L'alignement du cap est configuré exclusivement avec les paramètres `QYAW_*`
sur le FC. L'ancien superviseur Python et ses commandes de vol ont été supprimés.
Retirer le bloc `yaw_alignment` des anciens fichiers JSON : il est désormais
rejeté par la validation de configuration. Autolander publie les mesures de
position et d'orientation dans `LANDING_TARGET`.

## Descente de 20 m avec un objectif de 45 secondes

Le scénario tricopter vise un contact au sol en moins de 45 secondes en conditions
nominales. Le centrage garde la priorité : une correction qui impose une pause,
une intervention du pilote ou un failsafe peut prolonger cette durée. Il n'y a
pas de minuterie qui force l'atterrissage ou coupe les moteurs à 45 secondes.

Le profil est défini dans
[`tricopter-vtol.parm`](../simulation/config/tricopter-vtol.parm) :

```text
Q_WP_SPD_DN          1.5
Q_LAND_FINAL_ALT    8
Q_LAND_FINAL_SPD    0.3
PLND_XY_DIST_MAX    2.5
```

Sur la révision utilisée, `Q_WP_SPD_DN` est en m/s. L'ancien nom
`Q_WP_SPEED_DN` exprimait des cm/s et n'est plus utilisé pour ce réglage.
`PLND_XY_DIST_MAX=2.5` conserve la pause de descente demandée par l'applet de
centrage lorsque la cible détectée est à plus de 2,5 m horizontalement.

ArduPilot ralentit progressivement sur les six mètres au-dessus de
`Q_LAND_FINAL_ALT`. Le calcul idéal, sans pauses ni retard du contrôleur, donne :

| Hauteur | Consigne de descente | Durée calculée |
| --- | --- | --- |
| 20 → 14 m | 1,5 m/s | 4,0 s |
| 14 → 8 m | Transition progressive de 1,5 à 0,3 m/s | 8,0 s |
| 8 m → sol | 0,3 m/s | 26,7 s |
| Total | | **38,7 s** |

Cela laisse environ 6,3 secondes de marge nominale. Le centrage XY peut agir dès
l'acquisition de la cible et continue jusqu'à ce que son applet cesse de la suivre ;
l'alignement du cap reste autorisé de 8 m à `QYAW_MIN_HGT`, soit 4 m par défaut.
L'essai SITL natif, sans décentrage initial, a mesuré un contact en **38,45 s**
depuis **20,32 m**, dont **26,50 s** sous 8 m. Ces mesures ne constituent pas
une garantie de durée sur l'avion réel ou avec des pauses de centrage.

Pour appliquer les valeurs à un SITL déjà démarré, utiliser MAVProxy :

```text
param set Q_WP_SPD_DN 1.5
param set Q_LAND_FINAL_ALT 8
param set Q_LAND_FINAL_SPD 0.3
param set PLND_XY_DIST_MAX 2.5
```

Le firmware doit inclure l'API de lacet pendant la phase finale, présente dans
la révision patchée fixée par Docker. Les paramètres de descente seuls n'ajoutent
pas cette possibilité à un ancien binaire.

## Vérification

Les tests unitaires accompagnent le script dans ArduPilot. Avec une copie du
dépôt patché dans `../ardupilot`, depuis la racine d’Autolander :

```bash
lua5.4 ../ardupilot/libraries/AP_Scripting/tests/qland_yaw_alignment_test.lua ../ardupilot/libraries/AP_Scripting/applets/qland_yaw_alignment.lua
```

Essai natif isolé dans le conteneur, après construction du firmware corrigé :

```bash
docker compose exec -e ARDUPILOT_SITL_ROOT=/home/dev/ardupilot dev \
  .venv/bin/pytest -q -s tests/integration/test_qland_yaw_sitl.py
```

Cet essai charge les deux applets depuis `ARDUPILOT_SITL_ROOT`.
Il démarre sa propre instance SITL 41, port TCP 6170, dans un répertoire
temporaire, et la termine à la fin. Il vérifie une correction pendant la descente,
la perte de cible, l'expiration après arrêt de Lua et le maintien du failsafe
radio. Un second scénario charge les paramètres du profil de descente depuis le
dépôt, vérifie le contact au sol avant 45 secondes en conditions nominales,
introduit une erreur de cap après l'entrée en phase finale et vérifie la coupure
du lacet sous la hauteur minimale. Le contact est mesuré avec l'état physique
SITL, sans attendre le désarmement. Les mesures de cible sont synthétiques et
centrées sous le véhicule ; ce test ne couvre pas les pauses pour un décentrage
initial, la détection de
l'image Gazebo, les erreurs de pose du marqueur ou la consommation électrique.
