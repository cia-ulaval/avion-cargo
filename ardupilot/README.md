# Alignement du lacet pendant QLAND

Autolander publie une mesure MAVLink `LANDING_TARGET` contenant la position 3D,
l'orientation du marqueur et l'instant de capture. Le script
[`qland_yaw_alignment.lua`](scripts/qland_yaw_alignment.lua) calcule l'erreur de cap
sur le FC et commande uniquement une vitesse de lacet. Le véhicule reste en
`QLAND` et conserve sa descente native : le script ne commande ni changement de
mode, ni palier, ni gaz, ni override RC.

Cette suppression des paliers d'alignement peut réduire le temps de vol ; le gain
de batterie n'a pas été mesuré. Si le temps de descente restant est insuffisant,
le véhicule poursuit l'atterrissage sans garantir l'alignement complet.

## Firmware nécessaire

Le firmware est maintenu directement dans le dépôt voisin `../ardupilot`.
Les sources `ArduPlane/quadplane.{h,cpp}` et les bindings Lua de ce dépôt
fournissent `quadplane:set_landing_yaw_rate(deg_s)` et
`quadplane:clear_landing_yaw_rate()`. Aucun fichier patch n'est conservé dans
Autolander. L'override RC a été écarté car il peut rafraîchir l'activité radio et
masquer une perte du récepteur.
La consigne expire dans le firmware après **200 ms** sans rafraîchissement.
Elle est acceptée uniquement en QLAND armé, pendant la descente et la phase finale,
avec AHRS sain, sans failsafe actif et sans entrée pilote de roulis, tangage ou lacet.
En phase finale, le firmware refuse la consigne à un mètre ou moins selon sa hauteur
estimée au-dessus du sol. Le Lua conserve sa propre coupure, à quatre mètres
au-dessus du marqueur par défaut, même si la phase finale commence plus haut.
Chaque changement de mode efface la consigne. Cette API ne modifie pas les
entrées RC ni leurs horodatages. Un firmware standard sans cette API produit
un message explicite au chargement du script et n'effectue aucune correction.

Docker compile les sources locales de `../ardupilot`, y compris les modifications
non commitées, via le contexte de construction additionnel `avion-cargo` de Compose,
qui pointe vers le dossier parent.
Ce dépôt doit donc être présent à côté d'Autolander. Les sous-modules sont
initialisés dans l'image ; les scripts et le lanceur y sont également inclus.
Depuis la racine du dépôt Autolander :

```bash
docker compose -f compose.yaml -f compose-full-sim.yaml up --build --force-recreate -d dev
COMPOSE_FILE=compose.yaml:compose-full-sim.yaml bash scripts/launch-simulation.sh
```

Pour une construction Docker sans Compose, fournir le même contexte :

```bash
docker build --build-context avion-cargo=.. -t avion-cargo-dev:ubuntu24.04 .
```

Pour un FC physique, compiler ArduPlane depuis ce même dépôt `../ardupilot`
pour la carte avec le scripting activé, puis installer le firmware et copier le
Lua dans `APM/scripts/` sur la carte SD. Ce travail ne flashe aucun FC. Les essais
fournis couvrent SITL ; le comportement sur le matériel et le budget mémoire de
sa carte restent à valider.

Activer `SCR_ENABLE=1`, redémarrer le FC et vérifier le message `QYAW: loaded`.
Les paramètres `QYAW_*` deviennent disponibles après chargement du script ;
actualiser la liste des paramètres dans la station sol. Conserver également
`plane_precland.lua` et la configuration `PLND_*` pour le centrage XY. Ce script
de centrage peut toujours ralentir ou interrompre la descente selon ses propres
paramètres. `PLND_YAW_ALIGN` décrit l'orientation du capteur de précision : il
ne définit pas le cap d'atterrissage désiré.

## Réglages sur le FC

Le script réserve la table de paramètres Lua **93** et le préfixe `QYAW_`.
Vérifier l'absence de conflit avec d'autres scripts installés. Le contrôleur est
désactivé par défaut, y compris dans SITL. Après `QYAW: loaded`, l'activer dans
MAVProxy ou la station sol :

```text
param set QYAW_ENABLE 1
```

Cette activation est sauvegardée par le FC. Sur cette révision, les paramètres
créés par Lua ne sont pas initialisés par `--add-param-file`/`--defaults` de
SITL : les régler par MAVLink après le chargement du script.

| Paramètre | Défaut | Rôle |
| --- | --- | --- |
| `QYAW_ENABLE` | 0 | Mettre à 1 pour activer la correction. |
| `QYAW_SYSID` / `QYAW_COMPID` | 255 / 191 | Source MAVLink du compagnon autorisée pour les mesures. |
| `QYAW_MIN_HGT` | 4 m | Hauteur verticale minimale au-dessus du marqueur, déduite de la pose et de l'attitude. Minimum accepté : 1 m. |
| `QYAW_DIST_MAX` | 2 m | Distance horizontale maximale à la cible. |
| `QYAW_MAX_AGE` | 0,4 s | Durée maximale depuis la réception d'une nouvelle capture ; borne aussi l'accumulation de retard de transport. Plage : 0,05–0,5 s. |
| `QYAW_STABLE` | 0,5 s | Acquisition continue avant correction. Plage : 0,1–5 s. |
| `QYAW_TOL` | 3° | Tolérance d'alignement ; reprise au-delà de tolérance + 2°. Plage : 1–20°. |
| `QYAW_RATE_MAX` | 20°/s | Limite de vitesse commandée. Maximum accepté par le script : 45°/s. |
| `QYAW_GAIN` | 1/s | Gain proportionnel appliqué à l'erreur de cap. Plage : ]0, 5]. |
| `QYAW_FORWARD` | 0° | Axe avant désiré dans le plan du marqueur : +X à 0°, +Y à 90°. |

Garder `Q_WVANE_ENABLE=0` pour éviter deux objectifs de cap concurrents. Les
paramètres invalides empêchent la correction. Le script tourne à 20 Hz et
libère le lacet si la cible disparaît, devient invalide, sort de l'enveloppe,
si un failsafe se déclenche ou si le pilote reprend les commandes. Une perte
de mesures n'interrompt pas l'atterrissage. Après une exception Lua, la consigne
est libérée ; l'expiration native couvre aussi un arrêt complet du moteur Lua.

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

Le firmware du dépôt `../ardupilot` doit également être reconstruit pour autoriser le lacet
pendant la phase finale. Les paramètres de descente seuls n'ajoutent pas cette
possibilité à un ancien binaire.

## Mesure MAVLink

La publication utilise les extensions de
[`LANDING_TARGET` MAVLink 2](https://mavlink.io/en/messages/common.html#LANDING_TARGET) :

| Champ | Convention |
| --- | --- |
| `x`, `y`, `z` | Position du marqueur en mètres, repère `MAV_FRAME_BODY_FRD` : avant, droite, bas. |
| `q` | Quaternion unitaire `(w,x,y,z)` transformant les axes du marqueur vers BODY_FRD. |
| `time_usec` | Capture sur l'horloge monotone du compagnon, en microsecondes. Une capture n'est publiée qu'une fois. |
| `target_num` | 0 : une cible physique configurée, même lorsque le niveau FArUco visible change. |
| `type`, `position_valid` | `VISION_FIDUCIAL`, 1. |

Position et quaternion proviennent du même `body_from_marker`, qui inclut le
montage de caméra. Sans orientation connue, le quaternion est nul : le FC peut
utiliser la position pour le centrage, mais le script refuse de corriger le lacet.
Autolander cesse de publier les captures âgées de 400 ms ou plus.

Le Lua corrige le roulis et le tangage avant de projeter l'axe du marqueur dans
le plan horizontal. Une erreur positive demande une rotation horaire vue du
dessus. Une projection presque verticale est rejetée. La direction choisie
dépend donc de la convention d'axes du détecteur et de `QYAW_FORWARD` ; elle doit
être vérifiée avec le marqueur et le montage réellement utilisés.

Le timestamp 64 bits est conservé en quatre mots de 16 bits, compatibles avec
le Lua 32 bits d'ArduPilot. Les doublons et retours en arrière ne rafraîchissent
pas la cible. La durée locale utilise les compteurs `uint32_t` du FC, y compris
leur rebouclage. Les horloges n'ont pas besoin d'être synchronisées : le script
mesure l'âge à réception et les variations de retard de transport. Cela ne
mesure pas la latence absolue du premier paquet ni ne compense le mouvement
pendant l'exposition/le transport. Le lien doit conserver une faible latence.
Si l'horloge du compagnon repart en arrière après redémarrage, l'alignement
reste suspendu jusqu'à la prochaine sortie de QLAND/désactivation.

## Vérification

Tests du script, sans FC :

```bash
lua tests/ardupilot/qland_yaw_alignment_test.lua ardupilot/scripts/qland_yaw_alignment.lua
```

Essai natif isolé dans le conteneur, après construction du firmware corrigé :

```bash
docker compose exec -e ARDUPILOT_SITL_ROOT=/home/dev/ardupilot dev \
  .venv/bin/pytest -q -s tests/integration/test_qland_yaw_sitl.py
```

Cet essai démarre sa propre instance SITL 41, port TCP 6170, dans un répertoire
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
