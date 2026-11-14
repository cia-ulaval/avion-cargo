# Alignement autonome du lacet QuadPlane

Le superviseur tourne dans le thread `autolander-yaw-supervisor`. Il gère la séquence
`QLAND → GUIDED VTOL → alignement → QLAND` sans opérateur. La vision et la publication
de `LANDING_TARGET` continuent dans leurs boucles respectives. Il intervient une fois
que le drone est armé en `QLAND`; il ne déclenche ni l’armement ni le décollage.

## Pose et conventions

Les mesures ArUco et FArUco fournissent une transformation rigide homogène 4×4,
avec une translation en mètres. `RigidTransform` conserve une copie immuable des données;
`as_matrix()` retourne un tableau NumPy indépendant.

```python
body_from_marker = camera_mount.body_from_camera.compose(camera_from_marker)
# Équivalent matriciel :
T_body_marker = T_body_camera @ T_camera_marker
position_body = T_body_marker[:3, 3]
```

`T_camera_marker` transforme un point du repère marqueur vers le repère optique caméra.
`T_body_marker` le transforme vers BODY_FRD : avant, droite, bas. La configuration
`camera.mount` décrit la pose fixe de la caméra dans ce repère. Une caméra sur nacelle
mobile demande une transformation de montage actualisée; ce superviseur suppose un montage fixe.

La télémétrie expose ces matrices dans `transforms.camera_from_marker` et
`transforms.body_from_marker`. Les anciennes positions `poses` restent disponibles.
Le champ `yaw_alignment` contient l’état, l’erreur en degrés et le motif d’arrêt éventuel.

L’axe +X du marqueur représente son avant lorsque `marker_forward_deg` vaut zéro.
Une valeur de 90° choisit son axe +Y. Le calcul projette cet axe dans le plan horizontal
NED à l’aide de l’attitude du véhicule; il compense donc son roulis et son tangage.
Une erreur positive demande une rotation horaire vue du dessus.

## Activation

Le superviseur est désactivé lorsque la section suivante est absente. Elle est activée
dans `simulation/config/autolanding_config_tricopter_vtol.json`.

```json
"yaw_alignment": {
  "enabled": true,
  "marker_forward_deg": 0.0,
  "trigger_deg": 12.0,
  "tolerance_deg": 3.0,
  "stable_time_s": 1.0,
  "min_height_m": 3.0,
  "max_horizontal_distance_m": 2.0,
  "max_yaw_input": 0.25
}
```

Par défaut, la boucle tourne à 10 Hz. Elle exige une pose et des échantillons d’attitude
et de position âgés de moins de 0,4 s, une vitesse horizontale inférieure à 0,8 m/s,
et une cible à au moins 3 m sur l’axe bas du véhicule. Les paramètres du contrôleur sont
lus avant toute prise de contrôle. Le passage en `GUIDED` initialise sa destination
à proximité de la position et de l’altitude actuelles; le superviseur attend l’arrêt
de la descente (vitesse verticale absolue ≤ 0,3 m/s) avant de corriger le lacet.

La commande utilisée est `RC_CHANNELS_OVERRIDE` sur le seul canal `RCMAP_YAW`.
Le code lit `RCx_MIN`, `RCx_TRIM`, `RCx_MAX`, `RCx_DZ` et `RCx_REVERSED`, compense
la zone morte et limite la demande à une fraction de l’entrée normalisée.
Les autres canaux sont ignorés. Une demande nulle envoie le neutre calibré;
la fin de contrôle libère explicitement le canal, y compris les canaux MAVLink étendus.

Ce chemin est choisi parce que le traitement `SET_ATTITUDE_TARGET` d’ArduPlane
ne constitue pas une interface de consigne de cap pour le contrôleur VTOL utilisé ici.
La prise de contrôle exige :

- `Q_ENABLE > 0`, `Q_GUIDED_MODE = 1` et `STICK_MIXING != 0`;
- `Q_WVANE_ENABLE = 0`, pour éviter une commande concurrente face au vent;
- les overrides RC autorisés (`RC_OPTIONS` bit 1 désactivé);
- `0 < RC_OVERRIDE_TIME <= 1` seconde, pour borner la persistance d’une commande après interruption;
- un identifiant MAVLink du compagnon égal à `MAV_GCS_SYSID`, ou `SYSID_MYGCS` sur les versions antérieures;
- un canal de lacet entre 1 et 16, avec une calibration valide.

Les paramètres de simulation du tricopter fixent `STICK_MIXING=1` et
`RC_OVERRIDE_TIME=0.5`. Le code ne modifie aucun paramètre du contrôleur en cours de vol.

## États et reprises

Le superviseur confirme les modes à partir des heartbeats; l’envoi d’une demande
ne vaut pas confirmation. L’alignement doit rester dans la tolérance sur de nouvelles
images pendant une seconde. Le retour en `QLAND` doit également être confirmé par un
heartbeat postérieur à la demande. Une temporisation de dix secondes évite les basculements répétés.

Une perte de cible, une mesure périmée ou un dépassement des quinze secondes
d’alignement libère le lacet et demande le mode d’origine. Une perte de liaison
conserve la propriété de la séquence pour tenter ce retour à la reconnexion.
Un changement vers un mode externe, par exemple `QRTL`, suspend la séquence sans
forcer le mode précédent. Un échec reste visible dans la télémétrie et bloque une
nouvelle tentative jusqu’au désarmement. Si le contrôleur refuse le retour en `QLAND`,
l’état devient `FAILED`; le logiciel ne prétend pas avoir repris la descente.

À l’arrêt normal, le thread libère le canal et demande le retour s’il possède encore
la séquence `GUIDED`. Cette dernière demande est au mieux, puisque le processus termine
sa réception MAVLink. L’expiration RC côté contrôleur reste effective en cas d’arrêt brutal.

Chaque entrée dans `QLAND` réinitialise sa procédure d’atterrissage : cette séquence
interrompt la descente puis en démarre une nouvelle, plutôt que de restaurer ses états internes.

## Validation

Les tests couvrent la composition géométrique et la reprojection, le cap avec inclinaison,
les changements de mode, les observations périmées, la perte de cible et de liaison,
le thread et son arrêt, ainsi que les paquets MAVLink réellement encodés.
Un essai dynamique SITL/Gazebo reste nécessaire pour valider la réponse du tricopter,
le maintien de position et d’altitude pendant la correction, et le réglage du gain.

Références : [contrôleur VTOL QuadPlane](https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/quadplane.cpp),
[entrée en GUIDED](https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode_guided.cpp),
[entrée en QLAND](https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode_qland.cpp),
[entrées RC MAVLink](https://ardupilot.org/dev/docs/mavlink-rcinput.html).
