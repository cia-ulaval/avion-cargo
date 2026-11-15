# Alignement autonome du lacet QuadPlane

Le superviseur tourne dans le thread `autolander-yaw-supervisor`. Il gère la séquence
`QLAND → détection stable → GUIDED VTOL → maintien stable → tentative d'alignement → QLAND`
sans opérateur, avec un nombre limité de tentatives. La vision et la publication
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
Le champ `yaw_alignment` contient l’état, l’erreur en degrés, le motif d’arrêt éventuel,
le nombre de tentatives (`attempts`) et sa limite (`max_attempts`).

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
  "detection_stable_time_s": 1.0,
  "detection_loss_timeout_s": 1.5,
  "max_attempts": 3,
  "attempt_timeout_s": 30.0,
  "cooldown_s": 1.0,
  "min_height_m": 3.0,
  "max_horizontal_distance_m": 2.0,
  "max_yaw_input": 0.25,
  "hold_stable_time_s": 0.5,
  "hold_horizontal_tolerance_m": 0.5,
  "hold_altitude_tolerance_m": 0.3,
  "hold_speed_mps": 0.3
}
```

Par défaut, la boucle tourne à 10 Hz. Elle exige une pose et des échantillons d’attitude
et de position âgés de moins de 0,4 s, une vitesse horizontale inférieure à 0,8 m/s,
et une cible à au moins 3 m sous le véhicule, selon la verticale terrestre. La distance
horizontale à la cible est également calculée dans le repère NED après compensation de
l'inclinaison, et non à partir des seules composantes X/Y du repère drone. Les paramètres du contrôleur sont
lus avant toute prise de contrôle. Tant que la détection clignote, le superviseur laisse
le drone descendre en `QLAND`. Il exige une détection continue du même marqueur pendant
`detection_stable_time_s` avant de suspendre l'atterrissage. Cette continuité est évaluée
sur chaque image caméra, y compris les pertes entre deux passages de la supervision.
Une image répétée ne prolonge pas la fenêtre; une interruption de caméra, une pose inutilisable
ou un intervalle entre captures atteignant `max_age_s` la réinitialise.

La continuité visuelle est indépendante des seuils géométriques de rotation. Une cible
visible pendant le freinage reste détectée lorsque le drone s'incline; son décalage dans
l'image ne doit pas être assimilé à une disparition. La rotation attend toujours que
le maintien et les limites de distance soient respectés.

Pour FArUco, `marker_id` indique le niveau visible. `target_identity` identifie la cible
fractale commune : passer d'un niveau à un autre ne réinitialise pas la continuité.
Pour ArUco, cette identité inclut le dictionnaire et l'ID effectivement détecté.

Une fois la détection stable et l'erreur supérieure au seuil, le superviseur mémorise la latitude, la longitude
et l’altitude relative au home avant de demander `GUIDED`. Une fois ce mode confirmé,
il envoie `MAV_CMD_DO_REPOSITION` avec ce point fixe et attend son acceptation.
Cette commande ne force pas de changement de mode si le pilote ou un failsafe a
repris la main entre-temps. Le point mémorisé reste identique pendant toute la séquence.

L’état `HOLDING` attend pendant au moins 0,5 s des mesures renouvelées confirmant
une erreur horizontale ≤ 0,5 m, une erreur d’altitude ≤ 0,3 m et des vitesses horizontale
et verticale ≤ 0,3 m/s. Le lacet reste neutre pendant le freinage et la stabilisation.
Une dérive hors de ces tolérances suspend également la rotation jusqu’au retour au
point de maintien. La télémétrie `yaw_alignment.hold_position` expose la consigne.

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
heartbeat postérieur à la demande. Une temporisation `cooldown_s` d'une seconde par défaut
laisse reprendre la descente avant une nouvelle tentative. Chaque retour confirmé en `QLAND`
exige une nouvelle période de détection stable, même si le marqueur était visible avant le retour.

Pendant l’alignement, une détection manquée dispose d’un
délai `target_loss_grace_s` de 0,25 s, également borné par `max_age_s`. Ce délai est
compté depuis la capture de la dernière cible valide; les images `NOT_FOUND` ne le
prolongent pas. Pendant cette attente, l’alignement commande un lacet neutre et remet
à zéro la confirmation de stabilité. Il reprend la correction lorsque la détection
redevient continue pendant `detection_stable_time_s`, sans réutiliser un ancien cap
ni republier une ancienne position de cible.

Une perte de cible dépassant ce délai passe en `WAIT_TARGET` : le contrôleur conserve
le point et l’altitude en `GUIDED`, et le lacet reste neutre. Si aucune détection stable
n'est confirmée depuis `detection_loss_timeout_s` (1,5 s par défaut), le superviseur
abandonne la tentative et demande `QLAND`, même si quelques images détectent encore
le marqueur entre des pertes. Ce délai s'applique aussi pendant le freinage et l'entrée
en `GUIDED`. Il est configurable : il ne constitue pas la durée obligatoire de chaque pause.

Un délai d'alignement dépassé (`alignment_timeout_s`, 15 s à partir du début de la rotation)
ou une tentative dépassant `attempt_timeout_s` (30 s depuis la demande de `GUIDED`, freinage
compris) demande également `QLAND`. Une nouvelle tentative reste possible plus bas si
la détection est stable, l'erreur dépasse `trigger_deg` et la limite n'est pas atteinte.
Le retour en descente peut donc avoir lieu sans alignement terminé.

`max_attempts` est un entier positif, par défaut 3. Chaque demande de pause envoyée compte,
qu'elle aboutisse à un alignement ou soit abandonnée. Après la dernière tentative et le
retour confirmé en `QLAND`, l'état `EXHAUSTED` interdit toute nouvelle pause ou rotation.
Le compteur se réinitialise au désarmement, pas à chaque passage en `QLAND`.

Une télémétrie périmée suspend la rotation et impose une nouvelle stabilisation.
L’absence d’acquittement de la consigne (`hold_timeout_s`, 10 s) ou un refus de commande
reste un échec technique `FAILED`. Si le maintien a été établi, le véhicule reste en `GUIDED`
avec un lacet neutre. L’échec reste visible jusqu’au désarmement.
Une stabilisation lente après acceptation de la consigne reste récupérable : après
`hold_timeout_s`, un avertissement donne les écarts de position et les vitesses, mais
l’état reste `HOLDING`, dans la limite de `attempt_timeout_s`. La rotation démarre dès que
le maintien et la détection sont confirmés stables.
Une perte de liaison libère le canal RC; à la reconnexion, le superviseur vérifie le
mode et la stabilité avant toute reprise. Si le contrôleur refuse le mode ou la
consigne, le compagnon signale l’échec et ne peut pas garantir le maintien demandé.
Un changement vers un mode externe, par exemple `QRTL`, suspend la séquence sans
forcer le mode précédent. Si le contrôleur refuse le retour en `QLAND`,
l’état devient `FAILED`; le logiciel ne prétend pas avoir repris la descente.

À l’arrêt normal, le thread libère le canal sans demander `QLAND`. Le contrôleur conserve
sa consigne `GUIDED`; arrêter le compagnon n’autorise pas une descente avant alignement.
L’expiration RC côté contrôleur reste effective en cas d’arrêt brutal, et les failsafes
du contrôleur restent prioritaires.

Chaque entrée dans `QLAND` réinitialise sa procédure d’atterrissage : cette séquence
interrompt la descente puis en démarre une nouvelle, plutôt que de restaurer ses états internes.

## Validation

Les tests couvrent la composition géométrique et la reprojection, le cap avec inclinaison,
les changements de mode, la consigne géographique fixe, son acquittement, le freinage,
les observations périmées, les clignotements entre les ticks de supervision, les pertes de cible
et de liaison, les reprises de descente, la limite de tentatives, les échecs techniques,
le thread et son arrêt, ainsi que les paquets MAVLink réellement encodés.
Les tests natifs FArUco utilisent aussi la texture de simulation aux cinq niveaux et
dans les deux sens de rotation pour vérifier la chaîne détection → pose → commande de lacet.
La trace `tests/fixtures/quadplane_hold_settling.csv` rejoue un freinage réel SITL/Gazebo
à 4 Hz avec une supervision à 10 Hz : la stabilité n'est confirmée qu'après environ
10,2 s. Ce cas vérifie qu'un maintien lent ne bloque plus définitivement le lacet.
Un test de concurrence vérifie aussi que l'âge des mesures est calculé après lecture
des buffers, pour ne pas rejeter une mesure publiée pendant cette lecture.

Un essai séparé avec le modèle natif SITL `quadplane-tilttrivec` et une pose de marqueur
synthétique a validé les clignotements sans interruption de `QLAND`, l'abandon d'une
première pause devenue instable, puis une nouvelle tentative plus bas (consignes de
7,64 m puis 7,26 m). La seconde tentative a corrigé un écart initial proche de 60°,
avec une erreur finale de 1,8°, avant le retour en `QLAND`. Avec `max_attempts=2`,
le superviseur est ensuite passé en `EXHAUSTED`.
Cet essai valide le chemin de commande du contrôleur. Un essai complet avec la caméra
et la dynamique Gazebo reste nécessaire pour valider leur comportement ensemble.

Références : [contrôleur VTOL QuadPlane](https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/quadplane.cpp),
[entrée en GUIDED](https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode_guided.cpp),
[entrée en QLAND](https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode_qland.cpp),
[entrées RC MAVLink](https://ardupilot.org/dev/docs/mavlink-rcinput.html).
