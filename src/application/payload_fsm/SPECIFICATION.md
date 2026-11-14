# Avion Cargo — Payload : machine à états et architecture concurrente

**Statut : proposition technique V2, à valider avec l'équipe.** Aucun ordre de mouvement ou autorisation d'atterrissage réelle ne doit être généré par ce document seul.

## 1. Périmètre et invariants

**Dans cette contribution :** machine à états, décisions des transitions, contrats d'événements, orchestration des workers/threads, files et snapshots thread-safe, arrêt et tests sans matériel.

**Hors périmètre :** estimation UWB/DWM3000, traitement ArUco, contrôle moteur bas niveau, calcul géométrique des trajectoires, intégration ArduPilot, docking mécanique, gestion de plusieurs payloads (future extension).

**Faits actuels :** DLZ carrée **8 × 8 pieds = 2,4384 × 2,4384 m**, premier vol une seule payload; Raspberry Pi sur avion et payload (décisions), ESP32 échangent par Bluetooth; DWM3000 : trois sur avion et un sur payload (disposition non confirmée). La liaison Pi–ESP32 et les formats de données restent inconnus.

**Invariants de sécurité (à appliquer par les composants spécialisés et à vérifier à l'intégration) :**
1. Aucun mouvement autorisé tant que la pose utilisée n'est pas exploitable et que le déplacement n'est pas certifié dans la DLZ entière, en tenant compte du gabarit et de l'incertitude.
2. Ne jamais émettre `CLEARANCE_READY` sans confirmation du module de sécurité que la payload est immobile, hors de l'emprise d'atterrissage prévue. L'envoi seul ne signifie pas que l'avion a reçu/accepté la confirmation.
3. Pendant la descente finale, ne pas émettre de commandes de mouvement (sauf procédure d'urgence spécifiée séparément).
4. Pas d'approche avant `LANDING_CONFIRMED` ET pose de l'avion valide ET trajet approuvé.
5. Un arrêt d'urgence matériel/watchdog reste indépendant de la file d'événements et de la boucle Python; les commandes moteur expirent si non renouvelées.
6. Un redémarrage ne rétablit jamais automatiquement un état de mouvement.

## 2. États (V1, une payload)

| État | Responsabilité | Sortie normale | Action à l'entrée |
|---|---|---|---|
| `IDLE` | Aucun mouvement, attend mission | `MISSION_START` → `LOCALIZING` | STOP abstrait |
| `LOCALIZING` | Attend pose payload exploitable | `PAYLOAD_POSE_VALID` → `CLEARING_LZ` | Demande localisation |
| `CLEARING_LZ` | Demande dégagement hors zone d'atterrissage, **dans** la DLZ | `CLEARANCE_VERIFIED` → `READY_FOR_LANDING` | Demande trajet sûr de dégagement |
| `READY_FOR_LANDING` | Immobile, signale sa disponibilité à l'avion | `CLEARANCE_ACK` → `WAIT_LANDING` | Émettre `CLEARANCE_READY` (retransmission via module comms) |
| `WAIT_LANDING` | Reste immobile, attend avion au sol | `LANDING_CONFIRMED` → `LOCATE_AIRCRAFT` | STOP abstrait |
| `LOCATE_AIRCRAFT` | Attend une pose avion exploitable (source interchangeable) | `AIRCRAFT_POSE_VALID` → `PLAN_APPROACH` | Demande estimation de pose |
| `PLAN_APPROACH` | Attend trajet sûr vers l'arrière | `PATH_APPROVED` → `APPROACH` | Demande planification |
| `APPROACH` | Suit un trajet autorisé sous surveillance | `APPROACH_COMPLETE` → `READY_TO_DOCK` | Commande abstraite d'approche |
| `READY_TO_DOCK` | Immobilisée à la position d'accrochage; docking hors V1 | Fin V1 | STOP abstrait |
| `SAFE_STOP` | Aucune commande de mouvement | `RESET_APPROVED` → `IDLE` (à réarmer manuellement) | STOP, signal de faute |

**Attention :** `PAYLOAD_POSE_VALID`, `CLEARANCE_VERIFIED`, `AIRCRAFT_POSE_VALID`, `PATH_APPROVED`, `APPROACH_COMPLETE` sont produits par des sous-systèmes indépendants; la machine n'est pas autorisée à les inventer. Un événement « valide » porte les détails de validité et doit être vérifié par un adaptateur de confiance.

### Transitions de sûreté communes

- De tout état, `EMERGENCY_STOP` ou `CRITICAL_FAULT` → `SAFE_STOP` immédiatement dans la logique; l'arrêt matériel doit aussi être pris en charge indépendamment.
- En états de mouvement `CLEARING_LZ` et `APPROACH`, perte de pose, liaison critique perdue, chemin invalidé ou échec d'exécution → `SAFE_STOP` ou immobilisation sûre selon politique à valider.
- En `READY_FOR_LANDING` ou `WAIT_LANDING`, si la confirmation de dégagement n'est plus valide, retirer l'autorisation et déclencher la procédure de sécurité de l'avion (responsabilité à valider).
- Tout événement incompatible avec l'état courant : aucune transition; journaliser le rejet. Les doublons reconnus ne réexécutent pas les actions.
- Les délais (`timeout`) ne sont pas des constantes de sécurité inventées : ils proviennent de la configuration validée par l'équipe.

## 3. Contrat d'événements (proposition)

Champs communs : `mission_id`, `event_id`, `event_type`, `source`, `sequence_number`, `observed_at` (horloge ou timestamp source qualifié), `received_at_monotonic`, `payload`, `schema_version`.

| Événement | Producteur logique | Informations utiles | Type de transport |
|---|---|---|---|
| `MISSION_START` | Liaison avion | Identifiant mission | fiable / ACK |
| `PAYLOAD_POSE_VALID` | Localisation (externe) | repère DLZ, pose, incertitude, temps | snapshot actualisé + changement de validité |
| `CLEARANCE_VERIFIED` | Vérificateur dégagement (externe) | immobilisation et marge vérifiées | fiable / ACK local |
| `CLEARANCE_READY` | State machine → avion | mission + preuve/état de dégagement | émission fiable / ACK réseau |
| `CLEARANCE_ACK` | Avion | `event_id` acquitté et mission | fiable / ACK |
| `LANDING_CONFIRMED` | Avion | avion au sol/immobile, mission | fiable / ACK |
| `AIRCRAFT_POSE_VALID` | Localisation (externe) | pose avion `(x,y,θ)`, repère, incertitude, temps | snapshot + changement de validité |
| `PATH_APPROVED` | Planificateur / vérificateur externe | `path_id`, référence, durée/validité | fiable |
| `APPROACH_COMPLETE` | Contrôleur / vérificateur externe | arrêt et alignement validés | fiable |
| `EMERGENCY_STOP` | Tout dispositif autorisé | raison | **chemin prioritaire hors FIFO** |
| `CRITICAL_FAULT` | Superviseur | raison, module | chemin prioritaire hors FIFO |

Remarque : le protocole Bluetooth, la liaison Pi–ESP32, la localisation et le calcul du chemin sont des **adaptateurs à définir**, non implémentés ici. Les accusés de réception de messages ne prouvent pas que l'action physique est achevée.

## 4. Décision sur l'organisation des threads

**Organisation logique recommandée pour la Raspberry Pi de la payload :**

- **Worker d'I/O communication** : lit sans bloquer la mission, sérialise les messages, traite reconnexions/ACK et remonte les événements.
- **Worker d'entrée localisation** : reçoit les sorties *déjà calculées* (pas l'UWB brut), valide structure/horodatage/repère, publie un snapshot cohérent.
- **Worker de machine à états (unique propriétaire)** : consomme événements et snapshots, applique gardes et transitions, produit des intentions immuables. Aucun autre worker ne modifie son état.
- **Worker de sortie commandes** : remet les intentions au futur contrôleur moteur; aucune logique de mission, suivi des accusés d'exécution.
- **Superviseur + watchdog matériel** : surveille les workers, initie l'arrêt et assure que les commandes moteur expirent. Le watchdog critique ne doit pas dépendre d'un thread Python ou de la file FIFO.

Les boîtes ci-dessus sont des responsabilités **et non quatre threads obligatoires**. On peut fusionner comms et localisation si elles arrivent sur la même liaison, sans toucher au cœur de la machine. La capture PiCam/ArUco existante est indépendante; elle alimente la localisation de l'avion, pas nécessairement celle de la payload.

**Politique de synchronisation** :
- `EventQueue` **bornée** pour événements discrets : traitement par un seul consommateur; débordement → faute et arrêt, jamais suppression silencieuse.
- `LatestSnapshot` atomique pour poses fréquentes : remplace la donnée précédente, conserve horodatage, version, repère et incertitude. On ne garantit pas le traitement de chaque échantillon brut : c'est intentionnel.
- `EmergencyStop` : signal prioritaire indépendant de la file standard, avec coupure moteur assurée hors boucle Python.
- Numéros de séquence par *source + session/mission*; rejeter doublons et retours en arrière sans confondre sources. Ne pas comparer des horloges monotones de deux machines sans conversion/synchronisation explicite.
- Un worker ne garde pas un verrou pendant un appel réseau ou matériel. Les échanges entre threads utilisent des événements immuables.
- Arrêt propre : signal de fermeture, invalidation commandes, arrêt du moteur, `join` des workers avec délai, journalisation; redémarrage en `IDLE` et réinitialisation de session.

## 5. Tests nécessaires (avant intégration)

1. Parcours complet nominal avec adaptateurs simulés.
2. Impossible de passer `CLEARING_LZ` → `READY_FOR_LANDING` sur simple demande de mouvement.
3. Impossible d'entrer en `APPROACH` si avion non confirmé au sol, pose périmée ou chemin non approuvé.
4. Événements hors ordre, doublons et événements d'une autre mission rejetés.
5. Deux ou plusieurs producteurs simultanés; exactement un propriétaire de l'état.
6. File saturée : faute explicite, arrêt; pas de deadlock.
7. Décrochage comms/localisation pendant un déplacement : commande invalidée et arrêt indépendant.
8. Exceptions des workers propagées au superviseur; arrêt contrôlé.
9. Timeout d'ACK, transmission répétée avec `event_id` stable : pas de double-transition.
10. Démarrage et arrêt répétés sans threads orphelins; aucune reprise de commande moteur après reboot.

## 6. Décisions à faire valider par John-William

1. Quelle condition prouve que la payload est hors de la **zone dangereuse réelle** (empreinte de l'avion, incertitude, marge) au centre de la DLZ ?
2. L'avion bloque-t-il la descente tant que `CLEARANCE_READY` n'a pas été confirmé ? Que fait-il si la communication est perdue ensuite ?
3. Quel format de message l'ESP32 donne-t-il à la Pi et inversement ?
4. Qui publie les événements de `pose valide`, de `path approved` et de `mouvement terminé` ?
5. Quelles valeurs de timeout/fiabilité seront validées lors des essais ?

## 7. Livrables planifiés

- [x] Conception V2 de la FSM et de l'organisation des threads (ce document)
- [ ] Faire approuver les hypothèses et contrats par l'équipe
- [ ] Implémenter moteur Python pur et adaptateurs factices
- [ ] Implémenter superviseur et concurrence (threads réels) avec tests
- [ ] Validation sur Raspberry Pi, puis intégration avec autres sous-systèmes

**Note :** le prototype Python V1 précédent n'implémente pas encore toutes ces garanties. La présente spécification formalise les critères avant sa refonte; elle ne constitue pas une preuve de sûreté physique.

---

## Mise à jour V3.1 de la spécification

La V3.1 **étend** la transition `CLEARING_LZ` de la V2 : `CLEARANCE_PATH_APPROVED` (intention `EXECUTE_CLEARANCE`) → `CLEARANCE_EXECUTION_STARTED` → `CLEARANCE_VERIFIED`. Une demande de planification seule n'entraîne aucune motion. La confirmation de début de commande n'est **pas** une preuve que la payload est dégagée. L'état `APPROACH` exige de même `APPROACH_STARTED` avant `APPROACH_COMPLETE`.

Les événements non urgents sont vérifiés par un temps de réception monotone *local* et une politique configurable de sources autorisées (facultative dans le prototype, à rendre obligatoire en intégration). Les commandes dans la mailbox portent une révision pour écarter les anciennes intentions encore en attente. Cela ne stoppe pas une commande déjà engagée : TTL moteur et arrêt indépendant indispensables.

Voir `README.md` pour les détails de la V3.1 et `AUDIT.md` pour le bilan des défauts, tests et points bloquants. Les paragraphes V2 précédents demeurent historiques lorsqu'ils décrivent une séquence simplifiée.
