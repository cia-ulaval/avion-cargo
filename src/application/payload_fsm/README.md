# Avion Cargo — State machine de la payload, V3.1 (prototype)

**Statut : revue d'architecture / simulation logicielle uniquement, non prêt pour la propulsion ni le vol.**

Cette contribution concerne la **FSM + l'organisation des threads** sur une Raspberry Pi de payload. L'algorithme UWB, la caméra ArUco, la navigation géométrique, les contrôleurs moteurs, Bluetooth/ESP32 et ArduPilot sont **hors périmètre**. Ils sont représentés par des adaptateurs simulés.

La **DLZ est une exigence fixe de 8 × 8 pieds (2,4384 × 2,4384 m)**. La FSM ne valide pas la géométrie : un composant externe de navigation/sécurité doit fournir les attestations nécessaires.

## Lancer les tests

Depuis la racine du dossier :

```bash
PYTHONPATH=src/application python3 -m unittest discover -s tests/application/payload_fsm -p "test_*.py" -v
```

Résultat vérifié dans l'environnement de développement : **34 tests réussis**, suite répétée **12 fois**. Aucune dépendance externe.

## Fichiers

- `payload_fsm/core.py` : FSM pure, états, événements, transitions, actions et validation de métadonnées.
- `payload_fsm/concurrency.py` : événements thread-safe, snapshot de position, latch logiciel et unique thread propriétaire de la FSM.
- `payload_fsm/runtime.py` : workers de communication et localisation, sortie actions, superviseur et mailbox à génération.
- `tests/` : tests du moteur, de concurrence et de régression.
- `SPECIFICATION.md` : proposition de contrats et limites connues.
- `AUDIT.md` : anomalies V3 corrigées et risques restant ouverts.

## Architecture

```text
Communication ESP32 adapter ----> EventInbox (FIFO bornée) ----+
                                                               |
Localisation adapter ---------> EventInbox / LatestPose -------+--> FSMWorker (seul propriétaire de l'état)
                                                                    |
                                                                    v
                                                           ActionMailbox (bornée)
                                                                    |
                                                                    v
                                                           ActionWorker --> adaptateurs factices

Supervisor / EmergencyLatch --> alerte logicielle indépendante de la FIFO
VRAI watchdog moteur, TTL des commandes et interverrouillage = À FOURNIR PAR L'ÉQUIPE
```

Les entrées peuvent arriver de threads différents. **Une seule tâche modifie la FSM**. Les mises à jour de position haute fréquence passent par un snapshot, pas une file de milliers d'échantillons. Les événements critiques passent par une FIFO bornée, mais aucune livraison réseau n'est garantie par Python.

## Séquence du premier vol (abstraite)

`IDLE → LOCALIZING → CLEARING_LZ → READY_FOR_LANDING → WAIT_LANDING → LOCATE_AIRCRAFT → PLAN_APPROACH → APPROACH → READY_TO_DOCK`

En `CLEARING_LZ` :

1. `REQUEST_CLEARANCE_PATH` demande une trajectoire à un module extérieur.
2. `CLEARANCE_PATH_APPROVED` produit `EXECUTE_CLEARANCE` (intention envoyée au composant mouvement).
3. `CLEARANCE_EXECUTION_STARTED` confirme que le composant responsable a accepté de démarrer la commande.
4. `CLEARANCE_VERIFIED` est une **preuve externe distincte** : payload réellement immobile, hors de la zone dangereuse et toujours dans la DLZ. L'acceptation de la commande ne prouve PAS cela.
5. `SEND_CLEARANCE_READY` informe l'avion; `CLEARANCE_ACK` est reçu ensuite.

En `APPROACH`, `APPROACH_STARTED` (acceptation d'exécution) doit précéder `APPROACH_COMPLETE` (preuve externe de fin et d'alignement).

`SAFE_STOP` peut être demandé depuis n'importe quel état par une voie logicielle prioritaire. L'arrêt réel des moteurs exige une protection matérielle indépendante.

## Points importants pour l'intégration

- `Event.received_at` = horodatage *monotone du récepteur*, et non horodatage monotone de l'avion. `max_event_age_s` est configurable, 1 s par défaut **pour développement seulement**.
- `validated=True` n'est pas une preuve cryptographique ou physique : seuls des adaptateurs d'équipe réputés fiables doivent le renseigner après validation réelle.
- `allowed_sources` permet de définir les sources attendues par événement. Il n'est **pas configuré par défaut dans le prototype**, pour garder les tests factices génériques. Il devra être exigé à l'intégration.
- `Decision.revision` empêche la diffusion de décisions **encore en attente** après un changement d'état. Une action déjà en cours reste non interrompable par ce mécanisme.
- `communication_silence_s` est optionnel, à activer seulement avec un protocole de messages périodiques / heartbeat validé.
- Les callbacks `poll()` et `dispatch()` doivent être bornés et compatibles avec un arrêt propre. Un callback bloqué n'est pas tuable en Python.
- Les messages de plusieurs producteurs n'ont **pas d'ordre global garanti**.
- `RESET_APPROVED` est une transition du modèle pur; **le runtime ne réarme jamais automatiquement** un latch déclenché.

**Lire `AUDIT.md` avant toute PR** : la V3.1 améliore la robustesse logicielle mais ne satisfait pas encore les exigences d'une application critique physique.
