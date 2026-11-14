# Avion Cargo — Audit technique V3 → V3.1

**Portée vérifiée :** fichiers Python du prototype (`core.py`, `concurrency.py`, `runtime.py`), tests automatisés et contrats documentés. **Non vérifiés :** firmware ESP32, Bluetooth réel, capteurs DWM3000, contrôleur moteur, Raspberry Pi en conditions de vol et intégration avec le dépôt d'équipe. Ce document n'est pas une certification de sécurité.

## Anomalies constatées dans V3 et remédiations V3.1

| Gravité | Anomalie V3 | Modification / preuve |
| --- | --- | --- |
| Haute | Le dégagement demandait seulement une trajectoire, sans commande d'exécution | Ajout de `CLEARANCE_PATH_APPROVED` → `EXECUTE_CLEARANCE`, puis `CLEARANCE_EXECUTION_STARTED` avant `CLEARANCE_VERIFIED`. Tests de parcours et de refus. |
| Haute | La FSM acceptait des événements non urgents sans limite d'ancienneté | Vérification de `received_at` sur horloge monotone **locale**, seuil configurable (`max_event_age_s`, 1 s *pour les tests*, à valider). Arrêt prioritaire toujours accepté. |
| Haute | Un ancien ordre pouvait attendre dans la file d'actions après une transition plus récente | Génération `Decision.revision`, filtrage de commandes obsolètes par la mailbox. N'annule PAS une commande déjà en cours dans `dispatch`. |
| Moyenne | Fin d'approche sans preuve que l'exécution a même commencé | Ajout de `APPROACH_STARTED` validé avant `APPROACH_COMPLETE`. |
| Moyenne | Impossible de définir un contrôle des sources des événements | `allowed_sources` facultatif par type d'événement. **Obligatoire à renseigner lors de l'intégration réelle.** |
| Moyenne | Champ `received_at` de type inattendu pouvait causer une exception | Validation explicite des types, de la finitude, des timestamps futurs et des booléens employés comme séquences. |
| Moyenne | IDs de messages dupliqués entre producteurs pouvaient se bloquer mutuellement | Déduplication sur `(mission, source, event_id)` plutôt que sur un ID global. |
| Moyenne | Le superviseur détectait la boucle active, pas une communication silencieuse | Surveillance facultative `communication_silence_s` à activer **seulement** si l'équipe prévoit un heartbeat/flux périodique. |
| Faible | Des décisions acceptées sans action alimentaient la file d'action | Suppression de ces envois inutiles. |

## Résultat des tests

- **34 tests sur 34** réussis après modifications, couvrant le parcours nominal révisé, les gardes de transition, les doublons, les horodatages, la provenance configurable, les workers bloqués, l'arrêt logiciel prioritaire, la saturation et la fin de session.
- Suite complète exécutée **12 fois consécutivement** dans le même environnement sans échec.
- Pas de preuve de correction exhaustive : les scénarios de concurrence sont non déterministes, et les tests ne couvrent pas le matériel.

## Risques bloquants AVANT un essai motorisé ou la fusion comme système opérationnel

1. **Arrêt physique non implémenté.** `EmergencyLatch` est un événement Python. Si la sortie est bloquée, la coupure du moteur doit dépendre d'un watchdog / interverrouillage indépendant : expiration des commandes, interdiction de mouvement après faute, arrêt vérifié physiquement. `STOP` n'est pas émis à travers une mailbox qui a été désactivée par un latch.
2. **Provenance et preuves non garanties.** `validated=True` est une déclaration d'un adaptateur simulé. Un message Bluetooth non fiable ou provenant d'un mauvais appareil ne doit pas pouvoir le définir directement. Le mapping `allowed_sources` ne remplace ni l'authentification ni les contrôles physiques.
3. **L'exécution d'une action peut être bloquée.** Python ne peut pas interrompre de force `dispatch()` bloqué. Le superviseur détecte le blocage mais ne garantit pas l'arrêt du matériel.
4. **La machine ne calcule pas la géométrie.** Elle ne prouve pas l'absence de collision, le dégagement de la zone sous l'avion, le confinement dans la DLZ fixe **8 × 8 pieds**, ni le respect des incertitudes de localisation.
5. **Le transport et les accusés sont simulés.** Il faut spécifier retransmission/ACK, numéros de session sur redémarrage, authenticité, idempotence des commandes et délais côté avion. L'ACK de réception ne vaut pas permission d'atterrissage.
6. **L'état de sécurité n'est pas encore supervisé de bout en bout.** Il faudra invalider les trajets et les autorisations lorsque les mesures deviennent périmées *après* leur acceptation, prévoir les délais dans chaque état, et l'annulation des actions en cours.
7. **`ActionMailbox` filtre les commandes encore en file, pas celles déjà en cours.** Chaque exécuteur devra imposer un TTL/lease et un identifiant de génération; ce contrôle reste à intégrer.
8. **Le redémarrage de processus / crash brutal n'est pas éprouvé.** Des threads daemon ne garantissent ni libération correcte des ressources ni arrêt moteur après crash.
9. **Supervision du silence de communication facultative.** Les heartbeats, les seuils, et les cas où l'avion est légitimement silencieux sont à convenir. Les timeouts actuels sont de développement, non validés en sécurité.
10. **Intégration plateforme inconnue.** Aucun test Raspberry Pi, liaison Pi–ESP32, firmware UWB ou GitHub CI exécuté dans cette livraison.

## Critères de review conseillés

La version est adaptée à une **Draft PR pour discussion d'architecture seulement**. L'équipe doit valider le protocole avion–payload, le producteur de chaque preuve, l'effet des événements d'exécution, la commande d'arrêt indépendante, les seuils de temps et la séparation des composants. Ne pas présenter l'implémentation comme « prête au vol ».
