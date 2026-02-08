# Guide Complet - Système d'Atterrissage de Précision
## Avion Cargo - SAE AeroDesign 2026

---

## Vue d'ensemble

Ce système permet à l'avion d'atterrir avec précision (±10cm) en utilisant la détection de marqueurs ArUco par caméra embarquée. Le système envoie des commandes MAVLink au contrôleur de vol pour guider l'avion vers la cible.

### Architecture Globale

```
┌─────────────┐      ┌──────────────┐      ┌─────────────┐
│   Caméra    │─────▶│   Raspberry  │─────▶│  Contrôleur │
│  (Pi/USB)   │      │      Pi      │      │  de Vol     │
└─────────────┘      │              │      │ (ArduPilot) │
                     │ - Détection  │      └─────────────┘
                     │ - Calcul 3D  │
                     │ - MAVLink    │
                     └──────────────┘
```

---

## Modes de Fonctionnement

### 1. Mode DEMO (demo_gui.py)
**Usage:** Présentations et tests sans matériel

**Caractéristiques:**
- Caméra simulée avec marqueurs animés
- Véhicule simulé avec télémétrie réaliste
- Interface GUI Tkinter complète
- Aucun matériel requis

**Commande:**
```bash
cd src
python3 demo_gui.py --auto-start
```

**Quand utiliser:**
- Démonstrations
- Tests de l'interface utilisateur
- Validation de la logique de contrôle
- Formation des nouveaux membres (si applicable)

---

### 2. Mode TEST CAMERA (test_camera.py)
**Usage:** Test de la caméra réelle

**Caractéristiques:**
- Caméra USB ou Raspberry Pi Camera
- Véhicule simulé
- Interface GUI Tkinter
- Statistiques en temps réel (FPS, taux de détection, précision)

**Commande:**
```bash
cd src
python3 test_camera.py --auto-start
```

**Quand utiliser:**
- Valider la détection ArUco avec vraie caméra
- Tester l'éclairage et la qualité des marqueurs
- Optimiser les paramètres de détection
- Vérifier la performance (FPS)

**Critères de validation:**
- Taux de détection > 90%
- Taux de précision > 95%
- FPS stable > 20

---

### 3. Mode PRODUCTION (production.py)
**Usage:** Déploiement final sur Raspberry Pi avec drone réel

**Caractéristiques:**
- Caméra Raspberry Pi Camera Module
- Connexion MAVLink avec ArduPilot
- Commandes réelles envoyées au drone
- Interface GUI pour monitoring

**Commande:**
```bash
cd src
python3 production.py --auto-start
```

**AVERTISSEMENTS:**
- Toujours tester au sol d'abord!
- Vérifier que le drone est désarmé pour les tests initiaux
- S'assurer d'avoir un override manuel
- Zone d'atterrissage dégagée

**Vérifications avant vol:**
1. Caméra fonctionnelle (test_camera.py OK)
2. Connexion MAVLink établie
3. Télémétrie reçue (GPS, batterie, mode)
4. Tests au sol réussis

---

## Composants du Système

### 1. Caméra (camera.py)
Interface pour caméra embarquée.

**Backends supportés:**
- Picamera2 (Raspberry Pi Camera Module)
- OpenCV V4L2 (USB webcam)

**Fonctionnalités:**
- Auto-détection du matériel
- Reconnexion automatique
- Gestion des erreurs

### 2. Traitement d'Image (image_treatment.py)
Détection et localisation 3D des marqueurs ArUco.

**Paramètres optimisés:**
- Seuillage adaptatif pour éclairage variable
- Égalisation d'histogramme pour améliorer le contraste
- Raffinement des coins en subpixel
- Détection de petits marqueurs (min 3% du cadre)

**Sorties:**
- ID du marqueur
- Distance (mètres)
- Angles horizontal/vertical (radians)
- Vecteurs de rotation et translation

### 3. Interface Véhicule (vehicle_interface.py)
Communication MAVLink avec le contrôleur de vol.

**Messages MAVLink:**
- LANDING_TARGET - Position de la cible
- HEARTBEAT - État du véhicule
- GPS_RAW_INT - Position GPS
- BATTERY_STATUS - État de la batterie

**Connexions supportées:**
- /dev/serial0 (UART Raspberry Pi)
- /dev/ttyUSB0 (USB série)
- UDP (SITL simulation)

### 4. Contrôleur (interface/controller.py)
Coordination du système complet.

**Boucle principale (30 Hz):**
1. Capture frame de la caméra
2. Détecte les marqueurs ArUco
3. Calcule la position 3D
4. Envoie commande MAVLink
5. Met à jour l'interface GUI
6. Enregistre les statistiques

**Statistiques trackées:**
- Nombre de frames traitées
- Taux de détection (frames avec marqueurs)
- Taux de précision (poses réussies/détections)
- FPS moyen
- Temps d'exécution

### 5. Interface Graphique (interface/main_view.py)
GUI Tkinter pour monitoring et contrôle.

**Panneau gauche - Vidéo:**
- Flux caméra en temps réel
- Overlay de détection (boîtes vertes)
- Coins des marqueurs (points rouges)
- Labels ID et distance

**Panneau droit - Informations:**
- État du véhicule (mode, batterie, GPS)
- Marqueurs détectés (ID, distance, angles)
- Statistiques (FPS, détection, précision)
- Boutons de contrôle (Start, Stop, Quit)

---

## Génération de Marqueurs

### Outil: generate_markers.py

**Usage:**
```bash
# Générer une feuille de 4 marqueurs
python3 generate_markers.py --sheet --ids 0 1 2 3

# Marqueur individuel haute résolution
python3 generate_markers.py --id 0 --size 1000
```

**Instructions d'impression:**
1. Imprimer sur papier blanc mat (pas brillant)
2. IMPORTANT: Chaque marqueur doit faire exactement **5cm × 5cm**
3. Qualité maximale (pas de compression)
4. Bordures blanches essentielles pour la détection

**Tests après impression:**
1. Bon éclairage (pas de contre-jour)
2. Marqueur bien à plat
3. Distance: 0.5m - 3m de la caméra
4. Lancer: `python3 test_camera.py --auto-start`

---

## Installation et Configuration

### Dépendances

```bash
# Installation complète
pip install -r requirements.txt

# Installation minimale pour démo
pip install numpy opencv-python pillow

# Installation pour production (Raspberry Pi)
pip install picamera2 dronekit pymavlink
```

### Configuration Raspberry Pi

1. **Activer la caméra:**
```bash
sudo raspi-config
# Interface Options → Camera → Enable
```

2. **Tester la caméra:**
```bash
libcamera-hello --timeout 5000
```

3. **Connexion série UART:**
```bash
# Désactiver console série
sudo raspi-config
# Interface Options -> Serial Port
# Login shell: No
# Serial hardware: Yes
```

4. **Permissions:**
```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
```

---

## Workflow de Développement à Production

### Étape 1: Développement Local
```bash
# Tester la logique avec simulation
python3 demo_gui.py --auto-start
```

**Validation:**
- GUI fonctionne
- Détection de marqueurs OK
- Statistiques affichées
- Contrôleur répond aux commandes

---

### Étape 2: Test Caméra
```bash
# Générer les marqueurs
python3 generate_markers.py --sheet --ids 0 1 2 3

# Imprimer à 5cm × 5cm

# Tester avec caméra
python3 test_camera.py --auto-start
```

**Validation:**
- Taux de détection > 90%
- Taux de précision > 95%
- FPS > 20
- Distance calculée précise

**Dépannage:**
| Problème | Solution |
|----------|----------|
| Détection < 90% | Améliorer éclairage, réimprimer marqueurs |
| Précision < 95% | Vérifier taille marqueurs (5cm), améliorer qualité |
| FPS bas | Fermer applications, optimiser résolution |

---

### Étape 3: Tests au Sol avec Drone
```bash
# Sur Raspberry Pi, drone DÉSARMÉ
python3 production.py --auto-start
```

**Validation:**
- Connexion MAVLink établie
- Télémétrie reçue
- Commandes LANDING_TARGET envoyées
- Aucun crash ou freeze

**Vérifications:**
1. Messages MAVLink dans les logs
2. GPS verrouillé (> 6 satellites)
3. Batterie > 11.5V
4. Mode GUIDED ou LAND

---

### Étape 4: Vol Test
**Protocole:**
1. Zone dégagée, temps calme
2. Altitude modérée (10-15m)
3. Pilote manuel prêt pour override
4. Observer comportement en LAND mode
5. Enregistrer données pour analyse

---

## Dépannage

### Problème: Caméra non détectée

**Symptômes:**
```
Failed to initialize camera!
```

**Solutions:**
1. Vérifier connexion physique
2. `libcamera-hello` pour tester
3. Vérifier permissions: `groups $USER` (doit inclure video)
4. Redémarrer Raspberry Pi

---

### Problème: Pas de connexion MAVLink

**Symptômes:**
```
Vehicle not connected
```

**Solutions:**
1. Vérifier câble série (TX -> RX, RX -> TX)
2. Vérifier paramètres ArduPilot: SERIAL2_PROTOCOL = 2 (MAVLink)
3. Tester avec `mavproxy.py --master=/dev/serial0`
4. Vérifier baud rate (57600 par défaut)

---

### Problème: Détection faible

**Symptômes:**
```
Detection Rate: 45%
```

**Solutions:**
1. Améliorer éclairage (éviter soleil direct, ombres)
2. Vérifier qualité impression (noir profond, blanc pur)
3. Vérifier taille marqueurs (exactement 5cm)
4. Distance optimale: 0.5m - 3m
5. Nettoyer lentille caméra

---

### Problème: Précision faible

**Symptômes:**
```
Precision Rate: 60%
```

**Solutions:**
1. Vérifier taille marqueurs dans le code (0.05m)
2. Améliorer qualité d'impression
3. Éviter marqueurs pliés ou déformés
4. Calibrer caméra (optionnel mais recommandé)

---

## Métriques de Performance

### Objectifs pour la Compétition

| Métrique | Cible | Minimum |
|----------|-------|---------|
| Taux de détection | > 95% | > 90% |
| Taux de précision | > 98% | > 95% |
| FPS | > 25 | > 20 |
| Précision d'atterrissage | < 5cm | < 10cm |
| Temps de mission | < 3min | < 4min |

### Monitoring en Temps Réel

Les statistiques sont affichées dans l'interface GUI:
- **Frames**: Total de frames traitées
- **Detections**: Frames avec marqueurs visibles
- **Total Markers**: Nombre total de marqueurs détectés
- **Successful Poses**: Poses 3D calculées avec succès
- **FPS**: Images par seconde
- **Detection Rate**: % de frames avec marqueurs
- **Precision Rate**: % de marqueurs avec pose valide
- **Runtime**: Temps d'exécution total

---

## Architecture Logicielle

### Pattern Observer

Le système utilise le pattern Observer pour les mises à jour GUI:

```
Controller (Observable)
    ↓ notify
Interface (Observer)
    ↓ update
MainView (GUI)
```

**Avantages:**
- Découplage Controller <-> GUI
- Thread-safe
- Extensible (ajouter de nouveaux observers)

### Pattern MVC

```
Model: Camera + ImageTreatment + VehicleInterface
View: MainView (Tkinter)
Controller: Controller (coordination)
```

---

## Fichiers de Configuration

### Paramètres ArUco (image_treatment.py)

```python
# Taille des marqueurs (en mètres)
marker_length_m = 0.05  # 5cm

# Dictionnaire ArUco
marker_dict = cv2.aruco.DICT_4X4_50  # IDs 0-49

# Paramètres de détection
adaptiveThreshWinSizeMin = 3
adaptiveThreshWinSizeMax = 23
minMarkerPerimeterRate = 0.03
cornerRefinementMethod = CORNER_REFINE_SUBPIX
```

### Paramètres Controller

```python
# Fréquence de mise à jour
update_rate = 30  # Hz (30 images/seconde)
```

---

## Logs et Débogage

### Fichiers de Log

- `demo_gui.log` - Logs du mode démo
- `test_camera.log` - Logs des tests caméra
- `production.log` - Logs du mode production

### Niveau de Log

Modifier dans le code:
```python
logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(level=logging.INFO)
```

### Messages Importants

```
INFO: Camera connected          # Caméra OK
INFO: Vehicle connected         # Drone OK
INFO: Marker ID=0: dist=1.25m   # Détection OK
WARNING: Failed to capture      # Problème caméra
ERROR: No vehicle connection    # Problème MAVLink
```

---

## Contribution au Projet

### Structure des Commits

```bash
git commit -m "feat: add precision rate tracking"
git commit -m "fix: camera reconnection issue"
git commit -m "docs: update installation guide"
```

### Tests Avant Commit

```bash
# Vérifier syntaxe
python3 -m py_compile src/*.py

# Tester démo
python3 src/demo_gui.py
```

---

## Ressources

### Documentation Externe

- [ArduPilot Precision Landing](https://ardupilot.org/copter/docs/precision-landing-and-loiter.html#precision-landing-and-loiter)
- [MAVLink LANDING_TARGET](https://mavlink.io/en/messages/common.html#LANDING_TARGET)
- [OpenCV ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

### Contact

- **Team Lead**: Charles Clavet
- **Projet**: SAE AeroDesign 2026
- **Compétition**: Texas, Avril 2026

---

## Annexe: Spécifications Techniques

### Matériel Requis

**Raspberry Pi:**
- Modèle: Pi 4 (4GB RAM recommandé)
- OS: Raspberry Pi OS Lite 64-bit
- Stockage: 32GB minimum

**Caméra:**
- Raspberry Pi Camera Module V2 (8MP)
- Ou: USB webcam 720p minimum

**Connexion Drone:**
- Câble série UART (TX/RX)
- Ou: Télémétrie radio 433MHz/915MHz

### Performance Attendue

**Raspberry Pi 4:**
- FPS: 25-30
- Latence: < 50ms
- CPU: 40-60%

**Portée de Détection:**
- Minimum: 0.3m
- Optimal: 0.5m - 3m
- Maximum: 5m (dépend taille marqueur)

---

**Dernière mise à jour:** 6 février 2026
**Version:** 1.0
