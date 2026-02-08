# GUIDE DÉMARRAGE RAPIDE - Démo
## Avion Cargo - Système d'Atterrissage de Précision

### Composants du Système:
1. **Interface GUI** (Tkinter) - Visualisation complète des marqueurs ArUco
2. **Composants Simulés** - Caméra et véhicule pour démonstration
3. **Architecture Contrôleur** - Patron MVC
4. **Traitement d'Image** - Pipeline de détection ArUco complet

### Démarrage Rapide

#### Étape 1: Installer les Dépendances
```bash
cd avion-cargo
pip3 install -r requirements.txt
```

Ou installation minimale:
```bash
pip3 install numpy opencv-python pillow
```

#### Étape 2: Lancer la Démo
```bash
cd src
python3 demo_gui.py --auto-start
```

### Structure des Fichiers:

**Fichiers Principaux:**
- `src/demo_gui.py` - Démo avec caméra/véhicule simulés + GUI
- `src/test_webcam.py` - Test caméra réelle + GUI
- `src/production.py` - Mode production avec matériel réel

**Génération de Marqueurs:**
- `src/generate_markers.py` - Génération de marqueurs ArUco imprimables

**Fichiers Support:**
- `src/mock_camera.py` - Caméra simulée avec marqueurs ArUco animés
- `src/mock_vehicle.py` - Drone simulé avec télémétrie
- `src/interface/controller.py` - Contrôleur principal (patron MVC)
- `src/interface/main_view.py` - Vue GUI Tkinter
- `src/interface/interface_waiter.py` - Base du patron Observer
- `src/interface/movements.py` - Historique des commandes
- `src/image_treatment.py` - Détection ArUco et estimation de pose

### Interface de Démonstration:

1. **Panneau Gauche - Flux Vidéo:**
   - Vue caméra en temps réel (simulée)
   - Boîtes vertes autour des marqueurs ArUco détectés
   - Points rouges aux coins des marqueurs
   - Réticule vert au centre du marqueur
   - Label affichant "ID:X D:Y.YYm" pour chaque marqueur

2. **Panneau Droit - Informations:**
   - **État Véhicule**: Mode, batterie, GPS, position
   - **Marqueurs Détectés**: Liste avec ID, distance, angles
   - **Statistiques**: FPS, taux de détection, taux de précision
   - **Boutons de Contrôle**: Start, Stop, Quit

### Commandes:

```bash
# Démo avec matériel simulé (pour présentations)
python3 demo_gui.py --auto-start

# Test avec webcam réelle
python3 test_webcam.py --auto-start

# Mode production (caméra réelle + drone réel)
python3 production.py --auto-start

# Générer marqueurs ArUco imprimables
python3 generate_markers.py --sheet --ids 0 1 2 3
```

### Fonctionnalités:

- Interface GUI Tkinter (cohérente entre tous les modes)
- Détection ArUco animée (mode démo)
- Support caméra réelle (USB/Picamera2)
- Suivi de marqueurs en temps réel avec overlay visuel
- Télémétrie véhicule simulée (GPS, batterie, mode)
- Simulation de commandes d'atterrissage
- Statistiques de performance (FPS, détection, précision)
- Détecteur ArUco optimisé
- Patron Observer pour mises à jour GUI
- Opération thread-safe
- Outil de génération de marqueurs imprimables

### Lancer une Démonstration:

1. Ouvrir un terminal
2. Naviguer vers le projet: `cd avion-cargo/src`
3. Exécuter: `python3 demo_gui.py --auto-start`
4. L'interface GUI s'ouvrira avec:
   - Marqueurs ArUco animés en cercle
   - Overlay de détection en temps réel
   - État du véhicule mis à jour
   - Statistiques affichant FPS, détection et précision

### Workflow de Test:

1. **Mode Démo** - Tester sans matériel
   ```bash
   python3 demo_gui.py --auto-start
   ```

2. **Générer Marqueurs** - Imprimer marqueurs ArUco réels
   ```bash
   python3 generate_markers.py --sheet --ids 0 1 2 3
   # Imprimer à 5cm x 5cm
   ```

3. **Test Caméra** - Tester avec caméra réelle
   ```bash
   python3 test_webcam.py --auto-start
   ```

4. **Production** - Déployer sur Raspberry Pi avec drone
   ```bash
   python3 production.py --auto-start
   ```
