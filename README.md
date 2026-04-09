<div align="center">

[![Made with Python][python-shield]][python-url]
[![Built with Poetry][poetry-shield]][poetry-url]
[![Made for Raspberry Pi][rpi-shield]][rpi-url]
[![Contributing][contributing-shield]][contributing-url]

</div>

<div align="center" style="margin: 3rem 0 1rem 0">
  <a href="./">
    <img src="assets/img/landing.png" alt="Project logo" width="140" height="140">
  </a>

  <h1 align="center">
  AUTOLANDER
  </h1>
</div>


**AUTOLANDER** est un projet de vision embarquée visant à faciliter l’atterrissage de précision
à l’aide de tags ArUco et d’une caméra sur **Raspberry Pi**.  

Le dépôt inclut des scripts pour calibrer la caméra et estimer la pose (distance/orientation) d’un tag détecté.

## Prérequis

Pour exécuter ou contribuer à ce projet, assurez-vous d’avoir installé :

1. [Python 3 (>= 3.11, < 3.14)][python-installation-url]
2. [Poetry (>= 2.1.3)][poetry-installation-url]

> :information_source: Sur Raspberry Pi, il est possible que vous deviez installer des dépendances système supplémentaires.
> Elles sont généralement indiquées dans les messages d’erreur lors de l’installation ou de l’exécution.

---

## Commandes à connaître

### 1. Ajouter et supprimer des dépendances

Il est recommandé d’utiliser `poetry` pour la gestion des dépendances plutôt que `pip`.  
Poetry permet de reproduire exactement l’environnement de travail (versions de dépendances identiques) et crée un
environnement virtuel afin d’éviter de polluer vos packages Python système.

#### 1.1 Ajouter une dépendance

##### 1.1.1 Ajout d’une dépendance « principale »
```shell
poetry add <nom_de_la_dependance>
```

##### 1.1.2 Ajout d’une dépendance dans un groupe
```shell
poetry add --group <nom_du_groupe> <nom_de_la_dependance>
```

Par exemple, la commande `poetry add --group dev flake8` ajoute l’outil `flake8` dans le groupe `dev`.

#### 1.2 Supprimer une dépendance

```shell
poetry remove <nom_de_la_dependance>
```

> Il est normal qu’à l’ajout ou à la suppression d’une dépendance, les fichiers
> [pyproject.toml](pyproject.toml) et [poetry.lock](poetry.lock) soient modifiés automatiquement.
> Veuillez faire un commit lors de ces changements.

---

### 2. Exécution du programme et des scripts

Pour exécuter le programme ou certains scripts du projet, installez d’abord les dépendances requises (à la racine du dépôt).

#### 2.1 Installation des dépendances

```shell
poetry install
```

#### 2.2 Calibration de la caméra

Il est important de calibrer la caméra afin d’obtenir la matrice intrinsèque et les coefficients de distorsion.  
Une bonne calibration, pour l’estimation de pose, devrait idéalement fournir une erreur de reprojection
$\leq$ 1 px.

Pour plus d’informations sur le script de calibration :

```shell
poetry run calibrate_camera --help
```

#### 2.3 Atterrissage de précision

#### 2.3.1 Commande pour l'ordinateur de bord

Après avoir obtenu une bonne calibration de la caméra, vous pouvez opérer l'atterrissage de précision.

Pour ce, il vous faudra un fichier de configuration qui fournit au logiciel certaines informations pour son fonctionnement voir la section sur le [fichier de configuration](#5-le-fichier-de-configuration).

Pour lancer [autolander](#) pour operer un l'atterrissage sur ArUco:

```shell
poetry run precion_landing [PATH_TO_CONFIGURATION_FILE]
```

Par exemple, si le fichier de config est à la racine et s'appelle [landing_config.json](landing_config.json): 

```shell
poetry run precion_landing landing_config.json
```

Pour plus d’informations sur la commande `precision_landing`:

```shell
poetry run precion_landing --help
```
#### 2.3.2 Paramètres à mettre à jour sur le Flight Controller

---

### 3. Activer l’environnement virtuel Poetry dans le terminal courant

Activer l’environnement virtuel Poetry dans la session courante permet d’utiliser les commandes sans `poetry run ...`.  
Une fois activé, les scripts définis dans [pyproject.toml](pyproject.toml) (section `[tool.poetry.scripts]`) deviennent accessibles comme des commandes système.

```shell
eval "$(poetry env activate)"
```

Ensuite, par exemple :

```shell
calibrate_camera --help
```

---

### 4. Maintenance de la base de code

Les outils [black][black-python-tool-url], [flake8][flake8-python-tool-url] et [isort][isort-python-tool-url] ont été ajoutés au projet pour permettre de maintenir propre la base de code sur ce dépôt. 

#### 4.1. Appliquer le formatage de style dans le projet 

```shell
poetry run fmt
```

#### 4.2. Vérifier le formatage de style

```shell
poetry run fmt-check # ou
flake8 . # exécuter à la racine du projet
```

### 5. Le fichier de configuration

Le fichier de configuration est nécessaire pour faire l'atterrissage de précision. C'est un fichier `json` avant la structure suivante :

```text
{
  "camera": {
    "id": "entier représentant l'identifiant de la caméra. Généralement 0 pour la caméra par défaut du système.",
    "use_picamera": "booléen. true pour utiliser Picamera2 sur Raspberry Pi, false pour utiliser OpenCV / caméra système classique.",
    "fps": "nombre entier représentant le framerate souhaité pour la capture vidéo.",
    "calibration_filepath": "chemin absolu ou relatif vers le fichier de calibration de la caméra (formats supportés : .npz ou .yaml).",
    "gz_simulation": {
      "topic_name": "chaîne de caractères représentant le nom du topic Gazebo à utiliser pour récupérer les images de la caméra simulée. Utile seulement en simulation."
    }
  },

  "vision": {
    "targeted_marker": {
      "length": "taille réelle du côté du marqueur ArUco en mètres. Exemple : 0.896 pour un marqueur de 89.6 cm.",
      "id": "identifiant entier du marqueur ArUco cible à détecter.",
      "aruco_dictionary": "identifiant entier du dictionnaire ArUco utilisé pour générer et détecter le marqueur. Valeur entre 0 et 4 voir la table recapitulative en 6"
    }
  },

  "streaming": {
    "port": "port réseau utilisé pour exposer le flux de streaming ou le serveur associé.",
    "data": {
      "dps": "fréquence d'envoi des données de télémétrie ou de tracking, en données par seconde."
    },
    "video": {
      "fps": "framerate du flux vidéo diffusé. Peut être différent du fps de capture caméra."
    }
  },

  "drone_connection": {
    "use_serial": "booléen. true pour utiliser une connexion série UART, false pour utiliser une connexion réseau UDP/TCP selon l'implémentation.",
    "address": "adresse IP de la cible pour la connexion réseau. Par défaut : 127.0.0.1",
    "port": "port réseau utilisé pour la connexion au drone ou au simulateur. Par défaut : 14550",
    "baud_rate": "vitesse de communication série en bauds. Utilisée si use_serial = true. Par défaut: 921600."
  }
}
```

>:information_source: Le fichier de configuration recommandé pour RaspberryPi est [landing_config.json](landing_config.json)  
>
>:warning: La structure de ce fichier doit être respectée.

### 6. Table recapitulative pour dictionnaires ArUco

Le tableau suivant présente la correspondance entre les identifiants numériques et les noms des dictionnaires ArUco pris en charge :

| ID | Nom du dictionnaire | Taille de grille    | Nombre de marqueurs | Remarque                                                                         |
|----|---------------------|---------------------|---------------------|----------------------------------------------------------------------------------|
| 0  | DICT_4X4_50         | 4 × 4               | 50                  | Dictionnaire compact, utile si peu d’identifiants sont nécessaires               |
| 1  | DICT_5X5_50         | 5 × 5               | 50                  | Plus de bits que 4x4, meilleure capacité de distinction                          |
| 2  | DICT_6X6_50         | 6 × 6               | 50                  | Plus robuste pour des usages exigeant une meilleure unicité visuelle             |
| 3  | DICT_7X7_50         | 7 × 7               | 50                  | Très riche en information, mais plus exigeant en qualité d’image                 |
| 4  | DICT_ARUCO_ORIGINAL | Variable historique | Variable historique | Dictionnaire ArUco original, utilisé pour compatibilité avec d’anciens marqueurs |

### 5. Notes

Le code ayant été conçu spécifiquement pour fonctionner sur Raspberry Pi, son comportement sur d’autres plateformes
n’a pas été testé de manière rigoureuse.

------------------------------------------------------------------------------------------------------------------------

> Équipe Avion Cargo  
>
> Département de génie mécanique et génie industriel  
> Département d'informatique et de génie logiciel  
> Faculté des sciences et de génie  
> Université Laval  
>
> Hiver 2026
> 
------------------------------------------------------------------------------------------------------------------------

<div align="center" style="margin: 3rem 0 1rem 0; display:flex; justify-content: center; align-items: center; gap: 1rem">
  <a href="./">
    <img src="assets/img/CIA_LOGO.webp" alt="Project logo" width="140" height="140">
  </a>

  <a href="./">
    <img src="assets/img/ula_cropped.png" alt="Project logo" width="140" height="140">
  </a>
  
</div>

<!-- BADGES LINKS -->

[project-statement-shield]: https://img.shields.io/badge/Project%20statement-grey?style=for-the-badge
[project-statement-url]: https://projet2025.qualitelogicielle.ca/introduction/

[python-shield]: https://img.shields.io/badge/Made%20with-Python-3776AB?style=for-the-badge&logo=python&logoColor=yellow
[python-url]: https://www.python.org/

[poetry-shield]: https://img.shields.io/badge/Built%20with-Poetry-60A5FA?style=for-the-badge&logo=poetry&logoColor=1A2CA3
[poetry-url]: https://python-poetry.org/

[rpi-shield]: https://img.shields.io/badge/Made%20for-Raspberry%20Pi-C51A4A?style=for-the-badge&logo=raspberrypi&logoColor=red
[rpi-url]: https://www.raspberrypi.com/

[contributing-shield]: https://img.shields.io/badge/Contributing-Team%20only-0F766E?style=for-the-badge
[contributing-url]: ./CONTRIBUTING.md

<!-- Docs links -->

[python-installation-url]: https://www.python.org/downloads/
[poetry-installation-url]: https://python-poetry.org/docs/#installation
[black-python-tool-url]: https://github.com/psf/black
[flake8-python-tool-url]: https://flake8.pycqa.org/en/latest/user/
[isort-python-tool-url]: https://pycqa.github.io/isort/