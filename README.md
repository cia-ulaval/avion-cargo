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
\( \leq 1 \) px.

Pour plus d’informations sur le script de calibration :

```shell
poetry run calibrate_camera --help
```

#### 2.3 Estimation de pose

L’estimation de pose permet de détecter un tag ArUco et d’estimer sa position (et donc la distance) par rapport à la caméra.

Pour plus d’informations sur le script d’estimation de pose :

```shell
poetry run estimate_pose --help
```

> :warning: L’estimation de pose nécessite un fichier de calibration.  
> Les mêmes paramètres (board, tailles, etc.) utilisés lors de la calibration doivent être réutilisés pour l’estimation de pose.

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