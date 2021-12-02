# Python Recalage

Répertoire de stage à l'été 2021 sous la direction de Lama Séoud.
Actuellement, le répertoire permet de calibrer une caméra avec `calibrate.py` et une grille de cercles, de recaler deux caméras avec `rectify.py` et d'afficher le recalage avec `remapy.py`.

## Installation

### Prérequis
1. Installer [miniconda](https://docs.conda.io/en/latest/miniconda.html).
1. Installer [Azure Kinect SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md).
1. Ajouter l'emplacement `C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\bin` à la variable Path ([exemple ici](https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/))
1. Si Visual Studio n'est pas déjà installé, installer [Microsoft Visual C++ 14.0](https://visualstudio.microsoft.com/visual-cpp-build-tools) en choissisant l'option "C++ build tools" 

![image de l'installation](https://docs.microsoft.com/en-us/answers/storage/attachments/34873-10262.png "")

### Étapes sur ligne de commande
1. Cloner le répertoire.
1. Créer l'environnement avec la commande suivante (une fois) : 
    ```
    conda env create -f environment.yml
    ```
1. Utiliser la commande suivante pour activer l'environnement à chaque fois :
    ```
    conda activate stage2021 
    ```
1. Installer `pyk4a` séparément depuis la ligne de commande (en raison d'un bug dans pip):
    ```
    pip install pyk4a --no-use-pep517 --global-option=build_ext --global-option="-IC:\Program Files\Azure Kinect SDK v1.4.1\sdk\include" --global-option="-LC:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\lib"
    ```


## Utilisation

### Calibrage
La calibration fonctionne avec une grille de cercles peinte en rouge ou orange.
Cette grille peut être obtenue en découpant au laser du bois avec le fichier Grille.svg.
Pour calibrer une caméra, activer l'environnement, brancher la caméra voulue et effectuer :
```
python calibrate.py -c CAMERA
```
Paramètres :
-c : Type de caméra. Accepte K/Kinect, F/FLIR ou W/Webcam

Ensuite, déplacer la grile afin de couvrir toute l'image de la caméra de quadrilatères verts.
Il est également important de varier la distance entre la grille et la caméra (captures plus proches et plus lointaines).
Lorsque le compte d'image à prendre atteint 0, le logiciel peut demander de prendre des captures à nouveau.
Ceci signifie qu'il a rejeté des captures trop floues/imprécises, et il faut prendre de nouvelles captures.

Les recommendations suivantes sont à suivre pour un calibrage optimal : (tirés de [ce lien](https://stackoverflow.com/questions/12794876/how-to-verify-the-correctness-of-calibration-of-a-webcam/12821056#12821056)):
1. Avoir un bon éclairage
1. Bien fixer la caméra
1. Placer le checkerboard à un angle de la caméra (pas de face directement)
1. Couvrir toute l'image et tout le volume de l'espace à calibrer
1. Éviter le motion blur dans les images
1. S'assurer que le checkerboard est le plus rigide possible

Le calibrage est stocké dans le fichier `Results/CAMERA_Date/Calib.json`.

### Recalage
Le recalage permet de déformer deux images pour en faire correspondre les pixels.
Il fonctionne aussi avec une grille de cercles peinte en orange.
Pour recaler deux caméras branchées à l'ordinateur, effectuer :
```
python rectify.py -c CAMERAS --calib1 CALIB1.json --calib2 CALIB2.json
```
Paramètres :
-c : Types de caméra, dans l'ordre, 1 lettre chacune. Accepte K (Kinect), F (FLIR) ou W (Webcam). Exemple : KF (Kinect + FLIR)
--calib1 : Fichier de calibrage de la première caméra (calculé avec calibrate.py)
--calib2 : Fichier de calibrage de la seconde caméra (calculé avec calibrate.py)

Ensuite, il faut déplacer la grille de calibrage pour couvrir le plus possible le volume observé par les caméras.
La grille n'est acceptée que si elle est vue dans les deux caméras simultanément.
Les recommandations qui s'applique au calibrage s'appliquent également au recalage.

Le recalage est stocké dans le fichier `Results/CAMERAS_Date/RectifyCAMERAS.json`

### Affichage
Une fois le recalage effectué, il est possible d'afficher le résultat du recalage avec deux caméras.
Le script s'utilise de la manière suivante :
```
python remap.py -c CAMERAS --rectFile RECTIFYCAMERAS.json
```
Paramètres :
-c : Types de caméra, dans l'ordre, 1 lettre chacune. Accepte K (Kinect), F (FLIR) ou W (Webcam). Exemple : KF (Kinect + FLIR)
--rectFile : Fichier de rectifiation généré par `rectify.py`.

Pour l'affichage superposé, les paramètres manuels sont ajustables dans `remap.py`.


