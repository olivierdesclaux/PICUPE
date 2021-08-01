# Python Recalage

Répertoire de stage à l'été 2021 sous la direction de Lama Séoud.
Actuellement, le répertoire permet de calibrer une caméra RGB avec calibrate.py, et d'appliquer le calibrage sur une autre image avec undistort.py.

## Installation

### Prérequis
1. Installer [miniconda](https://docs.conda.io/en/latest/miniconda.html).
1. Installer [Azure Kinect SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md).
1. Ajouter l'emplacement `C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\bin` à la variable Path ([exemple ici](https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/))
3. Si Visual Studio n'est pas déjà installé, installer [Microsoft Visual C++ 14.0](https://visualstudio.microsoft.com/visual-cpp-build-tools) en choissisant l'option "C++ build tools" 

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

La calibration fonctionne avec une grille de cercles peinte en rouge ou orange.
Cette grille peut être obtenue en découpant au laser du bois avec le fichier Grille.svg.
Pour calibrer une caméra, simplement activer l'environnement et effectuer :
```
python calibrate.py
```
Ensuite, déplacer la grile afin de couvrir toute l'image de la caméra de quadrilatères verts.
Lorsque le compte d'image à prendre atteint 0, le logiciel peut demander de prendre des captures à nouveau.
Ceci signifie qu'il a rejeté des captures trop floues/imprécises, et il faut simplement en prendre davantage.
Les recommendations suivantes sont à suivre pour un calibrage optimal : (tirés de [ce lien](https://stackoverflow.com/questions/12794876/how-to-verify-the-correctness-of-calibration-of-a-webcam/12821056#12821056)):
1. Avoir un bon éclairage
2. Bien fixer la caméra
3. Placer le checkerboard à un angle de la caméra (pas de face directement)
4. Couvrir toute l'image et tout le volume de l'espace à calibrer
5. Éviter le motion blur dans les images
6. S'assurer que le checkerboard est le plus rigide possible

Le calibrage est stocké dans le fichier `calibrationFile.json`.

Une fois le calibrage effectué, utiliser la commande suivante pour appliquer le calibrage à la caméra actuelle :
```
python undistort.py [filename]
```
Par défaut, le nom de fichier utilisé est `calibrationFile.json`, mais un autre fichier de même format peut y être substitué.
