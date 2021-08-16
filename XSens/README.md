# Xsens Device MTw

Module IMU du projet PICUPE. Ce répertoire permet d'acquérir l'orientation et l'accélerations des capteurs inertiels MTw à partir un script python MTwReceiveData.py.

## Installation

### Prérequis
1. Installer miniconda avec python version 3.7
2. Installer Xsens MT Manager software suite [MT Manager 2019](https://www.xsens.com/cs/c/?cta_guid=ead8a68c-79b8-4b40-bb6d-2f39a1f9847a&signature=AAH58kGB_34RkwlhXNk7P-kzkX16xmLbqQ&pageId=27796161161&placement_guid=df26b080-cbde-4fb7-a9b1-7a9351551530&click=b7c20fa8-973f-4f72-a371-538ab87c8489&hsutk=f74d60f437d573572df922e693b12e24&canon=https%3A%2F%2Fwww.xsens.com%2Fsoftware-downloads&utm_referrer=https%3A%2F%2Fduckduckgo.com%2F&portal_id=3446270&redirect_url=APefjpHqm3mQzDGati4iSYrKagogzleEETAexx8Jc0bJQcgBnSiotpi0K950-59F1y_wn_jBEMAuWKXT_ppp2MJCSWIOTJlGlRnZo91Ytq-8psJGXqmWvWgeDhjp8A27t9T60q5PdYwohLHgx_GE4i2ZKHRFaGzWlIo_UN2MfukxhQF3QrYgdiifkZAJ19f7Q5tynDoyiUztZikw7cKIXv_Dpsii4YgpNuqM8ou7iq94ruR6l8kvRAGk4QQqmvZPw7xiO73lJ5MGivX0rbqfoLYdVVlq30ksSsZXYTt0x-XLwC9KUcDd_n5ExknOjAeBur6OtIczkl3h00L-QrlAI4LRp0nDXI5FfS_19jGmlLd_UGnBdd1xv5M&__hstc=81749512.f74d60f437d573572df922e693b12e24.1623932327552.1626901742023.1628781018493.11&__hssc=81749512.1.1628781018493&__hsfp=1948818673&contentType=standard-page)
3. À partir de l'interface MT Manager 2019, choisir *Check for new MT Software Suite versions* sous l'onglet *Help*.
4. Aller dans le répertoire d'installation de Xsens et choisir le dossier MT Software Suite 2021.0
5. Dans ce répertoire le fichier wheel à installer ce retrouve dans MT Software Suite 2021.0\MT SDK\Python\XX ou XX est le système d'opération de Windows.

### Étapes sur ligne de commande
1. Cloner le répertoire
2. Créer l'environnement avec la commande suivante (une fois) :
    ```
    conda env create -f environment.yml
    ```
1. Installer `XsensDeviceApi` séparément depuis la ligne de commande:
    ```
    pip install xsensdeviceapi-2021.0.0-cp37-none-win_amd64.whl
    ```
## Utilisation

1. Lancer le script python sur un terminal avec la commande suivante:
    ```
    python MTwReceiveData.py -c (Numéro de la fréquence d'acquisition) -d (Numéro du canal radio)
    ```
2. Par la suite, il faut attendre que les lumières LED des capteurs MTw clignotent en synchronisation avec la lumière LED de la station awinda. Pour démarrer la détection 
des capteurs MTw, il faut appuyer *ENTER*.
3. Confirmer la détection adéquate des capteurs MTw.
4. Appuyer sur la touche *ENTER* pour démarrer un enregistrement des données.
5. Appuyer sur la touche *ENTER* pour arrêter l'enregistrement des données.
6. Appuyer sur la touche *0* pour fermer les dispositifs

## Notes
Les fichiers texte sous le dossier **MTw Pickle** contiennent toutes les paquets de données réçus par les capteurs inertiels. Le dossier **MTw data** contient les fichiers texte des capteurs qui seront insérés dans le module de Open Sense de Open Sim.
