# Grilles de cercles

La grille utilisée pour le calibrage et recalage est fabriquée à partir d’une planche de bois, découpée au laser et peinte.

### Dessin
Le dessin de la grille peut être modifié/obtenu de la manière suivante :
1. Modifier le fichier `Grille.SLDPRT` avec SolidWorks.
1. Ensuite, le fichier `Grille.SLDDRW` sera mise à jour automatiquement, et il est possible d'exporter ce dessin vers un PDF dans SolidWorks.
1. Dans Inkscape (logiciel gratuit), on peut ouvrir le PDF et recadrer juste sur le dessin avec "Document Properties". 
	- Utiliser une marge de 0.5 mm.
	- Si voulu, il est possible d'ajouter un peu de texte (date, version de la grille) à graver dans la grille, en noir.
![Image de Document Properties](https://logosbynick.com/wp-content/uploads/2020/10/add-bleed-margin.png "")

### Découpe
La procédure pour la découpe et peinture est la suivante :
1. Suivre la formation du PolyFab/autre labo pour l'utilisation de la découpe laser.
1. Découper une plaque de bois de 1/8" ou 1/4" d'épais, plus grand que la grille.
	- Le maximum permis au PolyFab est 17" x 29". 
1. Mettre le fichier `.svg` de la grille sur une clé USB. Le plus récent fichier est `Grille v3.svg`.
1. Effectuer la découpe laser en utilisant la procédure du PolyFab. 
	- Attention d'utiliser les bons paramètres pour le bois choisi. 
	- Au PolyFab, les paramètres pour 1/8" sont disponibles dans `Guillaume/mdf 1/8`
1. Une fois la découpe effectuée, peindre la grille en rouge/orange.
	- Une peinture aérosol ou par pinceau peut être utilisée.

Si une autre couleur est désirée, il faut modifier une ligne de code dans `_grayify()`, de `circlegridfinder.py`.
- Pour bleu : `return cv.subtract(frame[:,:,0], frame[:,:,2])`
- Pour vert : `return cv.subtract(frame[:,:,1], frame[:,:,2])`
	

