# Keyence LJ-S640 - retour technique pour scan de facade

Date: 2026-08-20

## Objectif

Ce document resume ce que le Keyence LJ-S640 fait bien, ce qu'il fait moins bien,
et les points a surveiller pour scanner une facade complete avec une precision
utile au dimensionnement.

Le contexte considere ici est le suivant:

- Keyence LJ-S640 monte avec le systeme mecanique/Zaber.
- Scan passe par passe.
- Chaque position peut faire un ou plusieurs scans avec differents parametres.
- Les scans faits sans bouger le Zaber peuvent ensuite etre fusionnes localement.
- Distance nominale au mur visee: environ 1.1 m.
- Zone utile visee par passe: environ 640 mm x 640 mm dans le setup actuel.

## Resume rapide

Le LJ-S640 est tres bon pour obtenir des profils 3D denses et precis quand la
surface renvoie correctement le laser. Il est adapte a une logique de metrologie
locale: profils reguliers, resolution fine, bonne detection des plans, arêtes et
reliefs lorsque les conditions optiques sont bonnes.

Le point difficile n'est pas la precision intrinsèque du capteur, mais la
robustesse sur une facade reelle: surfaces noires, acier brillant, verre,
ombres, soleil, arêtes, joints et changements rapides de materiaux. Ces cas
generent des pixels invalides, du bruit, des dead zones ou des artefacts.

La strategie la plus pertinente est donc:

1. Identifier les conditions de surface avant la passe.
2. Lancer un ou plusieurs presets Keyence a la meme position.
3. Fusionner les scans localement en gardant les pixels les plus fiables.
4. Bouger le Zaber et repeter.
5. Assembler toutes les passes dans le repere global.

## Ce que le Keyence fait bien

### Haute resolution locale

Le capteur produit une image/profil de hauteur dense, adaptee a la reconstruction
3D locale. Pour du dimensionnement, c'est beaucoup plus interessant qu'une
camera RGB seule.

Points forts:

- Bonne resolution locale sur surfaces compatibles.
- Profils reguliers et exploitables pour reconstruire une zone.
- Mesure directe de hauteur/profondeur, pas seulement une image 2D.
- Tres bon comportement sur surfaces mates et assez uniformes.
- Bonne capacite a capturer plans, arêtes, joints et reliefs si le retour laser
  est propre.

### Repetabilite si le systeme ne bouge pas

Si le Zaber reste fixe entre plusieurs scans, les acquisitions Keyence sont dans
le meme repere. Cela rend possible une fusion locale simple:

- scan normal;
- scan surface sombre;
- scan surface brillante/metallique;
- fusion pixel par pixel.

Cette strategie est tres utile pour des cas comme:

- porte noire + poignee metallique;
- mur clair + cadre sombre;
- surface mate + piece reflechissante;
- facade avec zones d'ombre et zones tres lumineuses.

### Controle qualite exploitable

Les images invalid-red permettent de voir rapidement les zones non mesurees. Les
metadonnees donnent aussi des indicateurs utiles:

- pourcentage de pixels invalides;
- nombre de pixels invalides;
- min/max de hauteur brute;
- presence de trous ou artefacts visibles.

## Ce que le Keyence fait moins bien

### Depend fortement du retour optique

Le Keyence n'aime pas les surfaces qui renvoient mal ou trop fort le laser.

Cas difficiles:

- noir profond: retour trop faible;
- acier / chrome / poignee brillante: reflet speculaire, saturation ou retour
  instable;
- verre: transmission/reflexions parasites;
- arêtes franches: occultation geometrique et retours partiels;
- surfaces tres inclinees: le laser repart hors du capteur;
- fortes variations de lumiere ambiante.

### Un seul preset ne marche pas partout

Un preset qui marche sur une porte noire peut etre mauvais sur une poignee en
acier, et inversement:

- exposition haute: aide les surfaces sombres, mais peut saturer le metal;
- exposition basse: aide les surfaces brillantes, mais perd les surfaces noires;
- dynamic range eleve: aide dans les scenes contrastees, mais ne resout pas tout;
- filtres trop agressifs: peuvent supprimer des points utiles ou lisser des
  details.

Conclusion: il faut accepter plusieurs scans par position lorsque plusieurs
types de surfaces sont presents.

### Risque de dead zones

Une dead zone est une zone ou le capteur ne peut pas produire de hauteur valide,
souvent autour d'une arête, d'un changement brutal de profondeur ou d'une zone
occultee par la geometrie.

Ca arrive notamment quand:

- le laser touche une arête mais la camera du capteur ne voit pas le point utile;
- une surface cache une autre surface du point de vue du capteur;
- une zone reflechissante renvoie le signal ailleurs;
- la surface est trop proche/trop loin de la plage de mesure optimale;
- le scanner n'est pas bien aligne avec le bord ou le plan a scanner.

Impact:

- trous rouges dans l'image invalid-red;
- pertes autour des bords de poignee, cadre, joints, vitrage;
- artefacts dans le mesh si on triangule sans filtrer;
- necessite de rescanner depuis un autre angle ou avec un autre preset.

L'interpolation de dead zone peut aider visuellement, mais elle peut aussi
inventer de la geometrie. Pour du dimensionnement, il faut rester prudent.

## Distance au mur

Dans notre setup, la distance visee est autour de 1.1 m. Il ne faut pas trop
s'en eloigner.

Pourquoi:

- la largeur de profil utile depend de la distance;
- la zone couverte change avec la distance;
- la qualite de retour laser change avec la distance;
- certaines surfaces deviennent plus invalides si on sort de la plage optimale;
- la strategie de step/overlap depend de la largeur effective mesuree.

Effets d'une distance trop faible ou trop grande:

- largeur de profil differente de celle attendue;
- couverture incomplete ou overlap mal estime;
- augmentation des pixels invalides;
- dead zones plus visibles;
- risque de ne plus couvrir proprement les 640 mm x 640 mm vises.

Recommandation:

- viser environ 1.1 m;
- mesurer regulierement la distance avec le Keyence;
- calculer le pas Zaber a partir de la largeur effective du profil;
- garder une marge d'overlap pour eviter les gaps;
- verifier la calibration scanner/axe/mur avec une mire plane.

## Parametres Keyence disponibles

Les parametres ci-dessous sont ceux exposes par les scripts actuels.

### exposure

Valeurs:

- 0: 15 us
- 1: 30 us
- 2: 60 us
- 3: 80 us
- 4: 120 us
- 5: 160 us
- 6: 210 us
- 7: 240 us
- 8: 320 us
- 9: 380 us
- 10: 480 us
- 11: 640 us
- 12: 960 us
- 13: 1700 us
- 14: 4800 us
- 15: 9600 us

Effet:

- augmente ou diminue le temps d'exposition du capteur;
- plus haut: aide les surfaces sombres/noires;
- plus bas: aide les surfaces brillantes/metalliques;
- trop haut: saturation, reflets, invalides sur metal/verre;
- trop bas: pertes sur surfaces sombres.

Presets typiques:

- normal: 11-12;
- noir/sombre: 14-15;
- metal/brillant: 7-10.

### dynamic_range

Valeurs: 1 a 9.

Effet:

- augmente la capacite a gerer des scenes contrastees;
- utile quand il y a noir + clair + reflets dans la meme passe;
- valeur elevee recommandee pour surfaces mixtes ou brillantes.

Presets typiques:

- normal: 6-8;
- sombre/brillant/mixte: 8-9.

### light_mode

Valeurs:

- 0: MANUAL
- 1: AUTO
- 2: SLOPE

Effet:

- controle la logique d'intensite lumineuse;
- MANUAL donne un comportement plus reproductible;
- AUTO peut s'adapter mais rend les comparaisons moins stables;
- SLOPE peut aider selon la surface et le profil.

Usage:

- MANUAL pour tests comparables;
- SLOPE/AUTO a tester pour surfaces difficiles;
- eviter de changer trop de variables a la fois pendant l'analyse.

### light_upper et light_lower

Valeurs: 1 a 99.

Effet:

- bornes d'intensite lumineuse;
- 99 maximise la puissance disponible;
- reduire peut aider si metal/verre saturent;
- augmenter aide les surfaces sombres mais peut degrader les reflets.

Usage:

- 99/99 pour surfaces sombres;
- 60-90 a tester sur surfaces brillantes;
- garder fixe pendant un sweep si on veut isoler l'effet d'un autre parametre.

### x_subsample

Valeurs: 1 a 2.

Effet:

- sous-echantillonnage dans l'axe X;
- 1 conserve la resolution maximale;
- 2 reduit les donnees et peut accelerer/alleger, mais perd de la resolution.

Usage:

- garder a 1 pour dimensionnement precis.

### y_subsample

Valeurs: 1 a 8.

Effet:

- sous-echantillonnage dans l'axe Y;
- plus la valeur augmente, plus on perd de details dans cette direction.

Usage:

- garder a 1 pour les scans de qualite;
- augmenter seulement pour tests rapides ou debug.

### detection_sensitivity

Valeurs: 1 a 5.

Effet:

- sensibilite de detection du pic laser;
- plus haut: detecte des retours plus faibles;
- trop haut: peut accepter du bruit ou des faux retours;
- trop bas: peut perdre des surfaces sombres/faibles.

Usage:

- 5 pour maximiser les points detectes;
- tester plus bas si bruit/faux points sur surfaces brillantes.

### dead_zone_interpolation

Valeurs:

- 0: off
- 1: horizontal_vertical
- 2: linear

Effet:

- tente de remplir ou lisser les zones invalides;
- peut ameliorer l'apparence;
- peut masquer un vrai probleme de mesure;
- peut inventer de la geometrie au voisinage des arêtes.

Usage:

- off pour diagnostiquer les vrais trous;
- linear pour visualisation ou fusion si on accepte une interpolation;
- prudent pour du dimensionnement: mieux vaut garder une carte de confiance.

### peak_width_filter

Valeurs:

- off
- on strength 1 a 5

Effet:

- filtre selon la largeur du pic detecte;
- peut supprimer des retours parasites;
- peut aider sur metal/reflets;
- trop fort peut supprimer des details utiles ou des retours faibles.

Usage:

- off pour surfaces sombres si le signal est faible;
- on 2-3 pour surfaces brillantes/metalliques;
- tester plusieurs forces pour poignee, vitrage, acier.

## Strategies de presets

### Preset normal

Objectif: surface mate ou facade standard.

Exemple:

- exposure: 12
- dynamic_range: 6-8
- light_mode: 2 ou 0
- light_upper/lower: 99
- detection_sensitivity: 5
- dead_zone_interpolation: 2
- peak_width_filter: on 2

### Preset sombre

Objectif: porte noire, peinture sombre, zone a faible retour.

Exemple:

- exposure: 14-15
- dynamic_range: 8-9
- light_mode: 0
- light_upper/lower: 99
- detection_sensitivity: 5
- dead_zone_interpolation: 0 ou 2
- peak_width_filter: off

### Preset metal/brillant

Objectif: poignee acier, chrome, surface reflechissante.

Exemple:

- exposure: 7-10
- dynamic_range: 9
- light_mode: 0 ou 2
- light_upper/lower: 60-99
- detection_sensitivity: 4-5
- dead_zone_interpolation: 0 pour diagnostic, 2 pour rendu
- peak_width_filter: on 2-4

### Preset mixte

Objectif: zone avec noir + clair + metal.

Strategie:

- faire au moins un scan normal;
- ajouter un scan sombre si pixels noirs detectes;
- ajouter un scan metal si highlights/reflets detectes;
- fusionner localement les resultats.

## Fusion multi-scans

Comme le Zaber ne bouge pas entre les scans d'un meme set, les images Keyence
sont superposables. Une fusion simple peut deja aider:

1. Pour chaque pixel, garder une valeur valide si elle existe.
2. Si plusieurs scans sont valides, choisir celui avec le meilleur score local.
3. Garder une carte de confiance pour savoir d'ou vient chaque pixel.

Score possible:

- pixel valide ou invalide;
- luminance pas saturee;
- coherence avec voisins;
- absence de saut brutal de Z;
- faible bruit local;
- parametre utilise.

Important:

- ne pas laisser l'interpolation cacher les zones non mesurees;
- conserver les invalid-red ou une carte d'invalides;
- pour le dimensionnement, preferer une mesure valide reelle a une interpolation.

## Role d'une camera OpenMV / IA

La camera n'a pas besoin de segmenter parfaitement les objets. Comme le Keyence
fonctionne passe par passe, elle doit surtout decider quels presets tenter.

Sortie utile:

- DARK present?
- SHINY/METAL present?
- NORMAL/MATTE present?
- forte texture/arêtes?

Puis:

- DARK -> scan sombre;
- SHINY/METAL -> scan metal;
- NORMAL -> scan normal;
- si plusieurs classes -> plusieurs scans au meme endroit.

La precision finale vient du Keyence et de la calibration mecanique, pas de la
camera. La camera sert a automatiser le choix des acquisitions.

## Limitations principales pour dimensionnement

Les facteurs limitants sont:

- calibration Keyence/Zaber/mur;
- stabilite mecanique;
- distance au mur autour de 1.1 m;
- angle du capteur;
- surfaces optiquement difficiles;
- dead zones aux arêtes;
- fusion des passes successives;
- controle qualite des zones invalides.

Pour garantir du dimensionnement fiable, il faut valider avec:

- mire plane;
- objet de dimensions connues;
- arête connue;
- tests repetes a differents presets;
- comparaison entre passes avec overlap.

## Recommandations

1. Garder le Keyence comme capteur metrologique principal.
2. Ne pas chercher un preset unique pour toute la facade.
3. Utiliser une camera ou une logique de pre-analyse pour choisir les presets.
4. Faire plusieurs scans sans bouger quand la surface est mixte.
5. Fusionner localement les scans multi-parametres.
6. Garder les cartes d'invalides et de confiance.
7. Maintenir la distance autour de 1.1 m.
8. Mettre en place une calibration geometrique robuste.
9. Utiliser l'overlap entre passes pour verifier l'assemblage global.
10. Differencier resolution visuelle, densite de points et precision
    dimensionnelle reelle.

## Conclusion

Le Keyence LJ-S640 est un tres bon capteur pour reconstruire localement une
facade avec une haute densite de points, mais il n'est pas autonome face a la
diversite optique d'une facade reelle. Les problemes principaux sont les pixels
invalides, les dead zones, les reflets et la dependance aux parametres
d'acquisition.

La meilleure architecture n'est pas "un preset parfait", mais une boucle
adaptative:

camera/analyse -> choix de presets -> plusieurs scans Keyence fixes -> fusion
locale -> deplacement Zaber -> fusion globale.

Cette approche permet de garder la precision du Keyence tout en augmentant la
robustesse sur les surfaces difficiles.
