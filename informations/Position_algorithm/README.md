Algorithme de positionnement
====================

Introduction 
-------------

Dans un premier temps, en vue du développement, l'algorithme de positionnement est réalisé en Python. En effet, bien que moins performant pour la quantité de calcul à réaliser, le Python permettra de diminuer le temps de développement car il n'est pas nécessaire de compiler afin d'obtenir les résultats. Il est en effet plus un langage pour réaliser des scripts qu'un programme à efficacité maximale.

L'algorithme de positionnement a été réalisé avec deux variantes. L'une comportant 4 lasers, et l'autre en comportant 6. En efet, 4 lasers est le minimum afin de récupérer la position du drone dans l'espace et l'orientation autour de l'axe Z (yaw). Via 6 lasers, il est alors possible de récupérer l'état angulaire complet au prix de plus de calculs.

Énoncé
----------

Nous avons 4 (ou 6) lasers fixes sur le drone, et il faut retrouver sa localisation dans l'espace. De plus, le yaw, l'orientation selon l'axe z, n'est pas utilisable. En effet, les perturbation magnétiques dues aux moteurs, aux masses métalliques et au fait que l'on soit en intérieur rend le magnétomètre inutilisable pour déterminer l'orientation.

### Caractéristiques communes

* Algorithme convolutionnel
* Réalisable sur 600 mesures par seconde sur du matériel embarqué
* Nous ne pouvons pas faire confiance au yaw provenant de l'IMU (orientation selon z)

### Algorithme à 4 lasers

#### Demandé
* Calcule la distance du drone aux murs 
* Calcule l'état angulaire selon z du drone 

#### INPUT
* 4 Lasers fixes sur un drone, 1 dans chaque direction avec un supplémentaire en X (ou Y) afin de calculer le yaw
* q = Quaternion de rotation du drone, son état angulaire comprenant les rotations selon x, y et z (roll, pitch et yaw)
* Lx1, Lx2, Ly1, Lz1 = Position originale des lasers
* Vx1, Vx2, Vy1, Vz1 = Orientation originale des lasers 
* Mx1, Mx2, My1, Mz1 = Mesure des lasers

#### OUTPUT 
* Position en X, Y, Z
* Orientation réelle sous la forme d'un quaternion

### Algorithme à 6 lasers

#### Demandé
* Calcule la distance du drone aux murs 
* Calcule l'état angulaire du drone 
* L'état angulaire peut n'être que le yaw auquel on ajoute le pitch et le roll venant de l'IMU

#### INPUT
* 6 Lasers fixes sur un drone, 2 dans chaque direction (X,Y,Z).
* q = Quaternion de rotation du drone, son état angulaire comprenant les rotations selon x, y et z (roll, pitch et yaw)
* Lx1, Lx2, Ly1, Ly2, Lz1, Lz2 = Position originale des lasers
* Vx1, Vx2, Vy1, Vy2, Vz1, Vz2 = Orientation originale des lasers 
* Mx1, Mx2, My1, My2, Mz1, Mz2 = Mesure des lasers

#### OUTPUT 
* Position en X, Y, Z
* Orientation réelle sous la forme d'un quaternion


Critères
----------








LA SUITE D'HIER  (solution1) : 

Une fois que j'ai  : Le laser, son orienation, sa position finale, son orientation finale, le point touchant le mur, ALORS  => J'ai le mur ! 

Plan === ax + by + cz + d = 0
Point sur le plan = (kvx + lx, kvy + ly, kvz + lz) 
Laser à son origine (!)  =  (lx, ly, lz)
 
Équation du plan : 
(a,b,c) = sa normale = l'orientation initiale du laser = (pour x) (-1,0,0) 
d => utilisation du point existant, qui APPARTIENT au plan => 
plan === ax + by + cz + d = 0 => a*(kvx + lx) + b*(kvy + ly) + c*(kvz + lz) + d = 0 => (pour X, a = -1) a * (kvx + lx) + d = 0 => d = kvx + lx 
plan === x = kvx + lx

Intersection d'un point et d'un plan :
https://fr.wikipedia.org/wiki/Distance_d%27un_point_%C3%A0_un_plan
d = abs(a*lx + b*ly + c*lz + d)/sqrt(a² + b² + c²)
d = Valeur de X


--------------------------------------------

Solution 2 : Cherche à trouver yaw 
Je trouve non le point du mur, mais AUSSI le point qui touche le mur sachant qu'il est encore touché par le yaw. (implication de pitch et roll nulle), via deux rotations.

L'une par le quaternion inverse => Retour à la position initiale, l'autre par le yaw que l'on trouve dans le quaternion (original), soit le yaw du drone. 

-- Là, je devrais trouver l'intersection entre le point et la droite qui est la droite passant par le laser avec l'orientation du laser quand il est touché seulement par yaw, afin d'avoir la distance et commencer le calcul du yaw --

--------------------------------------------

Solution 3 : Je représente les 3 rotations qu'a fait le laser via UNE rotation autour d'un axe. Ainsi, les angles sont facile à corriger (UN SEUL COS) et pouf, on a la réponse 

http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm 
