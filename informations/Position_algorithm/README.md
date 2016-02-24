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
* Lx1 = (Lx1x, Lx1y, Lx1z)
* Vx1, Vx2, Vy1, Vy2, Vz1, Vz2 = Orientation originale des lasers 
* Vx1 = (Vx1x, Vx1y, Vx1z)
* Mx1, Mx2, My1, My2, Mz1, Mz2 = Mesure des lasers

#### OUTPUT 
* Position en X, Y, Z
* Orientation réelle sous la forme d'un quaternion


Algorithme à quatre lasers
--------------------------

### Repère

Le repère utilisé est un repère orthogonal centré sur le centre de gravité du drone, dont les axes sont perpendiculaire aux murs. Le laser en X vise donc vers (1, 0, 0) et la normale du mur est donc (-1, 0, 0).

### Position et orientation des lasers

Nous effectuons une rotation de la position et de l'orientation de chacun des lasers via le quaternion q de l'IMU. Pour le réaliser, nous convertissons le point/vecteur vers un quaternion donc w = 0. Ensuite, nous récupérons la matrice 4x4 de transformation depuis le quaternion. Enfin, il suffit d'appliquer un produit scalaire entre la matrice de transformation et notre point/vecteur.

La fonction rotate(p, q) s'en charge et est capable de réaliser une rotation autant sur un point, qu'un vecteur qu'un quaternion.

    def rotate(p, q):
        # if we got only a point/vector, convert it to a quaternion (x,y,z,w), with w = 0
        if(len(p) == 3):
            p = p + (0,)
        m = quaternion_matrix(q)
        return dot(m, p)

    Lrx1 = rotate(Lx1, q)
    Vrx1 = rotate(Vx1, q)

Une rotation selon un quaternion est lourde en calculs. Cette partie peut donc être optimisée en ne réalisant la rotation que d'un laser et en bougeant les autres points en conséquence.

NOTE : Nous devons utiliser un tuple pour réaliser la concaténation p = p + (0,). 

### Extrapolation de la mesure

Sachant que nous avons mesuré une distance Mx1 sur le laser x1 réellement positionné en Lrx1 avec l'orientation Vrx2, nous pouvons déterminer la position d'un point sur le mur. 

Nous déterminons donc K, le facteur de proportionnalité de Vrx2 afin que le vecteur ait la longueur M.

    # Vrx1 = Orientation du laser x1 après rotation
    # Vrx1 = (Vrx1x, Vrx1y, Vrx1z)
    # Lrx1 = Position du laser x1 après rotation
    # Lrx1 = (Lrx1x, Lrx1y, Lrx1z)
    # Longueur d'un vecteur (a,b,c) = sqrt(a² + b² + c²)
    # Soit Vrx1 = (k*Vrx1x, k*Vrx1y, k*Vrx1z)
    # k tel que la longueur du vecteur = M 
    # M = sqrt(k²*Vrx1x + k²*Vrx1y + k²*Vrx1z)
    # M² = k²*Vrx1x + k²*Vrx1y + k²*Vrx1z
    # Soit Px1 le point touchant le plan du mur

    k = M/sqrt(Vrx1x² + Vrx1y² + Vrx1z²)
    Px1 = (k*Vrx1x + Lrx1x, k*Vrx1y + Lrx1y, k*Vrx1z + Lrx1z)


Ce calcul est réalisé par la fonction extrapolate(p,v,M)

    # computes the X point where the laser points to on the well
    # input p : point with the laser
    # input v : the vector direction
    # input M : distance measured
    def extrapolate(p,v,M):
        K = sqrt(M*M/(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]))
        v = (K*v[0]+p[0], K*v[1]+p[1], K*v[2]+p[2])
        return v

### Calcul du plan du mur

L'équation générale du plan est P === ax + by + cz + d = 0 dont (a,b,c) est la normale.

La normale étant N = (a,b,c) et sachant que nous avons un point Px1(Px1x, Px1y, Px1z) passant par ce plan, son équation est déterminée via : 

    a*Px1x + b*Px1y + c*Px1z + d = 0
    d = -(a*Px1x + b*Px1y + c*Px1z)
    Px === ax + by + bz -(a*Px1x + b*Px1y + c*Px1z) = 0

### Calcul de la distance au mur

Il suffit désormais de déterminer la distance au plan du point (0,0,0) afin de déteminer la position du drone.
    
    # L'équation de la distance d'un point à un plan étant :
    d(A(xA,yA, zA),P) = abs(a*xA + b*yA + c*zA + d)/sqrt(a² + b² + c²)

    # La distance de l'origine au plan X, sachant que A = (0,0,0) et N = (-1,0,0) est :
    Dx = abs(d)/sqrt(a² + b² + c²)
    Dx = abs(a*Px1x + b*Px1y + c*Px1z)/sqrt(-1² + 0 + 0)
    Dx = abs(Px1x) # Logique

    # Cette étape est donc inutile, de par la convention des axes. Cela dit, il est intéressant de la réaliser afin que cet algorithme puisse être utilisé et modifié par d'autres, avec éventuellement un autre système d'axe.



Algorithme à six lasers
--------------------------

TODO

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
