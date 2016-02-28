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

Une rotation selon un quaternion est lourde en calculs. Cette partie peut donc être optimisée en ne réalisant la rotation que d'un laser et en bougeant les autres points en conséquence. **NOT IMPLEMENTED AND NOT PLANNED**

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
 
[Source](https://fr.wikipedia.org/wiki/Distance_d%27un_point_%C3%A0_un_plan)

Il suffit désormais de déterminer la distance au plan du point (0,0,0) afin de déteminer la position du drone.
    
    # L'équation de la distance d'un point à un plan étant :
    d(A(xA,yA, zA),P) = abs(a*xA + b*yA + c*zA + d)/sqrt(a² + b² + c²)

    # La distance de l'origine au plan X, sachant que A = (0,0,0) et N = (-1,0,0) est :
    Dx = abs(d)/sqrt(a² + b² + c²)
    Dx = abs(a*Px1x + b*Px1y + c*Px1z)/sqrt(-1² + 0 + 0)
    Dx = abs(Px1x) # Logique

    # Cette étape est donc inutile, de par la convention des axes. Cela dit, il est intéressant de la réaliser afin que cet algorithme puisse être utilisé et modifié par d'autres, avec éventuellement un autre système d'axe.

### Angle Yaw 

L'angle Yaw n'est pas fiable, ceci est du à l'application en intérieur, à la haute puissance des moteurs ainsi qu'aux perturbations dues aux masses métalliques. De plus, le yaw est primordial afin de déterminer notre position. C'est pourquoi nous devons le calculer selon les mesures réalisées.

Afin de le déterminer, nous avons paramétrisé une rotation autour de l'axe Z, et via deux mesures, il est possible de retrouver l'angle yaw. 

Mais attention à l'**absence** de convention pour les rotation angulaires en 3D. Ainsi, il y a 24 conventions possibles. Axes statiques, dynamiques, puis l'ordre dans lequel les angles est appliqué. Dans la suite, nous utiliserons principalement la convention "sxyz", axes statiques, appliquer la rotation selon l'axe X, Y puis Z.

#### Prérotation des vecteurs

La première chose à faire, afin d'être quitte du pitch et du roll, c'est de réaliser une prérotation de tous les vecteurs et points. Pour ce faire, il faut extraire les angles roll et pitch du quaternion, puis appliquer la rotation autour de X et Y

    # rotation
    roll, pitch, yaw = (10,20,17)
    # données d'origine
    p1 = (2,5,2)
    v1 = (3,0,0)
    M1 = 14.59
    p2 = (2,-5,2)
    v2 = (3,0,0)
    M2 = 11.41

    q = quaternion_from_euler(deg2radf(roll), deg2radf(pitch), deg2radf(yaw), axes="sxyz") # simulation d'un quaternion entrant
    roll, pitch, _ = euler_from_quaternion(q, axes="sxyz") # Récupération du roll & du pitch
    q = quaternion_from_euler(deg2radf(roll), deg2radf(pitch), 0, axes="sxyz") # Regénération du quaternion

    laser1 = rotate(p1, q)
    laser2 = rotate(p2, q)
    orientation1 = rotate(v1, q)
    orientation2 = rotate(v2, q)

#### Application d'une rotation autour de z en fonction de yaw

Le vecteur V résultant est : 

        [ -sin(yaw)*point.y + cos(yaw)*point.x  ]
    V = [  cos(yaw)*point.y + sin(yaw)*point.x  ]
        [  point.z                              ]

#### Extrapolation selon la mesure, en fonction de yaw

Dans ce cas-ci, la longueur du vecteur est toujours la même, il n'est pas nécessaire d'y incorporer l'angle Yaw. En effet, la rotation conserve la taille du vecteur.

    k = M / vector.length 
    C = (k*(X_v) + X_p, k*(Y_v) + Y_p, k*(Z_v) + Z_p)

#### Plans représentant le mur en fonction de yaw
Nous savons que le mur est perpendiculaire au plan XY. Sa normale est donc (-1,0,0)

    a === ax + by + cz + d = 0 => d === ax + d = 0

    a === -x + M/vector.length*(-sin(yaw)*vector.y + cos(yaw)*vector.x) + -sin(yaw)*point.y + cos(yaw)*point.x) = 0

#### Détermination de yaw

Sachant que l'on a deux lasers, nous avons deux mesures, soit deux points d'un même plan. Il suffit donc de mettre les équations ensemble pour en extraire le yaw.

    (1)  x=k*sin(yaw)*vecY+k*cos(yaw)*vecX-sin(yaw)*pointY+cos(yaw)*pointX
    (2)  x=k2*sin(yaw)*vecY2+k2*cos(yaw)*vecX2-sin(yaw)*pointY2+cos(yaw)*pointX2
    (1) - (2)  sin(yaw)=-(k*cos(yaw)*vecX+cos(yaw)*pointX-x)/(k*vecY-pointY)
    => tan(yaw) = (-k2*vecX2+k*vecX+pointX2-pointX)/(-k2*vecY2+k*vecY+pointY2-pointY)

-------
#### Meilleure visualisation d'une rotation avec Quaternion : Rotation Axe-Rotation

[Source](
http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm)

Nous représentons les 3 (ou deux) rotations qu'a fait le laser via UNE rotation autour d'un axe. Ainsi, les angles sont facile à corriger (UN SEUL COS) et pouf, on a la réponse.

### Réalisation

Afin de réaliser cette transformation, il faut normaliser le quaternion. Tout comme la normalisation d'un vecteur, il suffit de diviser chaque facteur du quaternion par sa longueur. Ainsi, la racine carrée de la somme des carrés des coéfficients du quaternions vaudra 1. Cela évitera par la suite d'avoir des problèmes pour le calcul de l'angle ou de l'axe.

    # input q : Quaternion
    # output normalized quaternion
    def normalizeQ(q):
        l = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        if(l == 1 or l == 0):
            return q
        return (q[0]/l, q[1]/l, q[2]/l, q[3]/l)

    # input q : Quaternion
    # output a tuple of a tuple (axis (x,y,z)) and  an angle
    def Quaternion2AxisAngle(q):
        q = normalizeQ(q)
        angle = 2 * acos(q[3]);
        s = sqrt(1-q[3]*q[3]);
        if(s <= 0.001):
            return (q[0], q[1], q[2]), angle
        return (q[0]/s, q[1]/s, q[2]/s), angle


    q = random_quaternion()
    print q, normalizeQ(q), Quaternion2AxisAngle(q)

 Il est aussi possible de réaliser cette transformation plus rapidement, en effet, que le vecteur soit plus long ou pas ne nous intéresse pas, c'est pour l'angle que nous devons avoir un quaternion normalisé. Ainsi, il est possible de réaliser les deux en même temps. Via la fonction ci-dessous (qui donne le même résultat), le calcul est 15% plus rapide. Sachant que cela doit être réalisé 800 à 1200 -selon le nombre de lasers- fois par seconde sur du matériel embarqué, ce n'est pas négligeable.

    # input q : Quaternion
    # output axis (x,y,z) and angle
    def fastQuaternion2AxisAngle(q):
        l = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        q3 = q[3]/l
        angle = 2 * acos(q3/l);
        s = sqrt(1-q3*q3);
        if(s <= 0.001):
            return (q[0], q[1], q[2]), angle
        return (q[0]/s, q[1]/s, q[2]/s), angle

Il est possible ensuite de réaliser la transformation via : https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle


Algorithme à six lasers
--------------------------

De la même façon que l'on a récupéré le yaw, il est possible d'obtenir le pitch et le roll en ayant 6 lasers et des équations beaucoup plus longues. Cela dit, ce n'est pas nécessaire.

TODO



