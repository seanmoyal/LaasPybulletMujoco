## Caractéristiques de l'environnement

Friction : LinearDampling/angularDampling = 4

Gravité : (0,0,-9.8)





## Caractéristiques Cube :

### conditions initiales :
position initiale : [0,0,0]
orientation initiale : [0,0,0] (Euler)



masse m = 1 

force avancée = 250/step

### Comment marche le jump :

ascendant : 100/step selon x, 100/step selon z

descendant : 100/step selon x, -100/step selon z

### RayCasting : 
3 rangées ( haut bas milieu ) pour un angle total de 20°

Chaque rangée : 7 rayons : angle total 70°
