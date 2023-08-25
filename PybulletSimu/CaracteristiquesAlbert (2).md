## Caractéristiques de l'environnement


## Caractéristiques Cube :

### conditions initiales :
position initiale : random entre i c [1;5] et j c [1;3]
orientation initiale : z c [-pi,pi] (Euler)


### Gravité : 
g = -100 m.s^-1

masse m = 1 

### Step (dt) : 
step = 1/240. s (secondes)


### Comment marche le jump :
x_factor : 
- 1 : saut vers l'avant
- 0 : saut sur place
- -1 : saut vers l'arrière

ori_jump : orientation d'albert au début du saut
new_ori : orientation actuelle d'albert

Ascendant :
- départ de saut : impulse $step * [500 * x_factor,0,1000]_{référentiel d'Albert}$ (N)
- ascension : impulse $step * [500 * x_factor * cos(new_ori - ori_jump), -500 * x_factor * sin(new_ori - ori_jump), 1000]_{référentiel d'Albert}$ (N)

Descendant : impulse $step * [500 * x_factor * cos(new_ori - ori_jump), -500 * x_factor * sin(new_ori - ori_jump), -1000]_{référentiel d'Albert}$ (N)

### Comment marche le move : 
avancée : impulse $step * [250,0,0]_{référentiel d'Albert}$ (N)
reculée : impulse $step * [-250,0,0]_{référentiel d'Albert}$ (N)

### RayCasting : 
3 rangées ( haut bas milieu ) pour un angle total de 20°

> Chaque rangée : 7 rayons : angle total 70°


## Equations utilisées : 

### Semi-Explicit Euler :
> - **F = m*a**
> - $$(τ = I * (dω/dt) + ω * I*ω)$$
> - $v_{t+Δt} = v_t + a * Δt = v_t + (F_{ext} + F_c)/m * Δt = v_t + F_{ext}/m * Δt + impulse_c/m$
> - $x_{t+Δt} = x_t + v_{t+Δt}*Δt$
>
$F_{ext}$ : gravity, wind force field, user forces ...

$F_c$ : constraint forces such as contact, friction, joints

> La fonction p.applyExternalForce() prend en parmètres un array[x,y,z] en Newtons (N)
> c'est une impulsion : (F*step)

### Friction : 

> linear damping : $F_{damping} = - v_{albert} * linear_damping_coef$
> linear_damping_coef = 4 kg/s

> angular damping : $τ_{damping} = - ω_{albert} * angular_damping_coef$
> angular_damping_coef = 4 kg*m^2/s

