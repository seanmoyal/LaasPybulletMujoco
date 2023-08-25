## Caractéristiques de l'environnement


## Caractéristiques Cube :

### conditions initiales :
position initiale : random entre y c [1;5] et x c [1;3]
orientation initiale : z c [-pi,pi] (Euler)


### Gravité : 
g = -10 m/s

masse m = 10 kg

### Step (dt) : 
step = 0.001. s (secondes)


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



## How to use this program : 

### MujocoSimu2/RLTests/test.py : 

 Moving Albert : 
  -  up key : move forward
  -  down key : move backward
  - left key : turn left
  - right key : turn right
  - space key : jump

### RLTests/PPO2.py : 

 Launches the training of the PPO model

### RLTests/TestModel.py : 

 Loads the trained model into the simulation
 
## Environment characteristics :
 dt = 1/1000

 actions  : [turn,move,jump]

 observations : s

 done : 
- simulation time > 20 sec
- albert falls off the level
- albert succeeds in passing the door

reset : 
- random position : x ∈ [ 1, 5 ]   y ∈ [ 1, 8 ]   z = 0.75
- random orientation : θx = 0, θy = 0, θz ∈ [-π,π]

