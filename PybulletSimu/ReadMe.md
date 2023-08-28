## Caractéristiques de l'environnement



## How to use this program : 

### PybulletSimu/RLTests/test.py : 

 Moving Albert : 
  -  up key : move forward
  -  down key : move backward
  - left key : turn left
  - right key : turn right
  - space key : jump

### RLTests/PPO2.py : 

 Launches the training of the PPO model


## Environment characteristics :
### conditions initiales :
position initiale : random entre y c [1;8] et x c [1;5]
orientation initiale : z c [-pi,pi] (Euler)


### Gravité : 
g = -100 m/s²

masse m = 10 kg

### Steps :
step(dt) = 1/240s


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

### Actions,Observations...

 actions  : [turn,move,jump]
- turn : 0 = doesn't turn, 1 = turns left, 2 = turns right
- move : 0 = doesn't move, 1 = moves backward, 2 = moves forward
- jump : 0  = doesn't jump, 1 = jumps

 observations :  5 observation memory length, 21 rays, returning : 
    (o[0],...,o[104],d[0],...,d[104])

where :
d : distance to object : [0 to 10] 

o : type of object : [0,1,2,3,4,5] = [none,button,ground,wall,fence,Iblock]

 done : 
- simulation time > 20 sec
- albert falls off the level
- albert succeeds in passing the door

reset : 
- random position : x ∈ [ 1, 3 ]   y ∈ [ 1, 5 ]   z = 0.75
- random orientation : θx = 0, θy = 0, θz ∈ [-π,π]

rewards :
- -0.05 if Albert jumps
- -0.1 if Albert has a contact with a wall,fence or iblock
- -0.5 if Albert falls off the level
- +1 if Albert achieves the maze
- +1 if Albert pushes a button


### Raycasting characteristics : 
All 21 rays are shot in front of Albert

ray length = 5 m

grid vision : 
 -  yaw : 7 rays covering 70° in [-35°,35°]
 - pitch : 3 rays covering 20° in [-10°,10°]

un comment the call to the show_grid function in raycasting() to visualize the raycasting




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

