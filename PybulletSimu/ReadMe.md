## Pybullet Readme



## How to use this program : 

### To test the simulation : 
    PybulletSimu/RLTests/test.py
 Moving Albert : 
  -  up key : move forward
  -  down key : move backward
  - left key : turn left
  - right key : turn right
  - space key : jump


### How to train the PPO model : 

    Launch RLTests/PPO2.py


## Environment characteristics :
### Initial conditions :

Albert's initial position : Randomized with y ∈ [1;8] et x ∈ [1;5]
Initial orientation : Randomized with z ∈ [-pi,pi] (Euler)


### Gravity : 
g = -100 m/s²

Albert's mass : m = 10 kg

### Integration Steps :
step(dt) = 1/240 s

## Movement and Raycasting (ObjetsEnvironnement/AlbertCube.py) : 

### How does the Albert's Jump function works :

#### variables:

x_factor : 
- 1 : jump forward
- 0 : jump on the spot
- -1 : jump backward

ori_jump : Albert's orientation at the beginning of the jump

new_ori : Albert's current orientation

#### jump phases :

Upward :
- beginning of the jump : impulse : $step * [500 * xFactor,0,1000]_{Albert's Referential}$ (N)
- ascension : impulse : $step * [500 * xFactor * cos(newOri - oriJump), -500 * xFactor * sin(newOri - oriJump), 1000]_{Albert's Referential}$ ( in N)

Downward :
- impulse : $step * [500 * xFactor * cos(newOri - oriJump), -500 * xFactor * sin(newOri - oriJump), -1000]_{Albert's Referential}$ ( in N )

### How does the move work : 
forward : impulse $step * [250,0,0]_{Albert's Referential}$ (in N)
backward : impulse $step * [-250,0,0]_{Albert's Referential}$ (in N)

### Raycasting characteristics : 
All 21 rays are shot in front of Albert

ray length = 10 m

#### grid vision : 
 -  yaw : 7 rays covering 70° in [-35°,35°]
 - pitch : 3 rays covering 20° in [-10°,10°]

#### uncomment the call to the show_grid function in raycasting() to visualize the raycasting




## Gym details : 

#### actions  : [turn,move,jump]
- turn : 0 = doesn't turn, 1 = turns left, 2 = turns right
- move : 0 = doesn't move, 1 = moves backward, 2 = moves forward
- jump : 0  = doesn't jump, 1 = jumps

#### observations : a 5 observation memory length, 21 rays, returning : 
    (o[0],...,o[104],d[0],...,d[104])

where :
d : distance to object : [0 to 10] 

o : type of object : [0,1,2,3,4,5] = [none,button,ground,wall,fence,Iblock]

 #### done if : 
- simulation time > 20 sec
- albert falls off the level
- albert succeeds in passing the door

#### reset : 
- random position : x ∈ [ 1, 3 ]   y ∈ [ 1, 5 ]   z = 0.75
- random orientation : θx = 0, θy = 0, θz ∈ [-π,π]

#### rewards :
- -0.05 if Albert jumps
- -0.1 if Albert has a contact with a wall,fence or iblock
- -0.5 if Albert falls off the level
- +1 if Albert achieves the maze
- +1 if Albert pushes a button


## Equations used : 

### Semi-Explicit Euler :
> - **F = m*a**
> - $(τ = I * (dω/dt) + ω * I*ω)$
> - $v_{t+Δt} = v_t + a * Δt = v_t + (F_{ext} + F_c)/m * Δt = v_t + F_{ext}/m * Δt + impulse_c/m$
> - $x_{t+Δt} = x_t + v_{t+Δt}*Δt$
>
$F_{ext}$ : gravity, wind force field, user forces ...

$F_c$ : constraint forces such as contact, friction, joints

> the p.applyExternalForce() function takes as a parameter an array[x,y,z] in Newtons (N)

 It's an impulse : (F*step)

### Friction : 

> linear damping : $F_{damping} = - v_{albert} * linearDampingCoef$
> 
> linear_damping_coef = 4 kg/s

> angular damping : $τ_{damping} = - ω_{albert} * angularDampingCoef$
> 
> angular_damping_coef = 4 kg*m^2/s

