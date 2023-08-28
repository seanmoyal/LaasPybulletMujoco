## Caractéristiques de l'environnement


## How to use this program : 

### xmlDirectory/Actor.xml
It's the MJCF file describing Albert

### How To load any wanted level :
    - go to XmlConversionDirectory/CreateMjcfRooms.py
    - choose which name you save the file with by changing save_mjcf_file_name
    - choose which level you want to save as a mjcf file by changing the i param in add_room_by_number()
### To test the simulation : 
    Launch MujocoSimu2/RLTests/test.py

 Moving Albert : 
  -  up key : move forward
  -  down key : move backward
  - left key : turn left
  - right key : turn right
  - space key : jump

### How to train the PPO model : 

    Launch RLTests/PPO2.py

### Loads the trained model into the simulation : 
    Launch RLTests/TestModel.py
 
 
## Environment characteristics :

### Initial conditions :
Albert's initial position :  Randomized with y ∈ [1;5] et x ∈ [1;3]
Initial orientation : Randomized with z ∈ [-pi,pi] (Euler)

### Gravity : 
g = -10 m/s²

Albert's mass : m = 10 kg

### Integration Steps : 
step(dt) = 0.001. s


## Movement and Raycasting (ObjetsEnvironnement/AlbertCube.py) : 

### How does the Albert's Jump function works :

#### variables:
x_jumping_factor : 
- 1 : saut vers l'avant
- 0 : saut sur place
- -1 : saut vers l'arrière

jump force : i = 13000

#### jump equation
 Beginning of the jump : impulse $step * [5 * xJumpingFactor,0,i]_{Albert's Referential}$ (in N)

### How does the move work :
forward : impulse $step * [250,0,0]_{Albert's Referential}$ (in N)
backward : impulse $step * [-250,0,0]_{Albert's Referential}$ (in N)


### Raycasting characteristics : 
All 21 rays are shot in front of Albert

ray length = 10m

#### grid vision : 
 -  yaw : 7 rays covering 70° in [-35°,35°]
 - pitch : 3 rays covering 15° in [-12°,+3°]

#### uncomment the call to the show_grid function in raycasting() to visualize the raycasting


## Gym details : 

#### actions  : [turn,move,jump]
- turn : 0 = doesn't turn, 1 = turns left, 2 = turns right
- move : 0 = doesn't move, 1 = moves backward, 2 = moves forward
- jump : 0  = doesn't jump, 1 = jumps

#### observations :  5 observation memory length, 21 rays, returning : 
    (o[0],...,o[630],d[0],...,d[104])

where :
d : distance to object : [0 to 10] 

o[i:i+6] : type of object : [x,x,x,x,x,x] = [none,button,ground,wall,fence,Iblock]

x=1 for the right type of object and x=0 elsewhere

#### done if : 
- simulation time > 20 sec
- albert falls off the level
- albert succeeds in passing the door

#### reset : 
- random position : x ∈ [ 1, 5 ]   y ∈ [ 1, 8 ]   z = 0.75
- random orientation : θx = 0, θy = 0, θz ∈ [-π,π]

##### rewards :
- -0.05 if Albert jumps
- -0.05 if Albert doesn't move
- -0.1 if Albert has a contact with a wall,fence or iblock
- -0.5 if Albert falls off the level
- +0.03 for each ray that perceives a button
- +2 if Albert achieves the maze
- +1+(1-time_spent_in_simu/time_episode) if Albert pushes a button ( the faster he gets to it the more reward he gets )


## Equations used : 

### Forward Dynamic equation :
> - $$M(dv/dt) + c = τ  $$
> - M : joint space inertia matrix, c = bias forces,τ = applied force
> - $v_{t+Δt} = v_t + a_t * Δt = v_t + (F_{ext} + F_c)/m * Δt = v_t + F_{ext}/m * Δt + impulse_c/m$
> - $x_{t+Δt} = x_t + v_{t+Δt}*Δt$
>
$F_{ext}$ : gravity, wind force field, user forces ...

$F_c$ : constraint forces such as contact, friction, joints

> the data.xfrx_applied[id] value has as value an array[x,y,z] in Newtons (in N)

It's an impulse (F*step)

### Friction : 

in xmlDirectory/Actor.xml :
damping = 1
diaginertia = [0.001 0.001 0.001]