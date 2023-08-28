## Caractéristiques de l'environnement


## How to use this program : 

### xmlDirectory/Actor.xml
mjcf file for Albert

### How To load a wanted level :
- go to XmlConversionDirectory/CreateMjcfRooms.py
- choose which name you save the file with by changing save_mjcf_file_name
- choose which level you want to save as a mjcf file by changing the i param in add_room_by_number()
### MujocoSimu2/RLTests/test.py : 
To test the simulation :

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

### conditions initiales :
position initiale : random entre y c [1;5] et x c [1;3]
orientation initiale : z c [-pi,pi] (Euler)


### Gravité : 
g = -10 m/s²

masse m = 10 kg

### Step (dt) : 
step = 0.001. s (secondes)

### Comment marche le jump :
x_jumping_factor : 
- 1 : saut vers l'avant
- 0 : saut sur place
- -1 : saut vers l'arrière

 départ de saut : impulse $step * [5 * x_jumping_factor,0,i]_{référentiel d'Albert}$ (N)

i=13000
### Comment marche le move : 
avancée : impulse $step * [250,0,0]_{référentiel d'Albert}$ (N)
reculée : impulse $step * [-250,0,0]_{référentiel d'Albert}$ (N)




### Actions,Observations...
 actions  : [turn,move,jump]
- turn : 0 = doesn't turn, 1 = turns left, 2 = turns right
- move : 0 = doesn't move, 1 = moves backward, 2 = moves forward
- jump : 0  = doesn't jump, 1 = jumps

 observations :  5 observation memory length, 21 rays, returning : 
    (o[0],...,o[630],d[0],...,d[104])

where :
d : distance to object : [0 to 10] 

o[i:i+6] : type of object : [x,x,x,x,x,x] = [none,button,ground,wall,fence,Iblock]
x=1 for the right type of object and x=0 elsewhere

 done : 
- simulation time > 20 sec
- albert falls off the level
- albert succeeds in passing the door

reset : 
- random position : x ∈ [ 1, 5 ]   y ∈ [ 1, 8 ]   z = 0.75
- random orientation : θx = 0, θy = 0, θz ∈ [-π,π]

rewards :
- -0.05 if Albert jumps
- -0.05 if Albert doesn't move
- -0.1 if Albert has a contact with a wall,fence or iblock
- -0.5 if Albert falls off the level
- +0.03 for each ray that perceives a button
- +2 if Albert achieves the maze
- +1+(1-time_spent_in_simu/time_episode) if Albert pushes a button ( the faster he gets to it the more reward he gets )




### Raycasting characteristics : 
All 21 rays are shot in front of Albert

ray length = 10m

grid vision : 
 -  yaw : 7 rays covering 70° in [-35°,35°]
 - pitch : 3 rays covering 15° in [-12°,+3°]

un comment the call to the show_grid function in raycasting() to visualize the raycasting

## Equations utilisées : 

### Forward Dynamic equation :
> - $$M(dv/dt) + c = τ  $$
> - M : joint space inertia matrix, c = bias forces,τ = applied force
> - $v_{t+Δt} = v_t + a_t * Δt = v_t + (F_{ext} + F_c)/m * Δt = v_t + F_{ext}/m * Δt + impulse_c/m$
> - $x_{t+Δt} = x_t + v_{t+Δt}*Δt$
>
$F_{ext}$ : gravity, wind force field, user forces ...

$F_c$ : constraint forces such as contact, friction, joints

> La valeur data.xfrx_applied[id] prend en parmètres un array[x,y,z] en Newtons (N)
> c'est une impulsion : (F*step)

### Friction : 

in xmlDirectory/Actor.xml :
damping = 1
diaginertia = [0.001 0.001 0.001]