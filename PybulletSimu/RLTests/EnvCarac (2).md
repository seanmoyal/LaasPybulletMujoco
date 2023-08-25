# Environment Caracs : 

## Env State : 
    (x,y,z,d,p1,...,pn)

with (x,y,z) cube's position

d : boolean : door opened(1)/closed(0)

pi : boolean : button pressed(1)/intact(0)

## Actions :

    ( Rot : (notMove,RotLeft, RotRight), Move : (notMove,Backwards,Forward), Jump : (don't jump, jump) )

Not sure for JumpForward

## Observation : 
    (Cell1, ..., Cell21)

where :

    Celli : (d,o)

d : distance to object [0 to 10] 

o : type of object [0,1,2,3,4,5] = [empty,button,ground,wall,fence,Iblock]