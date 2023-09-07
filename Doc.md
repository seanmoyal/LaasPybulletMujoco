# <ins>Project Presentation<ins/>
This project is based on the following video : https://www.youtube.com/watch?v=v3UBlEJDXR0,
please watch it before starting to work on the project.

### Recapitulation : 
In this project we work with an agent named Albert : It is a cube that can move forward, backward, jump and rotate around itself.
Albert is in a room containing various objects, including barriers and buttons.
The buttons allow, once all pressed, to open the door and allow Albert to exit and therefore solve the level.

![Alt text](./images/albert.png )

### The goal of the project : 
recreate the Reinforcement Learning Environments seen in the video in 3 different physics simulators (Pybullet, Mujoco, isaac Gym)

## Summary : 
#### I - What's an RL Environment

#### II - How To test the different environments

#### III - A presentation of the project's structure

#### IV - Specificities of each Simulator


## <ins>I - What's a RL Environment ?<ins/>

### 1 - Reinfocement Learning Problem
Reinforcement Learning is a type of machine learning paradigm where an agent
learns to make sequences of decisions by interacting with an environment. 

Some key elements of RL are the following : 

    - The Agent : it's the actor who's going to interact with the environment. The agent's goal is to learn a policy (a strategy) that maximizes the expected cumulative reward.
    - The Environment : The external system with which the agent interacts
    - The State : The environment's private representation
    - The Action : The set of possible choices or decisions that the agent can make.
    - The Policy : The environment's private representation
    - The Reward : A scalar value provided by the environment after each action is taken. It quantifies the immediate desirability or quality of the action. The agent's goal is to maximize the expected cumulative reward.

In our project we use PPO ( Proximal Policy Optimization ), an algorithm used to train the agent to learn the policy that maximizes the cumulative reward.

Check https://medium.com/mlearning-ai/ppo-intuitive-guide-to-state-of-the-art-reinforcement-learning-410a41cb675b for a first overview of PPO.

### 2 - The Environment 
![Alt text](./images/rl_environnement.png )
####  <ins>Interaction Loop</ins>

Here's a basic representation of how the environment works : 

At each step t the agent :
 - executes an action A_t

 - receives an observation O_t
 - receives a reward R_t

the environment :
 - receives action A_t
 - emits observation 0_{t+1}
 - emits scalar reward R_{t+1}


>The environment state S_t is the environment's private representation ( it's the data used to pick the next observation and reward )
>
>The Observation O_t is the environment's information that de agent perceives
>
>The Reward is calculated based on the State
> 
>The Action is chosen based on the Observation

## <ins>II - How To test the different environments<ins/>

### 1 - Installation requirements

#### Pybullet
> pip install pybullet
> 
> pip install numpy
> 
> pip install gym
> 
> pip install torchvision
>
> pip install stable-baselines3


#### Mujoco
> pip install mujoco
> 
> pip install numpy
> 
> pip install scipy
> 
> pip install gym
> 
> pip install torchvision
> 
> pip install stable-baselines3
