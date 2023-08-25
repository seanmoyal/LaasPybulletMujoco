import time
import numpy as np
import pybullet as p
import pybullet_data
from ObjetsEnvironnement.Cube import Cube


def euler_to_rotation_matrix(euler):
    # Convert Euler angles to rotation matrix
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    cos_r, sin_r = np.cos(roll), np.sin(roll)
    cos_p, sin_p = np.cos(pitch), np.sin(pitch)
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)

    rotation_matrix = np.array([
        [cos_y * cos_p, cos_y * sin_p * sin_r - sin_y * cos_r, cos_y * sin_p * cos_r + sin_y * sin_r],
        [sin_y * cos_p, sin_y * sin_p * sin_r + cos_y * cos_r, sin_y * sin_p * cos_r - cos_y * sin_r],
        [-sin_p, cos_p * sin_r, cos_p * cos_r]
    ])

    return rotation_matrix


# Connect to the physics server
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#  debug visualizer pour raycasting visible
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
# Set gravity
p.setGravity(0, 0, -9.8)

# Load plane
plane_id = p.loadURDF("plane.urdf")

# Load cube character
start_pos = [0, 0, 0.5]
start_orient = [0, 0, 0]
# cubeId = p.loadURDF("cube.urdf", start_pos)
Cube = Cube()
cube_id = Cube.create_cube(start_pos, orient_euler=start_orient, mass=1)
# Enable physics simulation for the cube character
p.setCollisionFilterGroupMask(cube_id, -1, 0, 0)
p.setCollisionFilterPair(plane_id, cube_id, -1, -1, 1)
# Movement variables
move_x = 0
move_z = 0
jumping = False
still_jumping = False
turning = False
count = 0
x_factor = 0
ori_jump = 0
# Friction
linear_friction = 4
angular_friction = 4
p.changeDynamics(cube_id, -1, linearDamping=linear_friction, angularDamping=angular_friction)


# Step simulation function
def step_simulation():
    global move_x, move_z, jumping, turning, count
    move()
    # Apply movement forces
    force = [move_x * 50, 0, 0]
    p.applyExternalForce(cube_id, -1, force, [0, 0, 0], p.LINK_FRAME)

    # Apply jump force
    jump(cube_id, 100)
    if turning:
        yaw_turn(cube_id, 2)
        turning = False

    # Step the simulation
    cr = raycasting(cube_id)  # C'EST CA QUI FAIT BUG LA SIMU, ENLEVER les rayons visibles PR PLUS DE FLUIDITE
    p.stepSimulation()
    reset_velocity()
    time.sleep(1 / 240)


# Keyboard event handler
def move():
    global move_x, move_z, jumping, turning, still_jumping
    keys = p.getKeyboardEvents()
    cam = p.getDebugVisualizerCamera()
    # Keys to change camera
    if keys.get(p.B3G_DOWN_ARROW):
        move_z = -5
        turning = True
    if keys.get(p.B3G_UP_ARROW):
        move_z = 5
        turning = True
    if keys.get(p.B3G_RIGHT_ARROW) and len(p.getContactPoints(cube_id)) != 0:
        move_x = 5
    if keys.get(p.B3G_LEFT_ARROW) and len(p.getContactPoints(cube_id)) != 0:
        move_x = -5
    if keys.get(p.B3G_SPACE) and len(p.getContactPoints(cube_id)) != 0:  # bon ca ca marche pas mais osef pr l'instant
        jumping = True
    if keys.get(p.B3G_SPACE):  # bon ca ca marche pas mais osef pr l'instant
        still_jumping = True


def raycasting(cube_id):
    cube_pos, ori = p.getBasePositionAndOrientation(cube_id)
    cube_ori = p.getEulerFromQuaternion(ori)
    matrice_ori = euler_to_rotation_matrix(cube_ori)
    ray_length = 5
    cube_pos_array = np.array([cube_pos for _ in range(7)])

    # départ des angles :
    dep_angles_yaw = -35 * np.pi / 180
    dep_angles_pitch = -10 * np.pi / 180
    # Pas yaw pour 70°
    step_yaw = 70 / 6
    step_yaw_rad = step_yaw * np.pi / 180

    # pas pitch pour 70°
    step_pitch = 20 / 2
    step_pitch_rad = step_pitch * np.pi / 180

    # rayVec1 : premier rayon droit devant le cube
    ray_vects = []
    for i in range(3):
        for n in range(7):
            base_ray = [np.cos(n * step_yaw_rad + dep_angles_yaw) * np.cos(i * step_pitch_rad + dep_angles_pitch),
                        np.sin(n * step_yaw_rad + dep_angles_yaw), np.sin(i * step_pitch_rad + dep_angles_pitch)]
            norm_ray = np.linalg.norm(base_ray)
            ray_vects.append(np.dot(matrice_ori, np.array([(base_ray[0] / norm_ray * ray_length + cube_pos[0]),
                                                           (ray_length * base_ray[1] / norm_ray + cube_pos[1]),
                                                           (ray_length * base_ray[2] / norm_ray + cube_pos[2])])))

    # tracé du rayon / faudra changer avec rayTestBatch() quand on aura plus de rayons
    contact_results = []
    for n in range(21):
        contact_results.append(p.rayTest(cube_pos, ray_vects[n]))

    # dans les resultats [0] : hitObjectId // [3] hit position
    # faudra changer les coordonnées de globales à locales

    for n in range(21):
        if contact_results[n][0][0] != -1:
            p.addUserDebugLine(cube_pos, contact_results[n][0][3], [0, 1, 0], 2, 0.05)
        else:
            p.addUserDebugLine(cube_pos, ray_vects[n], [1, 0, 0], 2, 0.05)
    return contact_results


def reset_velocity():
    global move_x, move_z
    move_x = 0
    move_z = 0


def jump(cube_id, i):
    global count, jumping, still_jumping, move_x, x_factor, ori_jump
    new_ori = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(cube_id)[1])[2]
    print(new_ori)
    print(ori_jump)
    if jumping:
        x_factor = move_x
        ori_jump = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(cube_id)[1])[2]
        impulse = [100 * x_factor, 0, i]
        p.applyExternalForce(cube_id, -1, impulse, [0, 0, 0], p.LINK_FRAME)
        count += 1
        jumping = False
    elif not jumping and still_jumping:
        if 1 <= count <= 100:
            impulse = [100 * x_factor * np.cos(new_ori - ori_jump), -100 * x_factor * np.sin(new_ori - ori_jump), i]
            p.applyExternalForce(cube_id, -1, impulse, [0, 0, 0], p.LINK_FRAME)
            count += 1
        elif len(p.getContactPoints(cube_id)) == 0:
            impulse = [100 * x_factor * np.cos(new_ori - ori_jump), -100 * x_factor * np.sin(new_ori - ori_jump), -i]
            p.applyExternalForce(cube_id, -1, impulse, [0, 0, 0], p.LINK_FRAME)
        elif len(p.getContactPoints(cube_id)) != 0:
            x_factor = 0
            still_jumping = False
            count = 0


def yaw_turn(cube_id, z):
    global move_z
    angular_velocity = [0, 0, z * move_z]
    p.resetBaseVelocity(cube_id, angularVelocity=angular_velocity)


# Register keyboard event handler


# Run the simulation
while True:
    step_simulation()

# Disconnect from the physics server
p.disconnect()
