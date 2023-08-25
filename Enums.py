from enum import Enum

class ObjectType(Enum):
    NONE = 0
    BUTTON = 1
    FLOOR = 2
    WALL = 3
    FENCE = 4
    IBLOCK = 5

class MoveType(Enum):
    NO_MOVE = 0
    BACKWARD_MOVE = 1
    FORWARD_MOVE = 2

class TurnType(Enum):
    NO_TURN = 0
    LEFT_TURN = 1
    RIGHT_TURN = 2

class JumpType(Enum):
    NO_JUMP = 0
    JUMP = 1

