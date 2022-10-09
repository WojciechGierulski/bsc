import math
from rcprg_ros_utils import exitError
import PyKDL


class GripperMoves:
    @staticmethod
    def close_grippers(velma, which):
        q = [math.pi, math.pi, math.pi, 0]
        if which == 'both':
            velma.moveHandLeft(q, [1, 1, 1, 1], [9999, 9999, 9999, 9999], 9999, hold=False)
            velma.moveHandRight(q, [1, 1, 1, 1], [9999,9999,9999,9999], 9999, hold=False)
            if velma.waitForHandLeft() != 0:
                exitError(2)
            if velma.waitForHandRight() != 0:
                exitError(4)
        if which == 'right':
            velma.moveHandRight(q, [1, 1, 1, 1], [9999,9999,9999,9999], 9999, hold=False)
            if velma.waitForHandRight() != 0:
                exitError(4)
        if which == 'left':
            velma.moveHandLeft(q, [1, 1, 1, 1], [9999, 9999, 9999, 9999], 9999, hold=False)
            if velma.waitForHandLeft() != 0:
                exitError(2)

    @staticmethod
    def open_grippers(velma, which):
        q = [0, 0, 0, 0]
        if which == 'both':
            velma.moveHandLeft(q, [1, 1, 1, 1], [9999, 9999, 9999, 9999], 9999, hold=False)
            velma.moveHandRight(q, [1, 1, 1, 1], [9999,9999,9999,9999], 9999, hold=False)
            if velma.waitForHandLeft() != 0:
                exitError(2)
            if velma.waitForHandRight() != 0:
                exitError(4)
        if which == 'right':
            velma.moveHandRight(q, [1, 1, 1, 1], [9999,9999,9999,9999], 9999, hold=False)
            if velma.waitForHandRight() != 0:
                exitError(4)
        if which == 'left':
            velma.moveHandLeft(q, [1, 1, 1, 1], [9999, 9999, 9999, 9999], 9999, hold=False)
            if velma.waitForHandLeft() != 0:
                exitError(2)