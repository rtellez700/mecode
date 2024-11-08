import numpy as np
from mecode import G
import warnings


class GMatrix(G):
    """This class passes points through a 2D transformation matrix before
    fowarding them to the G class.  A 2D transformation matrix was
    choosen over a 3D transformation matrix because GCode's ARC
    command cannot be arbitrary rotated in a 3 dimensions.

    This lets you write code like:

    def box(g, height, width):
        g.move(0, width)
        g.move(height, 0)
        g.move(0, -width)
        g.move(-height, 0)

    def boxes(g, height, width):
        g.push_matrix()
        box(g, height, width)
        g.rotate(math.pi/8)
        box(g, height, width)
        g.pop_matrix()

    To get two boxes at a 45 degree angle from each other.

    The 2D transformation matrices are arranged in a stack,
    similar to OpenGL.

    numpy is required.

    """

    def __init__(self, *args, **kwargs):
        super(GMatrix, self).__init__(*args, **kwargs)
        # self._matrix_setup()
        self.stack = [np.identity(3)]
        # self.position_savepoints = []

    def push_matrix(self):
        # Push a copy of the current matrix onto the stack
        self.stack.append(self.stack[-1].copy())

    def pop_matrix(self):
        # Pop the top matrix off the stack
        if len(self.stack) > 1:
            self.stack.pop()
        else:
            self.stack = [np.identity(3)]
            warnings.warn(
                "Cannot pop all items from stack. Setting stack to default identity matrix. To save transforms to stack, call g.push_matrix() before applying transformation."
            )
            # raise IndexError("Cannot pop from an empty matrix stack")

    def apply_transform(self, transform):
        # Apply a transformation matrix to the current matrix
        transormed_matrix = self.stack[-1] @ transform

        # get machine epsilon
        epsilon = np.finfo(transormed_matrix.dtype).eps

        # round values smaller than machine epsilon to zero
        self.stack[-1] = np.where(
            np.abs(transormed_matrix) < epsilon, 0, transormed_matrix
        )

    def get_current_matrix(self):
        # Get the current matrix (top of the stack)
        return self.stack[-1]

    def translate(self, x, y):
        # Create a translation matrix and apply it
        translation_matrix = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
        self.apply_transform(translation_matrix)

    def rotate(self, angle):
        # Create a rotation matrix for the angle
        c = np.cos(angle)
        s = np.sin(angle)
        rotation_matrix = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        self.apply_transform(rotation_matrix)

    def scale(self, sx, sy=None):
        if sy is None:
            sy = sx

        # Create a scaling matrix and apply it
        scaling_matrix = np.array([[sx, 0, 0], [0, sy, 0], [0, 0, 1]])
        self.apply_transform(scaling_matrix)

    def abs_move(self, x=None, y=None, z=None, **kwargs):
        # if x is None or y is None or z is None:
        #     raise ValueError('x, y, and z must be provided when using the GMatrix class.')

        if x is None:
            x = self.current_position["x"]
        if y is None:
            y = self.current_position["y"]
        if z is None:
            z = self.current_position["z"]

        # abs_move ends up invoking move, which means that
        # we don't need to do a matrix transform here.
        # NOTE: this also ends up calling `move` below instead of the parent class since method is overriden below
        super(GMatrix, self).abs_move(x, y, z, **kwargs)

    def move(self, x=None, y=None, z=None, **kwargs):
        x_p, y_p, z_p = self._transform_point(x, y, z)

        # x_p = np.round(x_p, self.output_digits)
        # y_p = np.round(y_p, self.output_digits)
        # z_p = np.round(z_p, self.output_digits)
        # z = np.round(z, self.output_digits)

        # x_p = 0 if x_p == 0 else x_p
        # y_p = 0 if y_p == 0 else y_p
        # z_p = 0 if z_p == 0 else z_p
        # z = 0 if z == 0 else z

        # NOTE: untransformed z is being used here. If support for 3D transformations is added, this should be updated
        super(GMatrix, self).move(x_p, y_p, z, **kwargs)

    def _transform_point(self, x, y, z):
        current_matrix = self.get_current_matrix()

        if x is None:
            x = 0
        if y is None:
            y = 0
        if z is None:
            z = 0

        return current_matrix @ np.array([x, y, z])

    # @property
    # def current_position(self):
    #     # x = self._current_position['x']
    #     # y = self._current_position['y']
    #     # z = self._current_position['z']

    #     # Ensure x and y are not None; default to 0.0
    #     # if x is None: x = 0.0
    #     # if y is None: y = 0.0

    #     # Get the latest matrix from the stack
    #     current_matrix = self.get_current_matrix()
    #     inverse_matrix = np.linalg.inv(current_matrix)

    #     # TODO: INVERSE OR CURRENT_MATRIX ???
    #     # x, y, z = current_matrix @ np.array([x, y, z])
    #     x_p, y_p, _ = current_matrix @ np.array([
    #         self._current_position['x'],
    #         self._current_position['y'],
    #         self._current_position['z']
    #     ])

    #     transformed_position = {**self._current_position}
    #     print('>>current position ', self._current_position)
    #     transformed_position.update({'x': x_p, 'y': y_p, 'z': self._current_position['z']})
    #     # return {'x': x, 'y': y, 'z': z_p}
    #     return transformed_position
