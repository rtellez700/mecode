import math
import os
import sys
import numpy as np
import copy
from collections import defaultdict
import warnings
import matplotlib.colors as mcolors

HERE = os.path.dirname(os.path.abspath(__file__))

# for python 2/3 compatibility
try:
    isinstance("", basestring)

    def is_str(s):
        return isinstance(s, basestring)

    def encode2To3(s):
        return s

    def decode2To3(s):
        return s

except NameError:

    def is_str(s):
        return isinstance(s, str)

    def encode2To3(s):
        return bytes(s, 'UTF-8')

    def decode2To3(s):
        return s.decode('UTF-8')

DEFAULT_FILAMENT_COLOR = (30/255, 144/255, 255/255)

class G(object):

    def __init__(self,
                    outfile=None,
                    print_lines=True,
                    header=None,
                    footer=None,
                    aerotech_include=True,
                    output_digits=6,
                    direct_write=False,
                    direct_write_mode='socket',
                    printer_host='localhost',
                    printer_port=8000,
                    baudrate=250000,
                    two_way_comm=True,
                    x_axis='X',
                    y_axis='Y',
                    z_axis='Z',
                    extrude=False,
                    filament_diameter=1.75,
                    layer_height=0.19,
                    extrusion_width=0.35,
                    extrusion_multiplier=1,
                    setup=True,
                    lineend='os'):
        """
        Parameters
        ----------
        outfile : path or None (default: None)
            If a path is specified, the compiled gcode will be writen to that
            file.
        print_lines : bool (default: True)
            Whether or not to print the compiled GCode to stdout

        Other Parameters
        ----------------
        header : path or None (default: None)
            Optional path to a file containing lines to be written at the
            beginning of the output file
        footer : path or None (default: None)
            Optional path to a file containing lines to be written at the end
            of the output file.
        aerotech_include : bool (default: True)
            If true, add aerotech specific functions and var defs to outfile.
        output_digits : int (default: 6)
            How many digits to include after the decimal in the output gcode.
        direct_write : bool (default: False)
            If True a socket or serial port is opened to the printer and the
            GCode is sent directly over.
        direct_write_mode : str (either 'socket' or 'serial') (default: socket)
            Specify the channel your printer communicates over, only used if
            `direct_write` is True.
        printer_host : str (default: 'localhost')
            Hostname of the printer, only used if `direct_write` is True.
        printer_port : int (default: 8000)
            Port of the printer, only used if `direct_write` is True.
        baudrate: int (default: 250000)
            The baudrate to connect to the printer with.
        two_way_comm : bool (default: True)
            If True, mecode waits for a response after every line of GCode is
            sent over the socket. The response is returned by the `write`
            method. Only applies if `direct_write` is True.
        x_axis : str (default 'X')
            The name of the x axis (used in the exported gcode)
        y_axis : str (default 'Y')
            The name of the z axis (used in the exported gcode)
        z_axis : str (default 'Z')
            The name of the z axis (used in the exported gcode)
        extrude : True or False (default: False)
            If True, a flow calculation will be done in the move command. The
            neccesary length of filament to be pushed through on a move command
            will be tagged on as a kwarg. ex. X5 Y5 E3
        filament_diameter: float (default 1.75)
            the diameter of FDM filament you are using
        layer_height : float
            Layer height for FDM printing. Only relavent when extrude = True.
        extrusion width: float
            total width of the capsule shaped cross section of a squashed filament.
        extrusion_multiplier: float (default = 1)
            The length of extrusion filament to be pushed through on a move
            command will be multiplied by this number before being applied.
        setup : Bool (default: True)
            Whether or not to automatically call the setup function.
        lineend : str (default: 'os')
            Line ending to use when writing to a file or printer. The special
            value 'os' can be passed to fall back on python's automatic
            lineending insertion.

        """
        self.outfile = outfile
        self.print_lines = print_lines
        self.header = header
        self.footer = footer
        self.aerotech_include = aerotech_include
        self.output_digits = output_digits
        self.direct_write = direct_write
        self.direct_write_mode = direct_write_mode
        self.printer_host = printer_host
        self.printer_port = printer_port
        self.baudrate = baudrate
        self.two_way_comm = two_way_comm
        self.x_axis = x_axis
        self.y_axis = y_axis
        self.z_axis = z_axis

        self.extrude = extrude
        self.filament_diameter = filament_diameter
        self.layer_height = layer_height
        self.extrusion_width = extrusion_width
        self.extrusion_multiplier = extrusion_multiplier

        self.history = [{
            'REL_MODE': True,
            'ACCEL' : 2500,
            'DECEL' : 2500,
            # 'P' : PRESSURE,
            # 'P_COM_PORT': P_COM_PORT,
            'PRINTING': {}, #{'Call togglePress': {'printing': False, 'value': 0}},
            'PRINT_SPEED': 0,
            'COORDS': (0,0,0),
            'ORIGIN': (0,0,0),
            'CURRENT_POSITION': {'X': 0, 'Y': 0, 'Z': 0},
            # 'VARIABLES': VARIABLES
            'COLOR': None
        }]

        self._current_position = defaultdict(float)
        self.is_relative = True
        self.position_history = [(0, 0, 0)]
        self.color_history = [DEFAULT_FILAMENT_COLOR]
        self.speed = 0
        self.speed_history = []
        self.extruding = [None, False, 0] # source, if_printing, printing_value
        self.extruding_history = []
        self.extrusion_state = {}#defaultdict()

        self.print_time = 0
        self.version = None

        self._socket = None
        self._p = None

        # If the user passes in a line ending then we need to open the output
        # file in binary mode, otherwise python will try to be smart and
        # convert line endings in a platform dependent way.
        if lineend == 'os':
            mode = 'w+'
            self.lineend = '\n'
        else:
            mode = 'wb+'
            self.lineend = lineend

        if is_str(outfile):
            self.out_fd = open(outfile, mode)
        elif outfile is not None:  # if outfile not str assume it is an open file
            self.out_fd = outfile
        else:
            self.out_fd = None

        if setup:
            self.setup()

        self._check_latest_version()

    @property
    def current_position(self):
        return self._current_position

    def _check_latest_version(self):
        import re, requests
        from packaging import version

        def read_version_from_setup():
            try:
                import pkg_resources  # part of setuptools

                version = pkg_resources.require("mecode")[0].version

                return version
            except:
                return None

        def read_version_from_github(username, repo, path='setup.py'):
            # GitHub raw content URL
            raw_url = f'https://raw.githubusercontent.com/{username}/{repo}/main/{path}'

            try:
                # Make a GET request to the raw content URL
                response = requests.get(raw_url)
                response.raise_for_status()  # Raise an exception for HTTP errors

                # Use regular expression to find the version string
                version_match = re.search(r"'version': ['\"]([^'\"]*)['\"]", response.text)

                if version_match:
                    version = version_match.group(1)
                    return version
                else:
                    print("Version not found in remote setup.py.")
                    return None

            except requests.exceptions.RequestException as e:
                print(f"Error: {e}")
                return None

        github_username = 'rtellez700'
        github_repo = 'mecode'

        remote_package_version = read_version_from_github(github_username, github_repo)

        local_package_version = read_version_from_setup()

        if local_package_version and 'unittest' not in sys.modules.keys():
            self.version = local_package_version
            print(f"\nRunning mecode v{local_package_version}")

        # confirm that a version is already installed first
        if 'unittest' not in sys.modules.keys():
            if local_package_version is not None and remote_package_version is not None:
                if version.parse(local_package_version) < version.parse(remote_package_version):
                    print("A new mecode version is available. To upgrade to the latest version run:\n\t>>> pip install git+https://github.com/rtellez700/mecode.git --upgrade")

    def __enter__(self):
        """
        Context manager entry
        Can use like:

        with mecode.G(  outfile=self.outfile,
                        print_lines=False,
                        aerotech_include=False) as g:
        """
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Context manager exit
        """
        self.teardown()

    # GCode Aliases  ########################################################

    def set_home(self, x=0, y=0, z=0, **kwargs):
        """ Set the current position to the given position without moving.

        Examples
        --------

        set the current position to X=0, Y=0
        >>> g.set_home(0, 0)

        """
        args = self._format_args(x, y, z, **kwargs)
        self.write('G92 ' + args)

        new_origin = (self.history[-1]['CURRENT_POSITION']['X'] + x,
                      self.history[-1]['CURRENT_POSITION']['Y'] + y,
                      self.history[-1]['CURRENT_POSITION']['Z'] + z)

        self.history[-1]['ORIGIN'] = new_origin

    def reset_home(self):
        """ Reset the position back to machine coordinates without moving.
        """
        # FIXME This does not work with internal current_position
        # FIXME You must call an abs_move after this to re-sync
        # current_position
        self.write('G92.1')

    def relative(self):
        """ Enter relative movement mode, in general this method should not be
        used, most methods handle it automatically.

        """
        if not self.is_relative:
            self.write('G91')
            self.is_relative = True

    def absolute(self):
        """ Enter absolute movement mode, in general this method should not be
        used, most methods handle it automatically.

        """
        if self.is_relative:
            self.write('G90')
            self.is_relative = False

    def feed(self, rate):
        """ Set the feed rate (tool head speed) in mm/s

        Parameters
        ----------
        rate : float
            The speed to move the tool head in mm/s.

        """
        self.write('G1 F{}'.format(rate))
        self.speed = rate

    def dwell(self, time):
        """ Pause code executions for the given amount of time.

        Parameters
        ----------
        time : float
            Time in milliseconds to pause code execution.

        """
        self.write('G4 P{}'.format(time))

    # Composed Functions  #####################################################

    def setup(self):
        """ Set the environment into a consistent state to start off. This
        method must be called before any other commands.

        """
        self._write_header()
        if self.is_relative:
            self.write('G91')
        else:
            self.write('G90')

    def teardown(self, wait=True):
        """ Close the outfile file after writing the footer if opened. This
        method must be called once after all commands.

        Parameters
        ----------
        wait : Bool (default: True)
            Only used if direct_write_model == 'serial'. If True, this method
            waits to return until all buffered lines have been acknowledged.

        """
        if self.out_fd is not None:
            if self.aerotech_include is True:
                with open(os.path.join(HERE, 'footer.txt')) as fd:
                    self._write_out(lines=fd.readlines())
            if self.footer is not None:
                with open(self.footer) as fd:
                    self._write_out(lines=fd.readlines())
            self.out_fd.close()
        if self._socket is not None:
            self._socket.close()
        if self._p is not None:
            self._p.disconnect(wait)

        # do not calculate print time during unittests
        if 'unittest' not in sys.modules.keys():
            self.calc_print_time()

    def home(self):
        """ Move the tool head to the home position (X=0, Y=0).
        """
        self.abs_move(x=0, y=0)

    def move_inc(self, disp=None, speed=None, axis=None, accel=None, decel=None):
        ''' Typically used to move linear actuator incrementally. Operates in
        relative mode.

        disp : float
            amount to displace `axis`. Negative values can be used for retraction
        speed : float
            Speed to move `axis` at
        accel : float
            If provided, will set the acceleration of `axis`
            TODO: NOT CURRENTLY SUPPORTED
        decel : float
            If provided, will set the deceleration of `axis`
            TODO: NOT CURRENTLY SUPPORTED
        '''
        # self.extrude = True
        # if accel is not None:

        self.write(f'MOVEINC {axis} {disp:.6f} {speed:.6f}')
        # self.extrude = False

    def move(self, x=None, y=None, z=None, rapid=False, color=DEFAULT_FILAMENT_COLOR, comment='', **kwargs):
        """ Move the tool head to the given position. This method operates in
        relative mode unless a manual call to [absolute][mecode.main.G.absolute] was given previously.
        If an absolute movement is desired, the [abs_move][mecode.main.G.abs_move] method is
        recommended instead.

        points : floats
            Must specify endpoint as kwargs, e.g. x=5, y=5
        rapid : Bool (default: False)
            Executes an uncoordinated move to the specified location.
        color : hex string or rgb(a) string
            Specifies a color to be added to color history for viewing.
        comment : str (default: '')
            Adds a comment to the end of the line.

        Examples
        --------
        >>> # move the tool head 10 mm in x and 10 mm in y
        >>> g.move(x=10, y=10)
        >>> # the x, y, and z keywords may be omitted:
        >>> g.move(10, 10, 10)

        >>> # move the A axis up 20 mm
        >>> g.move(A=20)

        """

        if self.speed == 0:
            msg = 'WARNING! no print speed has been set. Will default to previously used print speed.'
            self.write('; ' + msg)

            warnings.warn('''
                            >>> No print speed has been specified
                            e.g., to set print speed to 15 mm/s use:
                            \t\t g.feed(15)

                            If this is not the intended behavior please set a print speed. You can ignore this if your testing out features such as testing serial communication etc.
                            ''')

        if self.extrude is True and 'E' not in kwargs.keys():
            if self.is_relative is not True:
                x_move = self.current_position['x'] if x is None else x
                y_move = self.current_position['y'] if y is None else y
                x_distance = abs(x_move - self.current_position['x'])
                y_distance = abs(y_move - self.current_position['y'])
                current_extruder_position = self.current_position['E']
            else:
                x_distance = 0 if x is None else x
                y_distance = 0 if y is None else y
                current_extruder_position = 0
            line_length = math.sqrt(x_distance**2 + y_distance**2)
            area = self.layer_height*(self.extrusion_width-self.layer_height) + \
                3.14159*(self.layer_height/2)**2
            volume = line_length*area
            filament_length = ((4*volume)/(3.14149*self.filament_diameter**2))*self.extrusion_multiplier
            kwargs['E'] = filament_length + current_extruder_position

        self._update_current_position(x=x, y=y, z=z, color=color, **kwargs)
        self._update_print_time(x,y,z)
        # new_state = self.history[-1].copy()
        # new_state['COORDS'] = (x, y, z)
        # new_state['CURRENT_POSITION'] = {'X': self._current_position['x'], 'Y': self._current_position['y'], 'Z': self._current_position['z']}
        # self.history.append(new_state)
        args = self._format_args(x, y, z, **kwargs)
        cmd = 'G0 ' if rapid else 'G1 '
        self.write(cmd + args + f'; {comment}')

    def abs_move(self, x=None, y=None, z=None, rapid=False, **kwargs):
        """ Same as [move][mecode.main.G.move] method, but positions are interpreted as absolute.
        """
        if self.is_relative:
            self.absolute()
            self.move(x=x, y=y, z=z, rapid=rapid, **kwargs)
            self.relative()
        else:
            self.move(x=x, y=y, z=z, rapid=rapid, **kwargs)

    def rapid(self, x=None, y=None, z=None, **kwargs):
        """ Executes an uncoordinated move to the specified location.
        """
        self.move(x, y, z, rapid=True, **kwargs)

    def abs_rapid(self, x=None, y=None, z=None, **kwargs):
        """ Executes an uncoordinated abs move to the specified location.
        """
        self.abs_move(x, y, z, rapid=True, **kwargs)

    def retract(self, retraction):
        if self.extrude is False:
            self.move(E = -retraction)
        else:
            self.extrude = False
            self.move(E = -retraction)
            self.extrude = True

    def circle(self, radius, center=None,  direction='CW', linearize=True, **kwargs):
        """ Generates a circle starting from the current position if center is None,
        otherwise from center.

        Parameters
        ----------
        direction : str (either 'CW' or 'CCW') (default: 'CW')
            The direction to execute the arc in.
        radius : float
            The radius of the circle.
        center : (float, float)
            The center coordinates of the circle
        linearize : Bool (default: True)
            Represent the arc of the circle as a series of straight lines.

        Examples
        --------
        TODO: updates these
        >>> # arc 10 mm up in y and 10 mm over in x with a radius of 20.
        >>> g.arc(x=10, y=10, radius=20)

        >>> # move 10 mm up on the A axis, arcing through y with a radius of 20
        >>> g.arc(A=10, y=0, radius=20)

        >>> # arc through x and y while moving linearly on axis A
        >>> g.arc(x=10, y=10, radius=50, helix_dim='A', helix_len=5)

        """
        if direction == 'CW':
            self.arc(x=radius, y=radius, radius=radius, direction='CW', linearize=linearize, **kwargs)
            self.arc(x=radius, y=-radius, radius=radius, direction='CW', linearize=linearize, **kwargs)
            self.arc(x=-radius, y=-radius, radius=radius, direction='CW', linearize=linearize, **kwargs)
            self.arc(x=-radius, y=radius, radius=radius, direction='CW', linearize=linearize, **kwargs)
        elif direction == 'CCW':
            self.arc(x=-radius, y=radius, radius=radius, direction='CCW', linearize=linearize, **kwargs)
            self.arc(x=-radius, y=-radius, radius=radius, direction='CCW', linearize=linearize, **kwargs)
            self.arc(x=radius, y=-radius, radius=radius, direction='CCW', linearize=linearize, **kwargs)
            self.arc(x=radius, y=radius, radius=radius, direction='CCW', linearize=linearize, **kwargs)

    def arc(self, x=None, y=None, z=None, direction='CW', radius='auto',
            helix_dim=None, helix_len=0, linearize=True, color=(0,1,0,0.5), **kwargs):
        """ Arc to the given point with the given radius and in the given
        direction. If helix_dim and helix_len are specified then the tool head
        will also perform a linear movement through the given dimension while
        completing the arc. Note: Helix and flow calculation do not currently
        work with linearize.

        Parameters
        ----------
        direction : str (either 'CW' or 'CCW') (default: 'CW')
            The direction to execute the arc in.
        radius : 'auto' or float (default: 'auto')
            The radius of the arc. A negative value will select the longer of
            the two possible arc segments. If auto is selected the radius will
            be set to half the linear distance to desired point.
        helix_dim : str or None (default: None)
            The linear dimension to complete the helix through
        helix_len : float
            The length to move in the linear helix dimension.
        linearize : Bool (default: True)
            Represent the arc as a series of straight lines.

        Examples
        --------
        >>> # arc 10 mm up in y and 10 mm over in x with a radius of 20.
        >>> g.arc(x=10, y=10, radius=20)

        >>> # move 10 mm up on the A axis, arcing through y with a radius of 20
        >>> g.arc(A=10, y=0, radius=20)

        >>> # arc through x and y while moving linearly on axis A
        >>> g.arc(x=10, y=10, radius=50, helix_dim='A', helix_len=5)

        """
        dims = dict(kwargs)
        if x is not None:
            dims['x'] = x
        if y is not None:
            dims['y'] = y
        if z is not None:
            dims['z'] = z
        msg = 'Must specify two of x, y, or z.'
        if len(dims) != 2:
            raise RuntimeError(msg)
        dimensions = [k.lower() for k in dims.keys()]
        if 'x' in dimensions and 'y' in dimensions:
            plane_selector = 'G17'  # XY plane
            axis = helix_dim
        elif 'x' in dimensions:
            plane_selector = 'G18'  # XZ plane
            dimensions.remove('x')
            axis = dimensions[0].upper()
        elif 'y' in dimensions:
            plane_selector = 'G19'  # YZ plane
            dimensions.remove('y')
            axis = dimensions[0].upper()
        else:
            raise RuntimeError(msg)
        if self.z_axis != 'Z':
            axis = self.z_axis

        if direction == 'CW':
            command = 'G2'
        elif direction == 'CCW':
            command = 'G3'

        values = [v for v in dims.values()]
        if self.is_relative:
            dist = math.sqrt(values[0] ** 2 + values[1] ** 2)
            if radius == 'auto':
                radius = dist / 2.0
            elif abs(radius) < dist / 2.0:
                msg = 'Radius {} to small for distance {}'.format(radius, dist)
                raise RuntimeError(msg)
            vect_dir= [values[0]/dist,values[1]/dist]
            if direction == 'CW':
                arc_rotation_matrix = np.array([[0, -1],[1, 0]])
            elif direction =='CCW':
                arc_rotation_matrix = np.array([[0, 1],[-1, 0]])
            perp_vect_dir = np.array(vect_dir)*arc_rotation_matrix
            a_vect= np.array([values[0]/2,values[1]/2])
            b_length = math.sqrt(radius**2-(dist/2)**2)
            b_vect = b_length*perp_vect_dir
            c_vect = a_vect+b_vect
            # center_coords = c_vect
            final_pos = a_vect*2-c_vect
            initial_pos = -c_vect
        else:
            k = [ky for ky in dims.keys()]
            cp = self._current_position
            dist = math.sqrt(
                (cp[k[0]] - values[0]) ** 2 + (cp[k[1]] - values[1]) ** 2
            )

            if radius == 'auto':
                radius = dist / 2.0
            elif abs(radius) < dist / 2.0:
                msg = 'Radius {} to small for distance {}'.format(radius, dist)
                raise RuntimeError(msg)

            vect_dir= [(values[0]-cp[k[0]])/dist,(values[1]-cp[k[1]])/dist]
            if direction == 'CW':
                arc_rotation_matrix = np.array([[0, -1],[1, 0]])
            elif direction =='CCW':
                arc_rotation_matrix = np.array([[0, 1],[-1, 0]])
            perp_vect_dir = np.array(vect_dir)*arc_rotation_matrix
            a_vect = np.array([(values[0]-cp[k[0]])/2.0,(values[1]-cp[k[1]])/2.0])
            b_length = math.sqrt(radius**2-(dist/2)**2)
            b_vect = b_length*perp_vect_dir
            c_vect = a_vect+b_vect
            # center_coords = np.array(cp[k[:2]])+c_vect

            final_pos = np.array([cp[k] for k in k[:2]])+a_vect*2-c_vect
            initial_pos = np.array([cp[k] for k in k[:2]])

            # final_pos = np.array(cp[k[:2]])+a_vect*2-c_vect
            # initial_pos = np.array(cp[k[:2]])

        #extrude feature implementation
        # only designed for flow calculations in x-y plane
        if self.extrude is True:
            area = self.layer_height*(self.extrusion_width-self.layer_height) + 3.14159*(self.layer_height/2)**2
            if self.is_relative is not True:
                current_extruder_position = self.current_position['E']
            else:
                current_extruder_position = 0

            circle_circumference = 2*3.14159*abs(radius)

            arc_angle = ((2*math.asin(dist/(2*abs(radius))))/(2*3.14159))*360
            shortest_arc_length = (arc_angle/180)*3.14159*abs(radius)
            if radius > 0:
                arc_length = shortest_arc_length
            else:
                arc_length = circle_circumference - shortest_arc_length
            volume = arc_length*area
            filament_length = ((4*volume)/(3.14149*self.filament_diameter**2))*self.extrusion_multiplier
            dims['E'] = filament_length + current_extruder_position

        if linearize:
            #Curved formed from straight lines
            final_pos = np.array(final_pos.tolist()).flatten()
            initial_pos = np.array(initial_pos.tolist()).flatten()
            final_angle = np.arctan2(final_pos[1],final_pos[0])
            initial_angle = np.arctan2(initial_pos[1],initial_pos[0])

            if direction == 'CW':
                angle_difference = 2*np.pi-(final_angle-initial_angle)%(2*np.pi)
            elif direction == 'CCW':
                angle_difference = (initial_angle-final_angle)%(-2*np.pi)

            step_range = [0, angle_difference]
            step_size = np.pi/16
            angle_step = np.arange(step_range[0],step_range[1]+np.sign(angle_difference)*step_size,np.sign(angle_difference)*step_size)

            segments = []
            for angle in angle_step:
                radius_vect = -c_vect
                radius_rotation_matrix = np.array([[math.cos(angle), -math.sin(angle)],
                                 [math.sin(angle), math.cos(angle)]])
                int_point = radius_vect*radius_rotation_matrix
                segments.append(int_point)

            for i in range(len(segments)-1):
                move_line = segments[i+1]-segments[i]
                self.move(*move_line.tolist()[0], color=color)
        else:
            #Standard output
            if axis is not None:
                self.write('G16 X Y {}'.format(axis))  # coordinate axis assignment
            self.write(plane_selector)
            args = self._format_args(**dims)
            if helix_dim is None:
                self.write('{0} {1} R{2:.{digits}f}'.format(command, args, radius,
                                                            digits=self.output_digits))
            else:
                self.write('{0} {1} R{2:.{digits}f} G1 {3}{4}'.format(
                    command, args, radius, helix_dim.upper(), helix_len, digits=self.output_digits))
                dims[helix_dim] = helix_len

            self._update_current_position(**dims)

    def abs_arc(self, direction='CW', radius='auto', **kwargs):
        """ Same as [arc][mecode.main.G.arc] method, but positions are interpreted as absolute.
        """
        if self.is_relative:
            self.absolute()
            self.arc(direction=direction, radius=radius, **kwargs)
            self.relative()
        else:
            self.arc(direction=direction, radius=radius, **kwargs)

    def rect(self, x, y, direction='CW', start='LL'):
        """ Trace a rectangle with the given width and height.

        Parameters
        ----------
        x : float
            The width of the rectangle in the x dimension.
        y : float
            The height of the rectangle in the y dimension.
        direction : str (either 'CW' or 'CCW') (default: 'CW')
            Which direction to complete the rectangle in.
        start : str (either 'LL', 'UL', 'LR', 'UR') (default: 'LL')
            The start of the rectangle -  L/U = lower/upper, L/R = left/right
            This assumes an origin in the lower left.

        Examples
        --------
        >>> # trace a 10x10 clockwise square, starting in the lower left corner
        >>> g.rect(10, 10)

        >>> # 1x5 counterclockwise rect starting in the upper right corner
        >>> g.rect(1, 5, direction='CCW', start='UR')

        """
        if direction == 'CW':
            if start.upper() == 'LL':
                self.move(y=y)
                self.move(x=x)
                self.move(y=-y)
                self.move(x=-x)
            elif start.upper() == 'UL':
                self.move(x=x)
                self.move(y=-y)
                self.move(x=-x)
                self.move(y=y)
            elif start.upper() == 'UR':
                self.move(y=-y)
                self.move(x=-x)
                self.move(y=y)
                self.move(x=x)
            elif start.upper() == 'LR':
                self.move(x=-x)
                self.move(y=y)
                self.move(x=x)
                self.move(y=-y)
        elif direction == 'CCW':
            if start.upper() == 'LL':
                self.move(x=x)
                self.move(y=y)
                self.move(x=-x)
                self.move(y=-y)
            elif start.upper() == 'UL':
                self.move(y=-y)
                self.move(x=x)
                self.move(y=y)
                self.move(x=-x)
            elif start.upper() == 'UR':
                self.move(x=-x)
                self.move(y=-y)
                self.move(x=x)
                self.move(y=y)
            elif start.upper() == 'LR':
                self.move(y=y)
                self.move(x=-x)
                self.move(y=-y)
                self.move(x=x)

    def round_rect(self, x, y, direction='CW', start='LL', radius=0, linearize=True):
        """ Trace a rectangle with the given width and height with rounded corners,
            note that starting point is not actually in corner of rectangle.

        Parameters
        ----------
        x : float
            The width of the rectangle in the x dimension.
        y : float
            The height of the rectangle in the y dimension.
        direction : str (either 'CW' or 'CCW') (default: 'CW')
            Which direction to complete the rectangle in.
        start : str (either 'LL', 'UL', 'LR', 'UR') (default: 'LL')
            The start of the rectangle -  L/U = lower/upper, L/R = left/right
            This assumes an origin in the lower left.
        radius : radius of the corners of the rectangle

        Examples
        --------
        >>> # trace a 10x10 clockwise square with radius of 3, starting in the lower left corner
        >>> g.round_rect(10, 10, radius=3)

        >>> # 1x5 counterclockwise rect with radius of 2 starting in the upper right corner
        >>> g.round_rect(1, 5, direction='CCW', start='UR', radius=2)

                                    ______________
                                   /              \
                                  /                \
        starts here for 'UL' - > |                  | <- starts here for 'UR'
                                 |                  |
        starts here for 'LL' - > |                  | <- starts here for 'LR'
                                  \                /
                                   \______________/

        """
        if direction == 'CW':
            if start.upper() == 'LL':
                self.move(y=y-2*radius)
                self.arc(x=radius,y=radius,direction='CW',radius=radius, linearize=linearize)
                self.move(x=x-2*radius)
                self.arc(x=radius,y=-radius,direction='CW',radius=radius, linearize=linearize)
                self.move(y=-(y-2*radius))
                self.arc(x=-radius,y=-radius,direction='CW',radius=radius, linearize=linearize)
                self.move(x=-(x-2*radius))
                self.arc(x=-radius,y=radius,direction='CW',radius=radius, linearize=linearize)
            elif start.upper() == 'UL':
                self.arc(x=radius,y=radius,direction='CW',radius=radius, linearize=linearize)
                self.move(x=x-2*radius)
                self.arc(x=radius,y=-radius,direction='CW',radius=radius, linearize=linearize)
                self.move(y=-(y-2*radius))
                self.arc(x=-radius,y=-radius,direction='CW',radius=radius, linearize=linearize)
                self.move(x=-(x-2*radius))
                self.arc(x=-radius,y=radius,direction='CW',radius=radius, linearize=linearize)
                self.move(y=y-2*radius)
            elif start.upper() == 'UR':
                self.move(y=-(y-2*radius))
                self.arc(x=-radius,y=-radius,direction='CW',radius=radius, linearize=linearize)
                self.move(x=-(x-2*radius))
                self.arc(x=-radius,y=radius,direction='CW',radius=radius, linearize=linearize)
                self.move(y=y-2*radius)
                self.arc(x=radius,y=radius,direction='CW',radius=radius, linearize=linearize)
                self.move(x=x-2*radius)
                self.arc(x=radius,y=-radius,direction='CW',radius=radius, linearize=linearize)
            elif start.upper() == 'LR':
                self.arc(x=-radius,y=-radius,direction='CW',radius=radius, linearize=linearize)
                self.move(x=-(x-2*radius))
                self.arc(x=-radius,y=radius,direction='CW',radius=radius, linearize=linearize)
                self.move(y=y-2*radius)
                self.arc(x=radius,y=radius,direction='CW',radius=radius, linearize=linearize)
                self.move(x=x-2*radius)
                self.arc(x=radius,y=-radius,direction='CW',radius=radius, linearize=linearize)
                self.move(y=-(y-2*radius))
        elif direction == 'CCW':
            if start.upper() == 'LL':
                self.arc(x=radius,y=-radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(x=x-2*radius)
                self.arc(x=radius,y=radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(y=y-2*radius)
                self.arc(x=-radius,y=radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(x=-(x-2*radius))
                self.arc(x=-radius,y=-radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(y=-(y-2*radius))
            elif start.upper() == 'UL':
                self.move(y=-(y-2*radius))
                self.arc(x=radius,y=-radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(x=x-2*radius)
                self.arc(x=radius,y=radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(y=y-2*radius)
                self.arc(x=-radius,y=radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(x=-(x-2*radius))
                self.arc(x=-radius,y=-radius,direction='CCW',radius=radius, linearize=linearize)
            elif start.upper() == 'UR':
                self.arc(x=-radius,y=radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(x=-(x-2*radius))
                self.arc(x=-radius,y=-radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(y=-(y-2*radius))
                self.arc(x=radius,y=-radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(x=x-2*radius)
                self.arc(x=radius,y=radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(y=y-2*radius)
            elif start.upper() == 'LR':
                self.move(y=y-2*radius)
                self.arc(x=-radius,y=radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(x=-(x-2*radius))
                self.arc(x=-radius,y=-radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(y=-(y-2*radius))
                self.arc(x=radius,y=-radius,direction='CCW',radius=radius, linearize=linearize)
                self.move(x=x-2*radius)
                self.arc(x=radius,y=radius,direction='CCW',radius=radius, linearize=linearize)

    def meander(self, x, y, spacing, start='LL', orientation='x', tail=False,
                minor_feed=None, color=(0,0,0,0.5), mode='auto'):
        """ Infill a rectangle with a square wave meandering pattern. If the
        relevant dimension is not a multiple of the spacing, the spacing will
        be tweaked to ensure the dimensions work out.

        Parameters
        ----------
        x : float
            The width of the rectangle in the x dimension.
        y : float
            The height of the rectangle in the y dimension.
        spacing : float
            The space between parallel meander lines.
        start : str (either 'LL', 'UL', 'LR', 'UR') (default: 'LL')
            The start of the meander -  L/U = lower/upper, L/R = left/right
            This assumes an origin in the lower left.
        orientation : str ('x' or 'y') (default: 'x')
        tail : Bool (default: False)
            Whether or not to terminate the meander in the minor axis
        minor_feed : float or None (default: None)
            Feed rate to use in the minor axis
        color : hex string or rgb(a) string
            Specifies a color to be added to color history for viewing.
        mode : str (either 'auto' or 'manual')
            If set to auto (default value) will auto correct spacing to fit within x and y dimensions.

        Examples
        --------
        >>> # meander through a 10x10 square with a spacing of 1mm starting in
        >>> # the lower left.
        >>> g.meander(10, 10, 1)

        >>> # 3x5 meander with a spacing of 1 and with parallel lines through y
        >>> g.meander(3, 5, spacing=1, orientation='y')

        >>> # 10x5 meander with a spacing of 2 starting in the upper right.
        >>> g.meander(10, 5, 2, start='UR')

        """
        if start.upper() == 'UL':
            x, y = x, -y
        elif start.upper() == 'UR':
            x, y = -x, -y
        elif start.upper() == 'LR':
            x, y = -x, y

        # Major axis is the parallel lines, minor axis is the jog.
        if orientation == 'x':
            major, major_name = x, 'x'
            minor, minor_name = y, 'y'
        else:
            major, major_name = y, 'y'
            minor, minor_name = x, 'x'

        if mode.lower() == 'auto':
            actual_spacing = self._meander_spacing(minor, spacing)
            if abs(actual_spacing) != spacing:
                msg = ';WARNING! meander spacing updated from {} to {}'
                self.write(msg.format(spacing, actual_spacing))
                self.write(f";\t IF YOU INTENDED TO USE A SPACING OF {spacing:.4f} USE mode='manual'")
            spacing = actual_spacing
        sign = 1

        was_absolute = True
        if not self.is_relative:
            self.relative()
        else:
            was_absolute = False

        major_feed = self.speed
        if not minor_feed:
            minor_feed = self.speed

        n_passes = int(self._meander_passes(minor, spacing))

        for j in range(n_passes):
            self.move(**{major_name: (sign * major), 'color': color})
            if minor_feed != major_feed:
                self.feed(minor_feed)
            if (j < n_passes-1):
                self.move(**{minor_name: spacing, 'color': color})
            if (j==n_passes-1) and ( tail==True ):
                self.move(**{minor_name: spacing, 'color': color})

            if minor_feed != major_feed:
                self.feed(major_feed)
            sign = -1 * sign

        if was_absolute:
            self.absolute()

    def serpentine(self, L, n_lines, spacing, start='LL', orientation='x', color=(0,0,0,0.5)):
        """ Generate a square wave meandering/serpentine pattern. Unlike [meander][mecode.main.G.meander],
         will not tweak spacing dimension.

        Parameters
        ----------
        L : float
            Major axis dimension.
        n_lines : int
            The number of lines to generate
        spacing : float
            The space between parallel serpentine paths.
        start : str (either 'LL', 'UL', 'LR', 'UR') (default: 'LL')
            The start of the meander -  L/U = lower/upper, L/R = left/right
            This assumes an origin in the lower left.
        orientation : str ('x' or 'y') (default: 'x')
        color : hex string or rgb(a) string
            Specifies a color to be added to color history for viewing.

        Examples
        --------
        >>> # meander through a 10x10 square with a spacing of 1mm starting in
        >>> # the lower left.
        >>> g.meander(10, 10, 1)

        >>> # 3x5 meander with a spacing of 1 and with parallel lines through y
        >>> g.meander(3, 5, spacing=1, orientation='y')

        >>> # 10x5 meander with a spacing of 2 starting in the upper right.
        >>> g.meander(10, 5, 2, start='UR')

        """
        if orientation.lower() == 'x':
            major, major_name = L, 'x'
            minor, minor_name = spacing, 'y'
        else:
            major, major_name = L, 'y'
            minor, minor_name = spacing, 'x'

        sign_minor = +1
        sign_major = +1
        if start.upper() == 'UL':
            sign_major = +1 if orientation.lower()=='x' else -1
            sign_minor = -1 if orientation.lower()=='x' else +1
        elif start.upper() == 'UR':
            sign_major = -1
            sign_minor = -1
        elif start.upper() == 'LR':
            sign_major = -1 if orientation.lower()=='x' else +1
            sign_minor = +1 if orientation.lower()=='x' else -1

        was_absolute = True
        if not self.is_relative:
            self.relative()
        else:
            was_absolute = False

        for j in range(n_lines):
            self.move(**{major_name: sign_major*major, 'color': color})

            if j < (n_lines-1):
                self.move(**{minor_name: sign_minor*minor, 'color': color})

            sign_major = -1*sign_major

        if was_absolute:
            self.absolute()

    def clip(self, axis='z', direction='+x', height=4, linearize=False):
        """ Move the given axis up to the given height while arcing in the
        given direction.

        Parameters
        ----------
        axis : str (default: 'z')
            The axis to move, e.g. 'z'
        direction : str (either +-x or +-y) (default: '+x')
            The direction to arc through
        height : float (default: 4)
            The height to end up at

        Examples
        --------
        >>> # move 'z' axis up 4mm while arcing through positive x
        >>> g.clip()

        >>> # move 'A' axis up 10mm while arcing through negative y
        >>> g.clip('A', height=10, direction='-y')

        """
        secondary_axis = direction[1]
        if height > 0:
            orientation = 'CW' if direction[0] == '-' else 'CCW'
        else:
            orientation = 'CCW' if direction[0] == '-' else 'CW'
        radius = abs(height / 2.0)
        kwargs = {
            secondary_axis: 0,
            axis: height,
            'direction': orientation,
            'radius': radius,
            'linearize': linearize
        }
        self.arc(**kwargs)

    def triangular_wave(self, x, y, cycles, start='UR', orientation='x'):
        """ Perform a triangular wave.

        Parameters
        ----------
        x : float
            The length to move in x in one half cycle
        y : float
            The length to move in y in one half cycle
        start : str (either 'LL', 'UL', 'LR', 'UR') (default: 'UR')
            The start of the zigzag direction.
            This assumes an origin in the lower left, and move toward upper
            right.
        orientation : str ('x' or 'y') (default: 'x')

        Examples
        --------
        >>> # triangular wave for one cycle going 10 in x and 10 in y per half
        >>> # cycle.
        >>> # the lower left.
        >>> g.zigzag(10, 10, 1)

        >>> # triangular wave 4 cycles, going 3 in x and 5 in y per half cycle,
        >>> # oscillating along the y axis.
        >>> g.zigzag(3, 5, 4, orientation='y')

        >>> # triangular wave 2 cycles, going 10 in x and 5 in y per half cycle,
        >>> # oscillating along the x axis making the first half cycle towards
        >>> # the lower left corner of the movement area.
        >>> g.zigzag(10, 5, 2, start='LL')

        """
        if start.upper() == 'UL':
            x, y = -x, y
        elif start.upper() == 'LL':
            x, y = -x, -y
        elif start.upper() == 'LR':
            x, y = x, -y

        # Major axis is the parallel lines, minor axis is the jog.
        if orientation == 'x':
            major, major_name = x, 'x'
            minor, minor_name = y, 'y'
        else:
            major, major_name = y, 'y'
            minor, minor_name = x, 'x'

        sign = 1

        was_absolute = True
        if not self.is_relative:
            self.relative()
        else:
            was_absolute = False

        for _ in range(int(cycles*2)):
            self.move(**{minor_name: (sign * minor), major_name: major})
            sign = -1 * sign

        if was_absolute:
            self.absolute()

    def rect_spiral(self, n_turns, spacing, start='center', origin=(0,0), dwell=None, manual=False, **kwargs):
        """ Performs a square spiral.

        Parameters
        ----------
        n_turns : int
            The number of spirals
        spacing : float or iterable
            The spacing between lines of the spiral. Spacing can be a tuple or list to specify (dx, dy) spacings.
        start : str (either 'center', 'edge')
            The location to start the spiral (default: 'center').
        direction : str (either 'CW', 'CCW') #TODO: not being used right now
            Direction to print the spiral, either clockwise or counterclockwise. (default: 'CW')
        origin : tuple
            Absolute coordinates of spiral center. Helpful when printing in absolute coordinates

        Examples
        --------

        >>> # TODO


        """
        was_absolute = True
        if not self.is_relative:
            self.relative()
        else:
            was_absolute = False

        # d_F = spacing

        if hasattr(spacing, '__iter__'):
            dx = spacing[0]
            dy = spacing[1]
        else:
            dx = dy = spacing

        x_pts = [origin[0], dx]
        y_pts = [origin[1], 0]

        if hasattr(n_turns, '__iter__'):
            turn_0 = n_turns[0]
            turn_F = n_turns[1]
        else:
            turn_0 = 1
            turn_F = n_turns

        for j in range(1, turn_F + 1):
            top_right = (dx*j, dy*j)
            top_left = (-dx*j, dy*j)
            bottom_left = (-dx*j, -dy*j)
            bottom_right = (dx*j + dx, -dy*j)

            x_pts.extend([top_right[0], top_left[0], bottom_left[0], bottom_right[0]])
            y_pts.extend([top_right[1], top_left[1], bottom_left[1], bottom_right[1]])

        x_pts = np.array(x_pts)
        y_pts = np.array(y_pts)
        # adjust last point to ensure spiral is a square
        # TODO: if want adjustable spiral orientation / direction, will need to adjust this
        x_pts[-1] -= dx

        original_pts = (x_pts, y_pts)

        if turn_0 > 1:
            x_pts = x_pts[4*(turn_0-1)::]
            y_pts = y_pts[4*(turn_0-1)::]

        if start == 'edge':
            x_pts = x_pts[::-1]
            y_pts = y_pts[::-1]

        if self.is_relative:
            x_pts = x_pts[1:] - x_pts[:-1]
            y_pts = y_pts[1:] - y_pts[:-1]

        if not manual:
            for x_j, y_j in zip(x_pts, y_pts):
                self.move(x_j, y_j, **kwargs)

                if dwell is not None:
                    self.dwell(dwell)

        if was_absolute:
            self.absolute()

        if manual:
            return x_pts, y_pts, original_pts

    def square_spiral(self, n_turns, spacing, start='center', origin=(0,0), dwell=None, manual=False, **kwargs):
        """ Performs a square spiral.

        Parameters
        ----------
        n_turns : int
            The number of spirals
        spacing : float
            The spacing between lines of the spiral.
        start : str (either 'center', 'edge')
            The location to start the spiral (default: 'center').
        direction : str (either 'CW', 'CCW') #TODO: not being used right now
            Direction to print the spiral, either clockwise or counterclockwise. (default: 'CW')
        origin : tuple
            Absolute coordinates of spiral center. Helpful when printing in absolute coordinates

        Examples
        --------

        >>> # TODO


        """
        was_absolute = True
        if not self.is_relative:
            self.relative()
        else:
            was_absolute = False

        d_F = spacing

        x_pts = [origin[0], d_F]
        y_pts = [origin[1], 0]

        if hasattr(n_turns, '__iter__'):
            turn_0 = n_turns[0]
            turn_F = n_turns[1]
        else:
            turn_0 = 1
            turn_F = n_turns

        for j in range(1, turn_F + 1):
            top_right = (d_F*j, d_F*j)
            top_left = (-d_F*j, d_F*j)
            bottom_left = (-d_F*j, -d_F*j)
            bottom_right = (d_F*j + d_F, -d_F*j)

            x_pts.extend([top_right[0], top_left[0], bottom_left[0], bottom_right[0]])
            y_pts.extend([top_right[1], top_left[1], bottom_left[1], bottom_right[1]])

        x_pts = np.array(x_pts)
        y_pts = np.array(y_pts)
        # adjust last point to ensure spiral is a square
        # TODO: if want adjustable spiral orientation / direction, will need to adjust this
        x_pts[-1] -= d_F

        original_pts = (x_pts, y_pts)

        if turn_0 > 1:
            x_pts = x_pts[4*(turn_0-1)::]
            y_pts = y_pts[4*(turn_0-1)::]

        if start == 'edge':
            x_pts = x_pts[::-1]
            y_pts = y_pts[::-1]

        if self.is_relative:
            x_pts = x_pts[1:] - x_pts[:-1]
            y_pts = y_pts[1:] - y_pts[:-1]

        if not manual:
            for x_j, y_j in zip(x_pts, y_pts):
                self.move(x_j, y_j, **kwargs)

                if dwell is not None:
                    self.dwell(dwell)

        if was_absolute:
            self.absolute()

        if manual:
            return x_pts, y_pts, original_pts


    def spiral(self, end_diameter, spacing, feedrate, start='center', direction='CW',
                step_angle = 0.1, start_diameter = 0, center_position=None):
        """ Performs an Archimedean spiral. Start by moving to the center of the spiral location
        then use the 'start' argument to specify a starting location (either center or edge).

        Parameters
        ----------
        end_diameter : float
            The outer diameter of the spiral.
        spacing : float
            The spacing between lines of the spiral.
        feedrate : float
            Feedrate is the speed of the nozzle relative to the substrate
        start : str (either 'center', 'edge')
            The location to start the spiral (default: 'center').
        direction : str (either 'CW', 'CCW')
            Direction to print the spiral, either clockwise or counterclockwise. (default: 'CW')
        step_angle : float
            Resolution of the spiral in radians, smaller is higher resolution (default: 0.1).
        start_diameter : float
            The inner diameter of the spiral (default: 0).
        center_position : list
            Position of the absolute center of the spiral, useful when starting a spiral at the edge of a completed spiral

        Examples
        --------

        >>> # start first spiral, outer diameter of 20, spacing of 1, feedrate of 8
        >>> g.spiral(20,1,8)

        >>> # move to second spiral location and do similar spiral but start at edge
        >>> g.spiral(20,1,8,start='edge',center_position=[50,0])

        >>> # move to third spiral location, this time starting at edge but printing CCW
        >>> g.spiral(20,1,8,start='edge',direction='CCW',center_position=[50,50])

        >>> # move to fourth spiral location, starting at center again but printing CCW
        >>> g.spiral(20,1,8,direction='CCW',center_position=[0,50])

        """
        start_spiral_turns = (start_diameter/2.0)/spacing
        end_spiral_turns = (end_diameter/2.0)/spacing

        #Use current position as center position if none is specified
        if center_position is None:
            center_position = [self._current_position['x'],self._current_position['y']]

        #Keep track of whether currently in relative or absolute mode
        was_relative = True
        if self.is_relative:
            self.absolute()
        else:
            was_relative = False

        # SEE: https://www.comsol.com/blogs/how-to-build-a-parameterized-archimedean-spiral-geometry/
        b = spacing/(2*math.pi)
        t = np.arange(start_spiral_turns*2*math.pi, end_spiral_turns*2*math.pi, step_angle)

        #Add last final point to ensure correct outer diameter
        t = np.append(t,end_spiral_turns*2*math.pi)
        if start == 'center':
            pass
        elif start == 'edge':
            t = t[::-1]
        else:
            raise Exception("Must either choose 'center' or 'edge' for starting position.")

        #Move to starting positon
        if (direction == 'CW' and start == 'center') or (direction == 'CCW' and start == 'edge'):
            x_move = -t[0]*b*math.cos(t[0])+center_position[0]
        elif (direction == 'CCW' and start == 'center') or (direction == 'CW' and start == 'edge'):
            x_move = t[0]*b*math.cos(t[0])+center_position[0]
        else:
            raise Exception("Must either choose 'CW' or 'CCW' for spiral direction.")
        y_move = t[0]*b*math.sin(t[0])+center_position[1]
        self.move(x_move, y_move)

        #Start writing moves
        self.feed(feedrate)

        for step in t[1:]:
            if (direction == 'CW' and start == 'center') or (direction == 'CCW' and start == 'edge'):
                x_move = -step*b*math.cos(step)+center_position[0]
            elif (direction == 'CCW' and start == 'center') or (direction == 'CW' and start == 'edge'):
                x_move = step*b*math.cos(step)+center_position[0]
            else:
                raise Exception("Must either choose 'CW' or 'CCW' for spiral direction.")
            y_move = step*b*math.sin(step)+center_position[1]
            self.move(x_move, y_move)

        #Set back to relative mode if it was previsously before command was called
        if was_relative:
                self.relative()

    def gradient_spiral(self, end_diameter, spacing, gradient, feedrate, flowrate,
                start='center', direction='CW', step_angle = 0.1, start_diameter = 0,
                center_position=None, dead_delay=0):
        """ Identical motion to the regular spiral function, but with the control of two syringe pumps to enable control over
            dielectric properties over the course of the spiral. Starting with simply hitting certain dielectric constants at
            different values along the radius of the spiral.

        Parameters
        ----------
        end_diameter : float
            The outer diameter of the spiral.
        spacing : float
            The spacing between lines of the spiral.
        gradient : str
            Functioning defining the ink concentration along the radius of the spiral
        feedrate : float
            Feedrate is the speed of the nozzle relative to the substrate
        flowrate : float
            Flowrate is a measure of the amount of ink dispensed per second by the syringe pump
        start : str (either 'center', 'edge')
            The location to start the spiral (default: 'center').
        direction : str (either 'CW', 'CCW')
            Direction to print the spiral, either clockwise or counterclockwise. (default: 'CW')
        step_angle : float
            Resolution of the spiral in radians, smaller is higher resolution (default: 0.1).
        start_diameter : float
            The inner diameter of the spiral (default: 0).
        center_position : list
            Position of the absolute center of the spiral, useful when starting a spiral at the edge of a completed spiral
        dead_delay : float
            Printing composition offset caused by the dead volume of the nozzle which creates a delayed
            effect between the syringe pumps and the actual composition of the ink exiting the nozzle.

        Examples
        --------
        >>> g.gradient_spiral(start_diameter=7.62, #mm
        ...     end_diameter=30.48, #mm
        ...     spacing=1, #mm
        ...     feedrate=8, #mm/s
        ...     flowrate=2/60.0, #rot/s
        ...     start='edge', #'edge' or 'center'
        ...     direction='CW', #'CW' or 'CCW'
        ...     gradient="-0.322*r**2 - 6.976*r + 131.892") #Any function
        """

        import sympy as sy

        def calculate_extrusion_values(radius, length, feed = feedrate, flow = flowrate, formula = gradient, delay = dead_delay, spacing = spacing, start = start, outer_radius = end_diameter/2.0, inner_radius=start_diameter/2.0):
            """Calculates the extrusion values for syringe pumps A & B during a move along the print path.
            """

            def exact_length(r0,r1,h):
                """Calculates the exact length of an archimedean given the spacing, inner and outer radii.
                SEE: http://www.giangrandi.ch/soft/spiral/spiral.shtml

                Parameters
                ----------
                r0 : float
                    The inner diameter of the spiral.
                r1 : float
                    The outer diameter of the spiral.
                h  : float
                    The spacing of the spiral.
                """
                #t0 & t1 are the respective diameters in terms of radians along the spiral.
                t0 = 2*math.pi*r0/h
                t1 = 2*math.pi*r1/h
                return h/(2.0*math.pi)*(t1/2.0*math.sqrt(t1**2+1)+1/2.0*math.log(t1+math.sqrt(t1**2+1))-t0/2.0*math.sqrt(t0**2+1)-1/2.0*math.log(t0+math.sqrt(t0**2+1)))


            def exact_radius(r_0,h,L):
                """Calculates the exact outer radius of an archimedean given the spacing, inner radius and the length.
                SEE: http://www.giangrandi.ch/soft/spiral/spiral.shtml

                Parameters
                ----------
                r0 : float
                    The inner radius of the spiral.
                h  : float
                    The spacing of the spiral.
                L  : float
                    The length of the spiral.
                """
                d_0 = r_0*2
                if d_0 == 0:
                    d_0 = 1e-10

                def exact_length(d0,d1,h):
                    """Calculates the exact length of an archimedean given the spacing, inner and outer diameters.
                    SEE: http://www.giangrandi.ch/soft/spiral/spiral.shtml

                    Parameters
                    ----------
                    d0 : float
                        The inner diameter of the spiral.
                    d1 : float
                        The outer diameter of the spiral.
                    h  : float
                        The spacing of the spiral.
                    """
                    #t0 & t1 are the respective diameters in terms of radians along the spiral.
                    t0 = math.pi*d0/h
                    t1 = math.pi*d1/h
                    return h/(2.0*math.pi)*(t1/2.0*math.sqrt(t1**2+1)+1/2.0*math.log(t1+math.sqrt(t1**2+1))-t0/2.0*math.sqrt(t0**2+1)-1/2.0*math.log(t0+math.sqrt(t0**2+1)))

                def exact_length_derivative(d,h):
                    """Calculates the derivative of the exact length of an archimedean at a given diameter and spacing.
                    SEE: http://www.giangrandi.ch/soft/spiral/spiral.shtml

                    Parameters
                    ----------
                    d : float
                        The diameter point of interest in the spiral.
                    h  : float
                        The spacing of the spiral.
                    """
                    #t is diameter of interest in terms of radians along the spiral.
                    t = math.pi*d/h
                    dl_dt = h/(2.0*math.pi)*((2*t**2+1)/(2*math.sqrt(t**2+1))+(t+math.sqrt(t**2+1))/(2*t*math.sqrt(t**2+1)+2*t**2+2))
                    dl_dd = h*dl_dt/math.pi
                    return dl_dd

                #Approximate radius (for first guess)
                N = (h-d_0+math.sqrt((d_0-h)**2+4*h*L/math.pi))/(2*h)
                D_1 = 2*N*h + d_0
                tol = 1e-10

                #Use Newton's Method to iterate until within tolerance
                while True:
                    f_df_dt = (exact_length(d_0,D_1,h)-L)/1000/exact_length_derivative(D_1,h)
                    if f_df_dt < tol:
                        break
                    D_1 -= f_df_dt
                return D_1/2

            def rollover(val,limit,mode):
                if val < limit:
                    if mode == 'max':
                        return val
                    elif mode == 'min':
                        return limit+(limit-val)
                    else:
                        raise ValueError("'{}' is an incorrect selection for the mode".format(mode))
                else:
                    if mode == 'max':
                        return limit-(val-limit)
                    elif mode == 'min':
                        return val
                    else:
                        raise ValueError("'{}' is an incorrect selection for the mode".format(mode))

            def minor_fraction_calc(e,e_a=300,e_b=2.3,n=0.102,sr=0.6):
                """Calculates the minor fraction (fraction of part b) required to achieve the
                specified dielectric value

                Parameters
                ----------
                e : float
                    Dielectric value of interest
                e_a  : float
                    Dielectric value of part a
                e_b. : float
                    Dielectric value of part b
                n  : float
                    Morphology factor
                sr : float
                    Fraction of SrTi03 in part a
                """
                return 1 - ((e-e_b)*((n-1)*e_b-n*e_a))/(sr*(e_b-e_a)*(n*(e-e_b)+e_b))

            """
            This is a key line of the extrusion values calculations.
            It starts off by calculating the exact length along the spiral for the current
            radius, then adds/subtracts on the dead volume delay (in effect looking into the
            future path) to this length, then recalculates the appropriate radius at this new
            postiion. This is value is then used in the gradient function to determine the minor
            fraction of the mixed elements. Note that if delay is 0, then this line will have no
            effect. If the spiral is moving outwards it must add the dead volume delay, whereas if
            the spiral is moving inwards, it must subtract it.

            """
            if start == 'center':
                offset_radius = exact_radius(0,spacing,rollover(exact_length(0,radius,spacing)+delay,exact_length(0,outer_radius,spacing),'max'))
            else:
                offset_radius = exact_radius(0,spacing,rollover(exact_length(0,radius,spacing)-delay,exact_length(0,inner_radius,spacing),'min'))

            expr = sy.sympify(formula)
            r = sy.symbols('r')
            minor_fraction = np.clip(minor_fraction_calc(float(expr.subs(r,offset_radius))),0,1)
            line_flow = length/float(feed)*flow
            return [minor_fraction*line_flow,(1-minor_fraction)*line_flow,minor_fraction]

        #End of calculate_extrusion_values() function

        start_spiral_turns = (start_diameter/2.0)/spacing
        end_spiral_turns = (end_diameter/2.0)/spacing

        #Use current position as center position if none is specified
        if center_position is None:
            center_position = [self._current_position['x'],self._current_position['y']]

        #Keep track of whether currently in relative or absolute mode
        was_relative = True
        if self.is_relative:
            self.absolute()
        else:
            was_relative = False

        #SEE: https://www.comsol.com/blogs/how-to-build-a-parameterized-archimedean-spiral-geometry/
        b = spacing/(2*math.pi)
        t = np.arange(start_spiral_turns*2*math.pi, end_spiral_turns*2*math.pi, step_angle)

        #Add last final point to ensure correct outer diameter
        t = np.append(t,end_spiral_turns*2*math.pi)
        if start == 'center':
            pass
        elif start == 'edge':
            t = t[::-1]
        else:
            raise Exception("Must either choose 'center' or 'edge' for starting position.")

        #Move to starting positon
        if (direction == 'CW' and start == 'center') or (direction == 'CCW' and start == 'edge'):
            x_move = -t[0]*b*math.cos(t[0])+center_position[0]
        elif (direction == 'CCW' and start == 'center') or (direction == 'CW' and start == 'edge'):
            x_move = t[0]*b*math.cos(t[0])+center_position[0]
        else:
            raise Exception("Must either choose 'CW' or 'CCW' for spiral direction.")
        y_move = t[0]*b*math.sin(t[0])+center_position[1]
        self.move(x_move, y_move)

        #Start writing moves
        self.feed(feedrate)
        syringe_extrusion = np.array([0.0,0.0])

        #Zero a & b axis before printing, we do this so it can easily do multiple layers without quickly jumping back to 0
        #Would likely be useful to change this to relative coordinates at some point
        self.write('G92 a0 b0')

        for step in t[1:]:
            if (direction == 'CW' and start == 'center') or (direction == 'CCW' and start == 'edge'):
                x_move = -step*b*math.cos(step)+center_position[0]
            elif (direction == 'CCW' and start == 'center') or (direction == 'CW' and start == 'edge'):
                x_move = step*b*math.cos(step)+center_position[0]
            else:
                raise Exception("Must either choose 'CW' or 'CCW' for spiral direction.")
            y_move = step*b*math.sin(step)+center_position[1]

            radius_pos = np.sqrt((self._current_position['x']-center_position[0])**2 + (self._current_position['y']-center_position[1])**2)
            line_length = np.sqrt((x_move-self._current_position['x'])**2 + (y_move-self._current_position['y'])**2)
            extrusion_values = calculate_extrusion_values(radius_pos,line_length)
            syringe_extrusion += extrusion_values[:2]
            self.move(x_move, y_move, a=syringe_extrusion[0],b=syringe_extrusion[1],color=extrusion_values[2])

        #Set back to relative mode if it was previsously before command was called
        if was_relative:
                self.relative()

    def purge_meander(self, x, y, spacing, volume_fraction, flowrate, start='LL', orientation='x',
            tail=False, minor_feed=None):
        self.write('FREERUN a {}'.format(flowrate*volume_fraction))
        self.write('FREERUN b {}'.format(flowrate*(1-volume_fraction)))
        self.meander(x, y, spacing, start=start, orientation=orientation,
            tail=tail, minor_feed=minor_feed)
        self.write('FREERUN a 0')
        self.write('FREERUN b 0')

    def log_pile(self, L, W, H, RW, D_N, print_speed, com_ports, P, print_height=None, lead_in=0, dwell=0, jog_speed=10, jog_height=5):
        """ A solution for a 90° log pile lattice

        Parameters
        ----------
        L : float
            Length of log pile base
        W : float
            Width of log pile base
        H : float
            Height of log pile base
        RW : float
            Road width - spacing between filament centers
        D_N : float
            Nozzle diameter
        print_speed : float
            Printing speed
        com_ports : dict
            Dictionary of com_ports for pressure `P` and omnicure `UV`.
        P : float
            Printing pressure
        print_height : float
            Spacing between z-layers. If not provided, the default is 80% of `D_N` to provide better adhesion

        Examples
        --------

        Printing a 10 mm (L) x 15 mm (W) x 5 mm (H) log pile with a road width of 1.4 mm and nozzle size of 0.7 mm (700 um) extruding at 55 psi pressure via com_port 5
        >>> g.log_pile(10, 15, 1.4, 0.7, 1, {'P': 5}, 55)

        !!! note

            Currently, this assumes you are using a pressure-based printing method (e.g., Nordson).
            In the next version, this will be changed so that any arbitrary extruding source can be used.

        """
        COLORS = {
            'pre': (1,1,1),#(1,0,0,0),
            'post': (1,1,1),#(1,0,0,0),
            'even': (0,0,0, 1),
            'odd': (0,0,0, 1),
            'offset': (1,1,1,0)
            # 'post': (25/255,138/255,72/255,0.3)
            # 'even': (45/255, 36/255, 66/255, 1),
            # 'odd': (248/255, 214/255, 65/255, 1)
        }

        dz =  D_N*0.8 if print_height is None else print_height # [mm] z-layer spacing

        z_layers = int(H / dz)
        n_lines_L = int(np.floor(W/RW + 1))
        n_lines_W = int(np.floor(L/RW + 1))

        offset_L = L - (n_lines_W-1)*RW
        offset_W = W - (n_lines_L-1)*RW
        extra_offset = 5 # mm

        print(f'n_lines_L={n_lines_L:.1f} and offset_L={offset_L:.3f}')
        print(f'n_lines_W={n_lines_W:.1f} and offset_W={offset_W:.3f}')
        print(f'RW = {RW:.3f} = {RW/D_N:.3f}*d_N')
        print(f'z_layers = {z_layers:.1f}')
        print(f'rho = {2*D_N/ RW :.3f}')

        '''HELPER FUNCTIONS'''

        def initial_offset(start, orientation, offset):
            # LL
            if start == 'LL' and orientation == 'x':
                self.move(y=+offset/2, color=COLORS['pre'])
            elif start == 'LL' and orientation == 'y':
                self.move(x=+offset/2, color=COLORS['pre'])

            # UL
            elif start == 'UL' and orientation == 'x':
                self.move(y=-offset/2, color=COLORS['pre'])
            elif start == 'UL' and orientation == 'y':
                self.move(x=+offset/2, color=COLORS['pre'])

            # UR
            elif start == 'UR' and orientation == 'x':
                self.move(y=-offset/2, color=COLORS['pre'])
            elif start == 'UR' and orientation == 'y':
                self.move(x=-offset/2, color=COLORS['pre'])

            # LR
            elif start == 'LR' and orientation == 'x':
                self.move(y=+offset/2, color=COLORS['pre'])
            elif start == 'LR' and orientation == 'y':
                self.move(x=-offset/2, color=COLORS['pre'])

        def post_offset(next_start, next_orientation, offset):
            # LL
            if next_start == 'LL' and next_orientation == 'x':
                self.move(y=-extra_offset, color=COLORS['post'])
                self.move(x=-offset/2, color=COLORS['offset'])
                self.move(y=extra_offset, color=COLORS['post'])
            elif next_start == 'LL' and next_orientation == 'y':
                self.move(x=-extra_offset, color=COLORS['post'])
                self.move(y=-offset/2, color=COLORS['offset'])
                self.move(x=-extra_offset, color=COLORS['post'])

            # UL
            elif next_start == 'UL' and next_orientation == 'x':
                self.move(y=extra_offset, color=COLORS['post'])
                self.move(x=+offset/2, color=COLORS['offset'])
                self.move(y=-extra_offset, color=COLORS['post'])
            elif next_start == 'UL' and next_orientation == 'y':
                self.move(x=-extra_offset, color=COLORS['post'])
                self.move(y=+offset/2, color=COLORS['offset'])
                self.move(x=extra_offset, color=COLORS['post'])

            # UR
            elif next_start == 'UR' and next_orientation == 'x':
                self.move(y=extra_offset, color=COLORS['post'])
                self.move(x=+offset/2, color=COLORS['offset'])
                self.move(y=-extra_offset, color=COLORS['post'])
            elif next_start == 'UR' and next_orientation == 'y':
                self.move(x=extra_offset, color=COLORS['post'])
                self.move(y=+offset/2, color=COLORS['offset'])
                self.move(x=-extra_offset, color=COLORS['post'])

            # LR
            elif next_start == 'LR' and next_orientation == 'x':
                self.move(y=-extra_offset, color=COLORS['post'])
                self.move(x=+offset/2, color=COLORS['offset'])
                self.move(y=extra_offset, color=COLORS['post'])
            elif next_start == 'LR' and next_orientation == 'y':
                self.move(x=extra_offset, color=COLORS['post'])
                self.move(y=-offset/2, color=COLORS['offset'])
                self.move(x=-extra_offset, color=COLORS['post'])

            self.write('G92 X0 Y0')
        self.write('; >>> CHANGE PRINT SPEED IN THE FOLLOWING LINE ([=] mm/s) <<<')
        self.feed(print_speed)
        self.write('; >>> CAN CHANGE LEAD IN LENGTH HERE <<<')
        self.move(x=lead_in, color=(1,0,0,0.5)) # lead in

        self.write('; >>> CHANGE PRINT PRINT PRESSURE IN FOLLOWING LINE (0 -> 100, res=0.1) <<<')
        self.set_pressure(com_ports['P'], P)

        self.toggle_pressure(com_ports['P'])   # ON
        self.write('; >>> CHANGE INITIAL DWELL IN THE FOLLOWING LINE ([=] seconds) <<<')
        self.dwell(dwell)

        n_lines_list = [n_lines_L, n_lines_W]

        ''' START '''
        orientations = ['x','y']
        for j in range(z_layers):
            color = COLORS['even'] if j%2==0 else COLORS['odd']
            n_lines_local = n_lines_list[j%2]
            offset_local = offset_W if j%2==0 else offset_L

            # if both even-even or odd-odd
            if n_lines_list[0]%2 == n_lines_list[1]%2:
                if n_lines_local % 2 == 0: # if even
                    start_list = ['LL', 'UL', 'UR', 'LR']
                else:
                    # orientations = ['x','y']
                    start_list = ['LL', 'UR']*2
            # if even-odd
            elif n_lines_list[0]%2 ==0 and n_lines_list[1]%2==1:
                start_list = ['LL', 'UL', 'LR', 'UR']
            # if odd-even
            elif n_lines_list[0]%2 ==1 and n_lines_list[1]%2==0:
                start_list = ['LL', 'UR', 'UL', 'LR']


            self.write(f'; >>>  START LAYER #{j+1} <<<')
            start = start_list[j%4]
            orientation = orientations[j%2]

            next_start = start_list[(j+1)%4]
            next_orientation = orientations[(j+1)%2]

            initial_offset(start, orientation, offset_local)

            # print(start,orientation, ' --> ', next_start, next_orientation)

            if j%2==0: # runs first
                # print(f'> serpentine from {start} towards {orientation}')
                self.serpentine(L, n_lines_local, RW, start, orientation, color=color)
            else:
                # print(f'> serpentine from {start} towards {orientation}')
                self.serpentine(W, n_lines_local, RW, start, orientation, color=color)

            post_offset(next_start, next_orientation, offset_local)

            self.move(z=+dz)
            self.write(f'; >>>  END LAYER #{j+1} <<<')

            ''' STOP '''

            self.toggle_pressure(com_ports['P'])   # OFF

            # move away from lattice
            self.write('; MOVE AWAY FROM PRINT')
            self.feed(jog_speed)
            self.move(z=jog_height)
            self.abs_move(0, 0)
            self.move(z=-jog_height - z_layers*dz)

    # AeroTech Specific Functions  ############################################

    def get_axis_pos(self, axis):
        """ Gets the current position of the specified `axis`.
        """
        cmd = 'AXISSTATUS({}, DATAITEM_PositionFeedback)'.format(axis.upper())
        pos = self.write(cmd)
        return float(pos)

    def set_cal_file(self, path):
        """ Dynamically applies the specified calibration file at runtime.

        Parameters
        ----------
        path : str
            The path specifying the aerotech calibration file.

        """
        self.write(r'LOADCALFILE "{}", 2D_CAL'.format(path))

    def toggle_pressure(self, com_port):
        """ Toggles (On/Off) Nordson Ultimus V Pressure Controllers.

        Parameters
        ----------
        com_port : int
            The com port to communicate over RS-232

        Examples
        --------
        >>> #Turn on pressure on com 3
        >>> g.toggle_pressure(3)

        """
        self.write('Call togglePress P{}'.format(com_port))

        if com_port not in self.extrusion_state.keys():
            self.extrusion_state[com_port] = {'printing': True, 'value': 1}
        # if extruding source HAS been specified
        else:
            self.extrusion_state[com_port]['printing'] = not self.extrusion_state[com_port]['printing']

        # legacy code
        if self.extruding[0] == com_port:
            self.extruding = [com_port, not self.extruding[1], self.extruding[2] if not self.extruding[1] else 0]
        else:
            self.extruding = [com_port, True, self.extruding[2]]

    def set_pressure(self, com_port, value):
        """ Sets pressure on Nordson Ultimus V Pressure Controllers.

        Parameters
        ----------
        com_port : int
            The com port to communicate over RS-232.
        value : float
            The pressure value to set.
        Examples
        --------
        >>> #Set pressure on com 3 to 50.
        >>> g.set_pressure(com_port=3, value=50)

        """

        if com_port not in self.extrusion_state.keys():
            self.extrusion_state[com_port] = {'printing': False, 'value': round(value, 1)}
        else:
            self.extrusion_state[com_port] = {
                'printing': self.extrusion_state[com_port]['printing'],
                'value':  round(value, 1)
            }

        # legacy code
        if self.extruding[0] == com_port:
            self.extruding = [com_port, self.extruding[1], value if self.extruding else 0]
        else:
            self.extruding = [com_port, self.extruding[1], value if self.extruding else 0]
        self.write(f'Call setPress P{com_port} Q{value:.1f}')

    def linear_actuator_on(self, speed, dispenser):
        ''' Sets Aerotech (or similar) linear actuator speed and ON.

        Parameters
        ----------
        speed : float
            The linear actuator speed value to set [in local units].
        dispenser : int or str
            The linear actuator number (int) or full custom name (str).
        Examples
        --------
        >>> # Set extrusion speed to 3 mm/s on dispenser 2
        >>> g.linear_actuator_on(speed=3, dispenser=2)

        >>> # Set custom dispenser name to `PDISP22`
        >>> g.linear_actuator_on(speed=3, dispenser='PDISP22')
        '''

        if str(dispenser).isdigit():
            self.write(f'FREERUN PDISP{dispenser:d} {speed:.6f}')
        else:
            self.write(f'FREERUN {dispenser} {speed:.6f}')

        if dispenser not in self.extrusion_state.keys():
            self.extrusion_state[dispenser] = {'printing': True, 'value': f'{speed:.6f}'}
        # if extruding source HAS been specified
        else:
            self.extrusion_state[dispenser] = {'printing': True, 'value': f'{speed:.6f}'}

        # legacy code
        self.extruding = [dispenser, True]

    def linear_actuator_off(self, dispenser):
        ''' Turn Aerotech (or similar) linear actuator OFF.

        Parameters
        ----------
        dispenser : int or str
            The linear actuator number (int) or full custom name (str).
        Examples
        --------
        >>> # Turn linear actuator `PDISP2` off
        >>> g.linear_actuator_on(speed=3, dispenser='PDISP2')
        '''
        if str(dispenser).isdigit():
            self.write(f'FREERUN PDISP{dispenser:d} STOP')
        else:
            self.write(f'FREERUN {dispenser} STOP')

        if dispenser not in self.extrusion_state.keys():
            self.extrusion_state[dispenser] = {'printing': False, 'value': 0}
        # if extruding source HAS been specified
        else:
            self.extrusion_state[dispenser] = {'printing': False, 'value': 0}

        # legacy code

        self.extruding = [dispenser, False]

    def set_vac(self, com_port, value):
        """ Same as [set_pressure][mecode.main.G.set_pressure] method, but for vacuum.
        """
        self.write('Call setVac P{} Q{}'.format(com_port, value))

    def set_valve(self, num, value):
        """ Sets a digital output state (typically for valve).

        Parameters
        ----------
        num : int
            The com port to communicate over RS-232.
        value : bool
            On or off (1 or 0).
        Examples
        --------
        >>> #Turn on valve 2
        >>> g.set_valve(num=2, value=1)

        """
        self.write('$DO{}.0={}'.format(num, value))

    def omni_on(self, com_port):
        """ Opens the iris for the omnicure.

        Parameters
        ----------
        com_port : int
            The com port to communicate over RS-232

        Examples
        --------
        >>> #Turn on omnicure on com 3.
        >>> g.omni_on(3)

        """
        self.write('Call omniOn P{}'.format(com_port))

    def omni_off(self, com_port):
        """ Opposite to omni_on.
        """
        self.write('Call omniOff P{}'.format(com_port))

    def omni_intensity(self, com_port, value, cal=False):
        """ Sets the intensity of the omnicure.

        Parameters
        ----------
        com_port : int
            The com port to communicate over RS-232.
        value : float
            The intensity value to set.
        cal : bool
            Whether the omnicure is calibrated or not.
        Examples
        --------
        >>> #Set omnicure intensity on com 3 to 50%.
        >>> g.omni_intensity(com_port=3, value=50)

        """

        if cal:
            command = 'SIR{:.2f}'.format(value)
            data = self.calc_CRC8(command)
            self.write('$strtask4="{}"'.format(data))
        else:
            command = 'SIL{:.0f}'.format(value)
            data = self.calc_CRC8(command)
            self.write('$strtask4="{}"'.format(data))
        self.write('Call omniSetInt P{}'.format(com_port))

    def set_alicat_pressure(self, com_port, value):
        """ Same as [set_pressure][mecode.main.G.set_pressure] method, but for Alicat controller.
        """
        extruder_id = f'alicat_com_port{com_port}'
        if extruder_id not in self.extrusion_state.keys():
            self.extrusion_state[extruder_id] = {'printing': True, 'value': f'{value:.6f}'}
        # if extruding source HAS been specified
        else:
            self.extrusion_state[extruder_id] = {'printing': True, 'value': f'{value:.6f}'}

        self.write('Call setAlicatPress P{} Q{}'.format(com_port, value))

    def run_pump(self, com_port):
        '''Run pump with internally stored settings.
            Note: to run a pump, first call `set_rate` then call `run`'''

        extruder_id = f'HApump_com_port{com_port}'
        if extruder_id not in self.extrusion_state.keys():
            self.extrusion_state[extruder_id] = {'printing': True, 'value': 1}
        # if extruding source HAS been specified
        else:
            self.extrusion_state[extruder_id] = {'printing': True, 'value': 1}

        self.write(f'Call runPump P{com_port}')

        self.extruding = [com_port, True, 1]

    def stop_pump(self, com_port):
        '''Stops the pump'''

        extruder_id = f'HApump_com_port{com_port}'
        if extruder_id not in self.extrusion_state.keys():
            self.extrusion_state[extruder_id] = {'printing': False}#, 'value': 0}
        # if extruding source HAS been specified
        else:
            self.extrusion_state[extruder_id] = {'printing': False}#, 'value': 0}

        self.write(f'Call stopPump P{com_port}')

        self.extruding = [com_port, False, 0]


    def calc_CRC8(self,data):
        CRC8 = 0
        for letter in list(bytearray(data, encoding='utf-8')):
            for i in range(8):
                if (letter^CRC8)&0x01:
                    CRC8 ^= 0x18
                    CRC8 >>= 1
                    CRC8 |= 0x80
                else:
                    CRC8 >>= 1
                letter >>= 1
        return data +'{:02X}'.format(CRC8)


    def calc_print_time(self):
        print(f'''\n; Approximate print time:
; \t{self.print_time:.3f} seconds
; \t{self.print_time/60:.1f} min
; \t{self.print_time/60/60:.1f} hrs
''')

    # ROS3DA Functions  #######################################################


    def line_frequency(self,freq,padding,length,com_port,pressure,travel_feed):
        """ Prints a line with varying on/off frequency.

        Parameters
        ----------
        frequency : float
            The length to move in x in one half cycle
        """

        # Switch to relative if in absolute, but keep track of state
        was_absolute = True
        if not self.is_relative:
            self.relative()
        else:
            was_absolute = False

        # Use velocity on, required for switching like this
        self.write("VELOCITY ON")

        print_height = np.copy(self._current_position['z'])
        print_feed = np.copy(self.speed)

        self.set_pressure(com_port,pressure)
        for f in freq:
            # freq is in hz, ie 1/s. Thus dist = (m/s)/(1/s) = m
            dist = print_feed/f
            switch_points = np.arange(length+dist,step=dist)
            if len(switch_points)%2:
                switch_points = switch_points[:-1]
            for point in switch_points:
                self.toggle_pressure(com_port)
                self.move(x=dist)

            #Move to push into substrate
            self.move(z=-print_height)
            self.feed(travel_feed)
            self.move(z=print_height+5)

            if f != freq[-1]:
                self.move(x=-len(switch_points)*dist,y=padding)
                self.move(z=-5)
                self.feed(print_feed)

        # Switch back to velocity off
        self.write("VELOCITY OFF")
        # Switch back to absolute if it was in absolute
        if was_absolute:
            self.absolute()

        return [length,padding*(len(freq)-1)]

    def line_width(self,padding,width,com_port,pressures,spacing,travel_feed):
        """ Prints meanders of varying spacing with different pressures.

        Parameters
        ----------
        frequency : float
            The length to move in x in one half cycle
        """
        # Switch to relative if in absolute, but keep track of state
        was_absolute = True
        if not self.is_relative:
            self.relative()
        else:
            was_absolute = False

        print_height = np.copy(self._current_position['z'])
        print_feed = np.copy(self.speed)

        for pressure in pressures:
            direction = 1
            self.set_pressure(com_port,pressure)
            self.toggle_pressure(com_port)
            for space in spacing:
                #self.toggle_pressure(com_port)
                self.move(y=direction*width)
                self.move(space)
                if space == spacing[-1]:
                    self.move(y=-direction*width)
                #self.toggle_pressure(com_port)
                direction *= -1
            self.toggle_pressure(com_port)
            self.feed(travel_feed)
            self.move(z=5)
            if pressure != pressures[-1]:
                self.move(x=-np.sum(spacing),y=width+padding)
                self.move(z=-5)
                self.feed(print_feed)

        # Switch back to absolute if it was in absolute
        if was_absolute:
            self.absolute()

        return [np.sum(spacing)*2-spacing[-1],len(pressures)*width + (len(pressures)-1)*padding]

    def line_span(self,padding,dwell,distances,com_port,pressure,travel_feed):
        """ Prints meanders of varying spacing with different pressures.

        Parameters
        ----------
        frequency : float
            The length to move in x in one half cycle
        """
        # Switch to relative if in absolute, but keep track of state
        was_absolute = True
        if not self.is_relative:
            self.relative()
        else:
            was_absolute = False

        print_height = np.copy(self._current_position['z'])
        print_feed = np.copy(self.speed)

        for dist in distances:
            self.toggle_pressure(com_port)
            self.dwell(dwell)
            self.feed(print_feed*dist/distances[0])
            self.move(y=dist)
            self.dwell(dwell)
            self.toggle_pressure(com_port)

            self.move(z=-print_height)
            self.feed(travel_feed)
            self.move(z=print_height+5)
            if dist != distances[-1]:
                self.move(x=padding,y=-dist)
                self.move(z=-5)
                self.feed(print_feed)

        # Switch back to absolute if it was in absolute
        if was_absolute:
            self.absolute()

        return [padding*(len(distances)-1),np.max(distances)]


    def line_crossing(self,dwell,feeds,length,com_port,pressure,travel_feed):
        """ Prints meanders of varying spacing with different pressures.

        Parameters
        ----------
        frequency : float
            The length to move in x in one half cycle
        """
        # Switch to relative if in absolute, but keep track of state
        was_absolute = True
        if not self.is_relative:
            self.relative()
        else:
            was_absolute = False

        print_height = np.copy(self._current_position['z'])

        self.set_pressure(com_port,pressure)
        self.toggle_pressure(com_port)
        self.dwell(dwell)
        self.move(x=length)
        self.dwell(dwell)
        self.toggle_pressure(com_port)
        self.move(z=-print_height)
        self.feed(travel_feed)
        self.move(z=print_height+5)

        spacing = length/(len(feeds)+1)
        self.move(x=-spacing,y=8)
        for feed in feeds:
            self.move(z=-(print_height+5))
            self.feed(feed)
            self.move(y=-16)
            if feed != feeds[-1]:
                self.feed(travel_feed)
                self.move(z=print_height+5)
                self.move(x=-spacing,y=16)

        self.feed(travel_feed)
        self.move(z=print_height+5)

        # Switch back to absolute if it was in absolute
        if was_absolute:
            self.absolute()

        return length

    # EXPORT Functions  #######################################################
    def export_points(self, filename):
        ''' Exports a CSV file of the x, y, z coordinates with optional color column for multimaterial support

            Parameters
            ----------
            filename : str
                The name of the exported CSV file.

        '''
        _, file_extension = os.path.splitext(filename)
        if file_extension is False:
            file_extension = f'{file_extension}.csv'

        extruding_history = []
        color_history = []
        printing_history = []

        for h in self.history:
            any_on = any([entry['printing'] is True and entry['value'] != 0 for entry in h['PRINTING'].values()])

            extruding_history.append([h['CURRENT_POSITION']['X'],
                                      h['CURRENT_POSITION']['Y'],
                                      h['CURRENT_POSITION']['Z']])
            color_history.append(h['COLOR'] if h['COLOR'] is not None else DEFAULT_FILAMENT_COLOR)
            printing_history.append(1 if any_on else 0)


        extruding_history = np.array(extruding_history).reshape(-1,3)
        color_history = np.array(color_history).reshape(-1, 3)
        printing_history = np.array(printing_history).reshape(-1,1)

        np.savetxt(filename,
                    np.hstack([extruding_history, color_history, printing_history]),
                   delimiter=',',
                   header='x,y,z,R,G,B,ON',
                   comments='',
                   fmt=['%.6f']*3+['%.3f']*3 + ['%d']
                   )




    def gen_geometry(self,outfile,filament_diameter=0.8,cut_point=None,preview=False,color_incl=None):
        """ Creates an openscad file to create a CAD model from the print path.

        Parameters
        ----------
        outfile : str
            Location to save the generated .scad file
        filament_diameter : float (default: 0.8)
            The com port to communicate over RS-232.
        cut_point : int (default: None)
            Stop generating cad model part way through the path
        preview : bool (default: False)
            Show matplotlib preview of the part to be generated.
            Note that cut_point will affect the preview.
        color_incl : str (default: None)
            Used to export a single color when it is included in the code
            design. Useful for exporting mutlimaterial parts as different
            cad models.
        Examples
        --------
        >>> #Write geometry to 'test.scad'
        >>> g.gen_geometry('test.scad')

        """
        import solid as sld
        from solid import utils as sldutils
        import matplotlib.pyplot as plt

        # Matplotlib setup for preview
        fig = plt.figure(dpi=150)
        ax = plt.axes(projection='3d')

        def circle(radius,num_points=10):
            circle_pts = []
            for i in range(2 * num_points):
                angle = math.radians(360 / (2 * num_points) * i)
                circle_pts.append(sldutils.Point3(radius * math.cos(angle), radius * math.sin(angle), 0))
            return circle_pts

        # SolidPython setup for geometry creation
        extruded = 0
        filament_cross = circle(radius=filament_diameter/2)

        extruding_hist = dict(self.extruding_history)
        position_hist = np.array(self.position_history)

        #Stepping through all moves after initial position
        extruding_state = False
        for index, (pos, color) in enumerate(zip(self.position_history[1:cut_point],self.color_history[1:cut_point]),1):
            sys.stdout.write('\r')
            sys.stdout.write("Exporting model: {:.0f}%".format(index/len(self.position_history[1:])*100))
            sys.stdout.flush()
            #print("{}/{}".format(index,len(self.position_history[1:])))
            if index in extruding_hist:
                extruding_state =  extruding_hist[index][1]

            if extruding_state and ((color == color_incl) or (color_incl is None)):
                X, Y, Z = position_hist[index-1:index+1, 0], position_hist[index-1:index+1, 1], position_hist[index-1:index+1, 2]
                # Plot to matplotlb
                if color_incl is not None:
                    ax.plot(X, Y, Z,color_incl)
                else:
                    ax.plot(X, Y, Z,'b')
                # Add geometry to part
                extruded += sldutils.extrude_along_path(shape_pts=filament_cross, path_pts=[sldutils.Point3(*position_hist[index-1]),sldutils.Point3(*position_hist[index])])
                extruded += sld.translate(position_hist[index-1])(sld.sphere(r=filament_diameter/2,segments=20))
                extruded += sld.translate(position_hist[index])(sld.sphere(r=filament_diameter/2,segments=20))

        # Export geometry to file
        file_out = os.path.join(os.curdir, '{}.scad'.format(outfile))
        print("\nSCAD file written to: \n%(file_out)s" % vars())
        sld.scad_render_to_file(extruded, file_out, include_orig_code=False)

        if preview:
            # Display Geometry for matplotlib
            X, Y, Z = position_hist[:, 0], position_hist[:, 1], position_hist[:, 2]

            # Hack to keep 3D plot's aspect ratio square. See SO answer:
            # http://stackoverflow.com/questions/13685386
            max_range = np.array([X.max()-X.min(),
                                  Y.max()-Y.min(),
                                  Z.max()-Z.min()]).max() / 2.0

            mean_x = X.mean()
            mean_y = Y.mean()
            mean_z = Z.mean()
            ax.set_xlim(mean_x - max_range, mean_x + max_range)
            ax.set_ylim(mean_y - max_range, mean_y + max_range)
            ax.set_zlim(mean_z - max_range, mean_z + max_range)
            scaling = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz']); ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)
            plt.show()

    def export_APE(self):
        """ Exports a list of dictionaries describing extrusion moves in a
        format compatible with APE.

        Examples
        --------
        >>> #Write print geometry
        >>> geometry_def = g.meander()

        """
        extruding_hist = dict(self.extruding_history)
        position_hist = self.position_history
        cut_ranges=[*extruding_hist][1:]
        final_coords = []
        for i in range(0,len(cut_ranges),2):
            final_coords.append(position_hist[cut_ranges[i]-1:cut_ranges[i+1]])
        final_coords_dict = []
        for i in final_coords:
            keys = ['X','Y','Z']
            final_coords_dict.append([dict(zip(keys, l)) for l in i ])
        return final_coords_dict

    # Public Interface  #######################################################

    def view(self,
                backend='matplotlib',
                outfile=None,
                hide_travel=False,
                color_on=True,
                nozzle_cam=False,
                fast_forward = 3,
                framerate = 60,
                nozzle_dims=[1.0,20.0],
                substrate_dims=[0.0,0.0,-1.0,300,1,300],
                scene_dims = [720,720],
                ax=None,
                **kwargs):
        """ View the generated Gcode.

        Parameters
        ----------
        backend : str (default: 'matplotlib')
            The plotting backend to use, one of 'matplotlib' or 'mayavi'.
            'matplotlib2d' has been addded to better visualize mixing.
            'vpython' has been added to generate printing animations
            for debugging.
        outfile : str (default: 'None')
            When using the 'matplotlib' backend,
            an image of the output will be save to the location specified
            here.
        color_on : bool (default: 'False')
            When using the 'matplotlib' or 'matplotlib2d' backend,
            the generated image will display the color associated
            with the g.move command. This was primarily used for mixing
            nozzle debugging.
        nozzle_cam : bool (default: 'False')
            When using the 'vpython' backend and nozzle_cam is set to
            True, the camera will remained centered on the tip of the
            nozzle during the animation.
        fast_forward : int (default: 1)
            When using the 'vpython' backend, the animation can be
            sped up by the factor specified in the fast_forward
            parameter.
        nozzle_dims : list (default: [1.0,20.0])
            When using the 'vpython' backend, the dimensions of the
            nozzle can be specified using a list in the format:
            [nozzle_diameter, nozzle_length].
        substrate_dims: list (default: [0.0,0.0,-0.5,100,1,100])
            When using the 'vpython' backend, the dimensions of the
            planar substrate can be specified using a list in the
            format: [x, y, z, length, height, width].
        scene_dims: list (default: [720,720])
            When using the 'vpython' backened, the dimensions of the
            viewing window can be specified using a list in the
            format: [width, height]
        ax : matplotlib axes object
            Useful for adding additional functionailities to plot when debugging.

        """
        from mecode_viewer import plot2d, plot3d, animation
        # import matplotlib.cm as cm
        # from mpl_toolkits.mplot3d import Axes3D
        # import matplotlib.pyplot as plt
        # history = np.array(self.position_history)

        # use_local_ax = True if ax is None else False

        if backend == '2d':
           ax = plot2d(self.history, ax=ax, hide_travel=hide_travel, **kwargs)



        elif backend == 'matplotlib' or backend == '3d':
            ax = plot3d(self.history, ax=ax, hide_travel=hide_travel, **kwargs)

            return ax

        elif backend == 'mayavi':
            # from mayavi import mlab
            # mlab.plot3d(history[:, 0], history[:, 1], history[:, 2])
            raise ValueError(f'The {backend} backend is not currently supported.')

        elif backend == 'vpython' or backend == 'animated':
            animation(self.history,
                        outfile,
                        hide_travel,
                        color_on,
                        nozzle_cam,
                        fast_forward,
                        framerate,
                        nozzle_dims,
                        substrate_dims,
                        scene_dims,
                        **kwargs)

        else:
            raise Exception("Invalid plotting backend! Choose one of matplotlib or matplotlib2d or vpython.")

    def write(self, statement_in, resp_needed=False):
        if self.print_lines:
            print(statement_in)
        self._write_out(statement_in)
        statement = encode2To3(statement_in + self.lineend)
        if self.direct_write is True:
            if self.direct_write_mode == 'socket':
                if self._socket is None:
                    import socket
                    self._socket = socket.socket(socket.AF_INET,
                                                socket.SOCK_STREAM)
                    self._socket.connect((self.printer_host, self.printer_port))
                self._socket.send(statement)
                if self.two_way_comm is True:
                    response = self._socket.recv(8192)
                    response = decode2To3(response)
                    if response[0] != '%':
                        raise RuntimeError(response)
                    return response[1:-1]
            elif self.direct_write_mode == 'serial':
                if self._p is None:
                    from .printer import Printer
                    self._p = Printer(self.printer_port, self.baudrate)
                    self._p.connect()
                    self._p.start()
                if resp_needed:
                    return self._p.get_response(statement_in)
                else:
                    self._p.sendline(statement_in)

    def rename_axis(self, x=None, y=None, z=None):
        """ Replaces the x, y, or z axis with the given name.

        Examples
        --------
        >>> g.rename_axis(z='A')

        """
        if x is not None:
            self.x_axis = x
        elif y is not None:
            self.y_axis = y
        elif z is not None:
            self.z_axis = z
        else:
            msg = 'Must specify new name for x, y, or z only'
            raise RuntimeError(msg)

    # Private Interface  ######################################################

    def _write_out(self, line=None, lines=None):
        """ Writes given `line` or `lines` to the output file.
        """
        # Only write if user requested an output file.
        if self.out_fd is None:
            return

        if lines is not None:
            for line in lines:
                self._write_out(line)

        line = line.rstrip() + self.lineend  # add lineend character
        if 'b' in self.out_fd.mode:  # encode the string to binary if needed
            line = encode2To3(line)
        self.out_fd.write(line)


    def _meander_passes(self, minor, spacing):
        if minor > 0:
            passes = math.ceil(minor / spacing)
        else:
            passes = abs(math.floor(minor / spacing))
        return passes

    def _meander_spacing(self, minor, spacing):
        return minor / self._meander_passes(minor, spacing)

    def _write_header(self):
        if self.aerotech_include is True:
            with open(os.path.join(HERE, 'header.txt')) as fd:
                self._write_out(lines=fd.readlines())
        if self.header is not None:
            with open(self.header) as fd:
                self._write_out(lines=fd.readlines())

    def _format_args(self, x=None, y=None, z=None, **kwargs):
        d = self.output_digits
        args = []
        if x is not None:
            args.append('{0}{1:.{digits}f}'.format(self.x_axis, x, digits=d))
        if y is not None:
            args.append('{0}{1:.{digits}f}'.format(self.y_axis, y, digits=d))
        if z is not None:
            args.append('{0}{1:.{digits}f}'.format(self.z_axis, z, digits=d))
        args += ['{0}{1:.{digits}f}'.format(k, kwargs[k], digits=d) for k in sorted(kwargs)]
        args = ' '.join(args)
        return args

    def _update_current_position(self, mode='auto', x=None, y=None, z=None, color = (0,0,0),
                                 **kwargs):

        new_state = copy.deepcopy(self.history[-1])
        new_state['COORDS'] = (x, y, z)

        if mode == 'auto':
            mode = 'relative' if self.is_relative else 'absolute'
            new_state['REL_MODE'] = self.is_relative

        if self.x_axis != 'X' and x is not None:
            kwargs[self.x_axis] = x
        if self.y_axis != 'Y' and y is not None:
            kwargs[self.y_axis] = y
        if self.z_axis != 'Z' and z is not None:
            kwargs[self.z_axis] = z

        if mode == 'relative':
            if x is not None:
                self._current_position['x'] += x
            if y is not None:
                self._current_position['y'] += y
            if z is not None:
                self._current_position['z'] += z
            for dimention, delta in kwargs.items():
                self._current_position[dimention] += delta
        else:
            if x is not None:
                self._current_position['x'] = x
            if y is not None:
                self._current_position['y'] = y
            if z is not None:
                self._current_position['z'] = z
            for dimention, delta in kwargs.items():
                self._current_position[dimention] = delta

        x = self._current_position['x']
        y = self._current_position['y']
        z = self._current_position['z']

        new_state['CURRENT_POSITION'] = {'X': x, 'Y': y, 'Z': z}
        new_state['COLOR'] = color

        # if self.extruding[0] is not None:
        #     new_state['PRINTING'][self.extruding[0]] = {'printing': self.extruding[1], 'value': self.extruding[2]}
        # for k, v in self.extrusion_state.items():
        #     new_state['PRINTING'][k] = v
        new_state['PRINTING'] = copy.deepcopy(self.extrusion_state)

        self.position_history.append((x, y, z))

        try:
            color = mcolors.to_rgb(color)
        except ValueError as e:
            raise ValueError(f'Invalid color value provided and could not convert to RGB: {e}')

        self.color_history.append(color)
        new_state['COLOR'] = color
        new_state['PRINT_SPEED'] = self.speed


        len_history = len(self.position_history)
        if (len(self.speed_history) == 0
            or self.speed_history[-1][1] != self.speed):
            self.speed_history.append((len_history - 1, self.speed))
        if (len(self.extruding_history) == 0
            or self.extruding_history[-1][1] != self.extruding):
            self.extruding_history.append((len_history - 1, self.extruding))

        self.history.append(new_state)
        # print('updating state', self.history[-1]['COLOR'], self.history[-1]['PRINTING'] )

    def _update_print_time(self, x,y,z):
        if x is None:
            x = self.current_position['x']
        if y is None:
            y = self.current_position['y']
        if z is None:
            z = self.current_position['z']
        self.print_time += np.linalg.norm([x,y,z]) / self.speed
