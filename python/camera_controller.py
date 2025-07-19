#! /usr/bin/env python3

import atexit
import cv2
import math
import numpy as np
import re
import serial
import time

from inspect import currentframe, getframeinfo

 # If this value is returned for position or velocity there's an error
ERROR_VALUE = 1_000_000

# Camera image size
IMG_WIDTH = 640
IMG_HEIGHT = 480

def get_current_line_number():
    '''
    I use this for troubleshooting code
    '''
    f = currentframe()
    return getframeinfo(f).lineno


class ESP32_Controller:
    '''
    @brief used to communicate to and from the ESP32
    '''
    
    # Private data
    _SERIAL_PORT = "/dev/ttyUSB0"
    _SERIAL_BAUD = 115200
    _MAX_RETRIES = 5

    # Public data
    MAX_BUFF_LEN = 255

    def __init__(self):
        self.connected = False
        self.port = None

        self.connect_to_ESP()
        self.write_serial('s0')
        return

    def connect_to_ESP(self):
        '''
        @brief connects to ESP32 by trying _MAX_RETRIES times
        @param none
        @retval none
        '''
        connections_tried = 1
        prev = time.time()
        while (not self.connected):
            try:
                self.port = serial.Serial(self._SERIAL_PORT, self._SERIAL_BAUD, timeout=1)
            except:
                if (time.time() - prev > 2):
                    print("Can't connect to serial port: {}. Retrying {}".
                        format(self._SERIAL_PORT, connections_tried))
                    prev = time.time()
                    connections_tried += 1
                    
                # Only try to connect for a set number of attempts
                if connections_tried > self._MAX_RETRIES:
                    exit(2)
            if (self.port is not None):
                print("Connected to {}".format(self._SERIAL_PORT))
                self.connected = True


    def read_serial(self, num_char = 1):
        '''
        @brief reads from serial.
        @param num_char : number of bytes to read from serial
        '''
        if self.connected and self.port.is_open:
            return self.port.read(num_char).decode()
        else:
            print("ERROR: Not connected to ESP32 (line: {})".
                  format(get_current_line_number()))
            return ""


    def write_serial(self, cmd):
        '''
        @brief sends command postfixed by newline character
        @param cmd byte string
        '''
        if self.connected and self.port.is_open:
            cmd = cmd + '\n'
            self.port.write(cmd.encode())
        else:
            print("ERROR: Not connected to ESP32 (line: {})".
                  format(get_current_line_number()))     


class Camera_Interface:
    '''
    @brief used to get images from the camera and analyze them for red car
    '''

    # Private data
    _CAM_ID = 0

    # Color red thresholds
    _HUE_MIN = 0
    _HUE_MAX = 17
    _SAT_MIN = 31
    _SAT_MAX = 255
    _VAL_MIN = 255
    _VAL_MAX = 255
    _RED_LOWER = np.array([_HUE_MIN, _SAT_MIN, _VAL_MIN])
    _RED_UPPER = np.array([_HUE_MAX, _SAT_MAX, _VAL_MAX])

    # Use these for inverse perspective transform
    _TOP_LEFT  = (80, 2)
    _TOP_RIGHT = (528, 9)
    _BOT_LEFT  = (34, 472)
    _BOT_RIGHT = (580, 473)

    # minimum number of pixel in the mask to consider it a valid image
    _MIN_PIXELS = 10

    def __init__(self):
        # Initialize the camera
        self.camera_device = cv2.VideoCapture(self._CAM_ID)

        # Assume the scene doesn't change so I only need to find the warp 
        # transform once
        as_is_corners = np.float32([self._TOP_LEFT, self._TOP_RIGHT,
                                    self._BOT_LEFT, self._BOT_RIGHT])
        perfect_corners = np.float32([[0, 0], [IMG_WIDTH, 0], 
                                      [0, IMG_HEIGHT], [IMG_WIDTH, IMG_HEIGHT]])
        self.perspective_matrix = cv2.getPerspectiveTransform(as_is_corners, 
                                                         perfect_corners)

        self.prev_failed_img = True
        self._prev_time = 0
        self._prev_x = 0
        self._prev_y = 0
        return
    
    def find_car(self):
        '''
        @brief
        @retval (x, y, x_dot, y_dot, current_frame) position and velocity 
            of car if not enough pixels to determine car position
        '''
        # Acquire an image
        ret, raw_frame = self.camera_device.read()

        if ret:
            self.prev_failed_img = False
            (rows, cols, chnl) = raw_frame.shape
            #cv2.imshow("raw frame", raw_frame)
            #cv2.waitKey(1)

            corrected_frame = cv2.warpPerspective(raw_frame, 
                                                  self.perspective_matrix,
                                                  (IMG_WIDTH, IMG_HEIGHT))

            # Color threshold it for red:
            blurred_img = cv2.GaussianBlur(corrected_frame, (5, 5), 0)
            hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_img, self._RED_LOWER, self._RED_UPPER)
            cv2.imshow("red mask", mask)
            cv2.waitKey(1)

            # Find center of mass of red blob on X
            # I collapse the mask on X by summing each column and then I find
            # the center of mass of the sum
            mask_squish_on_x = np.sum(mask, axis=0)
            mask_x = np.sum([i*mask_squish_on_x[i] for i in range(cols)])
            sum_x = np.sum(mask_squish_on_x)
            if sum_x > (255 * self._MIN_PIXELS):
                x_pos = int(float(mask_x) / float(sum_x))
            else:
                # not enough pixels to call it
                self.prev_failed_img = False
                print("Can't get x position.")
                return (ERROR_VALUE, ERROR_VALUE, 
                        ERROR_VALUE, ERROR_VALUE,
                        corrected_frame)

            # Find center of mass of red blob on Y (same as X)
            mask_squish_on_y = np.sum(mask, axis=1)
            mask_y = np.sum([i*mask_squish_on_y[i] for i in range(rows)])
            sum_y = np.sum(mask_squish_on_y)
            if sum_x > (255 * self._MIN_PIXELS):
                y_pos = int(float(mask_y) / float(sum_y))
            else:
                # not enough pixels to call it
                self.prev_failed_img = False
                print("Can't get y position.")
                return (ERROR_VALUE, ERROR_VALUE, 
                        ERROR_VALUE, ERROR_VALUE, 
                        corrected_frame)

            # Calculate velocity:
            if self.prev_failed_img:
                (x_dot, y_dot) = (0, 0)
            else:
                current_time = time.time()
                time_delta = current_time - self._prev_time
                self._prev_time = current_time
                x_delta = (x_pos - self._prev_x)
                y_delta = (y_pos - self._prev_y)
                x_dot = int(x_delta / time_delta)
                y_dot = int(y_delta / time_delta)

                # ignore any velocity when x or y delta are smaller than 
                # THRESHOLD_DELTA (there is a lot of one pixel noise)
                THRESHOLD_DELTA = 4
                x_dot = x_dot if x_delta**2 > THRESHOLD_DELTA else 0
                y_dot = y_dot if y_delta**2 > THRESHOLD_DELTA else 0

            if (False):
                print("####")
                print("Time delta: {:.2f}".format(time_delta))
                print("Current position: {:d}, {:d}".format(x_pos, y_pos))
                print("Previou position: {:d}, {:d}".format(self._prev_x, self._prev_y))
                print("Current velocity: {:d}, {:d}".format(x_dot, y_dot))

            self._prev_x = x_pos
            self._prev_y = y_pos

            return (x_pos, y_pos, x_dot, y_dot, corrected_frame)
        else: 
            print("Can't get image.")
            self.prev_failed_img = True

            return (ERROR_VALUE, ERROR_VALUE, 
                    ERROR_VALUE, ERROR_VALUE,
                    raw_frame)
        

class Driver_Controller:
    '''
    @brief determines how the car gets driven and displays some telemetry
    '''

    # scale by which velocity should be amplified for display purposes
    _VELOCITY_SCALE = 1
    _HEADING_SCALE = 16

    _SET_POINTS_ARRAY = [[164, 215], [495, 215]]

    _THROTTLE_FWD = 4
    _THROTTLE_BKD = 1
    _STEER_LEFT = 4
    _STEER_RIGHT = 9

    # Thresholds to stop the car:
    _CLOSE_ENOUGH = 100 # when it is close enough to the setpoint
    # When it is close enough to the border
    BORDER_MIN = 200


    def __init__(self, car_coms):
        '''
        @param car_coms is the handler for ESP32 object
        '''
        (self.x_pos, self.y_pos, self.x_dot, self.y_dot) = (0, 0, 0, 0)
        (self.error_x, self.error_y) = (0, 0)
        self.error_magnitude = 0.0
        self.current_frame = None

        # The angle (in rad) the car is facing (0 is facing along x axis)
        self.heading = 0
        (self.heading_x, self.heading_y) = (0, 0)

        self.current_setpoint = 0
        self.setpoint_x = self._SET_POINTS_ARRAY[self.current_setpoint][0]
        self.setpoint_y = self._SET_POINTS_ARRAY[self.current_setpoint][1]

        self.out_of_bounds = 0
        self.undrivable = False

        self.car_command = 's5s5' #stop throttle and stop steering

        self.throttle_dir = 'f'
        self.throttle_mag = 0
        self.steering_dir = 'l'
        self.steering_mag = 0

        self.esp_control = car_coms
        return
    
    def drive(self, x, y, x_dot, y_dot, current_frame):
        '''
        @brief Calculate the difference between the current heading and the 
            desired velocity vector and adjust steering and throttle 
            appropriately
        @param x_dot
        @param y_dot
        @param current_frame
        '''
        (self.x_pos, self.y_pos) = (x, y)
        (self.x_dot, self.y_dot) = (x_dot, y_dot)

        self.error_x = x - self.setpoint_x
        self.error_y = y - self.setpoint_y
        self.error_magnitude = math.sqrt(self.error_x**2 + self.error_y**2)

        # Check if this state is drivable
        self.car_command = 's5s5'
        self.undrivable = False
        if ((self.x_pos is ERROR_VALUE) or
            (self.y_pos is ERROR_VALUE) or
            (self.x_dot is ERROR_VALUE) or
            (self.y_dot is ERROR_VALUE)):
            self.undrivable = True
            car_coms.write_serial(self.car_command)
            print("Error: position or velocity error. " + 
                  "Car command: {}".format(self.car_command))
        
        # Only update the heading if the velocities are non-zero:
        if ((x_dot**2 > 0)):
            self.heading = math.atan(self.y_dot/self.x_dot)
            self.heading_x = math.cos(self.heading)
            self.heading_y = math.sin(self.heading)
            #print("(x_dot, y_dot): ({:.2f},{:.2f})".format(x_dot, y_dot))
            #print("Updating heading 1: {:.2f}".format(self.heading))
        else:
            if (y_dot**2 > 0):
                self.heading = 1.57 * (1 if self.y_dot >= 0 else -1)
                self.heading_x = math.cos(self.heading)
                self.heading_y = math.sin(self.heading)
                #print("(x_dot, y_dot): ({:.2f},{:.2f})".format(x_dot, y_dot))
                #print("Updating heading 2: {:.2f}".format(self.heading))

        # Check if we are too close to the border 
        if ((not self.undrivable) and (self.x_pos < self.BORDER_MIN) or
            (self.y_pos < self.BORDER_MIN) or
            (self.x_pos > IMG_WIDTH - self.BORDER_MIN) or
            (self.y_pos > IMG_HEIGHT - self.BORDER_MIN)):
            # Only change direction once every FRAME_DELAY frames (this way we
            # won't dither back and forth)
            FRAME_DELAY = 60
            self.out_of_bounds += 1
            if self.out_of_bounds % FRAME_DELAY == 0:
                selected_command = False
                if (self.x_pos < self.BORDER_MIN):
                    print("Left border")
                    # Set throttle
                    self.throttle_dir = 'f' if self.throttle_dir == 'b' else 'b'
                    self.throttle_mag = (self._THROTTLE_FWD if self.throttle_dir == 'f' 
                                    else self._THROTTLE_BKD)
                    # Set steering
                    self.steering_dir = ('r' if 
                                         ((self.throttle_dir == 'f') and 
                                          (self.heading > 0)) 
                                          else 'l')
                    self.steering_mag = (self._STEER_LEFT if self.steering_dir == 'l' 
                                        else self._STEER_RIGHT)
                    selected_command = True
                
                if (self.x_pos > IMG_WIDTH - self.BORDER_MIN) and (not selected_command):
                    print("Right border")
                    # Set throttle
                    self.throttle_dir = 'f' if self.throttle_dir == 'b' else 'b'
                    self.throttle_mag = (self._THROTTLE_FWD if self.throttle_dir == 'f' 
                                    else self._THROTTLE_BKD)
                    # Set steering
                    self.steering_dir = ('l' if 
                                         ((self.throttle_dir == 'f') and 
                                          (self.heading > 0)) 
                                          else 'r')
                    self.steering_mag = (self._STEER_LEFT if self.steering_dir == 'l' 
                                        else self._STEER_RIGHT)
                    selected_command = True

                if (self.y_pos > IMG_HEIGHT - self.BORDER_MIN) and (not selected_command):
                    print("Bottom border")
                    # Set throttle
                    self.throttle_dir = 'f' if self.throttle_dir == 'b' else 'b'
                    self.throttle_mag = (self._THROTTLE_FWD if self.throttle_dir == 'f' 
                                    else self._THROTTLE_BKD)
                    # Set steering
                    self.steering_dir = ('r' if self.throttle_dir == 'f' else 'l')
                    self.steering_mag = (self._STEER_LEFT if self.steering_dir == 'l' 
                                        else self._STEER_RIGHT)
                    selected_command = True

                if (self.y_pos < self.BORDER_MIN) and (not selected_command):
                    print("Top border")
                    # Set throttle
                    self.throttle_dir = 'f' if self.throttle_dir == 'b' else 'b'
                    self.throttle_mag = (self._THROTTLE_FWD if self.throttle_dir == 'f' 
                                    else self._THROTTLE_BKD)
                    # Set steering
                    self.steering_dir = ('l' if self.throttle_dir == 'f' else 'r')
                    self.steering_mag = (self._STEER_LEFT if self.steering_dir == 'l' 
                                        else self._STEER_RIGHT)
                    selected_command = True
                
                # Compose and send the command
                self.car_command = "{}{}{}{}".format(
                    self.throttle_dir, int(self.throttle_mag),
                    self.steering_dir, int(self.steering_mag))
                car_coms.write_serial(self.car_command)
                print("Error: <<< OUT OF BOUNDS >>>. Reverse direction." + 
                    "Car command: {}".format(self.car_command))
        else:
            print(">>> In bounds <<<<")
            self.out_of_bounds = 0

        if ((not self.undrivable) and (self.out_of_bounds == 0)):
            # Determine the command for the car:
            # Determine the direction
            error_vector = np.array([self.error_x, self.error_y])
            heading_vector = np.array([self.heading_x, self.heading_y])
            # Go forward if the dot product is positive and backwards if negative
            self.direction = 'f' if np.dot(error_vector, heading_vector) > 0 else 'b'
            # Determine the speed        forward_magnitude = 3 if direction is 'f' else 2 #self.error_magnitude * forward_kp

            forward_kp = 0.01
            self.throttle_mag = (self._THROTTLE_FWD if self.throttle_dir == 'f' 
                            else self._THROTTLE_BKD) #self.error_magnitude * forward_kp

            # Check if we are close enough to current setpoint:
            if (self.error_magnitude > self._CLOSE_ENOUGH):
                self.car_command = '{}{}{}{}'.format(
                    self.throttle_dir,int(self.throttle_mag),
                    self.steering_dir,int(self.steering_mag))
            else:
                # We are close enough stop and go to next waypoint
                self.car_command = 's5s5'
                # Set the setpoint to the next setpoint:
                self.current_setpoint = ((self.current_setpoint + 1) 
                                        if (self.current_setpoint < (len(self._SET_POINTS_ARRAY) - 1)) 
                                        else 0)
                self.setpoint_x = self._SET_POINTS_ARRAY[self.current_setpoint][0]
                self.setpoint_y = self._SET_POINTS_ARRAY[self.current_setpoint][1]

            # Send command        
            car_coms.write_serial(self.car_command)

            if (False):
                print("*** Driving: ")
                print("Error (x, y): ({}, {}) Error magnitude: {:.2f}".
                    format(self.error_x, self.error_y, self.error_magnitude))
                print("Car heading: {}".format(self.heading))
                print("Car command: {}".format(car_command))
        
        self.current_frame = current_frame
        self.display_image()
    
    def display_image(self):
        txt_font = cv2.FONT_HERSHEY_SIMPLEX
        txt_font_scale = 0.5
        txt_font_thickness = 1
        txt_start_pos = (20, 20)
        txt_color = (255, 255, 0)
        txt_line_type = cv2.LINE_AA
        TXT_INCREMENT = 15

        # Display image with (x,y) coordinates on it
        circ_center = (self.x_pos, self.y_pos)
        circ_radius = 5
        circ_color = (255, 0, 255)
        circ_line_thickness = -1
        # circle is at pos (x,y)
        cv2.circle(self.current_frame, circ_center, circ_radius, circ_color, 
                   circ_line_thickness)
        cv2.putText(self.current_frame,'Car X, Y: {}, {}'.
                    format(self.x_pos, self.y_pos), 
                    txt_start_pos, txt_font, txt_font_scale, txt_color,
                    txt_font_thickness, txt_line_type)
        
        # Display car velocity vector (x_dot, y_dot)
        line_start = (self.x_pos, self.y_pos)
        line_end = (self.x_pos + int(self.x_dot * self._VELOCITY_SCALE), 
                    self.y_pos + int(self.y_dot * self._VELOCITY_SCALE))
        line_color = (0, 255, 255)
        line_thickness = 1
        # line is representing velocity vector
        cv2.line(self.current_frame, line_start, line_end, line_color, 
                 line_thickness)
        txt_start_pos = (txt_start_pos[0], txt_start_pos[1] + TXT_INCREMENT)
        cv2.putText(self.current_frame,'Velocity X, Y: {}, {}'.
                    format(self.x_dot, self.y_dot), 
                    txt_start_pos, txt_font, txt_font_scale, txt_color,
                    txt_font_thickness, txt_line_type)
        
        # Display car heading (x, y)
        line_end = (self.x_pos + int(self.heading_x * self._HEADING_SCALE), 
                    self.y_pos + int(self.heading_y * self._HEADING_SCALE))
        line_color = (255, 255, 0)
        cv2.line(self.current_frame, line_start, line_end, line_color, 
                 line_thickness)
        txt_start_pos = (txt_start_pos[0], txt_start_pos[1] + TXT_INCREMENT)
        cv2.putText(self.current_frame,'Heading rad | (x, y): {} | ({:.2f},{:.2f})'.
                    format(self.heading, self.heading_x, self.heading_y), 
                    txt_start_pos, txt_font, txt_font_scale, txt_color,
                    txt_font_thickness, txt_line_type)

        # Display location of set point
        circ_center = (self.setpoint_x, self.setpoint_y)
        circ_radius = 5
        circ_color = (0, 255, 0)
        cv2.circle(self.current_frame, circ_center, circ_radius, circ_color,
                   circ_line_thickness)
        txt_start_pos = (txt_start_pos[0], txt_start_pos[1] + TXT_INCREMENT)
        cv2.putText(self.current_frame,'Setpoint X, Y: {}, {}'.
                    format(self.setpoint_x, self.setpoint_y), 
                    txt_start_pos, txt_font, txt_font_scale, txt_color,
                    txt_font_thickness, txt_line_type)

        # Display error vector
        line_end = (self.setpoint_x, self.setpoint_y)
        line_color = (0, 255, 0)
        cv2.line(self.current_frame, line_start, line_end, line_color, 
                 line_thickness)
        txt_start_pos = (txt_start_pos[0], txt_start_pos[1] + TXT_INCREMENT)
        cv2.putText(self.current_frame,'Error magnitude: {:.2f}'.
                    format(self.error_magnitude), 
                    txt_start_pos, txt_font, txt_font_scale, txt_color,
                    txt_font_thickness, txt_line_type)
        
        # Car command
        txt_start_pos = (txt_start_pos[0], txt_start_pos[1] + TXT_INCREMENT)
        cv2.putText(self.current_frame,'Car command: {}'.
                    format(self.car_command), 
                    txt_start_pos, txt_font, txt_font_scale, txt_color,
                    txt_font_thickness, txt_line_type)

        cv2.imshow("Driver view", self.current_frame)
        cv2.waitKey(1)


# Sending commands to ESP32
car_coms = ESP32_Controller()
camera = Camera_Interface()
driver = Driver_Controller(car_coms)

def clean_up(car_coms):
    # Stop the car
    print("Stopping car")
    car_coms.write_serial('s5s5')

atexit.register(clean_up, car_coms)


while (True):
    (x, y, x_dot, y_dot, current_frame) = camera.find_car()
    driver.drive(x, y, x_dot, y_dot, current_frame)

while (False):
    command = input("Enter command (fx, bx, lx, rx, s): ")

    # Sanitize the command:
    regexp = re.compile(r"^[fblrs]\d$")
    if regexp.search(command):
        esp_32.write_serial(command)
        print("Sent command: {}".format(command))
        reply = esp_32.read_serial(esp_32.MAX_BUFF_LEN)
        if (len(reply) > 0):
            print(reply)
    else:
        print("Unrecognized command: {}".format(command))