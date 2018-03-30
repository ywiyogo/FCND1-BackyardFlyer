import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:

            # coordinate conversion 
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                self.calculate_box()
                self.waypoint_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        alt_threshold = 0.2
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < alt_threshold and abs(self.local_position[2]) < alt_threshold:
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implemented

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.WAYPOINT:
            self.waypoint_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()
        # elif self.flight_state == States.LANDING:
        #     self.disarming_transition()

    def calculate_box(self):
        """TODO: implemented
        
        1. Return waypoints to fly a box
        """
        target_alt = 3
        distance = 10
        self.all_waypoints=[
            np.array([distance,  0 , target_alt, 0]),
            np.array([distance, distance, target_alt, 0]),
            np.array([0, distance, target_alt, 0]),
            np.array([0, 0, target_alt, 0]) ]
        

    def arming_transition(self):
        """TODO: implemented
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        drone.set_home_position(drone.global_position[0], 
                                drone.global_position[1], 
                                drone.global_position[2])
        self.flight_state = States.ARMING


    def takeoff_transition(self):
        """TODO: implemented
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_alt = 3.0
        self.target_position[2] = target_alt
        self.takeoff(target_alt)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: implemented
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")

        dist_treshold = 0.4
        # if there is no more waypoints left -> landing
        if len(self.all_waypoints) == 0:
            self.in_mission = False
            self.landing_transition()

        # check if current location near the nord location
        #calculate the distance
        print("local pose: ", self.local_position[:2])
        print("next waypoint: ", self.all_waypoints[0][:2])
        dist = np.linalg.norm(self.local_position[:2]-self.all_waypoints[0][:2])
        print("Distance: ", dist)

        if dist > dist_treshold:
            self.target_position = self.all_waypoints[0]
            print("Target position: ", self.target_position)
            self.cmd_position(self.target_position[0],
                            self.target_position[1],
                            self.target_position[2],
                            self.target_position[3])
            self.flight_state = States.WAYPOINT
        else:
            #target reached, remove the current target waypoint
            self.all_waypoints.pop(0)
            print("Next waypoints: ", self.target_position)
            if len(self.all_waypoints) >0:
                self.target_position = self.all_waypoints[0]
                self.cmd_position(self.target_position[0],
                                self.target_position[1],
                                self.target_position[2],
                                self.target_position[3])
                self.flight_state = States.WAYPOINT
            else:
                self.in_mission = False
                self.landing_transition()

        

    def landing_transition(self):
        """TODO: implemented
        
        1. Command the drone to land
        2. Transition to the LANDING state  
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: implemented
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
