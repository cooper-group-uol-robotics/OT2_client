#! /usr/bin/python3
"""OT2 Node"""
import os
import glob
import json
import traceback
import yaml
from typing import List, Tuple
from pathlib import Path
from datetime import datetime
from copy import deepcopy
import time
from std_msgs.msg import String

from ot2_driver.ot2_driver_http import OT2_Config, OT2_Driver
# import opentrons.simulate
# from opentrons.simulate import format_runlog
from urllib.error import HTTPError, URLError
from urllib3.exceptions import ConnectionError, ConnectTimeoutError
from urllib3.connection import HTTPException, HTTPConnection
import requests



class OT2Client:

    """
    The init function is neccesary for the OT2Client class to initialize all variables, parameters, and other functions.
    Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
    """

    def __init__(self, TEMP_NODE_NAME = "OT2_Node"):
        """Setup OT2 node"""

        super().__init__(TEMP_NODE_NAME)
        self.node_name = self.get_name()

        self.declare_parameter("ip","127.0.0.1")

        # Receiving the real IP and PORT from the launch parameters
        self.ip =  self.get_parameter("ip").get_parameter_value().string_value

        self.get_logger().info("Received IP: " + self.ip + " Robot name: " + str(self.node_name))
        
        self.state = "UNKNOWN"
        self.action_flag = "READY"
        self.robot_status = ""
        self.past_robot_status = ""
        self.state_refresher_timer = 0

        self.resources_folder_path = '/home/uol/.ot2_temp/' + self.node_name + "/" + "resources/"  
        self.protocols_folder_path = '/home/uol/.ot2_temp/' + self.node_name + "/" + "protocols/"  
        
        self.check_resources_folder()
        self.check_protocols_folder()
        self.connect_robot()

        self.description = {
            "name": self.node_name,
            "type": "",
            "actions": {
                "execute": "config : %s",  ## takes in the yaml content as second string arg
                "run_protocol": "config_path: %s",  ## Temp inclusion
            },
        }

        action_cb_group = ReentrantCallbackGroup()
        description_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()
        self.timer_period = 1  # seconds

        # Publisher for ot2 state
        self.statePub = self.create_publisher(String, self.node_name + "/state", 10)

        # Timer callback publishes state to namespaced ot2_state
        self.stateTimer = self.create_timer(self.timer_period, self.stateCallback, callback_group = state_cb_group)
      
        # Control and discovery services
        self.actionSrv = self.create_service(
            WeiActions, self.node_name + "/action_handler", self.actionCallback, callback_group = action_cb_group
        )
        self.descriptionSrv = self.create_service(
            WeiDescription, self.node_name + "/description_handler", self.descriptionCallback, callback_group = description_cb_group
        )

    def connect_robot(self):
        try:
            self.ot2 = OT2_Driver(OT2_Config(ip = self.ip))

        except ConnectTimeoutError as connection_err:
            self.state = "OT2 CONNECTION ERROR"
            self.get_logger().error("Connection error code: " + connection_err)

        except HTTPError as http_error:
            self.get_logger().error("HTTP error code: " +  http_error)
            
        except URLError as url_err:
            self.get_logger().error("Url error code: " +  url_err)

        except requests.exceptions.ConnectionError as conn_err: 
            self.get_logger().error("Connection error code: "+ str(conn_err))
            
        except Exception as error_msg:
            self.state = "OT2 ERROR"
            self.get_logger().error("-------" + str(error_msg) +  " -------")

        else:
            self.get_logger().info(str(self.node_name) + " online")

    def check_resources_folder(self):
        """
        Description: Checks if the resources folder path exists. Creates the resource folder path if it doesn't already exists
        """

        isPathExist = os.path.exists(self.resources_folder_path)
        if not isPathExist:
            os.makedirs(self.resources_folder_path)
            self.get_logger().warn("Resource path doesn't exists")
            self.get_logger().info("Creating: " + self.resources_folder_path)
            
    def check_protocols_folder(self):
        """
        Description: Checks if the protocols folder path exists. Creates the resource folder path if it doesn't already exists
        """

        isPathExist = os.path.exists(self.protocols_folder_path)
        if not isPathExist:
            os.makedirs(self.protocols_folder_path)
            self.get_logger().warn("Protocols path doesn't exists")
            self.get_logger().info("Creating: " + self.protocols_folder_path)

    def stateCallback(self):
        """The state of the robot, can be ready, completed, busy, error"""
        try:
            self.robot_status = self.ot2.get_robot_status().upper()
            # self.get_logger().info(str(self.robot_status))
     
        except HTTPError as http_error:
            self.get_logger().err("HTTP error code: ", http_error)
            self.state = "OT2 CONNECTION ERROR"
            
        except URLError as url_err:
            self.get_logger().err("Url error code: ", url_err)

        except Exception as error_msg:
            self.state = "ERROR"
            self.get_logger().error("-------" + str(error_msg) +  " -------")

        # except Exception as err:
        #     self.get_logger().error("ROBOT IS NOT RESPONDING! ERROR: " + str(err))
        #     self.state = "OT2 CONNECTION ERROR"

        if self.state != "OT2 CONNECTION ERROR":
            msg = String()

            if self.robot_status == "FAILED" or (self.state == "ERROR" and self.action_flag == "BUSY"):
                self.state = "ERROR"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
                self.get_logger().error(self.ot2.get_robot_status())
                self.action_flag = "READY"
                self.ot2.reset_robot_data()

            elif self.state == "COMPLETED" and self.action_flag == "BUSY":
                self.state = "COMPLETED"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
                self.action_flag = "READY"

            elif self.robot_status == "RUNNING" or self.robot_status == "FINISHING" or self.robot_status == "PAUSED":
                self.state = "BUSY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            elif self.robot_status == "IDLE":
                self.state = "READY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
            elif self.robot_status == "OFFLINE":
                self.state = "ROBOT OFFLINE"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().warn(msg.data)
                self.get_logger().warn("Trying to connect again! IP: " + self.ip)
                self.connect_robot()
            else:
                self.state = "UNKOWN"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().warn(msg.data)
        else: 
            msg = String()
            msg.data = 'State: %s' % self.state
            self.statePub.publish(msg)
            self.get_logger().error(msg.data)
            # self.get_logger().warn("Trying to connect again! IP: " + self.ip)
            # self.connect_robot()

    def actionCallback(self, request, response):

        """The actions the robot can perform, also performs them
        Parameters:
        -----------
        request: str
            Request to the robot to perform an action
        respone: bool
            If action is performed
        Returns
        -------
        None
        """

        if self.state == "OT2 CONNECTION ERROR":
            msg = "Can not accept the job! OT2 CONNECTION ERROR"
            self.get_logger.error(msg)
            response.action_response = -1
            response.action_msg = msg
            return response

        while self.state != "READY":
            self.get_logger().warn("Waiting for OT2 to switch READY state...")
            time.sleep(0.5)
        
        self.action_flag = "BUSY"    

        self.action_command = request.action_handle
        # self.action_vars = json.loads(request.vars)
        # self.get_logger().info(f"{self.action_vars=}")

        self.get_logger().info(f"In action callback, command: {self.action_command}")

        if "run_protocol" == self.action_command:

            # protocol_config = self.action_vars.get("config_path", None)
            # resource_config = self.action_vars.get("resource_path", None) #TODO: This will be enbaled in the future 
            # resource_file_flag = self.action_vars.get("use_existing_resources", "False") #Returns True to use a resource file or False to not use a resource file. 
            
            # protocol_config = "/home/uol/ot2_ws/src/ot2_module/ot2_driver/protopiler/test_configs/basic_config.py"

            # Specify the path to your YAML file
            protocol_config = "/home/uol/ot2_ws/src/ot2_module/ot2_driver/config/protocol_1.py"

            self.get_logger().info("sending python yuan's protocol_1")

            resource_config = "/home/uol/ot2_ws/src/ot2_module/ot2_driver/config/resources.json"

            resource_file_flag = True

            if resource_file_flag:
                try:
                    list_of_files = glob.glob(self.resources_folder_path + '*.json') #Get list of files
                    if len(list_of_files) > 0: 
                        resource_config = max(list_of_files, key=os.path.getctime) #Finding the latest added file
                        self.get_logger().info("Using the resource file: " + resource_config)

                except Exception as er:
                    self.get_logger().error(er)
            if protocol_config:
                
                
                # config_file_path, resource_config_path = self.download_config_files(protocol_config, resource_config)
                config_file_path = protocol_config
                
                resource_config_path = resource_config

                # payload = deepcopy(self.action_vars)
                # payload.pop("config_path")

                # self.get_logger().info(f"ot2 {payload=}")

                print("config_file_path ", config_file_path)
                print("resource_config_path ", resource_config_path)

                self.get_logger().info(f"config_file_path: {config_file_path}")

                # response_flag, response_msg = self.execute(config_file_path, payload, resource_config_path)
                response_flag, response_msg = self.execute(config_file_path, resource_config_path)

                
                if response_flag == True:
                    self.state = "COMPLETED"
                    response.action_response = 0
                    response.action_msg = response_msg
                    if resource_config_path:
                        response.resources = str(resource_config_path)

                elif response_flag == False:
                    self.state = "ERROR"
                    response.action_response = -1
                    response.action_msg = response_msg
                    if resource_config_path:
                        response.resources = str(resource_config_path)

                self.get_logger().info("Finished Action: " + request.action_handle)
                return response

            else:
                response.action_msg = (
                    "Required 'config' was not specified in request.vars"
                )
                response.action_response = -1
                self.get_logger().error(response.action_msg)
                self.state = "ERROR"

                return response
        else:
            msg = "UNKOWN ACTION REQUEST! Available actions: run_protocol"
            response.action_response = -1
            response.action_msg= msg
            self.get_logger().error('Error: ' + msg)
            self.state = "COMPLETED"

            return response

    def descriptionCallback(self, request, response):
        """The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.
        Parameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: Tuple[str, List]
            The actions a robot can do, will be populated during execution
        Returns
        -------
        Tuple[str, List]
            The robot steps it can do
        """
        response.description_response = str(self.description)

        return response
        
    def download_config_files(self, protocol_config: str, resource_config = None):
        """
        Saves protocol_config string to a local yaml file locaton

        Parameters:
        -----------
        protocol_config: str
            String contents of yaml protocol file

        Returns
        -----------
        config_file_path: str
            Absolute path to generated yaml file
        """

        config_dir_path = Path.home().resolve() / self.protocols_folder_path
        config_dir_path.mkdir(exist_ok=True, parents=True)
        
        resource_dir_path = Path.home().resolve() / self.resources_folder_path
        resource_dir_path.mkdir(exist_ok=True, parents=True)
        
        time_str = datetime.now().strftime('%Y%m%d-%H%m%s')
        config_file_path = (
            config_dir_path
            / f"protocol-{time_str}.yaml"
        )

        self.get_logger().info(
            "Writing protocol config to {} ...".format(str(config_file_path))
        )

        with open(config_file_path, "w", encoding="utf-8") as pc_file:
            yaml.dump(protocol_config, pc_file, indent=4, sort_keys=False)
        if resource_config:
            resource_file_path = resource_dir_path / f"resource-{self.node_name}-{time_str}.json"
            with open(resource_config) as resource_content:
                content = json.load(resource_content)
            json.dump(content, resource_file_path.open("w"))
            return config_file_path, resource_file_path
        else:
            return config_file_path, None
        
    def execute(self, protocol_path, resource_config = None, payload=None):
        """
        Compiles the yaml at protocol_path into .py file;
        Transfers and Exececutes the .py file

        Parameters:
        -----------
        protocol_path: str
            absolute path to the yaml protocol

        Returns
        -----------
        response: bool
            If the ot2 execution was successful
        """


        try:
            (
                self.protocol_file_path,
                self.resource_file_path,
            ) = self.ot2.compile_protocol(protocol_path, payload=payload, resource_file = resource_config, resource_path = self.resources_folder_path, protocol_out_path = self.protocols_folder_path) 
            protocol_file_path = Path(self.protocol_file_path)
            self.get_logger().info(f"{protocol_file_path.resolve()=}")
            self.protocol_id, self.run_id = self.ot2.transfer(self.protocol_file_path)
            self.get_logger().info("OT2 " + self.node_name + " protocol transfer successful")
            resp = self.ot2.execute(self.run_id)
            self.get_logger().info("OT2 "+ self.node_name +" executed a protocol")
            # self.get_logger().warn(str(resp))

            if resp["data"]["status"] == "succeeded":
                # self.poll_OT2_until_run_completion()
                response_msg = "OT2 "+ self.node_name +" successfully completed running a protocol"
                return True, response_msg

            else: 
                response_msg = "OT2 "+ self.node_name +" failed running a protocol"
                return False, response_msg

        # except FileNotFoundError:
        #     from pathlib import Path

        #     response_msg = "Could not find protocol config file at {}, {}".format(protocol_path, Path(protocol_path).exists())
        #     self.get_logger().error(response_msg)
        #     self.stateCallback()

        except Exception as err:

            if "no route to host" in str(err.args).lower():
                response_msg = "No route to host error. Ensure that this container \
                has network access to the robot and that the environment \
                variable, robot_ip, matches the ip of the connected robot \
                on the shared LAN."
                self.get_logger().error(response_msg)

            response_msg = f"Error: {traceback.format_exc()}"
            self.get_logger().error(response_msg)
            return False, response_msg

            # rclpy.shutdown()  ## TODO: Could alternatively indent into the if block.
            ## TODO: Changed to as is to forestall any unexpected exceptions

    def poll_OT2_until_run_completion(self):
        """Queries the OT2 run state until reported as 'succeeded'"""

        self.get_logger().info("Polling OT2 run until completion")
        while self.state != "IDLE":

            run_status = self.ot2.get_run(self.run_id)

            if (
                run_status["data"]["status"]
                and run_status["data"]["status"] == "succeeded"
            ):
                self.state = "COMPLETED"
                self.get_logger().info("Stopping Poll")

            elif (
                run_status["data"]["status"]
                and run_status["data"]["status"] == "running"
            ):
                self.state = "BUSY"

            self.stateCallback()
            time.sleep(self.timer_period)


def main(args=None):


    rclpy.init(args=args)  # initialize Ros2 communication
    try:
        ot2_client = OT2Client()
        executor = MultiThreadedExecutor()
        executor.add_node(ot2_client)

        try:
            ot2_client.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            ot2_client.get_logger().info('Keyboard interrupt, shutting down.\n')
            ot2_client.ot2.change_lights_status(False)

        finally:
            executor.shutdown()
            ot2_client.destroy_node()
    finally:
        rclpy.shutdown()

    # else:
    #     print("The robot_ip environment variable has not been specified.")
    #     ## NOTE: TODO TO Verify that the IP Address is correct?


if __name__ == "__main__":

    main()
