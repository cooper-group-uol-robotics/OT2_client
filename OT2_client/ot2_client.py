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
from ot2_driver.ot2_driver_http import OT2_Config, OT2_Driver
from urllib.error import HTTPError, URLError
from urllib3.exceptions import ConnectionError, ConnectTimeoutError
from urllib3.connection import HTTPException, HTTPConnection
import requests



class OT2Client:

    """
    The init function is neccesary for the OT2Client class to initialize all variables, parameters, and other functions.
    Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
    """

    def __init__(self, ip = "169.254.227.210", device_name="OT2_alpha"):

        self.ip = ip
        self.node_name = device_name

        print("Received IP: " + self.ip + " Robot name: " + str(self.node_name))
        
        self.state = "UNKNOWN"
        self.action_flag = "READY"
        self.robot_status = ""
        self.past_robot_status = ""
        self.state_refresher_timer = 0

        self.resources_folder_path = r'C:\\Users\scrc112\Documents\\Ot2_ws\src\\Protocols\\ot2_temp/' + self.node_name + "/" + "resources/"  
        self.protocols_folder_path = r'C:\\Users\scrc112\Documents\\Ot2_ws\src\\Protocols\\ot2_temp/' + self.node_name + "/" + "protocols/"  
        
        self.check_resources_folder()
        self.check_protocols_folder()
        self.ot2_connected = self.connect_robot()

        self.description = {
            "name": self.node_name,
            "type": "",
            "actions": {
                "execute": "config : %s",  ## takes in the yaml content as second string arg
                "run_protocol": "config_path: %s",  ## Temp inclusion
            },
        }

        # action_cb_group = ReentrantCallbackGroup()
        # description_cb_group = ReentrantCallbackGroup()
        # state_cb_group = ReentrantCallbackGroup()
        self.timer_period = 1  # seconds

    def connect_robot(self):
        try:
            self.ot2 = OT2_Driver(OT2_Config(ip = self.ip))

        except ConnectTimeoutError as connection_err:
            self.state = "OT2 CONNECTION ERROR"
            print("Connection error code: " + connection_err)
            return False

        except HTTPError as http_error:
            print("HTTP error code: " +  http_error)
            return False
            
        except URLError as url_err:
            print("Url error code: " +  url_err)
            return False

        except requests.exceptions.ConnectionError as conn_err: 
            print("Connection error code: "+ str(conn_err))
            return False
            
        except Exception as error_msg:
            self.state = "OT2 ERROR"
            print("-------" + str(error_msg) +  " -------")
            return False

        else:
            print(str(self.node_name) + " online")
            return True

    def check_resources_folder(self):
        """
        Description: Checks if the resources folder path exists. Creates the resource folder path if it doesn't already exists
        """

        isPathExist = os.path.exists(self.resources_folder_path)
        if not isPathExist:
            os.makedirs(self.resources_folder_path)
            print("Resource path doesn't exists")
            print("Creating: " + self.resources_folder_path)
            
    def check_protocols_folder(self):
        """
        Description: Checks if the protocols folder path exists. Creates the resource folder path if it doesn't already exists
        """

        isPathExist = os.path.exists(self.protocols_folder_path)
        if not isPathExist:
            os.makedirs(self.protocols_folder_path)
            print("Protocols path doesn't exists")
            print("Creating: " + self.protocols_folder_path)

    def stateCallback(self):
        """The state of the robot, can be ready, completed, busy, error"""
        try:
            self.robot_status = self.ot2.get_robot_status().upper()
            print("robot status", self.robot_status)
     
        except HTTPError as http_error:
            print("HTTP error code: ", http_error)
            self.state = "OT2 CONNECTION ERROR"
            
        except URLError as url_err:
            print("Url error code: ", url_err)

        except Exception as error_msg:
            self.state = "ERROR"
            print("-------" + str(error_msg) +  " -------")

        # except Exception as err:
        #     self.get_logger().error("ROBOT IS NOT RESPONDING! ERROR: " + str(err))
        #     self.state = "OT2 CONNECTION ERROR"

        if self.state != "OT2 CONNECTION ERROR":
            msg = None

            if self.robot_status == "FAILED" or (self.state == "ERROR" and self.action_flag == "BUSY"):
                self.state = "ERROR"
                msg = 'State: %s' % self.state
                print(msg)
                print(self.ot2.get_robot_status())
                self.action_flag = "READY"
                self.ot2.reset_robot_data()

            elif self.state == "COMPLETED" and self.action_flag == "BUSY":
                self.state = "COMPLETED"
                msg = 'State: %s' % self.state
                print(msg)
                self.action_flag = "READY"

            elif self.robot_status == "RUNNING" or self.robot_status == "FINISHING" or self.robot_status == "PAUSED":
                self.state = "BUSY"
                msg = 'State: %s' % self.state
                print(msg)

            elif self.robot_status == "IDLE":
                self.state = "READY"
                msg = 'State: %s' % self.state
                print(msg)
            elif self.robot_status == "OFFLINE":
                self.state = "ROBOT OFFLINE"
                msg = 'State: %s' % self.state
                print(msg)
                print("Trying to connect again! IP: " + self.ip)
                self.connect_robot()
            else:
                self.state = "UNKOWN"
                msg = 'State: %s' % self.state
                print(msg)
        else: 
            msg = None
            msg = 'State: %s' % self.state
            print(msg)
            # self.get_logger().warn("Trying to connect again! IP: " + self.ip)
            # self.connect_robot()

    def actionCallback(self, cmd, id):

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

        protocol_path = protocol_path
        resource_config = resource_config

        if self.state == "OT2 CONNECTION ERROR":
            msg = "Can not accept the job! OT2 CONNECTION ERROR"
            return msg

        while self.state != "READY":
            print("Waiting for OT2 to switch READY state...")
            time.sleep(0.5)
        
        self.action_flag = "BUSY"    

        self.action_command = cmd
        # self.action_vars = json.loads(request.vars)
        # self.get_logger().info(f"{self.action_vars=}")

        print(f"In action callback, command: {self.action_command}")

        if "run_protocol" == self.action_command:

            # protocol_config = self.action_vars.get("config_path", None)
            # resource_config = self.action_vars.get("resource_path", None) #TODO: This will be enbaled in the future 
            # resource_file_flag = self.action_vars.get("use_existing_resources", "False") #Returns True to use a resource file or False to not use a resource file. 
            
            # protocol_config = "/home/uol/ot2_ws/src/ot2_module/ot2_driver/protopiler/test_configs/basic_config.py"



            # Specify the path to your YAML file


            protocol_config = f"W:\OT2_ws\src\OT2_client_package\Protocols\{id}"

            self.get_logger().info("sending python yuan's protocol_1")

            # resource_config = "/home/uol/ot2_ws/src/ot2_module/ot2_driver/config/resources.json"

            resource_file_flag = False
            

            if resource_file_flag:
                try:
                    list_of_files = glob.glob(self.resources_folder_path + '*.json') #Get list of files
                    if len(list_of_files) > 0: 
                        resource_config = max(list_of_files, key=os.path.getctime) #Finding the latest added file
                        print("Using the resource file: " + resource_config)

                except Exception as er:
                    print(er)
            if protocol_config:

                config_file_path = protocol_config
                
                # resource_config_path = resource_config
                # config_file_path, resource_config_path = self.download_config_files(protocol_config, resource_config)
                # payload = deepcopy(self.action_vars)
                # payload.pop("config_path")

                # self.get_logger().info(f"ot2 {payload=}")

                print("config_file_path ", config_file_path)
                # print("resource_config_path ", resource_config_path)

                print(f"config_file_path: {config_file_path}")

                # response_flag, response_msg = self.execute(config_file_path, payload, resource_config_path)
                # response_flag, response_msg = self.execute(config_file_path, resource_config_path)
                response_flag, response_msg = self.execute(config_file_path)
                
                if response_flag == True:
                    self.state = "COMPLETED"
                    response = True

                elif response_flag == False:
                    self.state = "ERROR"
                    response = False

                print("Finished Action: " + self.action_command)
                return response

            else:
                print( "Required 'config' was not specified in request.vars")
                self.state = "ERROR"

                return response
        else:
            msg = "UNKOWN ACTION REQUEST! Available actions: run_protocol"
            print('Error: ' + msg)
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
        
        # time_str = datetime.now().strftime('%Y%m%d-%H%m%s')
        time_str = datetime.now().strftime('%Y%m%d-%H%M%S')
        config_file_path = (
            config_dir_path
            / f"protocol-{time_str}.yaml"
        )

        print(
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
            print(f"{protocol_file_path.resolve()=}")
            self.protocol_id, self.run_id = self.ot2.transfer(self.protocol_file_path)
            print("OT2 " + self.node_name + " protocol transfer successful")
            resp = self.ot2.execute(self.run_id)
            print("OT2 "+ self.node_name +" executed a protocol")
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
                print(response_msg)

            response_msg = f"Error: {traceback.format_exc()}"
            print(response_msg)
            return False, response_msg

            # rclpy.shutdown()  ## TODO: Could alternatively indent into the if block.
            ## TODO: Changed to as is to forestall any unexpected exceptions

    def poll_OT2_until_run_completion(self):
        """Queries the OT2 run state until reported as 'succeeded'"""

        print("Polling OT2 run until completion")
        while self.state != "IDLE":

            run_status = self.ot2.get_run(self.run_id)

            if (
                run_status["data"]["status"]
                and run_status["data"]["status"] == "succeeded"
            ):
                self.state = "COMPLETED"
                print("Stopping Poll")

            elif (
                run_status["data"]["status"]
                and run_status["data"]["status"] == "running"
            ):
                self.state = "BUSY"

            self.stateCallback()
            time.sleep(self.timer_period)


# def main(args=None):


#     rclpy.init(args=args)  # initialize Ros2 communication
#     try:
#         ot2_client = OT2Client()
#         executor = MultiThreadedExecutor()
#         executor.add_node(ot2_client)

#         try:
#             print('Beginning client, shut down with CTRL-C')
#             executor.spin()
#         except KeyboardInterrupt:
#             print('Keyboard interrupt, shutting down.\n')
#             ot2_client.ot2.change_lights_status(False)

#         finally:
#             executor.shutdown()
#             ot2_client.destroy_node()
#     finally:
#         rclpy.shutdown()

#     # else:
#     #     print("The robot_ip environment variable has not been specified.")
#     #     ## NOTE: TODO TO Verify that the IP Address is correct?


# if __name__ == "__main__":

#     main()
