from opentrons import protocol_api
import opentrons.execute
from pprint import pprint
from collections import defaultdict
from pathlib import Path
import json
import requests
import math
import time

 
metadata = { 
            "protocolName": "test",
            "description": "test for connection",
            "author": "Yuan Li",
            "apiLevel": "2.18"
        }

#computer_url = 'http://169.254.142.17:5000'



deck_info = {
  "6": {
    "name": "reservoir",
    "type": "nest_12_reservoir_15ml",
    "A3": {
      "substance": "Water",
      "amount": 10000,
      "solvent_type": "aqueous"
    },
    "A5": {
      "substance": "Blue Water",
      "amount": 10000,
      "solvent_type": "aqueous"
    },   
    "A11": {
      "substance": "Orange Water",
      "amount": 10000,
      "solvent_type": "aqueous"
    }  
  }
}

move_info = [
    {
        "type": "move_substances",
        "amount": 100,
        "substance_name": "Orange Water",
        "position_to": {
            "labware": "plate",
            "slot": 5,
            "wells": ["A5"]
        },
        "mount": "right"
    }
]

offset_data = { 
  "camera_rack": {
    "type": "opentrons_96_tiprack_300ul",
    "pos": 3,
    "offset": {
      "x": 0.0,
      "y": 0.1,
      "z": 0.1
    }
  },
  "plate": {
    "type": "corning_96_wellplate_360ul_flat",
    "pos": 5,
    "offset": {
      "x": -0.3,
      "y": -0.2,
      "z": 0.8
    }
  },
  "reservoir": {
    "type": "azenta_12_reservoir_2100ul",
    "pos": 6,
    "offset": {
      "x": 0.0,
      "y": 0.0,
      "z": 11.0
    }
  },
  "tiprack300": {
    "type": "opentrons_96_tiprack_300ul",
    "pos": 10,
    "offset": {
      "x": -0.9,
      "y": 0.6,
      "z": 0.2
    }
  },
  "left_pipette": {
    "type": "p300_multi_gen2",
    "mount": "left"
  },
  "right_pipette": {
    "type": "p300_single_gen2",
    "mount": "right"
  }
}



# deck_info = json.loads(substances_string)
# move_info = json.loads(movement_string)
# offset_data = json.loads(system_config)



protocol = opentrons.execute.get_protocol_api('2.18')

class Opentrons:
    # Initialize Opentrons class
    def __init__(self,
                protocol: protocol_api.ProtocolContext,
                deck_info: dict,
               ):
        """
        Initialise the Opentrons object.

        Parameters
        ----------
        protocol : protocol_api.ProtocolContext
            The protocol context for the current protocol.
        
        deck_info : dict
            Dictionary of deck information.
            Contains information of substances and their quantities on the deck.
            keys are the deck slot. Value is pure labware/labware loaded on module containing liquids

        Returns
        -------
        None
        """

        self.protocol = protocol
        #Add dynamic loading of labware (camera_rack could also be used as tiprack), not including stock solvents
        self.camera_rack = self.protocol.load_labware("opentrons_96_tiprack_300ul", 3)
        self.plate = self.protocol.load_labware("corning_96_wellplate_360ul_flat", 5)
        self.tiprack300 = self.protocol.load_labware("opentrons_96_tiprack_300ul", 10)
        self.heater_shaker = self.protocol.load_module("heaterShakerModuleV1", 1)
        
        # A collection of used labwares
        self.labware = {} 
        self.labware[3] = self.camera_rack
        self.labware[5] = self.plate
        self.labware[10] = self.tiprack300

        # Load labware used for stocking solution (define substance and its position)
        for deck_number, details in deck_info.items():
            slot = int(deck_number)
            labware_type = deck_info[deck_number]['type']
            if slot != 12:
                self.reservoir = self.protocol.load_labware(labware_type, slot)
                self.labware[int(deck_number)] = self.reservoir
            else:
                pass
                #self.plate_on_heater_shaker = self.heater_shaker.load_labware(labware_type)
                #self.labware[int(deck_number)] = self.plate_on_heater_shaker
        #print(self.labware)

        # Delete type and name from deck_info
        for deck_number in deck_info:
            del deck_info[deck_number]['type']
            del deck_info[deck_number]['name']

        # Load pipette
        self.left_pipette = self.protocol.load_instrument(
        "p300_multi_gen2", mount="left", tip_racks=[self.tiprack300])
        self.right_pipette = self.protocol.load_instrument(
        "p300_single_gen2", mount="right", tip_racks=[self.camera_rack])  

        # Track substances amount
        self.substances = defaultdict(list)
        for position, details in deck_info.items():
            position_int = int(position)
            for well_position, substance_info in details.items():
                labware = self.labware[position_int]
                substance_position = labware.wells(well_position)
                substance_name = substance_info['substance']
                amount = substance_info['amount'] 
                solvent_type = substance_info['solvent_type']
                self.substances[substance_name].append({
                    'position': substance_position,
                    'amount': amount,
                    'solvent_type': solvent_type
                })
        #print(self.substances)

    # Add some specific functions(swell tips, move without drips for organic solvents), except for normal liquid handling 
    def swell_tip(self, mount, source, times_of_prewet=2):
        """
        Notes
        ----------
        Swell the tip in the 'stock' lavbware location at a specified flowrate.

        Parameters
        ----------
        mount : str
            Pipette to be used.
        source: 
            position to swell the tip in(substance_position).
        times_of_prewet: 
            prewet the tip when handling organic solvents.   
        Returns
        -------
        None
        """
        # confirm pipette to be used
        if mount == 'left':
            pipette = self.left_pipette 
            volume = 100
            pipette.flow_rate.aspirate = 50 # default 94 ul/s
            pipette.flow_rate.dispense = 50
        else:
            pipette = self.right_pipette
            volume = 20
            pipette.flow_rate.aspirate = 4 # default 7.56 ul/s
            pipette.flow_rate.dispense = 4
        for i in range(times_of_prewet):
            pipette.aspirate(volume, source)
            self.protocol.delay(10)
            pipette.move_to(source.top())
            pipette.dispense(volume, location=source)
            pipette.blow_out()
        pipette.swelled = True


    def move_without_drip(self, mount, source, amount, target_wells, new_tip):
        """
        Notes
        ----------
        Lower the gantry speed when transferring liquid

        Parameters
        ----------
        mount : str
            Pipette to be used.
        source: 
            position to swell the tip in.
        amount: float or int
            Amount of substance to be moved. (in uL)!!!
        target_wells: 
            Move substance to a list of wells.     
        Returns
        -------
        Nones
        """
        # Set gantry speed (default: X: 600, Y: 400, Z: 100, A: 100)
        self.protocol.max_speeds['X'] = 250
        self.protocol.max_speeds['Y'] = 250
        self.protocol.max_speeds['Z'] = 50
        if mount == 'left':
            pipette = self.left_pipette
        else: 
            pipette = self.right_pipette
        if new_tip == 'once':
            pipette.aspirate(amount, source)
            pipette.air_gap(15)
            pipette.dispense(location=target_wells.top(z=2))
        else: 
            pipette.drop_tip()
            pipette.pick_up_tip()
            pipette.aspirate(amount, source)
            pipette.air_gap(15)
            pipette.dispense(location=target_wells.top(z=2))

    def normal_liquid(self, mount, source, amount, target_wells, new_tip):
        """
        Notes
        ----------
        Handle the aqueous liquid or other normal liquid
        Parameters
        ----------
        mount : str
            Pipette to be used.
        source: 
            position to swell the tip in.
        amount: float or int
            Amount of substance to be moved. (in uL)!!!
        target_wells: 
            Move substance to a list of wells.     
        Returns
        -------
        Nones
        """
        # Set gantry speed (default: X: 600, Y: 400, Z: 100, A: 100)
        self.protocol.max_speeds['X'] = 600
        self.protocol.max_speeds['Y'] = 400
        self.protocol.max_speeds['Z'] = 100
        if mount == 'left':
            pipette = self.left_pipette 
            pipette.flow_rate.aspirate = 94 # default 94 ul/s
            pipette.flow_rate.dispense = 94
        else:
            pipette = self.right_pipette
            pipette.flow_rate.aspirate = 7.56 # default 7.56 ul/s
            pipette.flow_rate.dispense = 7.56 
        pipette.transfer(volume=amount, source=source, dest=target_wells, new_tip=new_tip)

    # Recorded in move_commands['type']
    # need to include one more step to decide the type of the substance 
    def move_substances(self, 
                        substance_name, 
                        amount,
                        labware,
                        slot,
                        target_wells, 
                        mount,
                        new_tip,
                        ):
        """
        amount : float or int
            Amount of substance to be moved. (in uL)!!!
        substance_name : str
            Name of the substance to be moved.
        labware:
            Labware, mostly a plate.
        slot: int
            Deck number of the labware.
        target_wells:
            Move substance to a list of wells. 
        mount: 'left' or 'right'
            The pipette to be used.
        new_tip: str
            Determine if it is necessary to change a tip during liquid transfer('always', 'once' or 'never') 
        """
        try:
            substance_position = self.substances[substance_name][0]["position"]
            solvent_type = self.substances[substance_name][0]["solvent_type"]
        except IndexError:
            pprint(self.substances)
            print(f'Substance{substance_name} not found in {substance_position}.')
            raise RuntimeError(
                f'Substance not found in deck. This could be due to the deck being being empty, or the amount of {substance_name} needed exceeding the amount placed on the deck.'
            ) 
        # Check to see if amount of substance is greater than the amount on the deck
        minimum_volume = 100
        if amount > (
            self.substances[substance_name][0]["amount"] - minimum_volume
        ):
            print(
                f"Amount of {substance_name} needed is greater than the amount on the deck.\n"
                f"Trying again to move after changing the well plate to move from {substance_name}."
            )
            # Change the wellposition to move from
            self.substances[substance_name].pop(0)
            if len(self.substances[substance_name]) == 0:
                raise RuntimeError(
                    f"No more {substance_name} left on the deck. Please check the deck and try again."
                )
        # Load the substance_position and solvent type
        substance_position = self.substances[substance_name][0]['position']
        solvent_type = self.substances[substance_name][0]['solvent_type']
        # Log the transfer action
        print(f"Transferred {amount} uL of '{substance_name}' from {substance_position} to the deck slot {slot}: {labware}{tuple(target_wells)} using {mount} pipette.")
        # Track the amount of substances (updata if pipette was changed)
        if mount == 'left':
            pipette = self.left_pipette 
            # Update the amount of substance
            self.substances[substance_name][0]["amount"] -= amount*8
        else:
            pipette = self.right_pipette
            # Update the amount of substance
            self.substances[substance_name][0]["amount"] -= amount
        # Transfer liquid
        # First class: organic solvent whether volatile or not
        if solvent_type == 'organic':
            self.swell_tip(mount, source=substance_position)
            self.move_without_drip(mount, source=substance_position, amount=amount, target_wells=target_wells)     
        # Second class: viscous liquid
        elif solvent_type == 'oily':
            self.swell_tip(mount, source=substance_position)
            pipette.transfer(volume=amount, source=substance_position, dest=target_wells, new_tip=new_tip)
        # The final class: aqueous liquid
        else:
            # this line is used for to recover the gantry speed and aspirate/dispense flowrate after dealing with organic solvent
            #self.normal_liquid(mount, source=substance_position, amount=amount, targer_wells=target_wells, new_tip=new_tip)
            # normal 'transfer' command used to handle aqueous liquid
            pipette.transfer(volume=amount, source=substance_position, dest=target_wells, new_tip=new_tip)
        

    # Recorded in move_commands['type']
    # capture_images function contains an internal function 'check_file_status'
    def capture_images(self, camera_position, delay_time):
        """
        Parameters
        ----------
        camera_position: a tip_position
            default_position: camera_position, slot 3: D1
                *** put here in case changing camera_position in the future. ***
        delay_time: 
            Keep pipette in the camera_position, slot3: D1 for a specified time.
        Returns
        -------
        None
        """
        pass
        # method to check file connected to AIOhttp server located on the working PC
       # def check_file_status(filename):
       #     """
       #     Check the status of the file via the aiohttp server.
       #     args:
       #         filename (str): The filename of the CSV to check.
       #     Returns:
       #         str: Status of the file ('ready', 'pending', 'not_found').
       #     """
       #     try:
       #         response = requests.get(f'{computer_url}/check_file_status', params={'filename': filename})
       #         response.raise_for_status()
       #         data = response.json()
       #         return data.get('status', 'not_found')
       #     except requests.RequestException as e:
       #         print(f'Error checking file status: {e}')
       #         return 'error'  
       #     
       # # First, use right pipette to fix the camera position
       # pipette = self.right_pipette
       # # Move to camera rack position
       # pipette.pick_up_tip(camera_position)
       # # Trigger the camera
       # response = requests.post(f'{computer_url}/send_message', json={
       #     'message': 'Hello from Opentrons!'
       # })
       # print('sent successfully!')
       # response = requests.post(f'{computer_url}/capture_images')
       # print('trigger successfully!')
       # # Pause the pipette to ensure the camera is fully started and then home pipette
       # print(f'Staying at a fixed position D3 for {delay_time} seconds')
       # self.protocol.delay(seconds=delay_time) 
       # # Start to check file status
       # csv_filename = 'analysis.csv'  
       # status = check_file_status(csv_filename)
       # while status == 'pending':
       #     print('CSV file is still being written to. Waiting...')
       #     time.sleep(5)  # Wait before checking again
       #     status = check_file_status(csv_filename)
       # if status == 'ready':
       #     print('CSV file is ready.')
       # # Return a tip
       # pipette.return_tip()
       # # Home pipette and then continue protocol
       # pipette.home()



    # Recorded in move_commands['type']
    #def use_heater_shaker(self, temperature: float, speed: int, time: int):
    #    """
    #    Set the procedures of using the Heater-Shaker module.
#
    #    Parameters
    #    ----------
    #    temperature : float
    #        Desired temperature in degrees Celsius.
    #    speed : int
    #        Shaking speed in RPM.
    #    time: int
    #        Shake for a specified time
    #    Returns
    #    -------
    #    None
    #    """
    #    ### Safety Information ###
    #    # The labware latch needs to be closed before: 
    #    # 1. Shaking; 
    #    # 2. Pipetting to or from the labware on the Heater-Shaker; 
    #    # 3. Pipetting to or from labware to the left or right of the Heater-Shaker
#
    #    print(self.heater_shaker.serial_number)
    #    print(self.heater_shaker.type)
    #    self.heater_shaker.close_labware_latch() 
#
    #    # Use this line to set temperature     
    #    #self.heater_shaker.set_temperature(temperature)
    #    
    #    # Before shaking, this command will retract the pipettes upward if they are parked adjacent to the Heater-Shaker.
    #    self.heater_shaker.set_and_wait_for_shake_speed(speed)
#
    #    # Shake for a specified time
    #    self.protocol.delay(minutes=time)
    #  
    #    # Stop shaking the plate on the Heater-Shaker module.       
    #    self.heater_shaker.deactivate_shaker() 
    #    
    #    print(f'completed shaking for {time} minute(s) at a speed of {speed} rpm')
    
    # Protect pipettes 
    def apply_offset(self):
        
        print('parsing offset json file')
        # system_path = Path('system_config.json')   
        # if not system_path.is_file():
        #     system_path = Path(r"/var/lib/jupyter/notebooks/Yuan_Li/system_config.json")
        # with open(str(system_path)) as f:
        #     offset_data = json.load(f)
           
        self.tiprack300.set_offset(x = offset_data['tiprack300']['offset']['x'] ,
                       y = offset_data['tiprack300']['offset']['y'], 
                       z = offset_data['tiprack300']['offset']['z'])
        self.camera_rack.set_offset(x = offset_data['camera_rack']['offset']['x'] ,
                        y = offset_data['camera_rack']['offset']['y'], 
                        z = offset_data['camera_rack']['offset']['z'])
        self.reservoir.set_offset(x = offset_data['reservoir']['offset']['x'] ,
                        y = offset_data['reservoir']['offset']['y'], 
                        z = offset_data['reservoir']['offset']['z'])
        self.plate.set_offset(x = offset_data['plate']['offset']['x'] ,
                        y = offset_data['plate']['offset']['y'], 
                        z = offset_data['plate']['offset']['z'])

        
# Run protocol as defined in move_commands.json
def run(protocol: protocol_api.ProtocolContext):

    """
    Execute protocol as defined in move_commands.json
    """   
    
    try:
        # parsing json file(substance.json as deck_info and move_commands.json as move_info)
        print('parsing json file')
        # substances_path = Path ('substances.json')
        # move_path = Path('move_commands.json')
        # if not substances_path.is_file():
        #     substances_path = Path(r"/var/lib/jupyter/notebooks/Yuan_Li/substances.json")       
        #     move_path = Path(r"/var/lib/jupyter/notebooks/Yuan_Li/move_commands.json")    
        # with open(str(substances_path)) as f:
        #     deck_info = json.load(f)
        # with open(str(move_path)) as f:
        #     move_info = json.load(f)
        print('loaded successfully!')

        # Update ot2 deck_info
        ot2 = Opentrons(protocol=protocol, deck_info=deck_info)
        print('successfully update ot2 object!')

        # Apply offset to the opentrons based on the simulate result in opentrons app
        ot2.apply_offset() 
        print('successfully apply offset!')
  
        # Extract list of moves from the move information
        for command in move_info:

            if command['type'] == 'move_substances':
                # Close labware latch in mandotory for the heater_shaker
                ot2.heater_shaker.close_labware_latch()
                # Extract parameters
                substance_name =command['substance_name']
                amount = int(command['amount'])
                labware = command['position_to']['labware']
                slot = int(command['position_to']['slot'])
                wells = command['position_to']['wells']
                mount = command['mount']   
                target_wells = [ot2.labware[slot].wells(well) for well in wells]
                pipette_max_volume = 300            
                #if mount  == 'left':
                #    pipette_max_volume = 300
                #else:
                #    pipette_max_volume = 20
                added_substances = []
                positions_added = defaultdict(str)
                amount_added = defaultdict(lambda: 0)
                num_of_transfer = math.ceil(amount/pipette_max_volume)
                added = 0
                new_tip = 'once'
                if len(added_substances) == 0:    
                    added_substances.append(substance_name) 
                # Check if substance matches the last substance added
                elif substance_name  != added_substances[-1]:
                    new_tip = 'always'
                for i in range(num_of_transfer):
                    # Check if move is the last move
                    if i == num_of_transfer - 1 :
                        amount_to_add = amount - added
                    else:
                        amount_to_add = pipette_max_volume
                    # Execute the move
                    ot2.move_substances(substance_name=substance_name, amount=amount_to_add, labware=labware, slot=slot, target_wells=target_wells, mount=mount, new_tip=new_tip)
                    # Track the move
                    added += amount_to_add
                    amount_added['slot'] = slot
                    if mount == 'left':
                        amount_added['type'] = 'column'
                        amount_added[tuple(wells)] += amount_to_add 
                    else:
                        amount_added['type'] = 'wells'
                        amount_added[tuple(wells)] += amount_to_add 
                positions_added[slot] = slot
                if mount == 'left':
                    positions_added['type'] = 'column'
                    positions_added[tuple(wells)] += substance_name + " "
                else:
                    positions_added['type'] = 'wells'
                    positions_added[tuple(wells)] += substance_name + " "
                # This loop iterates each item in ot2.protocol, but does nothing with them because of the 'continue' statement
                for line in ot2.protocol.commands():
                    continue
                # Track the progress of the move_commands
                pprint(amount_added)
                pprint(positions_added)

            #if command['type'] == 'use_heater_shaker':
            #    # Extract parameters
            #    temperature = command.get('temperature')
            #    shake_speed = command.get('shake_speed')
            #    shake_time = command.get('shake_time')
            #    # Execute the move
            #    ot2.use_heater_shaker(temperature,shake_speed,shake_time) 
        #
            #if command['type'] == 'capture_images':
            #    # Close labware latch in mandotory for the heater_shaker
            #    ot2.heater_shaker.close_labware_latch()
            #    # Extract parameters
            #    labware = command['position_to']['labware']
            #    slot = int(command['position_to']['slot'])
            #    wells = command['position_to']['wells']
            #    camera_position = [ot2.labware[slot].wells(well) for well in wells]         
            #    delay_time = command['dealy_time']
            #    # Execute the move
            #    ot2.capture_images(camera_position, delay_time)       
    except KeyboardInterrupt:
        print('interrupted')
        print('start home!')
        protocol.home()
        print('robot homed!')
        print('check if tips attached to the pipette')
        instruments = {}
        instruments = protocol.loaded_instruments
        if 'right' in instruments:
            print("Instrument on the right mount:", instruments['right'])
            pipette = instruments['right']
            if pipette.has_tip:
                pipette.drop_tip()
        else:
            print("No instrument on the right mount")
        if 'left' in instruments:
            print("Instrument on the left mount:", instruments['left'])
            pipette =instruments['left']
            if pipette.has_tip:
                pipette.drop_tip()
        else:
            print("No instrument on the left mount")
        print('Done!')
        del protocol
