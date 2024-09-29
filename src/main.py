# Import the necessary modules from the VEXcode IQ Python API
from vex import *
import sys

# Initialize the Brain
brain = Brain()

# Initialize Motors
drive_motor = Motor(Ports.PORT1, False)  # Single drive motor #TODO: Assign ports
extension_motor = Motor(Ports.PORT2, False)#TODO: Assign ports
arm_motor = Motor(Ports.PORT3, False)#TODO: Assign ports
gripper_motor = Motor(Ports.PORT4, False)#TODO: Assign ports

# Initialize Sensors
distance_sensor = Distance(Ports.PORT5)#TODO: Assign ports
optical_sensor = Optical(Ports.PORT6)#TODO: Assign ports
bumper_sensor = Bumper(Ports.PORT7)#TODO: Assign ports

# Variables to store block data and side profile
side_profile = []
smoothed_profile = []
block_positions = []
rescan_positions = {}
blocks_to_collect = []

# Global variables
motor_position_offset = 0  # To account for the initial motor position

# Constants
GEAR_RADIUS = 20  # Gear radius in millimeters (2 cm) #TODO: measure
GEAR_CIRCUMFERENCE = 2 * 3.1416 * GEAR_RADIUS  # Circumference in mm
DISTANCE_PER_DEGREE = GEAR_CIRCUMFERENCE / 360  # Distance per degree of motor rotation in mm
MEASUREMENT_INTERVAL = 10  # Take measurements every 10 mm
DISTANCE_THRESHOLD = 50  # Threshold in mm for significant distance change (adjustable)
BLOCK_WIDTH_MIN = 50  # Minimum block width in mm TODO: measure block
BLOCK_WIDTH_MAX = 100  # Maximum block width in mm TODO: measure block
MAX_RESCANS_PER_POSITION = 2  # Maximum number of rescans per position
DISTANCE_DIFFERENCE_THRESHOLD = 30  # Threshold for distinguishing blocks by distance
WALL_DISTANCE_THRESHOLD = 50  # Threshold to detect the wall (adjust as needed)
MAX_STACK_HEIGHT = 3  # Maximum number of blocks in a stack

# Motor direction variables TODO check directions
ARM_MOTOR_DIRECTION = 1        # Positive when lowering the arm
EXTENSION_MOTOR_DIRECTION = 1  # Positive when extending the mechanism
GRIPPER_MOTOR_DIRECTION = 1    # Positive when closing the gripper
DRIVE_MOTOR_DIRECTION = -1     # Positive when driving backwards

# Stacking positions (x, y coordinates)
STACK_OFFSET_X = 50  # Offset along the track in mm TODO: check distance between stacks
STACK_OFFSET_Y = 50  # Offset away from the train in mm TODO: check distance between stacks

# Generate stacking positions for red blocks (4 positions) TODO: assign reasonable stack positions
stacking_positions_red = [
    {'x': -200, 'y': STACK_OFFSET_Y},                             # Red Stack 1
    {'x': -200 - STACK_OFFSET_X, 'y': STACK_OFFSET_Y},            # Red Stack 2
    {'x': -200, 'y': STACK_OFFSET_Y + STACK_OFFSET_Y},            # Red Stack 3
    {'x': -200 - STACK_OFFSET_X, 'y': STACK_OFFSET_Y + STACK_OFFSET_Y}  # Red Stack 4
]

# Generate stacking positions for green blocks (4 positions) TODO: assign reasonable stack positions
stacking_positions_green = [
    {'x': -200, 'y': -STACK_OFFSET_Y},                            # Green Stack 1
    {'x': -200 - STACK_OFFSET_X, 'y': -STACK_OFFSET_Y},           # Green Stack 2
    {'x': -200, 'y': -STACK_OFFSET_Y - STACK_OFFSET_Y},           # Green Stack 3
    {'x': -200 - STACK_OFFSET_X, 'y': -STACK_OFFSET_Y - STACK_OFFSET_Y}  # Green Stack 4
]

# Stack levels for each stacking position
stack_levels_red = [0, 0, 0, 0]    # For red blocks
stack_levels_green = [0, 0, 0, 0]  # For green blocks

# Function to drive the robot forward
def drive_forward(speed):
    drive_motor.spin(FORWARD if DRIVE_MOTOR_DIRECTION > 0 else REVERSE, abs(speed), PERCENT)

# Function to stop the robot
def stop_driving():
    drive_motor.stop()

# Function to check for sufficient clearance at the block's position using side profile data
def check_clearance(block_position, block_distance):
    REQUIRED_WIDTH = 150  # Mechanism width in mm (15 cm)
    SAFE_DISTANCE_BEFORE_BLOCK = 30  # Stop 3 cm before the block

    # Calculate the required clearance distance
    required_clearance_distance = block_distance - SAFE_DISTANCE_BEFORE_BLOCK

    # Get the positions in side_profile within the width of the mechanism
    positions_to_check = [
        data for data in side_profile
        if abs(data['position'] - block_position) <= (REQUIRED_WIDTH / 2)
    ]

    if not positions_to_check:
        brain.screen.print(f"No side profile data near block position {block_position}.")
        return False  # Cannot determine clearance

    # Check for obstacles at each position
    for data in positions_to_check:
        distance_measurement = data['distance']
        if distance_measurement < required_clearance_distance:
            brain.screen.print(f"Obstacle detected at position {data['position']}. Distance: {distance_measurement} mm")
            return False  # Path is blocked

    # No obstacles detected within the corridor
    return True

# Function to map the side profile
def map_side_profile():
    global side_profile, motor_position_offset
    side_profile = []
    brain.screen.print("Mapping side profile...")

    # Record the initial motor position
    initial_motor_position = drive_motor.position(DEGREES)
    motor_position_offset = 0  # Will be set when wall is detected

    last_measurement_position = -MEASUREMENT_INTERVAL  # Ensure the first measurement is taken at position 0

    wall_detected = False

    while True:
        # Check if bumper sensor is pressed (end of track)
        if bumper_sensor.pressing():
            brain.screen.print("End of track detected by bumper sensor.")
            break  # Exit the mapping loop

        # Calculate current position in mm
        motor_degrees = drive_motor.position(DEGREES) - initial_motor_position
        current_position = motor_degrees * DISTANCE_PER_DEGREE  # Position in mm

        # Measure distance to the side
        distance = distance_sensor.object_distance(MM)

        # Detect wall
        if not wall_detected and distance > 0 and distance < WALL_DISTANCE_THRESHOLD:
            # Wall detected
            motor_position_offset = drive_motor.position(DEGREES)  # Set motor position offset at the wall
            brain.screen.print("Wall detected. Zero position set.")
            wall_detected = True
            # Continue mapping after wall
            last_measurement_position = 0
            # Skip adding wall distance to side_profile
            continue

        if wall_detected:
            # Positions are relative to the wall
            motor_degrees = drive_motor.position(DEGREES) - motor_position_offset
            current_position = motor_degrees * DISTANCE_PER_DEGREE  # Position in mm

            # Check if it's time to take a new measurement
            if current_position - last_measurement_position >= MEASUREMENT_INTERVAL:
                if distance > 0 and distance < 2000:  # Valid distance readings
                    # Store the position and distance
                    side_profile.append({'position': current_position, 'distance': distance})
                last_measurement_position = current_position  # Update last measurement position

        else:
            # Before wall is detected, keep moving forward
            pass

        drive_forward(20)  # Move forward at 20% speed
        wait(0.05, SECONDS)  # Small delay to prevent overloading the CPU

    stop_driving()
    brain.screen.print("Side profile mapping complete.")

# Function to analyze the side profile and calculate block positions
def analyze_side_profile():
    global block_positions, smoothed_profile
    block_positions = []
    brain.screen.print("Analyzing side profile...")

    smoothed_profile = smooth_distances(side_profile)

    i = 1  # Start from the second measurement
    while i < len(smoothed_profile) - 1:
        previous_data = smoothed_profile[i - 1]
        current_data = smoothed_profile[i]

        previous_distance = previous_data['distance']
        current_distance = current_data['distance']

        # Detect significant negative change (leading edge)
        if (previous_distance - current_distance) > DISTANCE_THRESHOLD:
            leading_edge_position = current_data['position']
            leading_edge_distance = current_distance
            # Look for significant positive change (trailing edge)
            trailing_edge_found = False
            j = i + 1
            while j < len(smoothed_profile):
                future_data = smoothed_profile[j]
                future_distance = future_data['distance']
                future_position = future_data['position']

                # Check if there is another leading edge before a trailing edge
                if (current_distance - future_distance) > DISTANCE_THRESHOLD:
                    # Found another leading edge before trailing edge
                    i = j - 1  # Set i to one before the new leading edge
                    trailing_edge_found = True
                    break  # Exit the inner loop to process new leading edge
                elif (future_distance - leading_edge_distance) > DISTANCE_THRESHOLD:
                    # Found trailing edge
                    trailing_edge_position = future_position
                    trailing_edge_distance = future_distance
                    block_width = trailing_edge_position - leading_edge_position
                    # Check if block width is within expected range
                    if BLOCK_WIDTH_MIN <= block_width <= BLOCK_WIDTH_MAX:
                        # Calculate middle position of the block
                        block_position = (leading_edge_position + trailing_edge_position) / 2
                        block_distance = (leading_edge_distance + trailing_edge_distance) / 2
                        # Record block position if not already in the list
                        if not any(
                            abs(pos['position'] - block_position) < MEASUREMENT_INTERVAL and
                            abs(pos['distance'] - block_distance) < DISTANCE_DIFFERENCE_THRESHOLD
                            for pos in block_positions
                        ):
                            block_positions.append({
                                'position': block_position,
                                'distance': block_distance,
                                'leading_edge_position': leading_edge_position,
                                'trailing_edge_position': trailing_edge_position,
                                'collected': False,
                                'color': 'unknown'  # Initialize color as unknown
                            })
                            rescan_positions[block_position] = 0  # Initialize rescan count
                    # Move i to the index after trailing edge
                    i = j
                    trailing_edge_found = True
                    break  # Exit the inner loop
                else:
                    # No significant change, continue searching
                    j += 1
            if not trailing_edge_found:
                # Trailing edge not found; discard this block and move to next leading edge
                i += 1
        else:
            i += 1
    brain.screen.print("Block analysis complete.")

# Function to smooth distance readings using a simple moving average
def smooth_distances(profile_data, window_size=3):
    smoothed = []
    for i in range(len(profile_data)):
        start_index = max(0, i - window_size // 2)
        end_index = min(len(profile_data), i + window_size // 2 + 1)
        window = profile_data[start_index:end_index]
        avg_distance = sum(d['distance'] for d in window) / len(window)
        smoothed.append({'position': profile_data[i]['position'], 'distance': avg_distance})
    return smoothed

# Function to get the color of a block using the optical sensor TODO: check if correct color is detected
def get_block_color():
    optical_sensor.set_light(100)
    wait(0.2, SECONDS)
    detected_color = optical_sensor.color()
    optical_sensor.set_light(0)
    if detected_color == Color.RED:
        return 'red'
    elif detected_color == Color.GREEN:
        return 'green'
    else:
        return 'unknown'

# Function to move the robot to a specific position
def move_to_position(target_position):
    # Calculate current position in mm
    motor_degrees = drive_motor.position(DEGREES) - motor_position_offset
    current_position = motor_degrees * DISTANCE_PER_DEGREE  # Position in mm

    distance_to_move = target_position - current_position  # Distance to move in mm
    degrees_to_move = distance_to_move / DISTANCE_PER_DEGREE  # Convert distance to degrees

    if degrees_to_move > 0:
        drive_motor.spin_for(FORWARD if DRIVE_MOTOR_DIRECTION > 0 else REVERSE, degrees_to_move, DEGREES, wait=True)
    else:
        drive_motor.spin_for(REVERSE if DRIVE_MOTOR_DIRECTION > 0 else FORWARD, -degrees_to_move, DEGREES, wait=True)

# Function to extend the mechanism to measure color
def extend_to_measure_color():
    desired_distance = 50  # Distance from block to get color reading (adjust as needed)
    extension_motor.set_velocity(50, PERCENT)
    extension_motor.spin(FORWARD if EXTENSION_MOTOR_DIRECTION > 0 else REVERSE)
    while True:
        distance = distance_sensor.object_distance(MM)
        if 0 < distance <= desired_distance:
            break
        wait(0.05, SECONDS)
    # Stop the motor and hold position
    extension_motor.stop(HOLD)
    return True  # Indicate successful extension

# Function to retract the mechanism
def retract_mechanism():
    # Retract the extension motor to position 0
    extension_motor.spin_to_position(0, DEGREES, wait=True)
    extension_motor.stop(HOLD)

# Function to map block colors
def map_block_colors():
    brain.screen.print("Mapping block colors...")
    for block in block_positions:
        move_to_position(block['position'])
        # Check for clearance before attempting to extend
        if check_clearance(block['position'], block['distance']):
            # Extend mechanism to measure color
            if extend_to_measure_color():
                color = get_block_color()
                block['color'] = color
                retract_mechanism()
            else:
                brain.screen.print("Cannot extend mechanism to measure color.")
                block['color'] = 'unknown'
        else:
            brain.screen.print("Not enough clearance to measure block color.")
            block['color'] = 'unknown'
    brain.screen.print("Block color mapping complete.")

# Function to collect blocks
def collect_blocks():
    global blocks_to_collect
    brain.screen.print("Collecting blocks...")
    # Initialize blocks_to_collect with blocks that are not collected
    blocks_to_collect = [block for block in block_positions if not block['collected']]
    while any(not block['collected'] for block in blocks_to_collect):
        # Prioritize red blocks
        blocks_to_collect = sorted(blocks_to_collect, key=lambda x: x.get('color') != 'red')
        for block in blocks_to_collect:
            if not block['collected']:
                move_to_position(block['position'])
                # Check for clearance before attempting to extend
                if not check_clearance(block['position'], block['distance']):
                    brain.screen.print("Not enough clearance to collect block.")
                    # Do not mark as collected; retry later
                    continue
                if extend_mechanism():
                    if block['color'] == 'unknown':
                        # Attempt to get color if not known
                        if extend_to_measure_color():
                            color = get_block_color()
                            block['color'] = color
                            retract_mechanism()
                        else:
                            brain.screen.print("Cannot extend mechanism to measure color.")
                            retract_mechanism()
                            continue  # Retry later
                    if block['color'] != 'unknown':
                        grab_block()
                        block['collected'] = True
                        retract_mechanism()
                        return_to_start()
                        stack_block(block['color'])
                    else:
                        # Unable to determine color; skip block for now
                        retract_mechanism()
                        continue  # Retry later
                    # Rescan the position for additional blocks
                    rescan_count = rescan_positions.get(block['position'], 0)
                    if rescan_count < MAX_RESCANS_PER_POSITION:
                        rescan_positions[block['position']] = rescan_count + 1
                        rescan_block_area(block)
                else:
                    # Unable to extend mechanism; skip block for now
                    # Do not mark as collected; retry later
                    continue
    brain.screen.print("Block collection complete.")

# Function to extend the mechanism for block collection TODO: adjust extension velocities according to distance
def extend_mechanism():
    desired_distance = 20  # Target distance in mm (2 cm)
    extension_motor.set_velocity(100, PERCENT)
    extension_motor.spin(FORWARD if EXTENSION_MOTOR_DIRECTION > 0 else REVERSE)

    while True:
        distance = distance_sensor.object_distance(MM)
        if 0 < distance <= desired_distance:
            break
        elif 0 < distance <= 50:
            # Slow down when close to the block
            extension_motor.set_velocity(20, PERCENT)
        elif 50 < distance <= 100:
            # Moderate speed when approaching
            extension_motor.set_velocity(50, PERCENT)
        else:
            # Full speed when far from the block
            extension_motor.set_velocity(100, PERCENT)
        wait(0.05, SECONDS)

    # Stop the motor and hold position
    extension_motor.stop(HOLD)
    return True  # Indicate successful extension

# Function to rescan the area around a block's position
def rescan_block_area(block):
    global side_profile, smoothed_profile, blocks_to_collect
    brain.screen.print(f"Rescanning area around position {block['position']}...")
    # Retract the mechanism if not already retracted
    retract_mechanism()
    # Define rescan range
    rescan_start = block['leading_edge_position'] - 25  # Subtract 2.5 cm TODO: select reasonable additional range for rescanning
    rescan_end = block['trailing_edge_position'] + 25    # Add 2.5 cm

    # Ensure rescan_start and rescan_end are within the bounds of side_profile
    rescan_start = max(0, rescan_start)
    rescan_end = min(side_profile[-1]['position'], rescan_end)

    # Move to rescan_start position
    move_to_position(rescan_start)

    # Initialize variables for continuous motion
    drive_motor.set_velocity(20, PERCENT)
    drive_motor.spin(FORWARD if DRIVE_MOTOR_DIRECTION > 0 else REVERSE)

    # Initialize variables for measurement
    last_measurement_position = rescan_start - MEASUREMENT_INTERVAL  # Ensure first measurement is taken
    new_side_profile_data = []

    while True:
        # Calculate current position
        motor_degrees = drive_motor.position(DEGREES) - motor_position_offset
        current_position = motor_degrees * DISTANCE_PER_DEGREE

        # Check if we've reached rescan_end
        if current_position >= rescan_end:
            break

        # Check if it's time to take a new measurement
        if current_position - last_measurement_position >= MEASUREMENT_INTERVAL:
            # Measure distance
            distance = distance_sensor.object_distance(MM)
            if 0 < distance < 2000:  # Valid readings
                new_side_profile_data.append({'position': current_position, 'distance': distance})
            last_measurement_position = current_position  # Update last measurement position

        wait(0.05, SECONDS)

    # Stop the motor after rescanning
    stop_driving()

    # Remove old data in the rescan range from side_profile
    side_profile = [data for data in side_profile if data['position'] < rescan_start or data['position'] > rescan_end]

    # Add new data to side_profile
    side_profile.extend(new_side_profile_data)

    # Keep side_profile sorted by position
    side_profile.sort(key=lambda x: x['position'])

    # Re-smooth the entire side_profile
    smoothed_profile = smooth_distances(side_profile)

    # Perform cube detection only on the rescanned part
    # Find the indices corresponding to rescan_start and rescan_end
    start_index = next((i for i, d in enumerate(smoothed_profile) if d['position'] >= rescan_start), 0)
    end_index = next((i for i, d in enumerate(smoothed_profile) if d['position'] > rescan_end), len(smoothed_profile))

    # Analyze the rescanned portion
    new_blocks = analyze_smoothed_profile(start_index, end_index)

    # For any new blocks
    for new_block in new_blocks:
        # Check if block is already collected or in the list
        if not any(
            abs(pos['position'] - new_block['position']) < MEASUREMENT_INTERVAL and
            abs(pos['distance'] - new_block['distance']) < DISTANCE_DIFFERENCE_THRESHOLD
            for pos in block_positions
        ):
            # Immediately get the color of the new block
            move_to_position(new_block['position'])
            if check_clearance(new_block['position'], new_block['distance']):
                if extend_to_measure_color():
                    color = get_block_color()
                    new_block['color'] = color
                    retract_mechanism()
                else:
                    brain.screen.print("Cannot extend mechanism to measure color.")
                    new_block['color'] = 'unknown'
            else:
                brain.screen.print("Not enough clearance to measure block color.")
                new_block['color'] = 'unknown'

            new_block['collected'] = False
            block_positions.append(new_block)
            blocks_to_collect.append(new_block)

    # Return to original block position
    move_to_position(block['position'])

# Function to analyze the smoothed profile within a specific index range
def analyze_smoothed_profile(start_index, end_index):
    new_blocks = []
    i = start_index
    while i < end_index - 1:
        previous_data = smoothed_profile[i - 1] if i > 0 else smoothed_profile[i]
        current_data = smoothed_profile[i]
        previous_distance = previous_data['distance']
        current_distance = current_data['distance']
        # Detect significant negative change (leading edge)
        if (previous_distance - current_distance) > DISTANCE_THRESHOLD:
            leading_edge_position = current_data['position']
            leading_edge_distance = current_distance
            # Look for significant positive change (trailing edge)
            trailing_edge_found = False
            j = i + 1
            while j < end_index:
                future_data = smoothed_profile[j]
                future_distance = future_data['distance']
                future_position = future_data['position']
                if (future_distance - leading_edge_distance) > DISTANCE_THRESHOLD:
                    # Found trailing edge
                    trailing_edge_position = future_position
                    trailing_edge_distance = future_distance
                    block_width = trailing_edge_position - leading_edge_position
                    if BLOCK_WIDTH_MIN <= block_width <= BLOCK_WIDTH_MAX:
                        block_position = (leading_edge_position + trailing_edge_position) / 2
                        block_distance = (leading_edge_distance + trailing_edge_distance) / 2
                        new_blocks.append({
                            'position': block_position,
                            'distance': block_distance,
                            'leading_edge_position': leading_edge_position,
                            'trailing_edge_position': trailing_edge_position
                        })
                    trailing_edge_found = True
                    i = j
                    break
                else:
                    j += 1
            if not trailing_edge_found:
                i += 1
        else:
            i += 1
    return new_blocks

# Function to grab a block using the gripper
def grab_block():
    # Lower the arm to grab position
    lower_arm_to_grab()
    # Set maximum torque to 0.5 Nm
    gripper_motor.set_max_torque(0.5, TorqueUnits.NM)
    # Close gripper until it reaches the torque limit
    gripper_motor.spin(FORWARD if GRIPPER_MOTOR_DIRECTION > 0 else REVERSE)
    while gripper_motor.torque(TorqueUnits.NM) < 0.5:
        wait(0.05, SECONDS)
    gripper_motor.stop(HOLD)  # Maintain pressure on the block
    # Lift the arm back to initial position
    arm_motor.spin_to_position(0, DEGREES, wait=True)
    arm_motor.stop(HOLD)

# Function to lower the arm to grab position
def lower_arm_to_grab():
    # Lower the arm to the position for grabbing a block
    grab_position = 150  # Adjust as needed (arm lowered to pick up block) TODO: get the degrees the motor needs to turn for grabing a block
    arm_motor.spin_to_position(grab_position * ARM_MOTOR_DIRECTION, DEGREES, wait=True)
    arm_motor.stop(HOLD)

# Function to lift the arm to stacking position while above
def lift_arm_to_stack_position_above(level):
    # Define arm positions for each level (calibrated during testing)
    ARM_POSITIONS_ABOVE = {
        1: 100,  # degrees above stacking level 1 TODO get the correct degrees the arm needs to be lifted in order to stack them to a specific level
        2: 80,   # degrees above stacking level 2
        3: 60    # degrees above stacking level 3
    }
    position = ARM_POSITIONS_ABOVE.get(level, 100)
    arm_motor.set_velocity(100, PERCENT)
    arm_motor.spin_to_position(position * ARM_MOTOR_DIRECTION, DEGREES, wait=True)
    arm_motor.stop(HOLD)

# Function to lower arm to stack with controlled speed
def lower_arm_to_stack_with_control(level):
    # Define arm positions for each level (calibrated during testing)
    ARM_POSITIONS = {
        1: 150,  # degrees for stacking level 1 TODO: get correct degrees to release a block on the stack
        2: 130,  # degrees for stacking level 2
        3: 110   # degrees for stacking level 3
    }
    target_position = ARM_POSITIONS.get(level, 150) * ARM_MOTOR_DIRECTION
    current_position = arm_motor.position(DEGREES)
    # Set lower velocity as approaching target
    arm_motor.set_velocity(50, PERCENT)
    arm_motor.spin_to_position(target_position, DEGREES, wait=True)
    arm_motor.stop(HOLD)

# Function to adjust extension for stacking based on stack level
def adjust_extension_for_stack(level, stacking_position):
    if level == 1:
        # For level one, use a predefined extension position based on the stack's distance from the track
        # Stacks further from the track require more extension
        # Calculate extension based on the absolute value of y-coordinate
        y_distance = abs(stacking_position['y'])  # Distance from track in mm

        # Define base extension positions (adjust these values based on calibration)
        BASE_EXTENSION_NEAR = 200  # Extension position for stacks closer to the track TODO check base extensions and also maybe stack positions
        BASE_EXTENSION_FAR = 250   # Extension position for stacks further from the track

        # Determine which base extension to use
        if y_distance <= STACK_OFFSET_Y:
            # Closer to the track
            predefined_extension_position = BASE_EXTENSION_NEAR
        else:
            # Further from the track
            predefined_extension_position = BASE_EXTENSION_FAR

        # Extend to the calculated position
        extension_motor.spin_to_position(predefined_extension_position * EXTENSION_MOTOR_DIRECTION, DEGREES, wait=True)
        extension_motor.stop(HOLD)
    else:
        # For levels two and three, use the distance sensor
        # Define desired distances for each level
        DESIRED_DISTANCES = {
            2: 50,  # 50 mm for level 2 TODO: measure needed distance for different stacking levels (distance between extension and blockstack)
            3: 70   # 70 mm for level 3
        }
        desired_distance = DESIRED_DISTANCES.get(level, 50)  # Default to 50 mm if level not found

        # Extend the mechanism until the distance sensor reads the desired distance
        extension_motor.set_velocity(50, PERCENT)
        extension_motor.spin(FORWARD if EXTENSION_MOTOR_DIRECTION > 0 else REVERSE)
        while True:
            distance = distance_sensor.object_distance(MM) #TODO adjust approach velocities
            if 0 < distance <= desired_distance:
                break
            elif distance > desired_distance + 20:
                # Move faster when far from desired distance
                extension_motor.set_velocity(100, PERCENT)
            elif desired_distance < distance <= desired_distance + 20:
                # Slow down as we approach desired distance
                extension_motor.set_velocity(50, PERCENT)
            else:
                # Distance sensor may not have valid reading
                # Implement timeout or error handling if necessary
                pass
            wait(0.05, SECONDS)
        # Stop the motor and hold position
        extension_motor.stop(HOLD)

# Update 'release_block' function to pass 'stacking_position' to 'adjust_extension_for_stack'
def release_block(stack_level, stacking_position):
    # Lift the arm to stacking position above
    lift_arm_to_stack_position_above(stack_level)
    # Move to stacking position with arm raised
    adjust_extension_for_stack(stack_level, stacking_position)
    # Lower the arm with controlled speed
    lower_arm_to_stack_with_control(stack_level)
    # Open gripper to release block
    gripper_motor.spin_to_position(0, DEGREES, wait=True)
    gripper_motor.stop()
    # Lift arm back up before retracting
    lift_arm_to_stack_position_above(stack_level)
    # Retract extension
    extension_motor.spin_to_position(0, DEGREES, wait=True)
    extension_motor.stop(HOLD)
    # Return arm to initial position
    arm_motor.spin_to_position(0, DEGREES, wait=True)
    arm_motor.stop(HOLD)

# Update 'stack_block' function to pass 'stacking_position' to 'release_block'
def stack_block(color):
    if color == 'red':
        global stack_levels_red #TODO check and adjust stacking logic
        # Find the next available stacking position for red blocks
        for index, level in enumerate(stack_levels_red):
            if level < MAX_STACK_HEIGHT:
                stack_levels_red[index] += 1
                stack_level = stack_levels_red[index]
                stacking_position = stacking_positions_red[index]
                brain.screen.print(f"Stacking red block at position {index + 1}, level {stack_level}...")
                # Move to stacking position
                move_to_stacking_position(stacking_position)
                # Perform stacking
                release_block(stack_level, stacking_position)
                # Return to starting position
                return_to_start()
                return
        brain.screen.print("Red stacks are full. Cannot stack more red blocks.")
    elif color == 'green':
        global stack_levels_green
        # Find the next available stacking position for green blocks
        for index, level in enumerate(stack_levels_green):
            if level < MAX_STACK_HEIGHT:
                stack_levels_green[index] += 1
                stack_level = stack_levels_green[index]
                stacking_position = stacking_positions_green[index]
                brain.screen.print(f"Stacking green block at position {index + 1}, level {stack_level}...")
                # Move to stacking position
                move_to_stacking_position(stacking_position)
                # Perform stacking
                release_block(stack_level, stacking_position)
                # Return to starting position
                return_to_start()
                return
        brain.screen.print("Green stacks are full. Cannot stack more green blocks.")
    else:
        brain.screen.print("Unknown block color; cannot stack.")
        return

# Update 'move_to_stacking_position' function if necessary
def move_to_stacking_position(stacking_position):
    # Move along x-axis (train track)
    move_to_position(stacking_position['x'])
    # Orientation or lateral adjustments can be added here if needed


# Function to return to the starting position
def return_to_start():
    # Lift arm to initial position if not already
    arm_motor.spin_to_position(0, DEGREES, wait=True)
    arm_motor.stop(HOLD)
    # Move back to position 0
    move_to_position(0)

# Main function
def main():
    # Initial mapping
    map_side_profile()
    analyze_side_profile()
    # Map block colors after analysis and rescanning
    map_block_colors()
    # Collect blocks using the mapped colors
    collect_blocks()
    # Process complete
    brain.screen.print("Process complete.")

# Start the program
main()
