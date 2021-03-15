
"""
    Data Testing & Updating

    This script contains the test_data function and is used to rewrite the data.

    It makes 3 main changes:
    - Removing false iterations where one or more drones are not properly working.
    - Between iterations, drones go to their new initial location.
    if this part was recorded in the data, this script deletes it.
    - if a collision has happened, and it affects the drone's behaviour,
    the iteration is removed.

    After making these changes, the data is rewriten to the input file.

    Parameters: 
        input_file_name (str) - path to the data file
"""

import csv
import pandas as pd

def test_data(input_file_name):

    # headers for csv file
    data_keys = ['drone', 'update_step', 'iter', 'script_time', 'sim_time', 'latitude', 'longitude', 'altitude',
                'angular_acceleration_x','angular_acceleration_y', 'angular_acceleration_z',
                'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
                'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
                'orientation_x', 'orientation_y', 'orientation_z',
                'position_x', 'position_y', 'position_z', 'collision', 'label']

    data = {data: [] for data in data_keys}

    # Get the original data from the input file
    old_data = pd.read_csv(input_file_name)

    num_of_iter = old_data.iter.max() + 1

    with open(input_file_name) as csvfile:
        # Read the data file
        csv_reader = csv.reader(csvfile, delimiter=',')

        # The original reader_by_iterations contains the original iterations
        reader_by_iterations = []
        [reader_by_iterations.append([]) for i in range(num_of_iter)]
        # The updated reader_by_iterations contains only the valid iterations
        reader_by_iterations_updated = []

        # Set the data in a list of iterations 
        for row in csv_reader:
            if row[0] != 'drone':
                # row[2] is the iteration number
                reader_by_iterations[int(row[2])].append(row)  

        # Part 1 - Remove false iterations
        for inum, iteration in enumerate(reader_by_iterations):
            # Check if all drones are moving.
            # If not, it is a false iteration and should be deleted
            is_moving = [False, False, False, False, False]
            try:
                # Check if the drones moved in the first 50 lines of data
                for i in range(50):
                    row = iteration[i]
                    # row[8] is the angular acceleration on the x axis
                    # If the drone is moving, its value is not 0.0
                    is_moving[i % 5] = float(row[8]) != 0.0

                # If all drones moved, keep the iteration
                if False not in is_moving:
                    reader_by_iterations_updated.append(iteration)
                else:
                    print(f"removing: {inum}")
            except:
                # If there was a major problem and the iteration was not recorded
                # (Should not happen)
                print(f"skipping iter {inum}")

        # Part 2 - Remove data at the end of iteration which should have not be recorded
        # If there is a dramatic change in the drones' location, then it is a part of their
        # relocation as a preparation to the next iteration. this part should be deleted.

        #counting the valid iterations
        iteration_count = 0

        for iteration in reader_by_iterations_updated:
            # If there was a change - save the row where the change has happened
            change_row = 0 
            # Save the previous location
            pre_vals = {"x": 0, "y": 0}

            # Check the 50 last rows
            for i in range(50):
                row = iteration[len(iteration) - 51 + i] 
                # row[23] - the drone's position on the x axis
                curr_x = float(row[23])
                # row[24] - the drone's position on the y axis
                curr_y = float(row[24])

                if i > 0:
                    # If the drone "jumped" - made in 0.05 seconds a distance longer than 2 meters,
                    # Then this part should have not been recorded
                    if abs(curr_x - pre_vals["x"]) > 2 or abs(curr_y - pre_vals["y"]) > 2:
                        # Save the row where the jump happened
                        change_row = len(iteration) - 51 + i 

                pre_vals["x"] = curr_x 
                pre_vals["y"] = curr_y 
            
            # Part 3 - if a collision has happened, and it affects the drone's behaviour,
            # the iteration is removed
            collided_drones = []
            # Count the rows where a collision has happened 
            [collided_drones.append(0) for i in range (5)]

            for row in iteration:
                # row[26] - collision label (1 if collision is occurring at this moment)
                collision = int(row[26])
                curr_drone = int(row[0][-1:]) - 1
                if collision:
                    # Count the collision rows
                    collided_drones[curr_drone] += 1

            
            # If a drone has collided for more than 10 rows, the iteration should be removed
            is_collision = False
            for drone_i in range(5):
                if collided_drones[drone_i] >= 10:
                    is_collision = True
            
            # if a collision didn't happen
            if not is_collision:
                # Writing all the updated data 
                for j, row in enumerate(iteration):
                    # Get drone index
                    curr_drone = int(row[0][-1:]) - 1

                    # Change the iteration index according the updated iteration list 
                    row[2] = iteration_count
                    
                    # If the moments between iterations were recorded, do not rewrite these lines
                    if change_row == 0:
                        [data[key].append(val) for key, val in zip(data.keys(), row)]
                    else:
                        if j < change_row:
                            [data[key].append(val) for key, val in zip(data.keys(), row)]
                iteration_count += 1

    print("finished processing")

    # Write the data to the file
    data_df = pd.DataFrame.from_dict(data)
    data_df.to_csv(input_file_name, index=False)
