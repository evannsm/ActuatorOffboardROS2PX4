import os
import csv
import numpy as np

class Logger:

    def __init__(self, filename):
        self.filename = filename[0]

        base_path = os.path.dirname(os.path.abspath(__file__))  # Get the script's directory        
        base_path = os.path.join(base_path, 'data_analysis/') # Append the 'data_analysis' folder to the path
        parts = base_path.split(os.sep)        # Split the path into components
        parts = ["src" if part == "build" else part for part in parts]        # Replace 'build' with 'src' if it exists in the path
        base_path = os.sep.join(parts)        # Reconstruct the new path

        print(f"logger {base_path = }")        # Print the base path
        self.filename = filename[0]        # Assuming 'filename' is passed or defined as a list
        self.full_path = os.path.join(base_path, self.filename)        # Combine the base path with the filename
        print(f"Logging to: {self.full_path}")        # Print the full path
        os.makedirs(os.path.dirname(self.full_path), exist_ok=True)        # Ensure the directory exists, and creates it if it doesn't


    def log(self, ControlNode):
        with open(self.full_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time',
                            'x', 'y', 'z', 'yaw',
                            'metadata'
                            ])
            
            time_history = ControlNode.get_time_log() #0
            x_history = ControlNode.get_x_log() #1
            y_history = ControlNode.get_y_log() #2
            z_history = ControlNode.get_z_log() #3
            yaw_history = ControlNode.get_yaw_log() #4
            metadata = ControlNode.get_metadata() #5

            """
            def get_time_log(self): return np.array(self.time_log).reshape(-1, 1)
            def get_x_log(self): return np.array(self.x_log).reshape(-1, 1)
            def get_y_log(self): return np.array(self.y_log).reshape(-1, 1)
            def get_z_log(self): return np.array(self.z_log).reshape(-1, 1)
            def get_yaw_log(self): return np.array(self.yaw_log).reshape(-1, 1)
            def get_metadata(self): return self.metadata.reshape(-1, 1)
            """
            
            # Pad the metadata to match the time history
            padding_length = time_history.shape[0] - metadata.shape[0]
            metadata = np.pad(metadata, ((0, padding_length), (0, 0)), 'constant', constant_values='0')
           

            # Combine the histories for logging
            data = np.hstack((time_history,
                              x_history, y_history, z_history, yaw_history,
                              metadata
                              ))
            # Write each row to the CSV file
            for row in range(data.shape[0]):
                writer.writerow(np.asarray(data[row, :]).flatten())

            print(f"\nWrote to {self.full_path}")
