# BOSSY - Basic Open-source Spacecraft Simulator with Yamcs

## Setup & Run

### Clone the repository
```
git clone https://github.com/Meridian-Space-Command/BOSSY.git
```

### Change directory to BOSSY
```
cd BOSSY
```

### Open two terminals

One terminal for the Yamcs server:
```
cd MCS
```

One terminal for the simulator:
```
cd SIM
```

### Run Yamcs

Run Yamcs from the MCS terminal:
```
./mvnw clean && ./mvnw compile && ./mvnw yamcs:run
```
This batch of commands will delete all previous simulation data (because otherwise the archive timestamps will overlap and it will confuse everything), then it will compile the Yamcs with its dependencies, and finally it will run Yamcs.

Go to a web browser and open the Yamcs web interface: `http://localhost:8090`

### Stop Yamcs

Go to the MCS terminal and press `Ctrl+c` (or `Cmd+c` on macOS).

### Run the simulator

- Edit the `BOSSY/SIM/config.py` file to set the initial state of the simulation, spacecraft, environment, orbit, etc.

Run the simulator from the SIM terminal:
```
python3 simulator.py
```

### Stop the simulator

Go to the SIM terminal and press `Ctrl+c` (or `Cmd+c` on macOS).

## ATS usage

ATS = Absolute Time Sequence

### Create a new ATS

- Go to the `BOSSY/MCS/upload` directory.
- Create a .csv file with the name of the new ATS, for example `ATS.csv`.
- Add the commands to the file, each command in a new line.
- Each line has the format: `timestamp, command_id, command_data, comment`
- Example:

```
789006610,14,None, # Reset OBC
789006630,33,2, # Set ADCS mode to SUNPOINTING
```

This means:
- cmd_1:
    - Timestamp = 789006610 (seconds since epoch (e.g. 2000-01-01 12:00:00) = 2025-01-01 12:30:10)
    - command_id = 14 (Reset OBC) 
    - command_data = None (no data as this command has no arguments).
- cmd_2:
    - Timestamp = 789006630 (seconds since epoch (e.g. 2000-01-01 12:00:00) = 2025-01-01 12:30:30)
    - command_id = 33 (ADCS_SET_MODE) 
    - command_data = 2 (SUNPOINTING)

Tips:
- For the comment, don't forget to have the `,` before the `#` character.
- To discover the command_id of a command, you can go to yamcs-web, then the Mission Database, then the Commands section, and select the desired command. It will show the command_id and the arguments with more description.

### Upload the ATS

- Send the `/spacecraft/DATASTORE_UPLOAD_FILE` command with the filename `ATS.csv` to upload the ATS from the `BOSSY/MCS/upload` directory onto the spacecraft's storage.

### Start the ATS

- Send the `/spacecraft/OBC_START_ATS` command with the filename `ATS.csv` to start the ATS.

### Things to know

- The `config.py` file sets the epoch and mission start time for the simulator. It also declares the upload and download directories for MCS. 
- Timestamps in the simulator's past will be discarded.
- The ATS will run indefinitely. There's currently no way to stop it. Good luck!
- Technically, you can upload multiple ATS files (with different names), and they'll run in parallel if timestamps overlap.
- This feature has not been fully tested. 

## PAYLOAD usage

- The payload has a camera that can take images.
- The camera's parameters are set in the `config.py` file.
- The camera can only take an image on demand (i.e. immediately).
- To schedule an image capture, use an ATS.
- The image filename will have the format: `EO_image_met_{met_sec}.png`, where `{met_sec}` is the MET seconds at the time of the image capture (i.e. total seconds since the mission start time).
- The camera will save the image in the "onboard storage" directory of the spacecraft, which is `BOSSY/SIM/apps/spacecraft/storage/`.
- You will then need to use the `/spacecraft/DATASTORE_DOWNLOAD_FILE` command to download the image to the `BOSSY/MCS/download` directory, which is the directory of the MCS (i.e. on the Ground). 

## Considerations
- As a BASIC simulator, the subsystems are not fully representative of real spacecraft subsystems. 
- The simulator does not yet fully support all the commands and telemetry.
- Some telemetry are shown, but not simulated properly (i.e. are not updated, just init values).
- This project is a work in progress.
