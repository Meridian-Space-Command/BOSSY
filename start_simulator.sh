#!/bin/bash

# Add current directory to PYTHONPATH   
export PYTHONPATH="${PYTHONPATH:-.}:$PWD"

# Export mission start time for YAMCS
export MISSION_START_TIME=$(python3 -c "from simulator.spacecraft_config import MISSION_START_TIME; print(MISSION_START_TIME.strftime('%Y-%m-%dT%H:%M:%SZ'))")

# Clean up the project (deletes target directory and all yamcs data)
./mvnw clean

# Compile the project
echo "Compiling project..."
./mvnw compile

if [[ "$OSTYPE" == "darwin"* ]]; then
    # Create new tabs with specific names and start processes
    osascript -e '
        tell application "Terminal"
            -- Create YAMCS tab
            tell application "System Events" to keystroke "t" using command down
            delay 0.5
            set custom title of selected tab of front window to "YAMCS_TAB"
            do script "cd '"$PWD"' && export PYTHONPATH='"'$PYTHONPATH'"' && export MISSION_START_TIME='"'$MISSION_START_TIME'"' && echo '\''Starting YAMCS...'\''" in selected tab of front window
            delay 1
            do script "./mvnw yamcs:run" in selected tab of front window
            
            -- Create Simulator tab
            tell application "System Events" to keystroke "t" using command down
            delay 0.5
            set custom title of selected tab of front window to "SIM_TAB"
            do script "cd '"$PWD"' && export PYTHONPATH='"'$PYTHONPATH'"' && export MISSION_START_TIME='"'$MISSION_START_TIME'"' && echo '\''Waiting for YAMCS to initialize...'\''" in selected tab of front window
            delay 10
            do script "python3 simulator.py" in selected tab of front window
        end tell'
    
    # Wait for user input in original window
    echo "Press Enter to stop the simulation..."
    read
    
    # Kill processes more aggressively
    echo "Stopping all processes..."
    pkill -9 -f "yamcs:run"
    pkill -9 -f "org.yamcs.YamcsServer"
    pkill -9 -f "simulator.py"
    
    # Wait for processes to stop
    sleep 2
    
else
    # For Linux/Unix systems
    if command -v gnome-terminal &> /dev/null; then
        # Start YAMCS in new tab
        gnome-terminal --tab -- bash -c "cd '$PWD' && export PYTHONPATH='$PYTHONPATH' && export MISSION_START_TIME='$MISSION_START_TIME' && echo 'Starting YAMCS...' && ./mvnw yamcs:run; exec bash"
        
        # Start simulator in new tab
        gnome-terminal --tab -- bash -c "cd '$PWD' && export PYTHONPATH='$PYTHONPATH' && export MISSION_START_TIME='$MISSION_START_TIME' && echo 'Waiting for YAMCS...' && sleep 10 && echo 'Starting simulator...' && python3 simulator.py; exec bash"
    else
        echo "No suitable terminal emulator found. Please install gnome-terminal."
        exit 1
    fi
    
    # Wait for user input to stop
    echo "Press Enter to stop the simulation..."
    read
    
    # Kill processes more aggressively
    echo "Stopping all processes..."
    pkill -9 -f yamcs
    pkill -9 -f "org.yamcs.YamcsServer"
    pkill -9 -f simulator.py
    
    # Wait for processes to stop
    sleep 2

fi