#!/bin/bash

# Add current directory to PYTHONPATH   
export PYTHONPATH="${PYTHONPATH:-.}:$PWD"

# Export mission start time for YAMCS
export MISSION_START_TIME=$(python3 -c "from simulator.spacecraft_config import MISSION_START_TIME; print(MISSION_START_TIME.strftime('%Y-%m-%dT%H:%M:%SZ'))")

# Clean up any existing .rdb files at start
echo "Cleaning up previous simulation data..."
rm -rf ./target/yamcs-data

# Compile the project
echo "Compiling project..."
./mvnw compile

if [[ "$OSTYPE" == "darwin"* ]]; then
    # Create new tabs and store their IDs
    YAMCS_TAB=$(osascript -e '
        tell application "Terminal"
            tell application "System Events" to keystroke "t" using command down
            delay 0.5
            do script "cd '"$PWD"' && export PYTHONPATH='"'$PYTHONPATH'"' && export MISSION_START_TIME='"'$MISSION_START_TIME'"' && echo '\''Starting YAMCS...'\''" in selected tab of the front window
            return id of the front window
        end tell')
    
    SIM_TAB=$(osascript -e '
        tell application "Terminal"
            tell application "System Events" to keystroke "t" using command down
            delay 0.5
            do script "cd '"$PWD"' && export PYTHONPATH='"'$PYTHONPATH'"' && export MISSION_START_TIME='"'$MISSION_START_TIME'"' && echo '\''Waiting for YAMCS to initialize...'\''" in selected tab of the front window
            return id of the front window
        end tell')
    
    # Start YAMCS in its tab
    osascript -e '
        tell application "Terminal"
            do script "./mvnw yamcs:run" in tab 1 of window id '"$YAMCS_TAB"'
        end tell'
    
    # Start simulator in its tab after delay
    osascript -e '
        tell application "Terminal"
            delay 10
            do script "python3 simulator.py" in tab 1 of window id '"$SIM_TAB"'
        end tell'
    
    # Wait for user input to stop
    echo "Press Enter to stop the simulation..."
    read

    # Kill processes more aggressively
    echo "Stopping all processes..."
    pkill -9 -f "yamcs:run"
    pkill -9 -f "org.yamcs.YamcsServer"
    pkill -9 -f "simulator.py"
    
    # Wait for processes to stop
    sleep 2
    
    # Close tabs using a different AppleScript approach
    osascript -e '
        tell application "Terminal"
            set windowList to windows where id is '"$YAMCS_TAB"' or id is '"$SIM_TAB"'
            repeat with currentWindow in windowList
                repeat with currentTab in tabs of currentWindow
                    tell currentTab
                        set currentSettings to current settings
                        write text "exit"
                    end tell
                end repeat
            end repeat
        end tell'
    
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