#!/bin/bash
# record_episodes.sh
# Records N episodes with automatic go_home between each.
# Usage: bash record_episodes.sh [num_episodes] [repo_id]

NUM_EPISODES=${1:-20}
REPO_ID=${2:-"PhillippGery/pick_place_001"}
EPISODE_TIME=${3:-30}

# Source environment
source /opt/ros/jazzy/setup.bash
source ~/TwinNexus-Admittance-Platform/10_src/install/setup.bash
source ~/lerobot_env/bin/activate

echo "================================================"
echo "  TwinNexus Episode Recording"
echo "  Episodes:     $NUM_EPISODES"
echo "  Repo:         $REPO_ID"
echo "  Episode time: ${EPISODE_TIME}s"
echo "================================================"
echo ""
echo "Make sure:"
echo "  1. boot_hw is running and Play is pressed"
echo "  2. spawntele is running"
echo ""
read -p "Press Enter to start recording..."

for i in $(seq 1 $NUM_EPISODES); do
    echo ""
    echo "================================================"
    echo "  Episode $i / $NUM_EPISODES"
    echo "================================================"

    # Record one episode
    python -m lerobot.record \
      --robot-type twinnexus \
      --robot-id twinnexus_right \
      --repo-id "$REPO_ID" \
      --num-episodes 1 \
      --episode-time-s "$EPISODE_TIME" \
      --reset-time-s 1

    # Check if recording succeeded
    if [ $? -ne 0 ]; then
        echo "Recording failed. Stopping."
        exit 1
    fi

    # Skip go_home after last episode
    if [ $i -lt $NUM_EPISODES ]; then
        echo "Returning to home position..."
        python3 ~/TwinNexus-Admittance-Platform/10_src/src/bimanual_ur5e_bringup/scripts/return_home.py

        echo ""
        echo "Reset the object to start position."
        echo "Match GELLO to robot home pose."
        read -p "Press Enter when ready for episode $((i+1))..."
    fi
done

echo ""
echo "================================================"
echo "  All $NUM_EPISODES episodes recorded!"
echo "  Dataset: $REPO_ID"
echo "================================================"
