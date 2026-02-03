# Navigation Workspace

This workspace contains the navigation stack and related packages for the Spot robot.

## Network Configuration (Cyclone DDS Unicast)

This repository uses **Cyclone DDS** in **Unicast** mode to communicate between the robot (Spot) and the operator's laptop. This configuration is necessary to avoid network congestion and ensure reliable communication over Wi-Fi.

### Prerequisites

*   **Robot:** Hostname must be `spot` or `spot-orin`.
*   **Laptop:** Hostname should ideally be `spot-laptop`. If not, you must manually ensure the correct Cyclone DDS configuration is loaded or set up.

### Configuration Files

The `config/` directory contains the DDS configuration files:

*   `cyclonedds_spot.xml`: Configuration for the robot. It is pre-configured to look for a peer at `${LAPTOP_IP}`.
*   `cyclonedds_laptop.xml`: Configuration for the operator's laptop. It is pre-configured to look for peers at `${SPOT_IP}` and `${ORIN_IP}`.

### Environment Variables

For the configuration to work, you must define the following environment variables. The `setup_unicast.sh` script (run automatically in the container) points Cyclone DDS to the correct XML file, but that XML file relies on these variables.

#### On the Laptop

You need to tell your laptop where the robot is.

1.  **Create a `config/spot_ip` file** (Recommended):
    Create a file named `spot_ip` in the `config/` directory and paste the robot's IP address into it.
    ```bash
    echo "192.168.86.XX" > ~/nav_ws/config/spot_ip
    ```
    The `container` script will read this file and automatically export `SPOT_IP` into the container.

2.  **Or, manually export `SPOT_IP`**:
    ```bash
    export SPOT_IP=192.168.86.XX
    ```

3.  **Export `ORIN_IP` (Optional)**:
    If communicating with the Orin payload separately:
    ```bash
    export ORIN_IP=192.168.86.YY
    ```

#### On the Robot

The robot needs to know where the laptop is.

1.  **Export `LAPTOP_IP`**:
    Add this to your `.bashrc` or export it before running the container:
    ```bash
    export LAPTOP_IP=192.168.86.ZZ  # Your laptop's IP
    ```

### How it Works

The `scripts/setup_unicast.sh` script runs inside the container and:
1.  Checks the hostname.
2.  If hostname is `spot` or `spot-orin`: Sets `CYCLONEDDS_URI` to `cyclonedds_spot.xml`.
3.  If hostname is `spot-laptop`: Sets `CYCLONEDDS_URI` to `cyclonedds_laptop.xml`.
4.  Otherwise: Falls back to default multicast `cyclonedds.xml`.

### Verification

To verify connectivity:

1.  Start the container: `./container start`
2.  Enter the shell: `./container shell`
3.  Check the config path:
    ```bash
    printenv CYCLONEDDS_URI
    ```
4.  Check that the variables are set:
    ```bash
    echo $SPOT_IP
    echo $LAPTOP_IP
    ```
5.  Test ROS 2 communication (e.g., `ros2 topic list`).
