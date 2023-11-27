# Modbus ROS Comm Package Instructions

This package is part of AgRobot 2 Project.

This package is only used for electromechanical devices' communication and control of low-level functions, advanced scheduling and control are completed by other ROS nodes.

See [examples](examples) for reference usage.

## Install pyModbus

- Fetch pyModbus source code

    ```bash
    cd ~
    git clone git://github.com/pymodbus-dev/pymodbus
    cd pymodbus && git checkout v3.5.2
    ```

- Prepare install requirements

    ```bash
    sudo apt install python3-pip
    pip install -r requirements.txt
    ```

    If errors occur when executing above commands and these errors are related to versions of `docutils`, the following commands may help a little.

    ```bash
    pip install --upgrade docutils==0.18.1
    pip install -r requirements.txt
    ```

- Install pyModbus

    ```bash
    pip install -e .
    pre-commit install
    ```

- Make a confirmation about the installation

    ```bash
    pip show pymodbus
    ```
