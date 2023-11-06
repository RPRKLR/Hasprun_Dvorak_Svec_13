Debugging guide for ros 2
Debugging for a single node
Once you have the code correctly implemented (At least compiled), the first thing you do is to compile the package the following way.

foo@bar:~$ cd ros_ws
foo@bar:~$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
foo@bar:~$ . install/setup.bash
This way we are compiling the code for debugging with optimization. If we want to compile the code without optimization use, do the following.

foo@bar:~$ cd ros_ws
foo@bar:~$ colcon --log-level debug build   --merge-install --event-handlers console_direct+ --cmake-args "-DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_C_FLAGS_RELWITHDEBINFO=-g -O0 -DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-g -O0" 
foo@bar:~$ . install/setup.bash
After compiling the code, we need to launch a GDB server. Here we will use a localhost:port for creating the server. Choose any free port for your GDB server.

foo@bar:~$ ros2 run --prefix 'gdbserver localhost:3000' package_name executable_name
After that, we need to create a launch.json file on VSCode. We are going to create a custom debugging configuration. In our case, create a GDB client and connect to the server.

Open VSCode on your workspace.
Go to your side bar, 'Run and Debug' section.
Add a new configuration (Select C++ enviroment or any other)
On your launch.json file, put the following information
{
    "version": "0.2.0",
        "configurations": [
            {
                "name": "C++ Debugger",
                "request": "launch",
                "type": "cppdbg",
                "miDebuggerServerAddress": "localhost:3000",
                "cwd": "/",
                "program": "[build-path-executable]"
            }
        ]
    }
name - Custom name of your debugger configuration
request - In this case we want to launch the client
type - cppdbg for c++ debugging
miDebuggerServerAddress - path_server:port
cwd - Where to find all the required files. We use root because ROS, the package, and other required files are distributed along the entire PC.
program - Change [build-path-executable] by your executable build file. You can find this path on the console when you launch the server.