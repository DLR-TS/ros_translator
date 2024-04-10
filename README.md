# ROS Translator

The ROS Translator is a docker compose project that setups up a 
[ros1_bridge 🔗](https://github.com/ros2/ros1_bridge) build and runtime context 
in Ubuntu 22.04.
This project has boilerplate code to automatically format and translate **ROS1
Noetic** and **ROS2 Iron** packages and message to and from ROS2.

In principle ROS1 messages do not need to be modified to be translated to and
from ROS2.

## Use Cases
Why would you want to use this project?
- Publish an arbitrary/custom topic in ROS 1 NOETIC and subscribe to it in ROS 2 IRON
- Publish an arbitrary/custom topic in ROS 2 IRON and subscribe to it in ROS 1 NOETIC 

## What does this project provide?
1. Docker build context that compiles ROS 1 NOETIC and [ros1_bridge 🔗](https://github.com/ros2/ros1_bridge)
2. Docker compose runtime context to run the [ros1_bridge 🔗](https://github.com/ros2/ros1_bridge)
  - This context has ROS 1 NOETIC and ROS 2 installed side-by-side on a Ubutnu 22.04 host 
3. Message package auto-generation. ROS 1 message packages can be auto-converted
   to a ROS 2 message package.

### ROS2 Package Generation 
This project contains a package generation tool that automatically takes ROS1
packages and translates them to a valid ROS2 package.

**ROS1** message packages are submodules placed in the [ros1_packages](ros1_packages)
directory. All packages in this directory are converted to an identical **ROS2**
package. Only packages containing selected messages defined in the 
[bridged_msgs.txt](bridged_msgs.txt) are converted to a ROS2 package. All
generated ROS2 packages use
[bridge_workspace/src/template](bridge_workspace/src/template) as the template
package. Once generated the output is placed in
[bridge_workspace/src/bridge_workspace/bridge_packages](bridge_workspace/src/bridge_packages).

Only messages explicitly listed in [bridged_msgs.txt](bridged_msgs.txt) will be
available in the `ros1_bridge`.

All message packages in [ros1_packages](ros1_packages) will be build with
**ROS1**. 

### Message Mapping
Messages can have different package names, message names, and data types.
Message mappings are defined in the [mapping_rules.yaml](mapping_rules.yaml).

Review [https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst](https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst) for more information on this configuration.

The following is a minimum example `mapping_rules.yaml` file:
```yaml
- enable_foreign_mappings: true
  ros1_package_name: 'adore_if_ros_msg'
  ros1_message_name: 'Action'
  ros2_package_name: 'adore_if_ros_msg'
  ros2_message_name: 'Action'
```
This assumes the package names, message names and type defs are identical.


### Basic Setup
Basic setup allows bridging/translation of ROS std_msgs data types.
a std_msgs data type can be published on a ROS1 node and received by a ROS2
node or published by a ROS2 node and received by a ROS1 node.

### Basic Usage
1. clone the project:
```bash
git clone --recurse-submodules git@gitlab.dlr.de:csa/ros2/ros_translator.git 
```

> ⚠️ **Warning:**
> 
> Failing to update submodules will result in build failures!

2. Build the ROS translator:
```bash
cd ros_translator
make build
```

3. Run the ros translator
```bash
make start
```

4. View the published topics:
```bash
make rostopic_list
```
This will run `rostopic list` in the context of ROS1 and `ros2 topic list` in
the context of ROS2.

## Advanced Setup With Message Generation
This project provides ROS2 message package generation capabilities.

1. Add the **ROS1** submodule to [ros1_packages](ros1_packages) with: 
```bash
cd ros1_packages 
git submodule add ...
```

2. Define the messages that should be bridged by adding the file names to 
[bridged_msgs.txt](bridged_msgs.txt).

3. Update the `ros1_bridge` mapping rules in the file [mapping_rules.yaml](mapping_rules.yaml).
The mapping rules need to be updated if one of the following questions is true:
- Are the package names of the **ROS1** and **ROS2** packages different?
- Are message names different?
- Are parameter names different?

## Advanced Setup Without Message Package Generation
1. Add the **ROS1** submodule to [ros1_packages](ros1_packages) with: 
```bash
cd ros1_packages 
git submodule add ...
```

2. Add the **ROS2** submodule to [ros2_packages](ros2_packages) with: 
```bash
cd ros2_packages 
git submodule add ...
```

3. Update the `ros1_bridge` mapping rules in the file [mapping_rules.yaml](mapping_rules.yaml).
The mapping rules need to be updated if one of the following questions is true:
- Are the package names of the **ROS1** and **ROS2** packages different?
- Are message names different?
- Are parameter names different?
