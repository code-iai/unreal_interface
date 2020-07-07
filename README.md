# unreal_interface
This ROS package is intended to provide a low-level interface to interact with an UE4 world in a RPC-fashion. 
It will allow you to modify the UE4 world without coding directly in UE4, but rather from external programs.

In order to accomplish this, we currently make use of the following UE4 plugins:
  * [UROSBridge](https://github.com/robcog-iai/UROSBridge) - General method to exchange messages between UE4 and external programs.
  * [UROSWorldControl](https://github.com/robcog-iai/UROSWorldControl) - Methods to manipulate a world, like spawning objects, modifying them, deleting them etc. Please make sure to also install the companion ROS package for this plugin.
  * [UnrealInterfaceObjectPlugin](https://github.com/code-iai/UnrealInterfaceObjectPlugin) - Functionality on the UE4-side that is rather specific to the UnrealInterface communication capabilities.

Please make sure to have these plugins running in your UE4 project, before using this ROS Package.
  
## Scope
Even though every application you want to tackle with UE4 has different requirements,
we aim to make this ROS package as general as possible to allow better re-use.
This means that this package should only act as an interface to alter UE4 worlds, 
but should not include application-specific code.

## State
Please note that this package is a very early development stage. Interfaces are subject to change.