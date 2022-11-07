# Introducing VPI - the Vex V5 port of WPILib

This library is intended both to provide more "advanced" functionality and as a guide to better programming practices, including object-oriented design, "don't repeat yourself", separation of concerns, etc.  Please learn from code here, and feel free to provide feedback to improve the quality of code or functionality.

## WPI Mission Statement

The WPI Mission Statement is very compelling, and the collaborative community that FRC seems to have is something the Vex community could strive to emulate.

```
    The WPILib Mission is to enable FIRST Robotics teams to focus on writing game-specific software rather than focusing on hardware 
    details - "raise the floor, don’t lower the ceiling". We work to enable teams with limited programming knowledge and/or mentor 
    experience to be as successful as possible, while not hampering the abilities of teams with more advanced programming capabilities.
```

Original WPILib documentation: https://docs.wpilib.org/

## Notes:

* Makes use of `#pragma once` directive, which is not standard (but supported by VexCode Pro), but is a nicer way to do things than `#ifndef`-style include guards. See: https://en.wikipedia.org/wiki/Pragma_once
* This is a fairly close / direct port of the FTC/FRC WPILib: https://github.com/wpilibsuite/allwpilib
    * WPILib uses https://gitlab.com/libeigen/eigen for Mecanum Kinematics and Odometry as well as path planning.
    * See section on Eigen later in this document
* Though the units are taken from OkapiLib: https://github.com/OkapiLib/OkapiLib/ as they seem more "natural"
* Most teams will find the best entry point to this through either of the two `vpi/chassis/` DifferentialDriveChassis classes
* Because the Chassis and PID classes use threads, it is STRONGLY RECOMMENDED that you do not declare your DifferentialDriveChassis in the global scope.
    * The examples will provide best practices
* Porting the Command-Based model will come next

(TODO - build doxygen documentation from comments)

Until a project template directory is created, follow these instructions to use this library in your code:

(TODO - make this easier)

## Note about threads

Please see this post https://www.vexforum.com/t/competition-code-and-user-created-tasks/88616 documenting the life-cycle of threads/tasks in V5 in the context of Competition states.

Note that this library creates several threads; for that reason (and others), it is recommended that users create objects in the scope of the `autonomous` or `usercontrol` functions.  Creating objects from this library in the `global` scope may lead to unpredictable behavior.

## Note about makefile changes

The example programs already have the changes in place, but this documents what those changes are.  New teams are encouraged to start from an example and customize from there.

### Makefile changes - details

To use the library in another project, build this project and then copy `libvpi.a` into a suitable place such as a "lib" directory.  Then copy the files in the `vpi/include` directory into your own project's `include` directory.

Open the makefile and add directories under the include directory right after the existing line starting with SRC_H:

    SRC_H += $(wildcard include/*/*.h)
    SRC_H += $(wildcard include/*/*.h)
    SRC_H += $(wildcard include/*/*/*.h)
    SRC_H += $(wildcard include/*/*/*/*.h)

Include the header files in the new project's include directory and use the library.

Now the project will still not build as we have to tell the linker to link the library

Open mkenv.mk and edit to add the new library (add `-lvpi`) so that it looks similar to:

    # libraries
    LIBS =  --start-group -lv5rt -lstdc++ -lc -lm -lgcc -lvpi --end-group

And tell the linker where it is by adding the path here (add -L"lib" assuming you have placed these libraries a directory you create called "lib" in your project)

    TOOL_LIB  = -L"$(TOOLCHAIN)/$(PLATFORM)/gcc/libs" -L"lib"

## Note about Eigen

During testing, code that called into Eigen on the V5 Brain would throw `assert` errors, pointing to: http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html

Based on that link, when `#include`-ing Eigen headers, VPI sets the `EIGEN_MAX_STATIC_ALIGN_BYTES` to 0, which generates a compile-time warning

Based on James Pearman's post here: https://www.vexforum.com/t/eigen-integration-issue/61474/7 the ARM_NEON flags are disabled for Eigen as well.  When `#include`-ing Eigen in your project, it is recommended to be done similar to:

    #undef __ARM_NEON__
    #undef __ARM_NEON
    #define  EIGEN_MAX_STATIC_ALIGN_BYTES 0
    #include "Eigen/Core"
