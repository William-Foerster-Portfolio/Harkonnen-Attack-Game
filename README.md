# Harkonnen Attack Game

The Harkonnen are attacking!
Our base has cobbled together a moving platform with a very powerful shield generator and amped-up auto-aiming laser on it.  They are dangerous if used together, but you may successfully preserve this base in our narrow canyon with these makeshift defenses.  Expect the enemy to drop tiny, super-dense masses on us—thankfully their local shortage of spice means that they will have a very limited number!
You may recall from history that the simultaneous overlapping use of lasers and shields can have disastrous consequences (a nuclear explosion), and we have been unable to get a safety interlock in place yet—so you’ll just have to make sure that your operators are…cautious.  Now if you’ll excuse me, I have an ornithopter to catch…

## Game Play

- **Objective**: Repell all three dense masses without letting a single mass hit the home world.

Using the capacitive sense slider and the 2 buttons on the SLSTK3402A EFM32, you will defend the home world. A 'super-dense' mass will fall from the sky and you have to bounce the mass back into space with the platform. The mass will only bounce higher if the shield is activated using the left button. If the shield is not activated when the mass hits the platform, a dead bounce will occure and the mass will lose momentum. If the mass loses too much momentum, it will fall through the platform. If you get into trouble, you have 1 lazer that will destroy the mass. 


## Project Technical Description
This project is a game simulation designed for the SLSTK3402A EFM32 Pearl Gecko PG Silicon Labs Development Board. It uses an RTOS (Real-Time Operating System) to handle multiple tasks such as physics calculations, user input, display updates, and LED indicators, all coordinated through semaphores, mutexes, and timers.


## Table of Contents

1. [Features](#features)
2. [System Requirements](#system-requirements)
4. [Main Project Structure](#main-project-structure)
5. [Tasks and Functions](#tasks-and-functions)
6. [Configuration Settings](#configuration-settings)
7. [Building and Running the Project](#building-and-running-the-project)
8. [Troubleshooting](#troubleshooting)
9. [Full Project Structure](#full-project-structure)

## Features

- **Real-Time Task Scheduling:** Utilizes an RTOS to manage multiple tasks concurrently.
- **Game Physics Simulation:** Implements physics for a platform and HoltzmanMasses (HM) objects with collision detection and response.
- **User Input Handling:** Handles button inputs and capacitive touch slider for game controls.
- **Display Output:** Updates an LCD screen with game state and graphics.
- **LED Indicators:** Uses LEDs to provide visual feedback based on game state.
- **Shield and Laser Mechanics:** Adds extra gameplay elements like shields and lasers with configurable settings.

## System Requirements

- **Microcontroller:** SLSTK3402A EFM32 Pearl Gecko PG.
- **Development Environment:** Simplicity Studio IDE with appropriate SDK and toolchain for Silicon Labs devices.
- **Hardware:** SLSTK3402A board with buttons, capacitive touch slider, LCD display, and LEDs.
- **RTOS:** Micrium OS (part of Silicon Labs' offering).

### Tasks and Functions

#### 1. Button Input Task
- **Function:** `ButtonInput_task(void *arg)`
- **Description:** Handles button press events and updates the game state accordingly (e.g., activating shields or lasers).

#### 2. Physics Task
- **Function:** `Physics_task(void *arg)`
- **Description:** Updates the physics simulation for game objects, including velocity, position, and collision detection.

#### 3. Platform Task
- **Function:** `Platform_task(void *arg)`
- **Description:** Manages the platform's position and velocity based on capacitive touch slider input.

#### 4. LCD Display Task
- **Function:** `LCDDisplay_task(void *arg)`
- **Description:** Periodically updates the LCD display with the current game state.

#### 5. LED0 Task
- **Function:** `led0_task(void *arg)`
- **Description:** Blinks LED0 based on the ratio of HoltzmanMasses (HM) velocity to maximum velocity.

#### 6. LED1 Task
- **Function:** `led1_task(void *arg)`
- **Description:** Blinks LED1 based on the difference in velocity and position between the platform and HM, and the distance to the ground.

#### 7. Idle Task
- **Function:** `idle_task(void *arg)`
- **Description:** Low-priority task to ensure the CPU is in sleep mode to optimize power consumption when no other tasks are running.

### Main Project Structure

#### Source Files (`./src/`)
- **main.c**: Main application source file.
- **Header_Files/**: Directory containing header files for the application.
  - **fifo.h**: Header file for FIFO buffer management.
  - **memlcd_app.h**: Header file for memory LCD application.
  - **bsp_os.h**: Header file for board support package and OS.
  - **app.h**: Header file for the main application.
  - **cmu.h**: Header file for clock management unit.
  - **gpio.h**: Header file for GPIO management.
  - **capsense.h**: Header file for capacitive sensing.
  - **physics.h**: Header file for physics calculations.
- **Source_Files/**: Directory containing source files for the application.
  - **cmu.c**: Source file for clock management unit.
  - **capsense.c**: Source file for capacitive sensing.
  - **gpio.c**: Source file for GPIO management.
  - **physics.c**: Source file for physics calculations.
  - **fifo.c**: Source file for FIFO buffer management.
  - **app.c**: Source file for the main application.
  - **memlcd_app.c**: Source file for memory LCD application.
  - **bsp_os.c**: Source file for board support package and OS.

### Configuration Settings

- **`WALL_BOUNCE_TypeDef`**: Configuration for wall bounce mechanics.
  - **`enable`**: Enables or disables wall bounce.
  - **`limited`**: Determines if the bounce is limited.
  - **`max_bounce_speed`**: Maximum speed for bouncing off walls.

- **`HM_SETTINGS_TypeDef`**: Settings for HoltzmanMasses objects.
  - **`num`**: Number of HoltzmanMasses.
  - **`display_diameter`**: Diameter for display purposes [cm].
  - **`initial_conditions`**: Initial conditions setting (e.g., fixed or user-defined).
  - **`v0`**: Initial velocity [cm/s] (x and y components).
  - **`max_xv`**: Maximum horizontal velocity [cm/s].
  - **`x0_position`**: Initial horizontal position [cm].
  - **`user`**: User-defined mode inputs.

- **`PLATFORM_SETTINGS_TypeDef`**: Platform-related settings.
  - **`max_force`**: Maximum force applied [N].
  - **`mass`**: Platform mass [kg].
  - **`length`**: Platform length [cm].
  - **`wall_bounce`**: Wall bounce settings (as defined in `WALL_BOUNCE_TypeDef`).
  - **`auto_control`**: Enables or disables automatic control.

- **`BOOST_TypeDef`**: Boost settings for game mechanics.
  - **`KE_increase`**: Kinetic energy increase percentage [%].
  - **`time_active`**: Time the boost is active [ms].
  - **`recharge_time`**: Time to recharge after boost [ms].

- **`SHIELD_TypeDef`**: Shield settings for defense mechanisms.
  - **`min_velocity`**: Minimum effective perpendicular speed [cm/s].
  - **`passive_KE_reduction`**: Kinetic energy reduction when shield is passive [%].
  - **`boost`**: Boost settings (as defined in `BOOST_TypeDef`).

- **`LASER_TypeDef`**: Laser settings for offense mechanisms.
  - **`num_activations`**: Number of activations allowed.
  - **`auto_control`**: Enables or disables automatic control.

- **`GAME_SETTINGS_TypeDef`**: General game settings.
  - **`version`**: Data structure version.
  - **`tau_physics`**: Physics update interval [ms].
  - **`tau_LCD`**: LCD update interval [ms].
  - **`gravity`**: Gravity constant [cm/s²].
  - **`canyon_size`**: Size of the game canyon [cm].
  - **`canyon_pixel_conversion`**: Conversion factor from canyon size to pixels [pixels/cm].
  - **`capsense_sample_rate`**: Capacitive sensor sample rate [ms].
  - **`hm`**: HoltzmanMasses settings (as defined in `HM_SETTINGS_TypeDef`).
  - **`platform`**: Platform settings (as defined in `PLATFORM_SETTINGS_TypeDef`).
  - **`shield`**: Shield settings (as defined in `SHIELD_TypeDef`).
  - **`laser`**: Laser settings (as defined in `LASER_TypeDef`).

### Building and Running the Project

1. **Download and Import the Project:**
   - Download the WF_RTOS_Harkonnens_Attack.sls file
   - Open Simplicity Studio and import existing project into workspace.

2. **Build the Project:**
   - Open Simplicity Studio and load the project.
   - Ensure all paths to libraries and headers are correctly configured.
   - Use the build tools in Simplicity Studio to compile the project. Make sure the toolchain is set up for the SLSTK3402A EFM32 Pearl Gecko PG microcontroller.

3. **Flash the Microcontroller:**
   - Connect your SLSTK3402A board to your computer using a USB cable.
   - In Simplicity Studio, use the Device Manager to detect your connected board.
   - Navigate to the Flash Programmer tool within Simplicity Studio.
   - Select the compiled binary from your project and click "Program" to flash the microcontroller.

4. **Run the Application:**
   - Once the microcontroller is flashed with the binary, press the reset button on the SLSTK3402A board.
   - Observe the game simulation on the connected hardware. The LCD should display the game state, and LEDs should provide visual feedback based on game events.
   
5. **Debugging:**
   - If you encounter issues, use the debugging tools in Simplicity Studio to step through your code and verify the behavior of your tasks and functions.
   - Ensure that all peripherals are properly initialized and that all tasks are running as expected.

### Troubleshooting

- **Build Errors:**
  - Ensure all library paths are correctly set in Simplicity Studio.
  - Verify that the toolchain is configured correctly for the SLSTK3402A EFM32 Pearl Gecko PG microcontroller.
  - Check for any missing or incorrect header files and ensure all dependencies are included in the project.

- **Runtime Errors:**
  - Use the debugger in Simplicity Studio to step through your code and monitor variable states, function calls, and task behavior.
  - Confirm that all tasks are being created and started correctly by checking task initialization and priority settings.
  - Ensure that all semaphores, mutexes, and timers are initialized properly and used according to the RTOS documentation.

- **Display Issues:**
  - Verify that the LCD connections are secure and that the display is receiving power.
  - Ensure that the `DMD` and `GLIB` libraries are initialized correctly in your code.
  - Check for any errors returned by display-related functions and handle them appropriately.

- **Button/Input Issues:**
  - Confirm that the GPIO configurations for buttons and capacitive touch sliders are correct.
  - Check that interrupts are enabled and configured properly for button inputs.
  - Use the debugger to verify that button press events are being detected and handled correctly.

- **LED Issues:**
  - Ensure that the LED GPIO pins are configured correctly for output.
  - Check that the LED tasks are running and that their logic for toggling the LEDs is correct.
  - Use a multimeter to verify that the LEDs are receiving power when expected.

### Full Project Structure

The project is organized into several directories and files, each serving a specific purpose in the development of the game simulation. Below is an overview of the key directories and files:

#### Root Directory
- **WF_RTOS_Harkonnens_Attack.slcp**: Simplicity Studio project configuration file.
- **README.md**: Documentation file for the project.
- **WF_RTOS_Harkonnens_Attack.pintool**: Pin tool configuration file for Simplicity Studio.

#### Configuration Files (`./config/`)
- **cpu_cfg.h**: Configuration settings for the CPU.
- **sl_device_init_dcdc_config.h**: Configuration for the DC-DC converter initialization.
- **pin_config.h**: GPIO pin configuration settings.
- **sl_simple_button_btn0_config.h**: Configuration for the first simple button.
- **sl_simple_button_btn1_config.h**: Configuration for the second simple button.
- **sl_simple_button_config.h**: General configuration for simple buttons.
- **sl_memory_config.h**: Memory configuration settings.
- **rtos_cfg.h**: Configuration for the Real-Time Operating System (RTOS).
- **sl_board_control_config.h**: Board control configuration settings.
- **sl_device_init_lfxo_config.h**: Configuration for the low-frequency crystal oscillator.
- **sl_device_init_emu_config.h**: EMU (Energy Management Unit) initialization configuration.
- **sl_memlcd_usart_config.h**: USART configuration for memory LCD.
- **os_cfg.h**: Operating system configuration settings.
- **rtos_err_cfg.h**: Error configuration settings for the RTOS.
- **sl_sleeptimer_config.h**: Sleep timer configuration.
- **emlib_core_debug_config.h**: Debug configuration for EMLib core.
- **common_cfg.h**: Common configuration settings.
- **sl_device_init_hfxo_config.h**: Configuration for the high-frequency crystal oscillator.

#### SystemView Directory (`./SystemView/`)
- **SEGGER_RTT_Conf.h**: Configuration for SEGGER RTT (Real-Time Transfer).
- **SEGGER_RTT.c**: Source code for SEGGER RTT.
- **SEGGER_SYSVIEW_MicriumOSKernel.c**: SystemView configuration for Micrium OS kernel.
- **SEGGER_RTT_ASM_ARMv7M.S**: Assembly file for SEGGER RTT on ARMv7-M.
- **SEGGER_SYSVIEW_Config_MicriumOSKernel.c**: Configuration for SystemView and Micrium OS kernel.
- **SEGGER_SYSVIEW.c**: Source code for SEGGER SystemView.
- **SEGGER_RTT.h**: Header file for SEGGER RTT.

#### Auto-Generated Files (`./autogen/`)
- **sl_simple_button_instances.c**: Auto-generated instances of simple buttons.
- **RTE_Components.h**: Auto-generated RTE (Run-Time Environment) components header.
- **sl_event_handler.h**: Auto-generated event handler.
- **sl_board_default_init.c**: Auto-generated board default initialization.
- **linkerfile.ld**: Auto-generated linker file.
- **rtos_description.h**: Auto-generated RTOS description.
- **sl_simple_button_instances.h**: Header file for auto-generated button instances.
- **.crc_config.crc**: CRC configuration file.
- **sl_component_catalog.h**: Component catalog header file.
- **sl_device_init_clocks.c**: Clock initialization source file.

#### Source Files (`./src/`)
- **main.c**: Main application source file.
- **Header_Files/**: Directory containing header files for the application.
  - **fifo.h**: Header file for FIFO buffer management.
  - **memlcd_app.h**: Header file for memory LCD application.
  - **bsp_os.h**: Header file for board support package and OS.
  - **app.h**: Header file for the main application.
  - **cmu.h**: Header file for clock management unit.
  - **gpio.h**: Header file for GPIO management.
  - **capsense.h**: Header file for capacitive sensing.
  - **physics.h**: Header file for physics calculations.
- **Source_Files/**: Directory containing source files for the application.
  - **cmu.c**: Source file for clock management unit.
  - **capsense.c**: Source file for capacitive sensing.
  - **gpio.c**: Source file for GPIO management.
  - **physics.c**: Source file for physics calculations.
  - **fifo.c**: Source file for FIFO buffer management.
  - **app.c**: Source file for the main application.
  - **memlcd_app.c**: Source file for memory LCD application.
  - **bsp_os.c**: Source file for board support package and OS.

#### Build Directory (`./GNU ARM v10.2.1 - Default/`)
- Contains build artifacts and intermediate files generated by the GNU ARM toolchain during the build process.
- **Example_LCD_Blink.axf**: Application executable file for the example LCD blink project.
- **WF_RTOS_Lab7_SharedResourse.bin**: Binary file for Lab 7 Shared Resource example.
- **makefile**: Makefile for building the project.

#### Gecko SDK (`./gecko_sdk_3.2.2/`)
- Contains libraries and components provided by the Silicon Labs Gecko SDK.
- **hardware/**: Directory containing hardware abstraction layer components.
- **platform/**: Platform-specific libraries and components, including middleware and RTOS support.
- **micrium_os/**: Directory containing Micrium OS source and configuration files.

Each file and directory is structured to support the development, configuration, and deployment of the game simulation on the EFM32 Pearl Gecko PG Development Board.
