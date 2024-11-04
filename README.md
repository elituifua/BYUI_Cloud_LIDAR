# BYUI_Cloud_LIDAR

## Project Description:
Spring 2024 Senior Project (ECEN 499) for the BYUI Physics Department ground-to-cloud lidar distancing project to measure cloud height.

The Brigham Young University Physics department wanted a system that could test the height of the clouds to decide on launching a balloon.

> This overall project consists of 4 parts:
> 1. STM32 NUCLEO-L476RG
> 2. Custom shield layout to connect on top of STM32 board
> 3. Comparator board with photodiode to catch signal pulse
> 4. Laser

The spring 2024 iteration is one that only consists of a red laser and it's respective photodiode comparator circuit. This is to ensure the main focus is for an overall system to work. Using this as a benchmark allows us, or potential future Senior project groups, to have a standard to work from.

## File Structure
MATLAB GUI Files
    app1.mlapp            -Matlab GUI source file
STM32Nucleo
    ECEN499_Lidar_Spring2024
        Core
            SRC
                main.c    -The main file for microcontroller code
                ...
        Debug
        Drivers
        ...
Verilog                   -Contains the codebase for the Verilog tester
README.md

## Team:
  Justin Ackerman (Team Leader), Samuel Adams, Tyler Brady, Andy Clarkson, Brighton Roskelley


## Sponsor:
Todd Lines