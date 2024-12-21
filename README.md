# OpenGL Orbit Simulation

This is an OpenGL program that lets you fly around in an orbital simulation with realistic physics!

![image](https://github.com/user-attachments/assets/0b81bf7a-23ca-45e9-afd8-9a4e11cc6847)


The program starts in *Sandbox* mode, where you can fly around without restrictions and can spawn as many new spacecrafts as you like. Beware that if all your crafts are destroyed, the simulation ends! 
New spacecraft can be summoned using the right click menu. They will spawn at a random height and speed, and you can adjust and move them around to your liking.

Using the right click menu, you can switch to *Challenge* mode, where you are limited to one spacecraft, and are presented with a target orbit that you must match as best you can to complete the challenge. The acceptance threshold is quite forgiving, so close enough is good enough!

![image](https://github.com/user-attachments/assets/caa1c07c-2e9d-47b6-a745-a825e4c91fe9)


The simulation can be reset using the menu if all crafts are destroyed or if you would like a new target orbit.

## Flight and controls

The spacecraft can be controlled using discrete impulses.

- W - Prograde burn
- S - Retrograde burn
- A - Radial in burn
- D - Radial out burn
- Q - Normal burn
- E - Anti-normal burn

The "throttle" level controls the size of each discrete impulse.

- Z - Throttle up
- X - Throttle down

Time can be sped up and slowed down using *Time Warp*

- . or \> - Increase time warp
- , or \< - Decrease time warp
- / or ? - Cut time warp

When more than one craft is present, you can swap control between them

- ] or } - Switch to next craft
- [ or { - Switch to previous craft

The camera can change focus between the active vessel and Earth

- ~ or ` - Focus current vessel
- TAB - Focus Earth

## Acknowledgements

This project started out as my submission for the final project in my Intro to Computer Graphics class in Oregon State University (CS 550), taught by Prof. Mike Bailey (mjb@cs.oregonstate.edu). 

The source code has been adapted from and built upon CS 4/550's "Sample2022" OpenGL program.

And if it wasn't immediately obvious, the look and feel of the game is *heavily* inspired by the video game Kerbal Space Program.
