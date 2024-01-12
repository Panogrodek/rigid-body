# rigid-body
<p align="center">
<b>Rigid body simulation using SFML library for graphics</b><br>
    <img width="680" src="https://github.com/Panogrodek/rigid-body/blob/main/repo/demo.gif">
</p>
  
# How to install
1. Download repository
2. Open build.bat to make visual studio files
3. Compile the project. Binaries should be located in bin folder
# Credits
This rigid body implementation uses code from c# flat physics engine tutorials for collision resolution. <br>
The original repository can be found <a href="https://github.com/twobitcoder101/FlatPhysics-part-24">here </a>
# Overview
This simulation is divided into: <br> <b>
-User input<br>
-Timestep body dynamics update (Fixed Update) <br>
-Broad phase collision detection <br>
-Narrow phase collision detection <br>
-Collision resolution
</b>

# Timestep
The fixed update is done using time accumulator and is located in <a href = "https://github.com/Panogrodek/rigid-body/blob/main/Sandbox/src/Application.cpp">Application.cpp</a>

# Broad phase
Broad phase is performed using dynamic tree from my other project <a href = "https://github.com/Panogrodek/bounding-volume-hierarchies">(here)</a>

# Narrow phase
Narrow phase is performed in <a href = "https://github.com/Panogrodek/rigid-body/blob/main/Sandbox/include/Collision/Collision.cpp">Collision.cpp</a>. <br>
It uses SAT collision detection algorithm.

# Collision resolution
Collision resolution is also done in <a href = "https://github.com/Panogrodek/rigid-body/blob/main/Sandbox/include/Collision/Collision.cpp">Collision.cpp</a>.<br>
It is based on flat physics engine collision resolution, where physical forces are calculated for each rigid body.

