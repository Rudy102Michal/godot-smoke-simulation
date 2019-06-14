# Smoke Simulation module for Godot Engine

Module implemented in C++.

Based on Jos Stam's canonical work:
http://www.dgp.toronto.edu/people/stam/reality/Research/pdf/GDC03.pdf

Thanks to Mike Ash for his "Fluid Simulation for Dummies":
https://www.mikeash.com/pyblog/fluid-simulation-for-dummies.html

This is NOT top-notch super-high-performance implementation. It does have bugs.
There seems to be some problems with Advection step, possibly some "corner cases", that are sometimes causing the whole thing to blow or crash.

Made for educational purposes.

In examples folder, there is a GDScript file, showing how this module can be used.