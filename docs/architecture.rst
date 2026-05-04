============
Architecture
============

The arquitecture of the software stack is designed to follow the typical structure of an autonomous vehicle software stack.
Our main objective is to create the most simple functional software that any type of AUV can use. With a focus that everything
could be run on a single edge computer.

Middleware
==========

We use ROS as our main middleware for communication and integration of different components. The software stack is organized into different packages,
each responsible for a specific aspect of the autonomous vehicle's functionality. These packages include perception, planning, control, and simulation components.
As far as we know, we are the first Open Source AUV project.

Simulation
==========

Currently we have Integrations of our own simulations enviroment and our own AUV models. We also have integrated famous
AUVs such as the BlueROV2 from bluerobotics, or the Girona500 from the University of Girona and we have plans to integrate more AUVs in the future.

Mission Planning and Controls
=============================

We developed our own mission planning and control algorithms, with a focus on simplicity and performance.
We believid in simplicity as the best way to structure and hack your own solutions.

Perception
==========

A Combination, of Extended Kalman Filters, YOLO Networks, Inertial Data Fusing, Feature Descriptions extractions and more, to achieve a robust perceptions
system that works mainly with cameras and inertial data. Where the visibility is strongly affected by the water conditions, \
and where the computational resources are limited.

AI Integrations
===============

We have a strong focus on AI integrations, so that we can automattely a lot of the hard problems of buildiing an AUV. Such the use of Video Generation models
to generate synthetic data for training our perceptions models. Or the use of Foundational models like SAM2 or Cotracker for automatic segmentations and labeling of our data.
