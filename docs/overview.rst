========
Overview
========

About us
=======

Rumarino Autonomy Stack is an open-source software stack for autonomous vehicles, developed by Rumarino.
A Research and Development team of the University of Puerto Rico at Mayaguez. We are a group of students
with a passion for robotics and autonomy, dedicated to advancing the state of the art in autonomous vehicle technology.


This repository contains the core building blocks for building an underwater autonomous vehicle, including perception,
planning, control, and simulation components.

Something that you will find in this repository is a collection of ideas and integrations of what we think is how these type of systems should be built.
The team originated from a desire to build an autonomous underwater vehicle for the RoboSub competition. We have gained experienced in the field of autonomy and robotics through our 
participation in the competition. Many of our core Developers are students from the University and thanks to to this experience we have been able to build a
strong foundation and experience in the field of autonomy and software development. If you want to learn about our core developer team check out our website: https://rumarino.com/ and our GitHub:



As far as we know, we are the first Open Source AUV project, that have all the necessary components to build, test and deploy an AUV.
With many of the components being developed by us, and some of them being integrations of other open source projects. 

In comparison to other open source AUV projects, like BlueOS, uuv_simulator, ardusub. We have a more complete software stack,

Where a lot of this projects are very good at either, control systems, navigations, simulations. None of them offer a full solution to the 
problems of building an AUV,


Some of the features that we offer in our code are:
- A stack that is fully integrated in Simulations and hardware, so that debugging in both environments is easy and seamless.
- A stack that Integrates a lot of pherifheral plugins or standlone applications so that Iterations are faster and easier.
- A Modern view of AI integrations in robotics, with a focus on performance ad real time applications.
- A team 






Architecture
============
The arquitecture  of the software stack  is designed to follow the typical structure of an autonomous vehicle software stack.
Our main objective is to create the most simple functional software that any type of AUV can use. With a focus that everything 
could be run on a single edge computer.

Middleware:

We use ROS as our main middleware for communication and integration of different components. The software stack is organized into different packages, 
each responsible for a specific aspect of the autonomous vehicle's functionality. These packages include perception, planning, control, and simulation components.
As far as we know, we are the first Open Source AUV project 


Simulation:

Currently we have Integrations of our own simulations enviroment and our own AUV models. We also have integrated famous 
AUVs such as the BlueROV2 from  bluerobotics , or the Girona500 from the University of Girona and we have plans to integrate more AUVs in the future.



Mission Planning and Controls:

We developed our own mission planning and control algorithms, with a focus on simplicity and performance.
We believid in simplicity as the best way to structure  and hack your own solutions.


Perception:

A Combination, of Extended Kalman Filters, YOLO Networks, Inertial Data Fusing, Feature Descriptions extractions and more, to achieve a robust perceptions
system that works mainly with cameras and inertial data. Where the visibility  is strongly affected by the water conditions, \
and where the computational resources are limited.



AI Integrations:
We have a strong focus on AI integrations, so that we can automattely a lot of the hard problems of buildiing an AUV. Such the use of Video Generation models
to generate synthetic data for training our perceptions models. Or the use of Foundational models like SAM2 or Cotracker for automatic segmentations and labeling of our data.







Sponsors
==============
The team has been sponsors trhoughout the years by many companies and organizations, and we are constantly looking for new sponsors to help us continue our work.
All this worked is mainly done by students that vlounteer their time and effort to build and inovate different ways to build an AUV.
And to run our experiments or buy the necessary hardware for our tests, we rely on the support of our sponsors. We also are determined to win the 
RoboSub competition, and we are always looking for new sponsors to help us achieve our goals. If you are interested in sponsoring our team, please contact us through our website or our GitHub.