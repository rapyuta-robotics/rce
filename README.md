# Warning : RCE Alpha Deprecated
**Note from the authors :** This is the Alpha version of the cloud engine and is hereby deprecated. A lot has changed since we initially started this project during our university, both from the perspective of the needs and requirements of robots and the underlying technologies RoboEarth Cloud Engine relied on. After a detailed analysis of the current scenario, at this point it is impractical to build on the existing code base and have decided to take a clean room approach to build something new. Drawing from our experience and the success and shortcomings of building the first edition and what we learnt in the interim we are currently in the process of building a faster, more scalable, secure and fault tolerant version of the cloud engine. 
We are excited to inform you that our team at [Rapyuta-Robotics](http://www.rapyuta-robotics.com) is hard at work to release an all new version of our work as "Rapyuta Core" in the coming months into the open-source domain. When ready we will update this repository to reflect the same. Thank you for your support

Content of git respository
==========================

- measure:
    Client-side scripts for a simple communication measurements and plotting.
0111
- rce-client:
    Python and ROS client for the RoboEarth Cloud Engine.
    (Tested with ROS fuerte & groovy)

- rce-comm:
    The files common to rce-core and rce-client.

- rce-core:
    The core files of the RoboEarth Cloud Engine.
    (Tested with ROS fuerte & groovy)

- rce-console:
    A console client for monitoring the Cloud Engine.

- rce-util:
    Utility package of RoboEarth Cloud Engine.

- rce++:
    C++ client for the RoboEarth Cloud Engine.
    
- setup:
    Setup files and utilities that are referenced from the INSTALL file.

- test:
    ROS packages which can be used with the framework. Contains server-side
    files for benchmark.
