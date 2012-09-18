ros-rssimap - work-in-progress
==============================

 A ROS package to enable positioning via wireless Reception Strength Indicators.
Right now this package provides the tools to automatically run the Offline phase
of wireless positioning algorithms. With the help of the
[hector_slam](http://www.ros.org/wiki/hector_slam) project and a portable IMU/Laser Scanner
the position of the wireless receiver is automatically gathered and correlated
with the scanned RSSI values. This provides a RSSI Signal Map of the
environment. Details can be found here,
[http://scholar.google.com/scholar?as_q=%22A+Feasibility+Study+of+Wrist-Worn+Accelerometer+Based+Detection+of+Smoking+Habits%22&btnG=Search+Scholar&as_epq=&as_oq=&as_eq=&as_occt=any&as_sauthors=Scholl[(Fast Indoor Radio-Map Building for
RSSI-based Localization Systems):

> @inproceedings{scholl2012fast,
>   title={Fast indoor radio-map building for RSSI-based localization systems},
>   author={Scholl, P.M. and Kohlbrecher, S. and Sachidananda, V. and Van Laerhoven, K.},
>   booktitle={Networked Sensing Systems (INSS), 2012 Ninth International
>   Conference on},
>   pages={1--2},
>   year={2012},
>   organization={IEEE}
> }
