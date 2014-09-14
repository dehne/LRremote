# The LRremote library for the Arduino

The LRremote library Copyright 2014 by D. L. Ehnebuske

Based heavily on IRRemote v0.11 by Ken Shirriff. See http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html

Restructured and cut down for the simple IR receiver use case of an action function for each button of interest. 

Normal use consists of instantiating a LRremote object in the static part of the sketch, invoking enable() in the 
sketch's setup() function and then invoking onButton() in the sketch's loop() function to get input from the IR remote.
