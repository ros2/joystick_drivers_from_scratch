joy
===
Welcome to the ROS 2 joy package.

Usage
*****
Make sure your system has a joystick plugged in, and then you can just run the
node:

::

    joy

Then, in a separate terminal, you can run a node which subscribes to joystick
messages and prints them to the terminal:

::

    joy_printer

At the moment, the ``joy`` node only works on Linux, since it calls the Linux
joystick driver directly.

.. doxygenindex::

.. toctree::
   :maxdepth: 2

