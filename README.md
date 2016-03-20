# Track+Stack

## Overview

Track+Stack is a multipurpose computer vision utility written in C, utilizing the OpenCV library.  Track+Stack takes a camera stream or video file (multiple formats) as input, tracks templates from a user-selected region of interest (ROI), and stacks a set of such ROIs from a video stream for output to image files or simply viewing on a monitor. Track+Stack was designed for realtime tracking+stacking of astronomical camera images, but it has many other applications.

There are two basic steps in the Track+Stack workflow:

* Template selection
* Template tracking / stacking

Template selection involves selecting an area of an image that you would like Track+Stack to follow in the input image stream from the camera or video file.  

<br>
<br>
Track+Stack attempts to match the template, and captures the ROI with the best match for the template.  An image is captured, and goes into a moving stack window with a user defined size (must be power of 2 for realtime algorithm).  Once images are stacked, they can be viewed on screen, or written to files (png, jpg, etc.).  The stacked window can be scaled so it appears larger on the screen.


## Instructions for Use

```
usage: trackstack [-mzvcnfph] [-i infile] [-a window_size (power of 2)] [-o outfile] [-d camera_index] [-s scale] [-O outformat]
  options:                                          
    --verbose,-v       Additional output on stdout
    --infile,-i        Stack a video file (not specifying -i uses webcam)
    --full,-f          Stack all frames (no window) from infile
    --avgwindow,-a     Number of frames to window (must be power of 2)
    --outfile,-o       Image output file
    --outformat,-O     Image output file (eg. png, bmp, etc.) default=png
    --device,-d        Camera device index (integer)
    --scale,-s         Scale output image size relative to input image (eg. 0.5, or 2.0)
    --coords,-c        Print CSV template position on stdout
    --nostack,-n       Don't stack. Track only.
    --showsource,-z    Show the source image stream.
    --fastmode,-m      Don't roll the windows (faster, but not continuous).
    --help,-h          Print out this help
```

### Template Selection

Templates are selected by dragging a box around the ROI while holding the left mouse button.  To advance a frame in a video, or pull a new image for template selection from a camera press (or hold) the space bar.  When the ROI highlighted by the green rectangle is acceptable, press the right mouse button to finalize selection and go to the tracking/stacking phase.<br><br>

### Tracking + Stacking

Pressing the enter key will write the current image to disk (PNG default), or, if <code>-f</code> was specified, the stacked image will be written when the entire image stream is processed. Windows that can be displayed are: The tracking window, the stack window (in realtime), and the source input.   Pressing the escape key exits the program. 

### Examples

To stack all frames in a particular file, myvideo.avi for example, and output to a file, myvideo[N].png, use: <br><br>
<code>trackstack -i myvideo.avi -f -o myvideo</code>
<br><br>
To stack frames from a (web)camera using a 50 frame window, and output to myvideo[N].png when enter is pressed, use:<br><br>
<code>trackstack -a 64 -o myvideo</code> 
<br><br>
To scale and stack the ROI to 5 times its size on the screen, use:<br><br>
<code>trackstack -s 5</code> 
There's also a track only mode that can output coordinates to stdout, use:
<code>trackstack -c -n</code> 
## Prerequisites

Track+Stack requires pkg-config, Make, gcc, and OpenCV 2.4 libraries.  On Ubuntu, these can be obtained by issuing the following command:

<code>sudo apt-get install pkg-config gcc make libopencv-dev</code> 

Track+Stack has been tested on Ubuntu 14.04 LTS/15.10, with version 2.4 of the OpenCV library.    

