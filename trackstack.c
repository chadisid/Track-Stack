#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>
#include <time.h>
#include <assert.h>
#include <limits.h>
#include <signal.h>
#include <math.h>
#include <sys/socket.h>
#include <sys/time.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"

/* HighGUI window name strings */
static const char sourceWindowName[] =  "Source";
static const char currentWindowName[] = "Tracking";
static const char shiftWindowName[] =   "Stacked";

#define UNUSED_PARAMETER(X) (void)X

/* Key constant(s) to compare against cvWaitKey() output */
#define RTSTACK_SPACE_BAR   (0x20)
#define RTSTACK_ENTER       (0x0a)
#define RTSTACK_ESC         (0x1b)

/* Over how many frames to compute frame rate */    
#define IMAGE_FRAME_RATE_COUNT (100)

/* Type of image file to output */
#define RTSTACK_IMAGE_NAME_DEFAULT      shiftWindowName 
#define RTSTACK_IMAGE_FORMAT_DEFAULT    "png"

/* Depth of stacked images */
#define RTSTACK_MATCHED_DEPTH (IPL_DEPTH_32F)
#define RTSTACK_MATCHED_CHANNELS (1)

/* Default number of images to stack together */
#define RTSTACK_DEFAULT_STACK_WINDOW (64)

/* Determine if the number is a power of two - for window size validation */ 
#define IS_POW2(X) (((X & ~(X-1))==X)? X : 0)

/* Round down to the nearest power of two */
#define ROUND_DOWN_POW2(X) ((unsigned int)(pow(2, floor(log(X)/log(2))))) 

typedef enum
{
  RTSTACK_FLAG_FULL    =1<<0,    /* Stack entire file */
  RTSTACK_FLAG_VERBOSE =1<<1,    /* Verbose print output */
  RTSTACK_FLAG_NOSTACK =1<<2,    /* Don't stack, just tracking */
  RTSTACK_FLAG_COORDS  =1<<3,    /* Output coordinates on screen */
  RTSTACK_FLAG_SHOWSRC =1<<4,    /* Display the source input window */
  RTSTACK_FLAG_FAST    =1<<5,    /* Don't roll the window, speeds computation */
} RTSTACK_FLAGS_T;

/* Global state */ 
typedef struct RTStackState_t
{
    IplImage** stackWindowImages; /* The array of images to stack */
    IplImage** stackTemp;         /* Working images for stacking */
    unsigned int stackTempAlloc;  /* Allocated stack temp images */
    unsigned long stackWindow;    /* Number of images to stack */
    unsigned long currentIndex;   /* Current index into stacking window */
    unsigned long totalFrames;    /* Total number of frames pulled */
    const char* outFile;          /* Outfile spec */
    const char* outFormat;        /* Outfile format spec */
    unsigned int outCurrent;      /* Current outfile */
    unsigned char flags;
    float scale;                  /* Flag to scale output image */
} RTSTACKSTATE_T;

static RTSTACKSTATE_T state;

#define RTSTACK_SET_FLAG(FLAG)    (state.flags |= FLAG)
#define RTSTACK_CLEAR_FLAG(FLAG)  (state.flags &= ~(FLAG))
#define RTSTACK_GET_FLAG(FLAG)    (state.flags & FLAG)

/* Socketpair for IPC between GUI events, and the main loop */
static int IPCSocketPair[2] = {-1};

/* Socketpair array indices */
enum
{
    RTSTACK_SOCK_MAIN=0,
    RTSTACK_SOCK_CALLBACK
};

/* Stack loop states */  
enum 
{
    RTSTACK_STATE_INITIAL,
    RTSTACK_STATE_IDENTIFY_TEMPLATE,
    RTSTACK_STATE_STACK,
    RTSTACK_STATE_TERMINATE,
    RTSTACK_STATE_MAX,
};

enum
{
    RTSTACK_EXTSIG_RUN,
    RTSTACK_EXTSIG_TERMINATE,
    RTSTACK_EXTSIG_MAX,
};

/* Indication of terminate from an external signal */
static int RTStackExtSignal = RTSTACK_EXTSIG_RUN;

/* Stack viewport definition states */
enum 
{
    RTSTACK_MOUSE_STATE_INITIAL,
    RTSTACK_MOUSE_STATE_DRAG_INIT,
    RTSTACK_MOUSE_STATE_DRAGGING,
    RTSTACK_MOUSE_STATE_DRAG_COMPLETE,
};

/* Event packet type -- HighGUI mouse callback events are 
 * sent to the main program using this packet structure. */
typedef struct RTStackMouseEventPacket_t
{
    int event;
    int x;
    int y;
    int flags;
} RTSTACKMOUSEEVENTPACKET_T;

/* This function intercepts SIGINT and 
 * sets RTStackState to RTSTACK_STATE_TERMINATE
 * so the main processing loop may exit gracefully */
void RTQuitSignalHandler(int signum) 
{
    UNUSED_PARAMETER(signum);

    if(RTSTACK_GET_FLAG(RTSTACK_FLAG_VERBOSE))
        fprintf(stderr, "Received signal %d:%s\r\n", signum, strsignal(signum));

    if (RTStackExtSignal == RTSTACK_EXTSIG_TERMINATE)
        exit(0); 

    RTStackExtSignal= RTSTACK_EXTSIG_TERMINATE;
}


/* This function is called in main() to set default
 * values for the global state. */
static void RTStackStateDefault(RTSTACKSTATE_T* state)
{
    assert(state != NULL);
    memset(state, 0, sizeof(*state));
    state->stackWindow = RTSTACK_DEFAULT_STACK_WINDOW; 
    state->scale = 0.0; /* No scaling */
    state->outFormat = RTSTACK_IMAGE_FORMAT_DEFAULT;
    state->outFile = RTSTACK_IMAGE_NAME_DEFAULT;
}

/* Writes an image out to a PNG file */
static void RTStackWriteImage(IplImage* image)
{
    int saveRet;
    char filename_array[NAME_MAX];

    /* Synthesize our filename */
    snprintf(filename_array, NAME_MAX, "%s%d.%s", state.outFile, state.outCurrent, state.outFormat);
    filename_array[NAME_MAX - 1] = '\0';

    /* Write the file */
    saveRet = cvSaveImage(filename_array, image, 0); 

    if(RTSTACK_GET_FLAG(RTSTACK_FLAG_VERBOSE))
        fprintf(stdout, "Wrote file %s (%d)\r\n", filename_array, saveRet);    

    state.outCurrent++;
}

/* This function sets up the global socket pair. 
 * It wraps a POSIX socketpair() call and sets
 * the sockets non-blocking. Non-blocking sockets
 * prevent the RTStackProcessImageStream() loop from blocking
 * in recv().
 */ 
static int RTStackSetupSocketpair(void)
{
    int sockRet = -1;

    sockRet = socketpair(AF_UNIX, SOCK_DGRAM, 0, IPCSocketPair);        

    assert(sockRet >= 0);

    if(sockRet < 0)
        return -1;

    sockRet = fcntl(IPCSocketPair[RTSTACK_SOCK_MAIN], F_SETFL, O_NONBLOCK);

    assert(sockRet >= 0);

    if(sockRet < 0)
        return -1;

    sockRet = fcntl(IPCSocketPair[RTSTACK_SOCK_CALLBACK], F_SETFL, O_NONBLOCK);

    assert(sockRet >= 0);

    if(sockRet < 0)
    {
        close(IPCSocketPair[RTSTACK_SOCK_MAIN]);
        return -1;
    } 

    return 0;
}

/* This function closes the global socketpair.
 * This function presumes that the socketpair
 * is initialized to -1 */
static void RTStackCleanupSocketpair(void)
{
    if(IPCSocketPair[RTSTACK_SOCK_MAIN] >= 0)
        close(IPCSocketPair[RTSTACK_SOCK_MAIN]);

    if(IPCSocketPair[RTSTACK_SOCK_CALLBACK] >= 0)
        close(IPCSocketPair[RTSTACK_SOCK_CALLBACK]);
}

/* This function is a callback from OpenCV HighGUI. It is called
 * asynchronously, and is presented to OpenCV through a call to
 * cvSetMouseCallback(). The function uses one end of the non-blocking
 * IPC socketpair to send events to the main processing loop using the 
 * RTSTACKMOUSEEVENTPACKET_T packet format. */
static void RTStackMouseCallback(int event, int x, int y, int flags, void* param)
{
    (void)param;
    RTSTACKMOUSEEVENTPACKET_T mouseEventPacket;

    switch(event)
    {
        /* These are the events that are of interest */
        case CV_EVENT_LBUTTONDOWN:   
        case CV_EVENT_MOUSEMOVE:
        case CV_EVENT_LBUTTONUP:
        case CV_EVENT_RBUTTONDOWN:
            /* Create our IPC packet */
            mouseEventPacket.event = event;        
            mouseEventPacket.x = x;        
            mouseEventPacket.y = y;        
            mouseEventPacket.flags = flags;        

            /* Send mouse events to our processing loop context */ 
            send(IPCSocketPair[RTSTACK_SOCK_CALLBACK], \
                    &mouseEventPacket, sizeof(mouseEventPacket), O_NONBLOCK);

            break;

        default:
            /* Unhandled */
            break;
    }

    return;
}

/* This function wraps cvWaitKey, and is called
 * by functions that require HighGUI key input.
 * The function sleeps 1 ms to ensure the highest
 * image processing throughput. */
static int RTStackProcessKeypress(void)
{
    return (cvWaitKey(1) & 255);
}

static IplImage* RTStackQueryFrame(CvCapture* capture)
{
    IplImage* nextFrame = cvQueryFrame(capture);

    if(nextFrame != NULL)
      state.totalFrames++;

    return nextFrame;
}

/* This function takes a list of images, ImageList, of length ImageListLen,
 * a power of two.  The function stacks images together, two at a time,
 * into TempList, ImageListLen/2 length list of intermediate output images. 
 * These images are stacked together, two at a time, in a 'binary tree' like
 * structure inside the array until only two remain. These last two images
 * are stacked into the final output, SumImage.  Caller is responsible for 
 * allocating ImageList, TempList, and SumImage */
static void RTStackCombineImagesWindowed(IplImage** ImageList,
                            IplImage** TempList,
                            unsigned int ImageListLen,
                            IplImage* SumImage)
{
    unsigned int width = ImageListLen >> 1;
    unsigned int stackIt = 0;

    /* Stack image list on temp list */ 
    for(stackIt = 0; stackIt < width; stackIt++)
      cvAddWeighted(ImageList[stackIt*2], .5, ImageList[(stackIt*2)+1], .5, 0, TempList[stackIt]);

    /* Merge temp entries two at a time until only two remain; SHR 1 is just
     * divide by 2 */
    for(width >>= 1; width > 2; width >>= 1) 
      for(stackIt = 0; stackIt < width; stackIt++)
        cvAddWeighted(TempList[stackIt*2], .5, TempList[(stackIt*2)+1], .5, 0, TempList[stackIt]);
       
    /* Merge final two temp images into the output */
    cvAddWeighted(TempList[0], .5, TempList[1], .5, 0, SumImage);
}

/* This function takes a list of images, ImageList,
 * to stack, or average together, the number of 
 * images in the list, ImageListLen, and the 
 * image to store the results of the average, SumImage. 
 * The caller is responsible for allocating SumImage. */
static void RTStackCombineImages(IplImage** ImageList, 
                            unsigned int ImageListLen,
                            IplImage* SumImage)
{
    unsigned int i = 0;

    for(i=0;i<ImageListLen; i++)
        cvAddWeighted(ImageList[i], 1.0 - (double)((double)i/(double)ImageListLen), SumImage, (double)((double)i/(double)ImageListLen), 0, SumImage);
    
    for(i=0;i<ImageListLen; i++)
        cvAddWeighted(ImageList[(ImageListLen-1)-i], 1.0 - (double)((double)i/(double)ImageListLen), SumImage, (double)((double)i/(double)ImageListLen), 0, SumImage);
}

/* This purpose of this function is to obtain the latest
 * capture frame and incorporate it into the stacked running
 * average. 
 *
 * An image fitness function is indicated as a future development. */
static int RTStackImageStack(CvCapture* capture,
        IplImage* imageCurrent,
        IplImage* imageTemplate)
{
    int returnState = RTSTACK_STATE_STACK;
    IplImage* imageMatched = NULL;
    IplImage* imageTemp= NULL;
    IplImage* imageTemp2= NULL;
    IplImage* imageZoom = NULL;
    CvSize sizeZoom;
    double minVal, maxVal;
    CvPoint minLoc, maxLoc;
    CvRect subRect;
    char write_image = 0;
    assert(capture != NULL);
    assert(imageCurrent != NULL);
    assert(imageTemplate != NULL);
    unsigned long arrayIndex;

    /* Check if a key has been pressed */
    switch(RTStackProcessKeypress())
    {
        /* If space bar is pressed, go back into
         * the 'identify template' state to reset
         * the image template (what we're looking 
         * for specifically in the image stream). */  
        case RTSTACK_ESC:
            {
                if(RTSTACK_GET_FLAG(RTSTACK_FLAG_VERBOSE))
                    fprintf(stdout, "Terminating on user input.");

                return RTSTACK_STATE_TERMINATE;
            }

            break;

            /* Enter depressed, write out the stacked image to
             * <outfile+state.outCount> if outFile has a value. */
        case RTSTACK_ENTER:
            if(state.outFile != NULL)
                write_image = 1;

            break;
    }

    /* Pull the current image out of the device, or video file */
    imageCurrent = RTStackQueryFrame(capture);

    if(RTSTACK_GET_FLAG(RTSTACK_FLAG_SHOWSRC))
      cvShowImage(sourceWindowName, imageCurrent);

    /* If the image is NULL, we have encountered 
     * a fatal error, or an end of file */
    if(imageCurrent == NULL)
    {
        fprintf(stderr, "Error: Fatal: Unable to pull frame from capture stream.\r\n");     
        return RTSTACK_STATE_TERMINATE;
    }


    /* Create our output image */
    imageMatched = cvCreateImage(cvSize((imageCurrent->width - imageTemplate->width) + 1, (imageCurrent->height - imageTemplate->height) + 1), RTSTACK_MATCHED_DEPTH, RTSTACK_MATCHED_CHANNELS);
   
    /* Find the part of the current image that best matches our template */
    cvMatchTemplate(imageCurrent, imageTemplate, imageMatched, CV_TM_SQDIFF_NORMED);

    cvMinMaxLoc(imageMatched, &minVal, &maxVal, &minLoc, &maxLoc, 0);

    subRect = cvRect(minLoc.x, minLoc.y, imageTemplate->width, imageTemplate->height);

    if(RTSTACK_GET_FLAG(RTSTACK_FLAG_COORDS))
        fprintf(stdout, "%d,%d\n", minLoc.x, minLoc.y);

    /* Set the ROI of the current image to the matched rectangle */
    cvSetImageROI(imageCurrent, subRect);

    /* Create an image to add to the rolling array */
    imageTemp = cvCreateImage(cvGetSize(imageCurrent), (imageCurrent)->depth, (imageCurrent)->nChannels);
    cvCopy(imageCurrent, imageTemp, NULL);

    /* Display the image ROI */ 
    cvShowImage(currentWindowName, imageCurrent);  

    /* If we are stacking, perform the operations */
    if(!RTSTACK_GET_FLAG(RTSTACK_FLAG_NOSTACK))
    {
      imageTemp2 = cvCreateImage(cvGetSize(imageCurrent), (imageCurrent)->depth, (imageCurrent)->nChannels);

      /* If we have filled our moving average window, we 
       * remove the first sample (FIFO) in the window 
       * before adding a new one. */
      arrayIndex = state.currentIndex % state.stackWindow;

      if(state.currentIndex >= state.stackWindow)
          cvReleaseImage(&state.stackWindowImages[arrayIndex]);

      /* Put the next image into the window */
      state.stackWindowImages[arrayIndex] = imageTemp;
     
      state.currentIndex++;

      /* If windowed stacking is being done, always show the output window.  If full stacking is done,
       * wait until all ROI images have been collected, and then stack at the end */
      if((RTSTACK_GET_FLAG(RTSTACK_FLAG_FULL) && state.totalFrames >= state.stackWindow) || 
          (!RTSTACK_GET_FLAG(RTSTACK_FLAG_FAST) && state.currentIndex >= state.stackWindow) ||
          (RTSTACK_GET_FLAG(RTSTACK_FLAG_FAST) && state.currentIndex >= state.stackWindow && arrayIndex == state.stackWindow - 1))
      {
          /* If this is the first image, initialize our temp with capture dims */
          if(RTSTACK_GET_FLAG(RTSTACK_FLAG_FULL))
          {
              if(RTSTACK_GET_FLAG(RTSTACK_FLAG_VERBOSE))
                  fprintf(stdout, "Processed full image stack. Writing output.\r\n");     

              /* Write the final stacked image */    
              returnState = RTSTACK_STATE_TERMINATE;
              write_image = 1;

              /* Use the slower 'full' algorithm for combining the entire file */
              RTStackCombineImages(state.stackWindowImages, state.currentIndex, imageTemp2); 
          }
          else
          {
              /* If we have enough images for a stack, allocate temp stack.
               * This is done once (or until we wrap 4.2 bil images) -- TODO */
              if(state.currentIndex == state.stackWindow)
                  for(state.stackTempAlloc = 0; state.stackTempAlloc<state.stackWindow/2; state.stackTempAlloc++)
                      state.stackTemp[state.stackTempAlloc] = cvCreateImage(cvGetSize(imageTemp2), imageTemp2->depth, imageTemp2->nChannels);

              /* Combine our current list of windowed images using stacktemp.
               * This is the faster 'realtime' algorithm. */
              RTStackCombineImagesWindowed(state.stackWindowImages, state.stackTemp, state.stackWindow, imageTemp2); 
          }
          /* If there is a scale factor, scale the image */
          if(state.scale)
          {
              /* Create the image to scale into */
              sizeZoom = cvGetSize(imageTemp2);
              sizeZoom.height = sizeZoom.height * state.scale;
              sizeZoom.width = sizeZoom.width * state.scale;

              imageZoom = cvCreateImage(sizeZoom, (imageTemp2)->depth, (imageTemp2)->nChannels);

              /* Resize the image; linear >= 1, and cubic for < 1 (for best results) */
              if(state.scale >= 1)
                  cvResize(imageTemp2, imageZoom, CV_INTER_LINEAR);
              else if (state.scale < 1)
                  cvResize(imageTemp2, imageZoom, CV_INTER_CUBIC);

              /* Show the zoomed stacked image */
              cvShowImage(shiftWindowName, imageZoom); 

              if(write_image)
                  RTStackWriteImage(imageZoom);

              /* Free the zoom temp */    
              cvReleaseImage(&imageZoom);
          }
          else
          {
              /* Show the stacked image */
              cvShowImage(shiftWindowName, imageTemp2); 

              if(write_image)
                  RTStackWriteImage(imageTemp2);
          }
      }
    
      cvReleaseImage(&imageTemp2);
    }

    /* Free image resources, reset the ROI */
    cvResetImageROI(imageCurrent);
    cvReleaseImage(&imageMatched);
    return returnState;
}


/* This function services the ROI template selection
 * user interface.  It processes keypresses for video queueing,
 * and receives mouse events from the OpenCV GUI callback
 * via one end of a socketpair (IPC).  The function 
 * draws a rectangle around the ROI, when dragged with the 
 * left mouse button held. The ROI is selected for processing
 * when the right mosue button is pressed. When this function
 * returns RTSTACK_STATE_STACK, the state machine begins 
 * processing the stack. */ 
static int RTStackIdentifyTemplate(CvCapture* capture, 
        IplImage** imageCurrent,
        IplImage** imageTemplate,
        IplImage** imageStacked,
        RTSTACKMOUSEEVENTPACKET_T* mouseEventPacket)
{
    static int RTStackMouseState = RTSTACK_MOUSE_STATE_INITIAL;
    static CvPoint dragPoints[2];
    int returnState = RTSTACK_STATE_IDENTIFY_TEMPLATE;
    IplImage* imageTemp = NULL;
    CvRect subRect;
    int tempCoord = 0;

    assert(capture != NULL);
    assert(imageCurrent != NULL);
    assert(imageTemplate != NULL);
    assert(imageStacked != NULL);

    switch(RTStackProcessKeypress())
    {
        /* Space bar depressed, queue to the next image on the camera,
         * or the next image in the video */ 
        case RTSTACK_SPACE_BAR:
            *imageCurrent = RTStackQueryFrame(capture);
            break;
    }

    /* Cue the first frame if we don't have one already */
    if(*imageCurrent == NULL)
        *imageCurrent = RTStackQueryFrame(capture);

    /* If we don't have a frame to get an ROI from, something has gone wrong */
    if(*imageCurrent == NULL)
    {
        fprintf(stderr, "Error: Fatal: Unable to pull template frame from capture stream.\r\n");     
        returnState = RTSTACK_STATE_TERMINATE;
    }
    else
    {
        /* Receive mouse events across the socketpair
         * from the callback RTStackMouseCallback */
        if(mouseEventPacket != NULL)
        {
            switch(mouseEventPacket->event)
            {
                /* Start the ROI selection */
                case CV_EVENT_LBUTTONDOWN:   
                    dragPoints[0] = cvPoint(mouseEventPacket->x, mouseEventPacket->y);
                    RTStackMouseState = RTSTACK_MOUSE_STATE_DRAG_INIT;
                    break;

                    /* ROI selection complete */
                case CV_EVENT_LBUTTONUP:   
                    RTStackMouseState = RTSTACK_MOUSE_STATE_DRAG_COMPLETE;
                    break;

                    /* Update the ROI selection */
                case CV_EVENT_MOUSEMOVE:
                    if(RTStackMouseState == RTSTACK_MOUSE_STATE_DRAG_INIT)
                        RTStackMouseState = RTSTACK_MOUSE_STATE_DRAGGING;

                    if(RTStackMouseState == RTSTACK_MOUSE_STATE_DRAGGING)
                    {
                        dragPoints[1] = cvPoint(mouseEventPacket->x, mouseEventPacket->y);
                        RTStackMouseState = RTSTACK_MOUSE_STATE_DRAGGING;    
                    }
                    break;

                    /* Update the ROI selection */
                case CV_EVENT_RBUTTONDOWN:
                    returnState = RTSTACK_STATE_STACK;
                    RTStackMouseState = RTSTACK_MOUSE_STATE_INITIAL;

                    if(dragPoints[0].x > dragPoints[1].x)
                    {
                        tempCoord = dragPoints[1].x;
                        dragPoints[1].x = dragPoints[0].x;
                        dragPoints[0].x = tempCoord;
                    }

                    if(dragPoints[0].y > dragPoints[1].y)
                    {
                        tempCoord = dragPoints[1].y;
                        dragPoints[1].y = dragPoints[0].y;
                        dragPoints[0].y = tempCoord;
                    }

                    subRect = cvRect(dragPoints[0].x, dragPoints[0].y, dragPoints[1].x - dragPoints[0].x, dragPoints[1].y - dragPoints[0].y);
                    cvSetImageROI(*imageCurrent, subRect);
                    *imageTemplate = cvCreateImage(cvGetSize(*imageCurrent), (*imageCurrent)->depth, (*imageCurrent)->nChannels);
                    *imageStacked = cvCreateImage(cvGetSize(*imageCurrent), (*imageCurrent)->depth, (*imageCurrent)->nChannels); 

                    cvCopy(*imageCurrent, *imageTemplate, NULL);
                    cvResetImageROI(*imageCurrent);

                    if(RTSTACK_GET_FLAG(RTSTACK_FLAG_VERBOSE))
                        fprintf(stdout, "Acquired %d x %d ROI\r\n", subRect.width, subRect.height);

                    fprintf(stdout, "[ESC]   Exits the program.\r\n");
                    fprintf(stdout, "[ENTER] Captures the next screenshot.\r\n");
                    break;

                default:
                    break;
            }
        }

        /* Update the display with the intial image if we haven't started dragging */    
        if(RTStackMouseState == RTSTACK_MOUSE_STATE_INITIAL)
            cvShowImage(currentWindowName, *imageCurrent);
        else if(RTStackMouseState == RTSTACK_MOUSE_STATE_DRAGGING ||
                RTStackMouseState == RTSTACK_MOUSE_STATE_DRAG_COMPLETE)
        {
            /* update the display with ROI rectangle */    
            imageTemp = cvCloneImage(*imageCurrent);
            cvRectangle(imageTemp, dragPoints[0], dragPoints[1], cvScalar(0, 255, 0, 255), 1, 0, 0);
            cvShowImage(currentWindowName, imageTemp);  
            cvReleaseImage(&imageTemp);
        }
    }

    return returnState;
}

static int RTStackProcessImageStream(CvCapture* capture)
{
    IplImage *imageCurrent  = NULL;
    IplImage *imageStacked  = NULL;
    IplImage *imageTemplate = NULL;
    RTSTACKMOUSEEVENTPACKET_T mouseEventPacket;
    RTSTACKMOUSEEVENTPACKET_T* mouseEventPacketParam;
    int sockRet = -1;
    int RTStackState = RTSTACK_STATE_INITIAL;
    struct timespec timeCurrent;
    struct timespec timeStart;
    double frameRate = 0.0;
    double deltaTime = 0.0;
    unsigned int frameDelta = 0;

    cvNamedWindow(currentWindowName, CV_WINDOW_AUTOSIZE);

    if((sockRet = RTStackSetupSocketpair()) < 0)
    {
        fprintf(stderr, "Error: Fatal: Unable to create socket pair: %d\r\n", sockRet); 
        return 1;
    }

    cvSetMouseCallback(currentWindowName, RTStackMouseCallback, NULL);

    while(RTStackState != RTSTACK_STATE_TERMINATE &&
          RTStackExtSignal != RTSTACK_EXTSIG_TERMINATE)
    {
        switch(RTStackState)
        {
            case RTSTACK_STATE_INITIAL:
                RTStackState = RTSTACK_STATE_IDENTIFY_TEMPLATE;

                /* Print instructions for user interface */
                fprintf(stdout, "Drag the ROI with the left mouse button.\r\n");
                fprintf(stdout, "When finished selecting the ROI, click the right mouse button.\r\n");
                fprintf(stdout, "[SPACE] advances to the next video frame/current camera snapshot.\r\n");
                break;

                /* Identify the ROI / template to match */
            case RTSTACK_STATE_IDENTIFY_TEMPLATE:

                sockRet = recv(IPCSocketPair[RTSTACK_SOCK_MAIN], \
                        &mouseEventPacket, sizeof(mouseEventPacket),\
                        O_NONBLOCK);

                if(sockRet > 0)
                    mouseEventPacketParam = &mouseEventPacket;
                else
                    mouseEventPacketParam = NULL;

                RTStackState = RTStackIdentifyTemplate(capture, &imageCurrent,\
                        &imageTemplate,\
                        &imageStacked,\
                        mouseEventPacketParam);

                if(RTStackState == RTSTACK_STATE_STACK)
                    clock_gettime(CLOCK_REALTIME, &timeStart);

                break;

                /* Stack the next image */
            case RTSTACK_STATE_STACK:

                RTStackState = RTStackImageStack(capture, imageCurrent, imageTemplate);

                /* Compute framerate, and print to stdout if verbose option is selected */ 
                if(RTSTACK_GET_FLAG(RTSTACK_FLAG_VERBOSE) && (state.currentIndex % IMAGE_FRAME_RATE_COUNT) == 0)
                {
                    clock_gettime(CLOCK_REALTIME, &timeCurrent);

                    frameDelta = state.currentIndex - frameDelta;

                    deltaTime = (timeCurrent.tv_sec - timeStart.tv_sec) + (timeCurrent.tv_nsec - timeCurrent.tv_nsec)/1000000; 

                    frameRate = frameDelta / deltaTime;

                    fprintf(stdout, "FPS: %f\r\n", frameRate);

                    clock_gettime(CLOCK_REALTIME, &timeStart);

                    frameDelta = state.currentIndex;
                }    

                break;

            default:
                fprintf(stderr, "Error: Fatal: Unknown state %d\r\n", RTStackState);     
                RTStackState = RTSTACK_STATE_TERMINATE;
                break;
        }
    }

    /* Release allocated resources */
    RTStackCleanupSocketpair();
    cvReleaseImage(&imageStacked);
    cvReleaseImage(&imageTemplate);
    cvDestroyWindow(currentWindowName);
    if(!RTSTACK_GET_FLAG(RTSTACK_FLAG_NOSTACK))
        cvDestroyWindow(shiftWindowName);
    if(RTSTACK_GET_FLAG(RTSTACK_FLAG_SHOWSRC))
      cvDestroyWindow(sourceWindowName);

    return 0;
}

static void RTStackCleanup(CvCapture* capture)
{
    unsigned int imgIt = 0;
    unsigned int imgMax = 0;

    cvReleaseCapture(&capture);

    for(imgIt = 0; imgIt < state.stackTempAlloc; imgIt++)
      cvReleaseImage(&(state.stackTemp[imgIt]));
      
    if(state.currentIndex)
    {
      imgMax = state.currentIndex > state.stackWindow ? state.stackWindow : state.currentIndex;
      for(imgIt = 0; imgIt < imgMax; imgIt++)
        cvReleaseImage(&(state.stackWindowImages[imgIt]));
    }

    free(state.stackWindowImages);
    free(state.stackTemp);
}

static void usage(const char* appname)
{
    fprintf(stdout, "usage: %s [-mzvcnfph] [-i infile] [-a window_size (power of 2)] [-o outfile] [-d camera_index] [-s scale] [-O outformat]\r\n", appname);
    fprintf(stdout, "  options:\r\n");
    fprintf(stdout, "    --verbose,-v       Additional output on stdout\r\n");
    fprintf(stdout, "    --infile,-i        Stack a video file (not specifying -i uses webcam)\r\n");
    fprintf(stdout, "    --full,-f          Stack all frames (no window) from infile\r\n");
    fprintf(stdout, "    --avgwindow,-a     Number of frames to window (must be power of 2)\r\n");
    fprintf(stdout, "    --outfile,-o       Image output file\r\n");
    fprintf(stdout, "    --outformat,-O     Image output file (eg. png, bmp, etc.) default=png\r\n");
    fprintf(stdout, "    --device,-d        Camera device index (integer)\r\n");
    fprintf(stdout, "    --scale,-s         Scale output image size relative to input image (eg. 0.5, or 2.0)\r\n");
    fprintf(stdout, "    --coords,-c        Print CSV template position on stdout\r\n");
    fprintf(stdout, "    --nostack,-n       Don't stack. Track only.\r\n");
    fprintf(stdout, "    --showsource,-z    Show the source image stream.\r\n");
    fprintf(stdout, "    --fastmode,-m      Don't roll the windows (faster, but not continuous).\r\n");
    fprintf(stdout, "    --help,-h          Print out this help\r\n\r\n");

    return;
}

int main(int argc, char* argv[])
{
    int c;
    int option_index=0;
    CvCapture* capture = NULL;
    const char* fileName = NULL;
    unsigned int cameraIndex = 0; 
    int exitValue = 1;

    static struct option long_options[] = {
        {"verbose",     no_argument,        0,    'v'},
        {"coords",      no_argument,        0,    'c'},
        {"nostack",     no_argument,        0,    'n'},
        {"full",        no_argument,        0,    'f'},
        {"infile",      required_argument,  0,    'i'},
        {"avgwindow",   required_argument,  0,    'a'},
        {"outfile",     required_argument,  0,    'o'},
        {"outformat",   required_argument,  0,    'O'},
        {"device",      required_argument,  0,    'd'},
        {"scale",       required_argument,  0,    's'},
        {"showsource",  no_argument,        0,    'z'},
        {"fastmode",    no_argument,        0,    'm'},
        {"help",        required_argument,  0,    'h'},
        {0,             0,                  0,    0},
    };

    /* Default Settings */
    RTStackStateDefault(&state);

    /* Setting opterr to 0 prevents getopt from writing errors to stderr */
    opterr = 0;

    /* This is from the man page example */
    while(1)
    {
        c = getopt_long(argc, argv, "mzvcnfhO:i:a:o:d:s:", long_options, &option_index);

        /* Test for end of options */
        if (c == -1)
            break;

        /* Process optino found by getopt_long */
        switch(c)
        {
            case 'i':
                fileName = optarg;
                break;
            
            case 'f':
                RTSTACK_SET_FLAG(RTSTACK_FLAG_FULL);
                break;
            
            case 'm':
                RTSTACK_SET_FLAG(RTSTACK_FLAG_FAST);
                break;

            case 'a':
                state.stackWindow = atoi(optarg);
                if(!IS_POW2(state.stackWindow))
                {
                    fprintf(stderr, "\r\nError: Fatal: stack window must be a power of 2 (eg. 16, 32, 64)\r\n\r\n");
                    return 1;
                }
                break;

            case 'o':
                state.outFile = optarg;
                break;

            case 'O':
                state.outFormat = optarg;
                break;

            case 'd':    
                cameraIndex = atoi(optarg);
                break;
            
            case 'z':    
                RTSTACK_SET_FLAG(RTSTACK_FLAG_SHOWSRC);
                break;
            
            case 'n':
                RTSTACK_SET_FLAG(RTSTACK_FLAG_NOSTACK);
                break;
            
            case 's':
                state.scale= atof(optarg);
                break;

            case 'v':
                RTSTACK_SET_FLAG(RTSTACK_FLAG_VERBOSE);
                break;

            case 'c':
                RTSTACK_SET_FLAG(RTSTACK_FLAG_COORDS);
                break;

            case 'h':
                usage(argv[0]);
                return 0;

            case '?':
            case ':':
            default:
                fprintf(stderr, "\r\nError: Fatal: Invalid command syntax.\r\n\r\n");
                usage(argv[0]);
                return 1;
        }

    }
  
    /* If there's not a filename specified, pull it out of the camera for live stacking */
    if(fileName != NULL)
        capture = cvCaptureFromFile(fileName);
    else
        capture = cvCaptureFromCAM(cameraIndex);
   
    signal(SIGINT, RTQuitSignalHandler); 

    /* Ensure the capture has been successfully allocated */
    if(capture == NULL)
    {
        if(fileName)
            fprintf(stderr, "Error: Fatal: Unable to load capture file: %s\r\n", fileName);
        else
            fprintf(stderr, "Error: Fatal: Unable to open camera at index: %d\r\n", cameraIndex);

        exit(1);
    }
    
    /* If we select full for the video, get the number of frames in the sequence
     * so that number of frames can be allocated to stack */
    if(fileName != NULL && RTSTACK_GET_FLAG(RTSTACK_FLAG_FULL))
        state.stackWindow = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT);

    /* Allocate the required number of images, either the window size, or the entire video */
    state.stackWindowImages = (IplImage**)malloc(sizeof(IplImage*) * state.stackWindow);

    /* Allocate working images. We need half the size of the number of windows 
     * in the stack since we combine each of the images, two at a time. */
    if(state.stackWindowImages == NULL)
    {
        cvReleaseCapture(&capture);
        fprintf(stderr, "Error: Fatal: Unable to allocate window buffer.\r\n");
        exit(1);
    }
    else
    {
        state.stackTemp = (IplImage**)malloc((sizeof(IplImage*)) * (state.stackWindow/2));
    
        if(state.stackTemp == NULL)
        {
            cvReleaseCapture(&capture);
            free(state.stackWindowImages);
            fprintf(stderr, "Error: Fatal: Unable to allocate temp window buffer.\r\n");
            exit(1);
        }
    }

    /* Main loop */
    exitValue = RTStackProcessImageStream(capture);

    RTStackCleanup(capture);

    exit(exitValue);
}

