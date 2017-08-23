#include "ros/ros.h"
#include <iostream>
#include <px4_autonomy/Position.h>
#include <px4_autonomy/Velocity.h>
#include <px4_autonomy/Takeoff.h>
#include <std_msgs/UInt8.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>
#include <ARToolKitPlus/TrackerSingleMarker.h>
#include "sensor_msgs/Image.h"

using ARToolKitPlus::TrackerSingleMarker;
using namespace std;

const bool useBCH = false;
static const int width=320, height=240, bpp=1;
static const int numPixels = width *height * bpp;
unsigned char cameraBuffer[numPixels];
const int foundthld=3;
const nummarker 2003

void imageCallback(const sensor_msgs::Image &msg)
{
	printf("callback established\n");
	cout << msg.data.size() << endl;
	for (int i=0;i<numPixels;i++){
		cameraBuffer[i] = 0.2989*msg.data[3*i]+0.5870*msg.data[3*i+1]+0.1140*msg.data[3*i+2];
		//printf("%s\n", );
	}
	printf("successful callback\n");
	
	//memcpy(cameraBuffer,&msg.data[0],msg.data.size()*sizeof(unsigned char));
	//printf("%s\n", (*cameraBuffer));
	//*cameraBuffer = msg.data;//need check! if the whole data is transferred to cameraBuffer
	
};



int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "qr_detection_multi");
    ros::NodeHandle node;
    ros::Subscriber image_sub;
    //ros::Publisher _pub = n.advertise<std_msgs::Bool>("/image_loss",1) ;
    ros::Rate loop_rate(20);
    image_sub = node.subscribe("/usb_cam/image_raw",5,imageCallback);
    float conf;
    size_t numBytesRead;
    int i=0;
    int id=0;
    static bool found = false;
    bool identified[nummarker] = {false};
    //c markerId;
    /*printf("error:creat image instance\n");
    const char *fName = useBCH ? "/home/ubuntu/src/ARToolKitPlus/sample/data/image_320_240_8_marker_id_bch_nr0100.raw"
            : "/home/ubuntu/src/ARToolKitPlus/sample/data/image_320_240_8_marker_id_simple_nr031.raw";
    if (FILE* fp = fopen(fName, "rb")) {
        numBytesRead = fread(cameraBuffer, 1, numPixels, fp);
        fclose(fp);
    } else {
        printf("Failed to open %s\n", fName);
        return -1;
    }
    for(int i=0;i<numPixels;i++){

    	cout << int(cameraBuffer[i]) << endl;
    }*/

    // create a tracker that does:
    //  - 6x6 sized marker images (required for binary markers)
    //  - samples at a maximum of 6x6
    //  - works with luminance (gray) images
    //  - can load a maximum of 0 non-binary pattern
    //  - can detect a maximum of 8 patterns in one imagege
    TrackerMultiMarker tracker(width, height, 8, 6, 6, 6, 0);
    printf("initialized tracker\n");
    tracker.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    //tracker.setLoadUndistLUT(true);
    
    // load a camera file.
    if (!tracker.init("/home/ubuntu/catkin_ws/src/qrcode/data/Logitech_Notebook_Pro.cal", 1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR: init() failed\n");
        return -1;
    }

    tracker.getCamera()->printSettings();

    // define size of the marker in OpenGL units
    tracker.setPatternWidth(2.0);

    // the marker in the BCH test image has a thin border...
    tracker.setBorderWidth(useBCH ? 0.125 : 0.25);

    // set a threshold. alternatively we could also activate automatic thresholding
    tracker.setThreshold(160);

    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    tracker.setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker.setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);

    // do the OpenGL camera setup
    // glMatrixMode(GL_PROJECTION)
    // glLoadMatrixf(tracker.getProjectionMatrix());

    // here we go, just two calls to find the camera pose
    while (ros::ok())
    {
    	//printf("afterrosok\n");
    int numDetected = tracker.calc(cameraBuffer);
    ros::spinOnce();	
    std::vector<int> markerId = tracker.calc(cameraBuffer);
    //markerId.push_back(1);
    int x = tracker.selectBestMarkerByCf();
    conf = tracker.getConfidence();
    if(markerId.empty()){
    	cout << "none idetified" << endl;
    }
    else{
    	if(markerId[0]==id&&!identified[id]){
    		i++;
    		if(i>=foundthld){//continuously identified foundthld times, we save the photo and publish the result
    			i=0;
    			identified[id] = true;
    			//save photo and publish result
    		}
    	}	
    	if(markerId[0]!=id){
    		id = markerId[0];
    	}

    	cout << "Found marker:" << markerId[0] << " at confidence: " << int(conf*100.0f) << "%" << endl;
    	//printf("Found marker %d  (confidence %d%%)\n\  ", markerId[0], (int(conf * 100.0f)));
    }
    // use the result of calc() to setup the OpenGL transformation
    // glMatrixMode(GL_MODELVIEW)
    // glLoadMatrixf(tracker.getModelViewMatrix());

    //printf("\nFound marker %d  (confidence %d%%)\n\nPose-Matrix:\n  ", markerId[0], (int(conf * 100.0f)));
    /*for (int i = 0; i < 16; i++)
        printf("%.2f  %s", tracker.getModelViewMatrix()[i], (i % 4 == 3) ? "\n  " : "");
    */
    loop_rate.sleep();
    }

    return 0;
}