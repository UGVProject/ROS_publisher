#include "flycap.h"
// Opencv Library
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>

using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;
   
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub1 = it.advertise("my_stereo/left/image_raw", 1);
    image_transport::Publisher pub2 = it.advertise("my_stereo/right/image_raw", 1);

    type = atoi(argv[1]);
    //std::cout << "numCap is " << capNum << std::endl;
 //    time(&now);
	// strftime(datestr, 20, "%b. %d, %Y", localtime(&currentdate));
	// timenow = localtime(&now);
	// strftime(timestr,20,"%H %M",timenow);
 //    if(!CreateDirectory(datestr,timestr))
 //    {
 //    std::cout << "Directory is not created!\n" << std::endl;
 //    return -1;
 //    }
    FlyCapture2::BusManager busMgr;
    FlyCapture2::Error error;
    FlyCapture2::PGRGuid guid[2];
    FlyCapture2::Camera cam[2];
    FlyCapture2::CameraInfo camInfo[2];
    FlyCapture2::Property camProperty[2];
    FlyCapture2::Image rawImage[2]; 
    FlyCapture2::Image bgrImage[2];
    FlyCapture2::TriggerMode triggerMode[2];
    // opencv variable
    cv::Mat m_res[2];
    cv::Mat m_show=cv::Mat (1024,1280,CV_8UC1);
    cv::Mat m_raw[2];
    //std::vector<cv::Mat> rawleft;
    //std::vector<cv::Mat> rawright;
    //limit the number of files to capNum
    //rawleft.resize(capNum);
    //rawright.resize(capNum);

    // Get the camera information
    error = busMgr.GetCameraFromSerialNumber(15231263, &guid[0]);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
    error = busMgr.GetCameraFromSerialNumber(15231302, &guid[1]);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    // Connect to a camera
    error = cam[0].Connect(&guid[0]);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    error = cam[1].Connect(&guid[1]);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    if(type == 0)		//type = 0 means output for calibration
    {
    	error = cam[0].GetCameraInfo(&camInfo[0]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		camProperty[0].type = FlyCapture2::SHUTTER;
		camProperty[0].absControl = true;
		camProperty[0].onePush = false;
		camProperty[0].onOff = true;
		camProperty[0].autoManualMode = false;
		camProperty[0].absValue = 100;
		error = cam[0].SetProperty(&camProperty[0]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		error = cam[0].GetProperty(&camProperty[0]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		PrintProperty_l(&camProperty[0]);
	   
		error = cam[1].GetCameraInfo(&camInfo[1]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		camProperty[1].type = FlyCapture2::SHUTTER;
		camProperty[1].absControl = true;
		camProperty[1].onePush = false;
		camProperty[1].onOff = true;
		camProperty[1].autoManualMode = false;
		camProperty[1].absValue = 100;

		error = cam[1].SetProperty(&camProperty[1]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		error = cam[1].GetProperty(&camProperty[1]);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		PrintProperty_r(&camProperty[1]);

		error = cam[0].GetTriggerMode( &triggerMode[0] );

		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		
		// Set left camera to trigger mode 0
		triggerMode[0].onOff = false;
		
		error = cam[0].SetTriggerMode( &triggerMode[0] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		error = cam[1].GetTriggerMode( &triggerMode[1] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}

		// Set right camera to trigger mode 0
		triggerMode[1].onOff = false;

		error = cam[1].SetTriggerMode( &triggerMode[1] );
		if (error != FlyCapture2::PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
			// Display Camera Settings
		PrintCameraInfo_l(&camInfo[0]);
		PrintCameraInfo_r(&camInfo[1]);
		std::cout << "Now We are Calibrating ... \n" << std::endl;
    }
    else{
	    error = cam[0].GetCameraInfo(&camInfo[0]);
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
	    PrintError( error );
	    return -1;
	    }
	    camProperty[0].type = FlyCapture2::SHUTTER;
	    camProperty[0].absControl = true;
	    camProperty[0].onePush = false;
	    camProperty[0].onOff = true;
	    camProperty[0].autoManualMode = false;
	    camProperty[0].absValue = shuttlespeed;
	    error = cam[0].SetProperty(&camProperty[0]);
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
	    PrintError( error );
	    return -1;
	    }
	    error = cam[0].GetProperty(&camProperty[0]);
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
	    PrintError( error );
	    return -1;
	    }
	    PrintProperty_l(&camProperty[0]);

	    error = cam[1].GetCameraInfo(&camInfo[1]);
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
	    PrintError( error );
	    return -1;
	    }
	    camProperty[1].type = FlyCapture2::SHUTTER;
	    camProperty[1].absControl = true;
	    camProperty[1].onePush = false;
	    camProperty[1].onOff = true;
	    camProperty[1].autoManualMode = false;
	    camProperty[1].absValue = shuttlespeed;

	    error = cam[1].SetProperty(&camProperty[1]);
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
	    PrintError( error );
	    return -1;
	    }
	    error = cam[1].GetProperty(&camProperty[1]);
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
	    PrintError( error );
	    return -1;
	    }
	    PrintProperty_r(&camProperty[1]);

	    error = cam[0].GetTriggerMode( &triggerMode[0] );

	    if (error != FlyCapture2::PGRERROR_OK)
	    {
	    PrintError( error );
	    return -1;
	    }

	    // Set left camera to trigger mode 0
	    triggerMode[0].onOff = true;
	    triggerMode[0].mode = 0; //0
	    triggerMode[0].parameter = 0;
	    triggerMode[0].source = 0;
	    triggerMode[0].polarity = 0;    //falling edge

	    error = cam[0].SetTriggerMode( &triggerMode[0] );
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
	    PrintError( error );
	    return -1;
	    }

	    error = cam[1].GetTriggerMode( &triggerMode[1] );
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
	    PrintError( error );
	    return -1;
	    }

	    // Set right camera to trigger mode 0
	    triggerMode[1].onOff = true;
	    triggerMode[1].mode = 0;
	    triggerMode[1].parameter = 0;
	    triggerMode[1].source =0; // 7 means software tirgger. 0 is camera external trigger
	    triggerMode[1].polarity = 0;  //falling edge

	    error = cam[1].SetTriggerMode( &triggerMode[1] );
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
	    PrintError( error );
	    return -1;
	    }
    }
    
    error = cam[0].StartCapture();
    if ( error != FlyCapture2::PGRERROR_OK )
    {
    PrintError( error );
    return -1;
    }

    error = cam[1].StartCapture();
    if ( error != FlyCapture2::PGRERROR_OK )
    {
    PrintError( error );
    return -1;
    }

    sensor_msgs::ImagePtr msg1;
    sensor_msgs::ImagePtr msg2;	
    //ros::Rate loop_rate(100);

    while (nh.ok()) {
        error = cam[0].RetrieveBuffer( &rawImage[0] );
        if (error != FlyCapture2::PGRERROR_OK)
        {
        PrintError(error);
        continue;
        }
        error = cam[1].RetrieveBuffer( &rawImage[1] );
        if (error != FlyCapture2::PGRERROR_OK)
        {
        PrintError( error );
        continue;
        }
        else
        { 
            error = rawImage[0].Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &bgrImage[0] );
            if (error != FlyCapture2::PGRERROR_OK)
            {
                PrintError(error);
                std::cout << "Left convert wrong\n" << std::endl; 
                continue;
            }
            error = rawImage[1].Convert( FlyCapture2::PIXEL_FORMAT_MONO8, &bgrImage[1] );
            if (error != FlyCapture2::PGRERROR_OK)
            {
                PrintError(error);
                std::cout << "Right convert wrong\n" << std::endl; 
                continue;
            }

            unsigned int rowBytes = (double)bgrImage[0].GetReceivedDataSize() / (double)bgrImage[0].GetRows();       
            m_raw[0] = cv::Mat( bgrImage[0].GetRows(), bgrImage[0].GetCols(), CV_8UC1, 
            bgrImage[0].GetData(), rowBytes );
            m_raw[1] = cv::Mat( bgrImage[1].GetRows(), bgrImage[1].GetCols(), CV_8UC1, 
            bgrImage[1].GetData(), rowBytes );
            if( m_raw[0].cols > 640 ) 
            {
                cv::resize( m_raw[0], m_res[0], cv::Size( 640, 640*m_raw[0].rows/m_raw[0].cols ) );
                cv::resize( m_raw[1], m_res[1], cv::Size( 640, 640*m_raw[1].rows/m_raw[1].cols ) );
                
                
                m_show = cv::Mat( m_res[0].rows, m_res[0].cols*2, CV_8UC1);

                m_res[0].copyTo(m_show( cv::Rect( 0, 0, m_res[0].cols, m_res[0].rows ) ) );
                m_res[1].copyTo(m_show( cv::Rect( m_res[0].cols, 0, m_res[1].cols, m_res[1].rows ) ) );
            
            }
            msg1 = cv_bridge::CvImage(std_msgs::Header(), "mono8", m_raw[0]).toImageMsg();
            pub1.publish(msg1);

            msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", m_raw[1]).toImageMsg();
            pub2.publish(msg2);            
            
                    
            //m_raw[0].copyTo(rawleft[currentindex]);
            //m_raw[1].copyTo(rawright[currentindex]);
            currentindex ++;
            
            // if( currentindex >= capNum )
            // {
            //     std::cout << "Saving Pictures..." << std::endl;
            //     for(int i=0; i<currentindex;i++)
            //     {
                    
            //         sprintf(filename0,"%s/L%05d.png",finalarray,i);
            //         sprintf(filename1,"%s/R%05d.png",finalarray,i);
            //         cv::imwrite(filename0,rawleft[i]);
            //         cv::imwrite(filename1,rawright[i]);
            //     }
                
            //     std::cout << "Stop Capturing" << std::endl;
            //     // Stop capturing images
            //     error = cam[0].StopCapture();
            //     if ( error != FlyCapture2::PGRERROR_OK )
            //     {
            //         PrintError( error );
            //         return -1;
            //     }
            //     error = cam[1].StopCapture();
            //     if ( error != FlyCapture2::PGRERROR_OK )
            //     {
            //         PrintError( error );
            //         return -1;
            //     }
                
            //     waitKey(1);
            //     std::cout << "Set Trigger Mode\n" << std::endl;
            //     // Trigger
            //     triggerMode[0].onOff = false;    
            //     error = cam[0].SetTriggerMode( &triggerMode[0] );
            //     if (error != FlyCapture2::PGRERROR_OK)
            //     {
            //         PrintError( error );
            //         return -1;
            //     }
            //     triggerMode[1].onOff = false; 
            //     error = cam[1].SetTriggerMode( &triggerMode[1] );
            //     if (error != FlyCapture2::PGRERROR_OK)
            //     {
            //         PrintError( error );
            //         return -1;
            //     }
            //     std::cout << "Disconnect \n" << std::endl;
            //     // Disconnect the camera
            //     error = cam[0].Disconnect();
            //     if (error != FlyCapture2::PGRERROR_OK)
            //     {
            //         PrintError( error );
            //         return -1;
            //     }

            //     error = cam[1].Disconnect();
            //     if (error != FlyCapture2::PGRERROR_OK)
            //     {
            //         PrintError( error );
            //         return -1;
            //     }
            //     std::cout << "Done! Now exiting..." << std::endl;

            //     break;
            
            // }
        
        
        }
    //ros::spinOnce();
    //loop_rate.sleep();
    }
  return 0;
}

