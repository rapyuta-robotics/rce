//      readText.cxx
//      
//      Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
//      
//      This program is free software; you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation; either version 2 of the License, or
//      (at your option) any later version.
//      
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//      
//      You should have received a copy of the GNU General Public License
//      along with this program; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//      MA 02110-1301, USA.
//      
//      

#include "ros/ros.h"                        // ROS standard library
#include "ReadTextService/QueryReadText.h"  // Service Definition
#include "read_text/text_detect.h"          // Image/Text Reader
#include "cv_bridge/cv_bridge.h"            // OpenCV/ROS bridge
#include <opencv2/core/mat.hpp>             // Used image representation in openCV
#include <boost/shared_ptr.hpp>             // Used shared pointer class which for image conversion

// Global constants; paths to configuration files
string *filenameCorrelation = NULL;
string *filenameWordList = NULL;

// Callback function for service
bool readText(ReadTextService::QueryReadText::Request  &req, ReadTextService::QueryReadText::Response &res)
{
    // Initialize the reader
    DetectText reader = DetectText();
    reader.readLetterCorrelation(filenameCorrelation->c_str());
    reader.readWordList(filenameWordList->c_str());
    
    // Load and convert the received image
    boost::shared_ptr<void const> imgPtr = boost::shared_ptr<void const>();
    cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(req.image, imgPtr, "");
    cv::Mat img = cv::Mat(cvImg->image);
    
    // Free memory
    imgPtr.reset();
    cvImg.reset();
    
    // Scan the image, retrieve the read text and write it to the response message
    reader.detect(img);
    res.text = reader.getWords();
    
    return true;
}

int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "ReadText");
    ros::NodeHandle n;
    
    // Retrieve the configuration from parameter server
    string filename;
    
    filename = "";
    
    if (n.getParam(ros::names::append(ros::this_node::getName(), "correlation"), filename))
        filenameCorrelation = new string(filename);
    
    filename = "";
    
    if (n.getParam(ros::names::append(ros::this_node::getName(), "wordList"), filename))
        filenameWordList = new string(filename);
    
    if (filenameCorrelation == NULL || filenameWordList == NULL)
        return -1;
    
    // Start the service
    ros::ServiceServer service = n.advertiseService("ReadText", readText);
    
    // Enter mainloop
    ros::spin();
    
    delete filenameCorrelation;
    delete filenameWordList;
    
    return 0;
}
