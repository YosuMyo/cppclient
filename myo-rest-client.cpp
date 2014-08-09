// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include "restclient.h"
#include "json/json.h"
#include <stdlib.h>
#include <time.h>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

std::string myoId = "53e621c7af755b5a17000002";
std::string host = "http://localhost:3000";

static const std::string arms[] = {"armLeft", "armRight", "armUnknown"};
static const std::string xDirections[] = {"xDirectionTowardWrist", "xDirectionTowardElbow", "xDirectionUnknown"};

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
    
    std::string buildURLParameters(const Json::Value & root) {
        std::string parameters;
        auto keys = root.getMemberNames();
        for (auto it = keys.begin(); it != keys.end(); it++) {
            auto key = *it;
            parameters.append(key);
            parameters.append("=");
            parameters.append(root[key].asString());
            if (it != keys.end()) {
                parameters.append("&");
            }
        }
        return parameters;
    }
    
    DataCollector()
    : onArm(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }
    
    /// Called when a Myo has been paired.
    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
        Json::Value root;
        root["eventType"] = "onPair";
        root["timestamp"] = timestamp;
        root["firmwareVersion.firmwareVersionMajor"] = firmwareVersion.firmwareVersionMajor;
        root["firmwareVersion.firmwareVersionMinor"] = firmwareVersion.firmwareVersionMinor;
        root["firmwareVersion.firmwareVersionPatch"] = firmwareVersion.firmwareVersionPatch;
        root["firmwareVersion.firmwareVersionHardwareRev"] = firmwareVersion.firmwareVersionHardwareRev;
        
        RestClient::post(host + "/myo/" + myoId + "/event", "application/x-www-form-urlencoded", buildURLParameters(root));
    }
    
    /// Called when a paired Myo has been connected.
    void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
        Json::Value root;
        root["eventType"] = "onConnect";
        root["timestamp"] = timestamp;
        root["firmwareVersion.firmwareVersionMajor"] = firmwareVersion.firmwareVersionMajor;
        root["firmwareVersion.firmwareVersionMinor"] = firmwareVersion.firmwareVersionMinor;
        root["firmwareVersion.firmwareVersionPatch"] = firmwareVersion.firmwareVersionPatch;
        root["firmwareVersion.firmwareVersionHardwareRev"] = firmwareVersion.firmwareVersionHardwareRev;
        
        RestClient::post(host + "/myo/" + myoId + "/event", "application/x-www-form-urlencoded", buildURLParameters(root));
    }
    
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        
        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(2.0f * (quat.w() * quat.y() - quat.z() * quat.x()));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                          1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
        
        // Convert the floating point angles in radians to a scale from 0 to 20.
        roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
        yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
        
        Json::Value root;
        root["eventType"] = "onOrientationData";
        root["timestamp"] = timestamp;
        root["rotation.x"] = quat.x();
        root["rotation.y"] = quat.y();
        root["rotation.z"] = quat.z();
        root["rotation.w"] = quat.w();
        
        RestClient::post(host + "/myo/" + myoId + "/event", "application/x-www-form-urlencoded", buildURLParameters(root));
    }

    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;

        // Vibrate the Myo whenever we've detected that the user has made a fist.
        if (pose == myo::Pose::fist) {
            myo->vibrate(myo::Myo::vibrationMedium);
        }
        
        Json::Value root;
        root["eventType"] = "onPose";
        root["timestamp"] = timestamp;
        root["pose"] = pose.toString();
        
        RestClient::post(host + "/myo/" + myoId + "/event", "application/x-www-form-urlencoded", buildURLParameters(root));
    }

    // onArmRecognized() is called whenever Myo has recognized a setup gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmRecognized(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
    {
        onArm = true;
        whichArm = arm;
        
        Json::Value root;
        root["eventType"] = "onArmRecognized";
        root["timestamp"] = timestamp;
        root["arm"] = arms[arm];
        root["xDirection"] = xDirections[xDirection];
        
        RestClient::post(host + "/myo/" + myoId + "/event", "application/x-www-form-urlencoded", buildURLParameters(root));
    }

    // onArmLost() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmLost(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
        
        Json::Value root;
        root["eventType"] = "onArmLost";
        root["timestamp"] = timestamp;
        
        RestClient::post(host + "/myo/" + myoId + "/event", "application/x-www-form-urlencoded", buildURLParameters(root));
    }
    
    void onDisconnect(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
        
        Json::Value root;
        root["eventType"] = "onDisconnect";
        root["timestamp"] = timestamp;
        
        RestClient::post(host + "/myo/" + myoId + "/event", "application/x-www-form-urlencoded", buildURLParameters(root));
    }
    
    /// Called when a paired Myo has provided new accelerometer data in units of g.
    void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel) {
        Json::Value root;
        root["eventType"] = "onAccelerometerData";
        root["timestamp"] = timestamp;
        root["accel.x"] = accel.x();
        root["accel.y"] = accel.y();
        root["accel.z"] = accel.z();
        
//        srand(time(nullptr));
//        if (rand() % 2) {
//            RestClient::post(host + "/myo/" + myoId + "/event", "application/x-www-form-urlencoded", buildURLParameters(root));
//        }
    }
    
    /// Called when a paired Myo has provided new gyroscope data in units of deg/s.
    void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro) {
        Json::Value root;
        root["eventType"] = "onGyroscopeData";
        root["timestamp"] = timestamp;
        root["gyro.x"] = gyro.x();
        root["gyro.y"] = gyro.y();
        root["gyro.z"] = gyro.z();
        
//        RestClient::post(host + "/myo/" + myoId + "/event", "application/x-www-form-urlencoded", buildURLParameters(root));
    }
    
    /// Called when a paired Myo has provided a new RSSI value.
    /// @see Myo::requestRssi() to request an RSSI value from the Myo.
    void onRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi) {
        Json::Value root;
        root["eventType"] = "onGyroscopeData";
        root["timestamp"] = timestamp;
        root["rssi"] = rssi;
        
        RestClient::post(host + "/myo/" + myoId + "/event", "application/x-www-form-urlencoded", buildURLParameters(root));
    }

    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';

        // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
        std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
                  << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
                  << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';

        if (onArm) {
            // Print out the currently recognized pose and which arm Myo is being worn on.

            // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
            // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
            // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
            std::string poseString = currentPose.toString();

            std::cout << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
                      << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
        } else {
            // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
            std::cout << "[?]" << '[' << std::string(14, ' ') << ']';
        }

        std::cout << std::flush;
    }

    // These values are set by onArmRecognized() and onArmLost() above.
    bool onArm;
    myo::Arm whichArm;

    // These values are set by onOrientationData() and onPose() above.
    int roll_w, pitch_w, yaw_w;
    myo::Pose currentPose;
};

int main(int argc, char** argv)
{
    auto response = RestClient::get(host + "/myo/" + myoId);
    std::cout << "Response : \n code=" << response.code << ", body=" << response.body << std::endl;
    Json::Reader reader;
    Json::Value root;
    reader.parse(response.body, root);
    std::string id = root["_id"].asString();
    myoId = id;
    
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {

    // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
    // publishing your application. The Hub provides access to one or more Myos.
    myo::Hub hub("com.example.hello-myo");

    std::cout << "Attempting to find a Myo..." << std::endl;

    // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
    // immediately.
    // waitForAnyMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
    // if that fails, the function will return a null pointer.
    myo::Myo* myo = hub.waitForMyo(10000);

    // If waitForAnyMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
    if (!myo) {
        throw std::runtime_error("Unable to find a Myo!");
    }

    // We've found a Myo.
    std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

    // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
    DataCollector collector;

    // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
    // Hub::run() to send events to all registered device listeners.
    hub.addListener(&collector);

    // Finally we enter our main loop.
    while (1) {
        // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
        // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
        hub.run(1000/20);
        // After processing events, we call the print() member function we defined above to print out the values we've
        // obtained from any events that have occurred.
        collector.print();
    }

    // If a standard exception occurred, we print out its message and exit.
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
