#include "YarpCameraControl.hh"

using namespace gazebo;



GZ_REGISTER_SENSOR_PLUGIN(YarpCameraControl)

/////////////////////////////////////////////////
YarpCameraControl::YarpCameraControl() : SensorPlugin()
{
}

/////////////////////////////////////////////////
YarpCameraControl::~YarpCameraControl()
{
    stopRecording();
    _rpcServer.close();
    delete _callback;
}

/////////////////////////////////////////////////
void YarpCameraControl::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
    _recordingNumber = 0;
    _videoExt = ".mp4";
    _imageExt = ".jpg";


    _isRecording = false;


    // Get the parent sensor.
    _cameraSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!_cameraSensor)
    {
        gzerr << "YarpCameraControl requires a CameraSensor.\n";
        return;
    }


    _camera = _cameraSensor->Camera();

    // Connect to the sensor update event.
    _updateConnection = _cameraSensor->ConnectUpdated(std::bind(&YarpCameraControl::OnUpdate, this));

    // Make sure the parent sensor is active.
    _cameraSensor->SetActive(true);
    _cameraName = _cameraSensor->Name();

    _rpcServerPortName = "/Gazebo/"+ _cameraName +"/rpc:i";
    _rpcServer.open(_rpcServerPortName);
    _callback = new RpcCallback(this);
    _rpcServer.setReader(*_callback);


}

/////////////////////////////////////////////////
void YarpCameraControl::OnUpdate()
{
    if(_isRecording) {
        _camera->SaveFrame(getFrameFilename());
    }
}
/////////////////////////////////////////////////

std::string YarpCameraControl::getFrameFilename()
{
    _relativeRecordingTime = yarp::os::Time::now() - _recordStartTime;
    std::string frameName{"image-"};
    // TODO: Add timestamp to the frames - can't get it to work with ffmpeg command yet.
    // std::string frameName{"image-" + std::to_string(_relativeRecordingTime) + "-"};
    std::stringstream fcStream;
    fcStream << std::setfill ('0') << std::setw (ZERO_PADDING) << _frameCount;
    frameName += fcStream.str();

    ++_frameCount;
    return _imageSaveDir + "/" + frameName + _imageExt;
}

bool YarpCameraControl::startRecording()
{
    if(!_isRecording) {
        _recordStartTime = yarp::os::Time::now();
        _relativeRecordingTime = 0.0;
        _isRecording = true;
        ++_recordingNumber;
        _frameCount = 0;
        return true;
    }
    return false;
}

bool YarpCameraControl::stopRecording()
{
    if(_isRecording) {
        _isRecording = false;
        generateVideoFromImages();
        return true;
    }
    return false;
}

void YarpCameraControl::generateVideoFromImages()
{
    auto fps = std::to_string(30);//(int)std::round(_camera->AvgFPS()));
    auto width = std::to_string(_camera->ImageWidth());
    auto height = std::to_string(_camera->ImageHeight());

    auto frameArgs          = " -framerate " + fps;
    auto inputArgs          = " -i " + _imageSaveDir + "/image-%0" + std::to_string(ZERO_PADDING) + "d" +_imageExt;
    auto resArgs            = " -s " + width + "x" + height;
    auto overwriteArgs      = " -y";
    auto outputArgs         = " " + _saveDir + "/" +_videoName;
    auto nonBlockingArgs    = " & ";
    auto pixFormatArgs      = "";

    // http://superuser.com/questions/533695/how-can-i-convert-a-series-of-png-images-to-a-video-for-youtube#answers-header
    if ((_imageExt == ".png") && (_videoExt == ".mp4")) {
        pixFormatArgs = " -pix_fmt yuv420p";
    }

    auto args = "ffmpeg" + frameArgs + inputArgs + resArgs + overwriteArgs + pixFormatArgs + outputArgs + nonBlockingArgs;

    std::cout << "Generating video with the following arguments:\n" << args << std::endl;

    int uselessInt = system(args.c_str());
}

void YarpCameraControl::parseAndReply(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    auto tag = in.get(0).asString();
    if ( tag == "record" ) {
        parseRecordMessage(in, out);
    } else if ( tag == "get_pose" ) {
        parseGetPoseMessage(in, out);
    } else if ( tag == "set_resolution" ) {
        parseSetResolutionMessage(in, out);
    } else if ( tag == "set_video_format" ) {
        parseSetVideoFormatMessage(in, out);
    } else if ( tag == "set_still_format" ) {
        parseSetStillFormatMessage(in, out);
    } else if ( tag == "help" ) {
        parseHelpMessage(in, out);
    } else {
        parseHelpMessage(in, out);
    }
}

void YarpCameraControl::parseRecordMessage(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if ( in.size() >= 2 ) {
        bool start = in.get(1).asBool();
        if( start ) {
            if ( in.size() >= 3 ) {
                setSaveDir( in.get(2).asString() );
                if ( in.size() == 4 ) {
                    setVideoName( in.get(3).asString() );
                } else {
                    setVideoName(_cameraName + "_" + std::to_string(_recordingNumber));
                }
            } else {
                setSaveDir(boost::filesystem::current_path().string());
                setVideoName(_cameraName + "_" + std::to_string(_recordingNumber));
            }

            if( startRecording() ) {
                out.addInt(SUCCESS);
                out.addString(_saveDir);
                out.addString(_imageSaveDir);
                out.addString(_videoName);
                std::cout << "Recording started." << '\n';
                std::cout << "Files will be saved to: " << _saveDir << '\n';
                std::cout << "Stills will be saved to: " << _imageSaveDir << '\n';
                std::cout << "Video title will be: " << _videoName << '\n';
            } else {
                out.addInt(FAILURE);
            }
        } else {
            if ( stopRecording() ) {
                out.addInt(SUCCESS);
                out.addString(_saveDir);
                out.addString(_imageSaveDir);
                out.addString(_videoName);
                std::cout << "Recording stopped." << '\n';
                std::cout << "Files saved to: " << _saveDir << '\n';
                std::cout << "Stills saved to: " << _imageSaveDir << '\n';
                std::cout << "Video title: " << _videoName << '\n';
                std::cout << "Total recording time: " << _relativeRecordingTime << " seconds" << std::endl;
            } else {
                out.addInt(FAILURE);
            }
        }
    } else {
        out.addInt(FAILURE);
    }
}

void YarpCameraControl::setSaveDir(const std::string& newSaveDir)
{
    auto newDir = boost::filesystem::path(newSaveDir);
    if ( ! boost::filesystem::exists( newDir ) ) {
        boost::filesystem::create_directories(newDir);
    }
    _saveDir = newSaveDir;
}

void YarpCameraControl::setVideoName(const std::string& newVideoName)
{
    _videoName = newVideoName;
    if ( newVideoName.find(_videoExt) == std::string::npos ) {
        _videoName += _videoExt;
    }
    createImageSaveDir( _videoName.substr( 0, _videoName.size()-4 ) );
}

void YarpCameraControl::createImageSaveDir(const std::string& newImageSaveDir)
{
    _imageSaveDir = _saveDir + "/" + newImageSaveDir + "_images";
    auto newDir = boost::filesystem::path(_imageSaveDir);
    if ( ! boost::filesystem::exists( newDir ) ) {
        boost::filesystem::create_directories(newDir);
    } else {
        if ( !boost::filesystem::is_empty(newDir) ) {
            std::cout << "WARNING: Overwriting images in " << newDir << ". Old data will be lost." << '\n';

            boost::filesystem::directory_iterator rdi(newDir);
            boost::filesystem::directory_iterator end_rdi;
            for (; rdi != end_rdi; ++rdi) {
                if (_imageExt.compare(rdi->path().extension().string()) == 0) {
                    try {
                        if( boost::filesystem::is_regular_file(rdi->status()) ) {
                            boost::filesystem::remove(rdi->path());
                        }
                    } catch (const std::exception& ex ) {
                        ex;
                    }
                }
            }
        }
    }

}

void YarpCameraControl::parseGetPoseMessage(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    auto pose = _camera->WorldPose();
    auto pos = pose.Pos();
    auto rot = pose.Rot().Euler();
    out.addDouble(pos[0]); // x
    out.addDouble(pos[1]); // y
    out.addDouble(pos[2]); // z
    out.addDouble(rot[0]); // roll
    out.addDouble(rot[1]); // pitch
    out.addDouble(rot[2]); // yaw
}

void YarpCameraControl::parseSetResolutionMessage(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    int width, height;
    if( in.size()>=2 ) {
        width = in.get(1).asInt();
        if ( in.size()==3 ) {
            height = in.get(2).asInt();
        } else {
            // If height is not given then provide in 16:9 ratio
            height = std::round(width * 9 / 16);
        }
    } else {
        out.addInt(FAILURE);
        return;
    }
    _camera->SetImageSize(width, height);
    if ( (width == _camera->ImageWidth()) && (height == _camera->ImageHeight()) ) {
        out.addInt(SUCCESS);
        out.addInt(width);
        out.addInt(height);
        std::cout << "Recording video in " << width << " x " << height << " resolution." << '\n';
    } else {
        out.addInt(FAILURE);
        std::cout << "Failed to set new resolution." << '\n';
    }

}

void YarpCameraControl::parseSetVideoFormatMessage(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    auto newFormat = in.get(1).asString();
    std::transform(newFormat.begin(), newFormat.end(), newFormat.begin(), ::tolower);
    if ( (newFormat=="mp4") || (newFormat=="webm") ) {
        _videoExt = "." + newFormat;
        out.addInt(SUCCESS);
        out.addString(_videoExt);
        std::cout << "Videos shall be encoded in the " << _videoExt << " format." << std::endl;
    } else {
        out.addInt(FAILURE);
        std::cout << "WARNING: Unable to change video format to " << newFormat << ". Valid options are: mp4, and webm."<< std::endl;
    }
}

void YarpCameraControl::parseSetStillFormatMessage(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    auto newFormat = in.get(1).asString();
    std::transform(newFormat.begin(), newFormat.end(), newFormat.begin(), ::tolower);
    if ( (newFormat=="jpg") || (newFormat=="png") ) {
        _imageExt = "." + newFormat;
        out.addInt(SUCCESS);
        out.addString(_imageExt);
        std::cout << "Still images shall be encoded in the " << _imageExt << " format." << std::endl;
    } else {
        out.addInt(FAILURE);
        std::cout << "WARNING: Unable to change still image format to " << newFormat << ". Valid options are: jpg, and png."<< std::endl;
    }
}

void YarpCameraControl::parseHelpMessage(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{

}



////////////////////////////////////////////////////////////////////////////////////////////
YarpCameraControl::RpcCallback::RpcCallback(YarpCameraControl* camCtrl)
: _camCtrl{camCtrl}
{

}

bool YarpCameraControl::RpcCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle in, out;
    bool ok = in.read(connection);
    if (!ok) return false;
    _camCtrl->parseAndReply(in, out);
    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender!=NULL) {
        out.write(*returnToSender);
    }

    return true;
}
