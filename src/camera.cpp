#include"camera.h"

//函数实现

void PrintCameraInfo( CameraInfo* pCamInfo ){
  cout << endl;
  cout << "*** CAMERA INFORMATION ***" << endl;
  cout << "Serial number -" << pCamInfo->serialNumber << endl;
  cout << "Camera model - " << pCamInfo->modelName << endl;
  cout << "Camera vendor - " << pCamInfo->vendorName << endl;
  cout << "Sensor - " << pCamInfo->sensorInfo << endl;
  cout << "Resolution - " << pCamInfo->sensorResolution << endl;
  cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
  cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;
}

void PrintFormat7Capabilities( Format7Info fmt7Info ){
  cout << "Max image pixels: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")" << endl;
  cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")" << endl;
  cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")" << endl;
  cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << endl;
}

int CameraInitialize(Error &error, Camera &cam){
  BusManager busMgr;
  unsigned int numCameras;
  const Mode k_fmt7Mode = MODE_0;
  const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_MONO8;
  
  error = busMgr.GetNumOfCameras(&numCameras);
  if(error != PGRERROR_OK){ //未检测到设备
    error.PrintErrorTrace();
    return -1;
  }
  
  cout<<"Number of cameras detected: " << numCameras << endl;
  
  if(numCameras<1){
    cout<<"No Cameras Founded! Please Check and Run Again!"<<endl;
    return -1;
  }
  
  PGRGuid guid;
  error = busMgr.GetCameraFromIndex(0, &guid);
  if(error != PGRERROR_OK){
    error.PrintErrorTrace();
    return -1;    
  }
  
  error = cam.Connect(&guid);
  if(error != PGRERROR_OK){
    error.PrintErrorTrace();
    return -1;     
  }
  
  CameraInfo camInfo;
  error = cam.GetCameraInfo(&camInfo);
  if(error != PGRERROR_OK){
    error.PrintErrorTrace();
    return -1;     
  }
  
  PrintCameraInfo(&camInfo);
  
  Format7Info fmt7Info;
  bool supported;
  fmt7Info.mode = k_fmt7Mode;
  error = cam.GetFormat7Info( &fmt7Info, &supported );
  if(error != PGRERROR_OK){
    error.PrintErrorTrace();
    return -1;     
  }
  
  PrintFormat7Capabilities( fmt7Info );
  
  if((k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0){
    // Pixel format not supported!
    cout << "Pixel format is not supported" << endl;
    return -1;
  }
  
  Format7ImageSettings fmt7ImageSettings;
  fmt7ImageSettings.mode = k_fmt7Mode;
  fmt7ImageSettings.offsetX = 0;
  fmt7ImageSettings.offsetY = 0;
  fmt7ImageSettings.width = fmt7Info.maxWidth;
  fmt7ImageSettings.height = fmt7Info.maxHeight;
  fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

  bool valid;
  Format7PacketInfo fmt7PacketInfo;

  // Validate the settings to make sure that they are valid
  error = cam.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo );
  if (error != PGRERROR_OK){
    error.PrintErrorTrace();
    return -1;
  }
  if ( !valid ){
    // Settings are not valid
    cout << "Format7 settings are not valid" << endl;
    return -1;
  }

  // Set the settings to the camera
  error = cam.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket );
  if (error != PGRERROR_OK){
    error.PrintErrorTrace();
    return -1;
  }
  
  //Start capturing images
  error = cam.StartCapture();
  if(error != PGRERROR_OK){
    error.PrintErrorTrace();
    return -1;
  }
  
  Property frmRate;
  frmRate.type = FRAME_RATE;
  error = cam.GetProperty( &frmRate );
  if (error != PGRERROR_OK){
    error.PrintErrorTrace();
    return -1;
  }
  cout << "Frame rate is " << frmRate.absValue << " fps" << endl;

  return 1;
}

int QueryImages(Error & error, Camera & cam, Image & convertedImage){
  Image rawImage;
  static int image_ID = 1;
  error = cam.RetrieveBuffer( &rawImage );
  if(error != PGRERROR_OK){
    error.PrintErrorTrace();
    return -1;
  }
  
  // Get the raw image dimensions
  PixelFormat pixFormat;
  unsigned int rows, cols, stride;
  rawImage.GetDimensions( &rows, &cols, &stride, &pixFormat );
  
  //Convert the raw image
  error = rawImage.Convert(PIXEL_FORMAT_BGRU, &convertedImage);
  if(error != PGRERROR_OK){
    error.PrintErrorTrace();
    return -1;
  }
  
  // Create a unique filename
//   ostringstream filename;
//   filename << "image" << "-" << image_ID << ".bmp";
//   
//   //Save the image
//   error = convertedImage.Save(filename.str().c_str());
//   if(error != PGRERROR_OK){
//     error.PrintErrorTrace();
//     return -1;
//   }
//   cout<<endl;
//   cout<<"Successfully grabbing image:"<<image_ID<<endl;
//   image_ID = image_ID+1;

  return 1;
}
