#include "kameraFunktioner.h"

using namespace cv;
using namespace std;
using namespace Pylon;

void tagBillede(Mat& grayImage, bool show)
{
    Pylon::PylonAutoInitTerm autoInitTerm;
    try
    {
        CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());

        GenApi::INodeMap& nodemap = camera.GetNodeMap();
        camera.Open();


        GenApi::CIntegerPtr width = nodemap.GetNode("Width");
        GenApi::CIntegerPtr height = nodemap.GetNode("Height");

        camera.MaxNumBuffer = 5;

        CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = PixelType_BGR8packed;

        CPylonImage pylonImage;
        Mat openCvImage;

        /*GenApi::CEnumerationPtr userSet(nodemap.GetNode("UserSetSelector"));

        if(GenApi::IsWritable(userSet))
        {
            userSet->FromString("Off");
        }

        GenApi::CEnumEntryPtr userDefault = nodemap.GetNode("EnumEntry_UserSetSelector_Default");

        GenApi::CEnumerationPtr exposureAuto( nodemap.GetNode( "ExposureAuto"));
        if ( GenApi::IsWritable( exposureAuto))
        {
            exposureAuto->FromString("Off");
        }



        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        if(exposureTime.IsValid())
        {
            if(cameraExposure >= exposureTime->GetMin() && cameraExposure <= exposureTime->GetMax())
            {
                exposureTime->SetValue(cameraExposure);
            }
            else
            {
                exposureTime->SetValue(exposureTime->GetMin());
            }
        }
        else
        {
            std::cout << ">> Failed to set exposure value." << std::endl;
        }*/

        camera.StartGrabbing(5, GrabStrategy_LatestImageOnly);

        CGrabResultPtr ptrGrabResult;

        while(camera.IsGrabbing())
        {
            camera.RetrieveResult(2000, ptrGrabResult, TimeoutHandling_ThrowException);

            if(ptrGrabResult->GrabSucceeded())
            {
                formatConverter.Convert(pylonImage, ptrGrabResult);
                openCvImage = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());

                cvtColor(openCvImage, grayImage, COLOR_BGR2GRAY);
                if(show)
                {
                    namedWindow("test", 1);
                    imshow("test", grayImage);
                    waitKey(0);
                    destroyAllWindows();
                }
                break;
            }
        }
    }
    catch (const GenericException &e)
    {
        // Error handling
        cerr << "An exception occurred." << endl
             << e.GetDescription() << endl;
    }
}
