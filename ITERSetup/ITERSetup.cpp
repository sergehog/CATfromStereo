// Setups cameras into a correct configuration
#include "../common/vimba_helper.h"

using namespace AVT::VmbAPI;

int main()
{
	VimbaSystem &system = VimbaSystem::GetInstance();
	
	checkStatus(system.Startup());

	CameraPtr camera1, camera2;
	FeaturePtr feature1, feature2;


	checkStatus(system.OpenCameraByID("169.254.0.111", VmbAccessModeFull, camera1));
	checkStatus(system.OpenCameraByID("169.254.0.113", VmbAccessModeFull, camera2));


	setFeature(camera1, feature1, "BinningHorizontal", 2);
	setFeature(camera1, feature1, "BinningVertical", 2);
	setFeature(camera1, feature1, "PixelFormat", "Mono8");
	auto width = readFeature(camera1, feature1, "WidthMax");
	auto height = readFeature(camera1, feature1, "HeightMax");
	setFeature(camera1, feature1, "Width", width);
	setFeature(camera1, feature1, "Height", height);
	//setFeature(camera1, feature1, "Width", 960);
	//setFeature(camera1, feature1, "Height", 540);

	setFeature(camera2, feature2, "BinningHorizontal", 2);
	setFeature(camera2, feature2, "BinningVertical", 2);
	setFeature(camera2, feature2, "PixelFormat", "Mono8");
	setFeature(camera2, feature2, "Width", width);
	setFeature(camera2, feature2, "Height", height);
	
	setFeature(camera1, feature1, "AcquisitionMode", "Continuous");
	setFeature(camera1, feature1, "TriggerSource", "Software");
	setFeature(camera1, feature1, "SyncOutSource", "Imaging");

	setFeature(camera2, feature2, "AcquisitionMode", "Continuous");
	setFeature(camera2, feature2, "TriggerSource", "Line1");

	system.Shutdown();

    return 0;
}

