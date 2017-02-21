#pragma once

#include <VimbaCPP/Include/VimbaCPP.h>
//#include <VimbaImageTransform/Include/VmbTransform.h>


using namespace AVT::VmbAPI;

void checkStatus(const int status)
{
	if (VmbErrorSuccess == status)
	{
		return;
	}

	switch (status)
	{
	case VmbErrorInternalFault: throw std::exception("- 1 Unexpected fault in Vimba or driver"); break;
	case VmbErrorApiNotStarted: throw std::exception(" - 2 Startup was not called before the current command"); break;
	case VmbErrorNotFound: throw std::exception(" - 3 The designated instance(camera, feature etc.) cannot be found"); break;
	case VmbErrorBadHandle: throw std::exception(" - 4 The given handle is not valid"); break;
	case VmbErrorDeviceNotOpen: throw std::exception(" - 5 Device was not opened for usage"); break;
	case VmbErrorInvalidAccess: throw std::exception(" - 6 Operation is invalid with the current access mode"); break;
	case VmbErrorBadParameter: throw std::exception(" - 7 One of the parameters is invalid(usually an illegal pointer)"); break;
	case VmbErrorStructSize: throw std::exception("- 8 The given struct size is not valid for this version of the API"); break;
	case VmbErrorMoreData: throw std::exception(" - 9 More data available in a string / list than space is provided"); break;
	case VmbErrorWrongType: throw std::exception("- 10 Wrong feature type for this access function"); break;
	case VmbErrorInvalidValue: throw std::exception(" - 11 The value is not valid; either out of bounds or not an increment of the minimum"); break;
	case VmbErrorTimeout: throw std::exception(" - 12 Timeout during wait"); break;
	case VmbErrorOther: throw std::exception("- 13 Other error"); break;
	case VmbErrorResources: throw std::exception("- 14 Resources not available(e.g.memory)"); break;
	case VmbErrorInvalidCall: throw std::exception("- 15 Call is invalid in the current context(e.g.callback)"); break;
	case VmbErrorNoTL: throw std::exception(" - 16 No transport layers are found"); break;
	case VmbErrorNotImplemented: throw std::exception("- 17 API feature is not implemented"); break;
	case VmbErrorNotSupported: throw std::exception(" - 18 API feature is not supported"); break;
	case VmbErrorIncomplete: throw std::exception(" - 19 A multiple registers read or write is partially completed"); break;
	default: throw std::exception(" Unknown error"); break;
	}
}

VmbInt64_t readFeature(CameraPtr camera, FeaturePtr feature, const char* name)
{
	VmbInt64_t value;
	checkStatus(camera->GetFeatureByName(name, feature));
	checkStatus(feature->GetValue(value));
	return value;
}
void setFeature(CameraPtr camera, FeaturePtr feature, const char* name, const char* value)
{
	checkStatus(camera->GetFeatureByName(name, feature));
	checkStatus(feature->SetValue(value));
}
void setFeature(CameraPtr camera, FeaturePtr feature, const char* name, VmbInt32_t value)
{
	checkStatus(camera->GetFeatureByName(name, feature));
	checkStatus(feature->SetValue(value));
}

void runCommand(CameraPtr camera, FeaturePtr feature, const char* name)
{
	checkStatus(camera->GetFeatureByName(name, feature));
	checkStatus(feature->RunCommand());
}

void setCamera(CameraPtr camera, FeaturePtr feature, const int width, const int height)
{
	setFeature(camera, feature, "BinningHorizontal", 2);
	setFeature(camera, feature, "BinningVertical", 2);
	setFeature(camera, feature, "PixelFormat", "Mono8");
	//VmbInt32_t MaxWidth = readFeature(camera, feature, "MaxWidth");
	//VmbInt32_t MaxHeight = readFeature(camera, feature, "MaxHeight");
	//setFeature(camera, feature, "Width", MaxWidth);
	//setFeature(camera, feature, "Height", MaxHeight);
	setFeature(camera, feature, "Width", width);
	setFeature(camera, feature, "Height", height);
	
}