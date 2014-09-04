// ==============================================================================================
// This file is part of the VRmagic VRmUsbCam C API v2 Demo Application
// ==============================================================================================
// Main Function
// ----------------------------------------------------------------------------------------------

#include <vrmusbcam2.h>

// some stuff we need here:
#ifdef WIN32
#include <windows.h>
#endif
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstdlib>

// small helper macro to check function for success and call LogExit when function fails
#define VRMEXECANDCHECK(function)\
{\
	if (VRM_SUCCESS!=function)\
	LogExit();\
}

void LogExit()
{
	std::stringstream msg;
	msg << "VRmUsbCam Error: " <<  VRmUsbCamGetLastError();
#ifdef WIN32
	MessageBox(0, msg.str().c_str(), "VRmUsbCam Demo", MB_ICONERROR);
#endif
	std::cerr << msg.str() << "\nApplication exit" << std::endl;
	exit(-1);
}

int main(int argc, char** argv)
{
	// at first, be sure to call VRmUsbCamCleanup() at exit, even in case
	// of an error
	atexit(VRmUsbCamCleanup);

	// read libversion (for informational purposes only)
	VRmDWORD libversion;
	VRMEXECANDCHECK(VRmUsbCamGetVersion(&libversion));

	std::cout << "========================================================" << std::endl
		<< "===         VRmagic VRmUsbCam Device Info            ===" << std::endl
		<< "========================================================" << std::endl
		<< "(v." << libversion << ")" << std::endl << std::endl;

	// uncomment one of the following lines to enable logging features of VRmUsbCam (for customer support)
	//VRmUsbCamEnableLogging(); // save logfile to default location
	//VRmUsbCamEnableLoggingEx("mylogfile.log"); //save logfile to user defined location

	// get connected devices list
	VRmDWORD device_list_size=0;
	VRMEXECANDCHECK(VRmUsbCamGetDeviceKeyListSize(&device_list_size));

	// open all devices
	VRmDeviceKey* p_device_key=0;
	for(VRmDWORD i=0; i<device_list_size;++i)
	{
		VRMEXECANDCHECK(VRmUsbCamGetDeviceKeyListEntry(i, &p_device_key));
		if(!p_device_key->m_busy)
		{
			// read device information
			VRmWORD vendor_id;
			VRMEXECANDCHECK(VRmUsbCamGetVendorId(p_device_key, &vendor_id));

			VRmWORD product_id;
			VRMEXECANDCHECK(VRmUsbCamGetProductId(p_device_key, &product_id));

			VRmWORD group_id;
			VRMEXECANDCHECK(VRmUsbCamGetGroupId(p_device_key, &group_id));

			VRmSTRING serial_string;
			VRMEXECANDCHECK(VRmUsbCamGetSerialString(p_device_key, &serial_string));

			std::cout << "\ndevice IDs: vendor <" << std::hex << vendor_id
				<< "> - product <" << product_id
				<< "> - group <" << group_id
				<< "> - serial <" << std::dec << p_device_key->m_serial << ">" << std::endl;

			std::cout << "device strings: manufacturer <" << p_device_key->mp_manufacturer_str
				<< "> - product <" << p_device_key->mp_product_str
				<< "> - serial <" << serial_string
				<< ">" << std::endl;

			VRmUsbCamDevice device=0;
			VRMEXECANDCHECK(VRmUsbCamOpenDevice(p_device_key, &device));

			// check number of connected sensors
			VRmDWORD num_sensorports=0;
			VRMEXECANDCHECK(VRmUsbCamGetSensorPortListSize(device, &num_sensorports));

			// get source format descriptions for each sensor port
			for(VRmDWORD ii=0; ii<num_sensorports;ii++)
			{
				VRmDWORD port;
				VRMEXECANDCHECK(VRmUsbCamGetSensorPortListEntry(device, ii, &port));
				const char* format_str;
				VRMEXECANDCHECK(VRmUsbCamGetSourceFormatDescription(device, port, &format_str));

				std::cout << "format port#" << port << ": "	<< format_str << std::endl;
			}

			VRmBOOL supported;
			VRMEXECANDCHECK(VRmUsbCamGetPropertySupported(device, VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E, &supported));

			//now select the first sensor port
			if(supported)
			{
				VRmDWORD port;
				VRMEXECANDCHECK(VRmUsbCamGetSensorPortListEntry(device, 0, &port));
				VRmPropId id = VRmPropId(VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_1+port-1);
				VRMEXECANDCHECK(VRmUsbCamSetPropertyValueE(device, VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E, &id));
			}

			// list all supported properties
			std::cout << "\nProperties for first sensor port:\n" << std::endl;
			VRmDWORD size = 0;
			VRMEXECANDCHECK(VRmUsbCamGetPropertyListSize(device, &size));

			for (i=0; i<size; ++i)
			{
				VRmPropId id;
				VRMEXECANDCHECK(VRmUsbCamGetPropertyListEntry(device, i, &id));
				VRmPropInfo info;
				// read property meta information
				VRMEXECANDCHECK(VRmUsbCamGetPropertyInfo(device, id, &info));
				std::cout << "[" << i << "]: 0x" << std::hex << info.m_id << std::dec << ", " << info.m_id_string
					<< " \"" << info.m_description << "\"" << (info.m_writeable?"":" (read-only)") << std::endl;

				// for enum properties, also print out all possible values
				if (info.m_type == VRM_PROP_TYPE_ENUM)
				{
					std::cout << "  =>  Possible values:" << std::endl;
					VRmPropAttribsE attribs;
					VRMEXECANDCHECK(VRmUsbCamGetPropertyAttribsE(device, id, &attribs));
					for (VRmPropId cid=attribs.m_min; cid<=attribs.m_max; cid=VRmPropId(cid+1))
					{
						VRMEXECANDCHECK(VRmUsbCamGetPropertySupported(device, cid, &supported));
						if (!supported)
							continue;
						VRmPropInfo cinfo;
						VRMEXECANDCHECK(VRmUsbCamGetPropertyInfo(device, cid, &cinfo));
						std::cout << "      0x" << std::hex << cinfo.m_id << std::dec << ", " << cinfo.m_id_string << " \""
							<< cinfo.m_description << "\"" << std::endl;
					}
				}
			}

			VRMEXECANDCHECK(VRmUsbCamCloseDevice(device));
		}
		VRMEXECANDCHECK(VRmUsbCamFreeDeviceKey(&p_device_key));
	}
	std::cout << "\nexit." << std::endl;

	return 0;
}
