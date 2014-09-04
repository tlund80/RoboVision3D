/*************************************************************************************
  @file		  ProjectorCtrl.cpp                                        
  @author	  Kent Hansen <kenh@teknologisk.dk>
  @date		  2013-11-28
  @version	1.0
  @brief	  Control class for TI LightCrafter 4500 projector
**************************************************************************************/

#include <iostream>
#include "ProjectorCtrl.hpp"

namespace vrm3dvision 
{

ProjectorCtrl::ProjectorCtrl() :
	trigger_(1),
	exposure_(25000),
	framePeriod_(1000000),
	last_exposure_(-1),
	last_splash_idx_(-1),
	last_add_ambient_image_(false),
	patternRecovered_(false),
	currentMode_(ProjectorPattern::NONE)
{
}

ProjectorCtrl::~ProjectorCtrl()
{
  close();
}

bool ProjectorCtrl::initialize()
{
	bool ret = false;
	if(connect())
	{
		LCR_SetPowerMode(0);
		//LCR_SetLedCurrents(255-20, 255-20, 255-20);     // Very low intensity
		//LCR_SetLedCurrents(255-104, 255-135, 255-130);  //TODO Defines (Default)
		LCR_SetLedCurrents(100,0,0);
		LCR_SetMode(1);
		ret = true;
	}
	return ret;
}


bool ProjectorCtrl::connect()
{
  USB_Init();
	if(USB_IsConnected())
		USB_Close();
	USB_Open();

	if(USB_IsConnected())
	{
		LOG_INFO("Connected to LightCrafter4500");
    return true;
  }
  else
  {
    LOG_ERROR("Failed to connect to LightCrafter4500. Is it powered on and connected to computer?");
    return false;
  }
}

void ProjectorCtrl::close()
{
	USB_Close();
	USB_Exit();
}

bool ProjectorCtrl::isAlive()
{
  return connect();
}

bool ProjectorCtrl::addPatternToLut(int trigger, int patternNo, int bitDepth, int ledSelect, bool invertPattern, bool insertBlack, bool bufferSwap, bool trigOutPrev)
{
  if(LCR_AddToPatLut(trigger, patternNo, bitDepth, ledSelect, invertPattern, insertBlack, bufferSwap, trigOutPrev) < 0)
  {
    LOG_ERROR("LCr4500 error: Failed to add pattern to LUT");
    return false;
  }
  return true;
}



bool ProjectorCtrl::addGreyCodeLut(int splash_index, int startPlane, bool inverted, bool add_ambient_image)
{
  int planeOrder[24] = {16,17,18,19,20,21,22,23,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; // For longrun gray code
//  int planeOrder[24] = {6,7,18,19,20,21,22,23,0,1,2,3,4,5,8,9,10,11,12,13,14,15,16,17}; // For normal graycode (level7 first for occlusion mask)
  int numPatterns = 0;

  if(add_ambient_image)
  {
		addPatternToLut(trigger_, planeOrder[23], 1, 7, false, true, true, false);
		numPatterns++;
  }

  int next_plane = startPlane;

  while(next_plane < 24 && numPatterns < 24)
  {
	if(next_plane % 2 == 0 || !inverted)
	{
		addPatternToLut(trigger_, planeOrder[next_plane], 1, 7, false, true, true, false);
		numPatterns++;
		next_plane++;
	}
	if(inverted && numPatterns < 24)
	{
		addPatternToLut(trigger_, planeOrder[next_plane], 1, 7, false, true, true, false);
		numPatterns++;
		next_plane++;
	}
  }

  // Avoid LightCrafter4500 bug
  if(numPatterns == 20)
  {
    addPatternToLut(trigger_, planeOrder[20], 1, 7, false, true, true, false);
    numPatterns++;
  }

  return loadPattern(numPatterns, splash_index); // 17 for normal GrayCode, 15 for LongRun GrayCode
}


bool ProjectorCtrl::addNoiseLut(int startPlane)
{
  addPatternToLut(trigger_, 10, 1, 7, false, true, true, false); //   addPatternToLut(trigger_, startPlane, 1, 7, false, true, true, false);
  return loadPattern(1, 17); //   return loadPattern(1, 14); RANDOM NOISE IN SPLASH IMAGE 14
}



bool ProjectorCtrl::loadPattern(int numPatterns, int splashLutFrame)
{
  unsigned int status;
  unsigned char splashLut[64];
  int numSplashLutEntries = 1;  // Only single splash frame
  splashLut[0] = splashLutFrame;

  if(LCR_SetPatternDisplayMode(0) < 0)
  {
    LOG_ERROR("LCr4500 error: Failed to set pattern display mode.");
    return false;
  }

  if(LCR_SetPatternConfig(numPatterns, true, 1, numSplashLutEntries) < 0)
  {
    LOG_ERROR("LCr4500 error: Failed to set pattern config.");
    return false;
  }

  LOG_INFO("Setting exposure to : " << exposure_);
  if(LCR_SetExpsosure_FramePeriod(exposure_, framePeriod_) < 0)
  {
    LOG_ERROR("LCr4500 error: Failed to set exposure time / frame period.");
    return false;
  }

  if(LCR_SetPatternTriggerMode(1) < 0)
  {
    LOG_ERROR("LCr4500 error: Failed to set pattern trigger mode.");
    return false;
  }

  if(LCR_SendPatLut() < 0)
  {
    LOG_ERROR("LCr4500 error: Failed to send pattern LUT.");
    return false;
  }

  if(LCR_SendSplashLut(&splashLut[0], numSplashLutEntries) < 0)
  {
    LOG_ERROR("LCr4500 error: Failed to send splash LUT.");
    return false;    
  }

  if(LCR_ValidatePatLutData(&status) < 0)
  {
    LOG_ERROR("LCr4500 error: Failed to validate LUT data.");
    if(status & 0x01)
        LOG_ERROR("LCr4500 error: Exposure/Period OOR.");
    if(status & 0x02)
        LOG_ERROR("LCr4500 error: Pattern number OOR.");
    if(status & 0x041)
        LOG_ERROR("LCr4500 error: Cont trig out overlaps black.");
    if(status & 0x08)
        LOG_ERROR("LCr4500 error: Black vector missing.");
    if(status & 0x10)
        LOG_ERROR("LCr4500 error: Period, Exposure diff < 230.");
    return false;
  }
  return true;
}



bool ProjectorCtrl::startPattern(ProjectorPattern mode, int splash_index, int start_plane, bool add_ambient_image)
{
  // Stop pattern sequence
	stopPattern();
  
  // Check if pattern must be updated
  if((mode != currentMode_) || (patternRecovered_ == true) || (start_plane != 0) ||
		  last_exposure_ != exposure_ || last_splash_idx_ != splash_index || last_add_ambient_image_ != add_ambient_image)
  {
	  last_exposure_ = exposure_;
	  last_splash_idx_ = splash_index;
	  last_add_ambient_image_ = add_ambient_image;
	  if(LCR_ClearPatLut() < 0)
    {
      LOG_ERROR("LCr4500 error: Failed to clear pattern LUT.");
      return false;
    }

    switch(mode)
    {
      case ProjectorPattern::GREY_CODE:
        if(addGreyCodeLut(splash_index, start_plane, false, add_ambient_image))
          LOG_INFO("Loaded GREY_CODE pattern");
        break;
      case ProjectorPattern::GREY_CODE_INVERTED:
        if(addGreyCodeLut(splash_index, start_plane, true, add_ambient_image))
          LOG_INFO("Loaded GREY_CODE_INVERTED pattern");
        break;
      case ProjectorPattern::RANDOM_NOISE:
        if(addNoiseLut(start_plane))
          LOG_INFO("Loaded RANDOM_NOISE pattern");
        break;
      default:
        LOG_WARNING("Unknown pattern selected");
        break;
    }
    currentMode_ = mode;
    if(start_plane != 0)
    {
      patternRecovered_ = true;
    }
    else
    {
      patternRecovered_ = false;
    }
  }

  // Start pattern
  if(LCR_PatternDisplay(2) < 0)
  {
      LOG_ERROR("Error starting pattern display");
      return false;
  }

  return true;
}



bool ProjectorCtrl::recoverPattern(ProjectorPattern mode, int start_plane)
{
  return startPattern(mode, last_splash_idx_, start_plane, last_add_ambient_image_);
}



bool ProjectorCtrl::stopPattern()
{
  if(LCR_PatternDisplay(0) < 0)
  {
	  LOG_ERROR("LCr4500 error: Failed to stop pattern display.");
	  return false;
  }

  usleep(15000); // Wait before next LCR command
  return true;
}



void ProjectorCtrl::setExposure(int exposure)
{
	exposure_ = exposure;
}

} /* namespace vrm3dvision */
