/*************************************************************************************
  @file		  ProjectorCtrl.hpp                                        
  @author	  Kent Hansen <kenh@teknologisk.dk>
  @date		  2013-11-28
  @version	1.0
  @brief	  Control class for TI LightCrafter 4500 projector

**************************************************************************************/

#ifndef PROJECTOR_CTRL_HPP_
#define PROJECTOR_CTRL_HPP_

#include "lcr_api.h"
#include "lcr_usb.h"
#include "../logger.hpp"


namespace vrm3dvision {

enum class ProjectorPattern { NONE, GREY_CODE, GREY_CODE_INVERTED, RANDOM_NOISE };

class ProjectorCtrl 
{
  public:

    ProjectorCtrl();
    ~ProjectorCtrl();
    bool initialize();
    bool startPattern(ProjectorPattern mode, int splash_index, int start_plane, bool add_ambient_image);
    bool stopPattern();
    bool recoverPattern(ProjectorPattern mode, int startPlane);
    bool isAlive();
    void setExposure(int exposure);

  private:
    bool connect();
    void close();
    bool loadPattern(int numPatterns, int splashLutFrame);
    bool addPatternToLut(int trigger, int patternNo, int bitDepth, int ledSelect, bool invertPattern, bool insertBlack, bool bufferSwap, bool trigOutPrev);
    bool addGreyCodeLut(int splash_index, int startPlane, bool inverted, bool add_ambient_image);
    bool addNoiseLut(int startPlane);

    int trigger_;
    int exposure_;
    int framePeriod_;
    int last_exposure_;
    int last_splash_idx_;
    bool last_add_ambient_image_;
    bool patternRecovered_;
    ProjectorPattern currentMode_;
    
};

} /* namespace vrm3dvision */

#endif /* PROJECTOR_CTRL_HPP_ */
