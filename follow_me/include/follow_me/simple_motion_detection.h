/***********************************************************************************************************************
 *
 * Copyright (c) 2022, LIG / MARVIN Team / O. Aycard
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of MARVIN nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */
 
#pragma once

#ifndef ROBAIR_SIMPLE_MOTION_DETECTION_H
#define ROBAIR_SIMPLE_MOTION_DETECTION_H

#include "ros/ros.h"

namespace robair{

class SimpleMotionDetection{

public:
    /**
     * \brief Default Class Constructor.
     */
    SimpleMotionDetection();

    /**
     * \brief Initialization.
     */
    bool init(int &num_beams, float (& r_)[1000], bool curr_robot_moving, bool prev_robot_moving);

    /**
     * \brief Run Motion Detection.
     */  
    void run();

    /**
     * \brief Detect Motion.
     */  
    void detectMotion();

    /**
     * \brief Store Background.
     */  
    void storeBackground();

    /**
     * \brief Reset Motion Dynamic Array.
     */  
    void resetMotion();
    //------------------------------------
    // Getter Functions
    //------------------------------------
    /**
     * \brief Get Dynamic Array.
     * \return bool array dynamic
     */  
    bool *getDynamicrArr(){
        return dynamic;
    }

    //------------------------------------
    // Setter Functions
    //------------------------------------
    /**
     * \brief Set stored_background Value.
     */ 
    void setStoredBackground(bool val){
        stored_background = val;
    }
    /**
     * \brief Set current_robot_moving Value.
     */ 
    void setCurrentRobotMoving(bool val){
        current_robot_moving = val;
    }
    /**
     * \brief Set r Values.
     */ 
    void setR(float (& r_)[1000]){
        std::copy(std::begin(r_), std::end(r_), std::begin(r));;
    }
private:

    /**
     * \brief TODO.
     */  
    bool stored_background;

    /**
     * \brief TODO.
     */  
    float background[1000];

    /**
     * \brief TODO.
     */  
    bool dynamic[1000];
    
    /**
     * \brief Number of Laser Scan Beams.
     */
    int nb_beams;
    
    /**
     * \brief Laser Radius & Angles.
     */
    float r[1000];

    /**
     * \brief Robot Currently Moving Flag.
     */    
    bool current_robot_moving;

    /**
     * \brief TODO.
     */  
    bool previous_robot_moving; 
};
}

#endif
