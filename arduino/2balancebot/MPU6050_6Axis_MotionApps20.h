// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//  2021/09/27 - split implementations out of header files, finally
//  2025/12/27 - Rue Mohr culled the dead code.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _MPU6050_6AXIS_MOTIONAPPS20_H_
#define _MPU6050_6AXIS_MOTIONAPPS20_H_

// take ownership of the "MPU6050" typedef
#define I2CDEVLIB_MPU6050_TYPEDEF

#include "helper_3dmath.h"
#include "MPU6050.h"

class MPU6050_6Axis_MotionApps20 : public MPU6050_Base {
    public:
        MPU6050_6Axis_MotionApps20(uint8_t address=MPU6050_DEFAULT_ADDRESS, void *wireObj=0) : MPU6050_Base(address, wireObj) { }

        uint8_t dmpInitialize();

        // Get Fixed Point data from FIFO   
        uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet=0);
        uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet=0);    
        uint16_t dmpGetFIFOPacketSize();

    private:
        uint8_t *dmpPacketBuffer;
        uint16_t dmpPacketSize;
};

typedef MPU6050_6Axis_MotionApps20 MPU6050;

#endif /* _MPU6050_6AXIS_MOTIONAPPS20_H_ */
