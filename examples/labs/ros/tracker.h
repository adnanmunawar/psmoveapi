
 /**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "psmove_tracker_opencv.h"
#include "psmove_fusion.h"

extern "C" {
    void cvShowImage(const char *, void*);
};

inline float SIGN(float x) { 
	return (x >= 0.0f) ? +1.0f : -1.0f; 
}

inline float NORM(float a, float b, float c, float d) { 
	return sqrt(a * a + b * b + c * c + d * d); 
}

// Copied from https://gist.github.com/lb5160482/e812a671f778c0c7b14f80612c5389f7
// quaternion = [w, x, y, z]'
void transMat2Quat(float* mat, float* quat) {
	// float r11 = mat[0];
	// float r21 = mat[1];
	// float r31 = mat[2];
	// float r12 = mat[4];
	// float r22 = mat[5];
	// float r32 = mat[6];
	// float r13 = mat[8];
	// float r23 = mat[9];
	// float r33 = mat[10];

	float r11 = mat[0];
	float r21 = mat[4];
	float r31 = mat[8];
	float r12 = mat[1];
	float r22 = mat[5];
	float r32 = mat[9];
	float r13 = mat[2];
	float r23 = mat[6];
	float r33 = mat[10];

	float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
	float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
	float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
	if (q0 < 0.0f) {
		q0 = 0.0f;
	}
	if (q1 < 0.0f) {
		q1 = 0.0f;
	}
	if (q2 < 0.0f) {
		q2 = 0.0f;
	}
	if (q3 < 0.0f) {
		q3 = 0.0f;
	}
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		q0 *= +1.0f;
		q1 *= SIGN(r32 - r23);
		q2 *= SIGN(r13 - r31);
		q3 *= SIGN(r21 - r12);
	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		q0 *= SIGN(r32 - r23);
		q1 *= +1.0f;
		q2 *= SIGN(r21 + r12);
		q3 *= SIGN(r13 + r31);
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		q0 *= SIGN(r13 - r31);
		q1 *= SIGN(r21 + r12);
		q2 *= +1.0f;
		q3 *= SIGN(r32 + r23);
	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
		q0 *= SIGN(r21 - r12);
		q1 *= SIGN(r31 + r13);
		q2 *= SIGN(r32 + r23);
		q3 *= +1.0f;
	}
	else {
		printf("coding error\n");
	}
	float r = NORM(q0, q1, q2, q3);
	q0 /= r;
	q1 /= r;
	q2 /= r;
	q3 /= r;

	quat[0] = q0;
	quat[1] = q1;
	quat[2] = q2;
	quat[3] = q3;
}

void transMat2Pos(float* mat, float* pos){
    pos[0] = mat[12];
    pos[1] = mat[13];
    pos[2] = mat[14];
}

class Tracker
{
    public:
        Tracker(){
            m_move = psmove_connect();
            int quit = 0;

            if (m_move == NULL) {
                fprintf(stderr, "INFO! Could not connect to controller.\n");
            }

            m_tracker = psmove_tracker_new();

            while (psmove_tracker_enable(m_tracker, m_move) != Tracker_CALIBRATED) {
                // Retry calibration until it works
            }

            psmove_enable_orientation(m_move, true);

            m_fusion = psmove_fusion_new(m_tracker, 0.1, 100);

            m_quaternion = new float[4];
            m_position = new float[3];
        }

        void update()
        {
            while (psmove_poll(m_move)) {
                if (psmove_get_buttons(m_move) & Btn_PS) {
                    break;
                }
                if (psmove_get_buttons(m_move) & Btn_MOVE) {
                    psmove_reset_orientation(m_move);
                }
                psmove_get_orientation(m_move, &m_quaternion[0], &m_quaternion[1], &m_quaternion[2], &m_quaternion[3]);
                // psmove_tracker_get_position(tracker, move, &x, &y, &z);

                m_modelViewMat = psmove_fusion_get_modelview_matrix(m_fusion, m_move);
                //                transMat2Quat(m_modelViewMat, m_quaternion);
                transMat2Pos(m_modelViewMat, m_position);
            }
            psmove_tracker_update_image(m_tracker);
            psmove_tracker_update(m_tracker, NULL);
            cvShowImage("asdf", psmove_tracker_opencv_get_frame(m_tracker));
        }

        void cleanup(){
            psmove_tracker_free(m_tracker);
            psmove_disconnect(m_move);
        }
public:
    float* m_quaternion;
    float* m_position;
    float* m_modelViewMat;

private:
    PSMove *m_move;
    PSMoveTracker *m_tracker;
    PSMoveFusion* m_fusion;
};

