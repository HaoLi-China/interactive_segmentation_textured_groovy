/**
 * Copyright (c) 2011, Robert Bosch LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Robert Bosch LLC nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
  * \author Christian Bersch
  */

#ifndef TIMER_H_
#define TIMER_H_

#include <sys/time.h>
#include <time.h>
#include <iostream>

class Timer {
public:
	Timer(){
		 gettimeofday(&tv_start, NULL);
	}

	float stop(){
		 gettimeofday(&tv_end, NULL);
		 int curtimesec = tv_end.tv_sec - tv_start.tv_sec;
		 int curtimeusec = tv_end.tv_usec - tv_start.tv_usec;

		 if (curtimeusec < 0){
			 curtimesec--;
			 curtimeusec = 1000000 + curtimeusec;
		 }


		 float time = (float)  curtimeusec;
		 time /= 1000000;
		 time += curtimesec;
		 return time;

	}

	void start(){
		 gettimeofday(&tv_start, NULL);
	}



private:
	  struct timeval tv_start;
	  struct timeval tv_end;



};

#endif /* TIMER_H_ */
