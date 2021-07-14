#include <stdio.h>
#include <stdlib.h>
#include "fastcv.h"

int* ball_tracking(uint32_t srcHeight, uint32_t srcWidth, uint8_t * img_data);

int* ball_tracking(uint32_t srcHeight, uint32_t srcWidth, uint8_t * img_data) {

	fcvOperationMode mode = FASTCV_OP_PERFORMANCE;//FASTCV_OP_CPU_PERFORMANCE;
	fcvSetOperationMode(mode);//Selects HW units for all routines at run-time
	fcvMemInit();//Initialize the Memory sub-system in FastCV

        fcvCircle *circles = (fcvCircle *) fcvMemAlloc(sizeof(fcvCircle)*5, 16);
        uint32_t numCircle = 0;
        uint32_t maxCircle = 0;
        uint32_t minDist = 50;
        uint32_t cannyThreshold = 80;
        uint32_t accThreshold = 35;
        uint32_t minRadius = 40;
        uint32_t maxRadius = 150;
	uint32_t dstWidth = 640;
	uint32_t dstHeight = 480;

	
        uint8_t * pSrc = (uint8_t *)fcvMemAlloc(srcWidth*srcHeight, 16);//Allocates aligned memory
        void * data = fcvMemAlloc(dstWidth*dstHeight*16, 16);

	uint8_t * resized_img = (uint8_t *) fcvMemAlloc(dstWidth*dstHeight, 16);

        pSrc = (uint8_t *)img_data;

	fcvScaleDownBLu8(pSrc, srcWidth, srcHeight, 0, resized_img, dstWidth, dstHeight, 0);


	
        fcvHoughCircleu8(resized_img, dstWidth, dstHeight, (uint32_t) 0, circles, &numCircle, maxCircle, minDist, cannyThreshold, accThreshold, minRadius, maxRadius, data);

	printf("numCircle %d\n", numCircle);

	int* result = (int *) malloc(sizeof(int)*3*numCircle);
	if(numCircle == 0) {
	       	result[0] = 0;
		result[1] = 0;
		result[2] = 0;
	}
	else {
		for(unsigned int i=0; i<numCircle; i++) {
			result[0+i*3] = circles[i].x;
			result[1+i*3] = circles[i].y;
			result[2+i*3] = circles[i].radius;
		}
	}
	fcvMemDeInit();//De-Initialize the Memory subsystem initialized by fcvMemInit() function
	fcvCleanUp();//Clean up FastCV resources

	return result;

}




