/* ----------------------------------------------------------------------    
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.    
*    
* $Date:        19. March 2015
* $Revision: 	V.1.4.5
*    
* Project: 	    CMSIS DSP Library    
* Title:	    arm_fir_decimate_fast_q31.c    
*    
* Description:	Fast Q31 FIR Decimator.    
*    
* Target Processor: Cortex-M4/Cortex-M3
*  
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the 
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE. 
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**    
 * @ingroup groupFilters    
 */

/**    
 * @addtogroup FIR_decimate    
 * @{    
 */

/**    
 * @brief Processing function for the Q31 FIR decimator (fast variant) for Cortex-M3 and Cortex-M4.    
 * @param[in] *S points to an instance of the Q31 FIR decimator structure.    
 * @param[in] *PsrcLeft points to the block of input data.    
 * @param[out] *pDstLeft points to the block of output data    
 * @param[in] blockSize number of input samples to process per call.    
 * @return none    
 *    
 * <b>Scaling and Overflow Behavior:</b>    
 *    
 * \par    
 * This function is optimized for speed at the expense of fixed-point precision and overflow protection.    
 * The result of each 1.31 x 1.31 multiplication is truncated to 2.30 format.    
 * These intermediate results are added to a 2.30 accumulator.    
 * Finally, the accumulator is saturated and converted to a 1.31 result.    
 * The fast version has the same overflow behavior as the standard version and provides less precision since it discards the low 32 bits of each multiplication result.    
 * In order to avoid overflows completely the input signal must be scaled down by log2(numTaps) bits (where log2 is read as log to the base 2).    
 *    
 * \par    
 * Refer to the function <code>arm_fir_decimate_q31()</code> for a slower implementation of this function which uses a 64-bit accumulator to provide higher precision.    
 * Both the slow and the fast versions use the same instance structure.    
 * Use the function <code>arm_fir_decimate_init_q31()</code> to initialize the filter structure.    
 */

void arm_fir_decimate_fast_q31_2ch(
  //arm_fir_decimate_instance_q31 * S,
  uint8_t M,
  uint32_t numTaps,
  q31_t *pCoeffs,
  q31_t *pStateInitLeft,
  q31_t *pStateInitRight,
  q31_t * PsrcLeft,
  q31_t * pDstLeft,
  q31_t * PsrcRight,
  q31_t * pDstRight,
  uint32_t blockSize)
{

// common
  q31_t *pb;
  q31_t c0;
  //uint32_t numTaps = S->numTaps;                 /* Number of taps */
  uint32_t i, tapCnt, blkCnt, outBlockSize = blockSize / M;  /* Loop counters */
  uint32_t blkCntN2;

// per channel (left)
  q31_t *pStateLeft = pStateInitLeft;                     /* State pointer */
//  q31_t *pCoeffs = S->pCoeffs;                   /* Coefficient pointer */
  q31_t *pStateLeftCurnt;                            /* Points to the current sample of the state */
  q31_t x0Left;                                /* Temporary variables to hold state and coefficient values */
  q31_t *pxLeft;                                     /* Temporary pointers for state buffer */
  q31_t sum0Left;                                    /* Accumulator */
  q31_t x1Left;
  q31_t acc0Left, acc1Left;
  q31_t *pxLeft0, *pxLeft1;

// per channel (right)
  q31_t *pStateRight = pStateInitRight;                     /* State pointer */
  q31_t *pStateRightCurnt;                            /* Points to the current sample of the state */
  q31_t x0Right;                                /* Temporary variables to hold state and coefficient values */
  q31_t *pxRight;                                     /* Temporary pointers for state buffer */
  q31_t sum0Right;                                    /* Accumulator */
  q31_t x1Right;
  q31_t acc0Right, acc1Right;
  q31_t *pxRight0, *pxRight1;




  /* S->pStateLeft buffer contains previous frame (numTaps - 1) samples */
  /* pStateLeftCurnt points to the location where the new input data should be written */
  pStateLeftCurnt = pStateLeft + (numTaps - 1u);
  pStateRightCurnt = pStateRight + (numTaps - 1u);


  /* Total number of output samples to be computed */

  blkCnt = outBlockSize / 2;
  blkCntN2 = outBlockSize - (2 * blkCnt);

  while(blkCnt > 0u)
  {
    /* Copy decimation factor number of new input samples into the state buffer */
    i = 2 * M;

    do
    {
      *pStateLeftCurnt++ = *PsrcLeft++;
      *pStateRightCurnt++ = *PsrcRight++;


    } while(--i);

    /* Set accumulator to zero */
    acc0Left = 0;
    acc1Left = 0;
    acc0Right = 0;
    acc1Right = 0;

    /* Initialize state pointer */
    pxLeft0 = pStateLeft;
    pxLeft1 = pStateLeft + M;
    pxRight0 = pStateRight;
    pxRight1 = pStateRight + M;

    /* Initialize coeff pointer */
    pb = pCoeffs;

    /* Loop unrolling.  Process 4 taps at a time. */
    tapCnt = numTaps >> 2;

    /* Loop over the number of taps.  Unroll by a factor of 4.       
     ** Repeat until we've computed numTaps-4 coefficients. */
    while(tapCnt > 0u)
    {
      /* Read the b[numTaps-1] coefficient */
      c0 = *(pb);

      /* Read x[n-numTaps-1] for sample 0 sample 1 */
      x0Left = *(pxLeft0);
      x1Left = *(pxLeft1);
      x0Right = *(pxRight0);
      x1Right = *(pxRight1);

      /* Perform the multiply-accumulate */
      acc0Left = (q31_t) ((((q63_t) acc0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      acc1Left = (q31_t) ((((q63_t) acc1Left << 32) + ((q63_t) x1Left * c0)) >> 32);

      acc0Right = (q31_t) ((((q63_t) acc0Right << 32) + ((q63_t) x0Right * c0)) >> 32);
      acc1Right = (q31_t) ((((q63_t) acc1Right << 32) + ((q63_t) x1Right * c0)) >> 32);

      /* Read the b[numTaps-2] coefficient */
      c0 = *(pb + 1u);

      /* Read x[n-numTaps-2]  for sample 0 sample 1  */
      x0Left = *(pxLeft0 + 1u);
      x1Left = *(pxLeft1 + 1u);
      x0Right = *(pxRight0 + 1u);
      x1Right = *(pxRight1 + 1u);

      /* Perform the multiply-accumulate */
      acc0Left = (q31_t) ((((q63_t) acc0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      acc1Left = (q31_t) ((((q63_t) acc1Left << 32) + ((q63_t) x1Left * c0)) >> 32);

      acc0Right = (q31_t) ((((q63_t) acc0Right << 32) + ((q63_t) x0Right * c0)) >> 32);
      acc1Right = (q31_t) ((((q63_t) acc1Right << 32) + ((q63_t) x1Right * c0)) >> 32);

      /* Read the b[numTaps-3] coefficient */
      c0 = *(pb + 2u);

      /* Read x[n-numTaps-3]  for sample 0 sample 1 */
      x0Left = *(pxLeft0 + 2u);
      x1Left = *(pxLeft1 + 2u);

      x0Right = *(pxRight0 + 2u);
      x1Right = *(pxRight1 + 2u);
      pb += 4u;

      /* Perform the multiply-accumulate */
      acc0Left = (q31_t) ((((q63_t) acc0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      acc1Left = (q31_t) ((((q63_t) acc1Left << 32) + ((q63_t) x1Left * c0)) >> 32);

      acc0Right = (q31_t) ((((q63_t) acc0Right << 32) + ((q63_t) x0Right * c0)) >> 32);
      acc1Right = (q31_t) ((((q63_t) acc1Right << 32) + ((q63_t) x1Right * c0)) >> 32);

      /* Read the b[numTaps-4] coefficient */
      c0 = *(pb - 1u);

      /* Read x[n-numTaps-4] for sample 0 sample 1 */
      x0Left = *(pxLeft0 + 3u);
      x1Left = *(pxLeft1 + 3u);

      x0Right = *(pxRight0 + 3u);
      x1Right = *(pxRight1 + 3u);


      /* Perform the multiply-accumulate */
      acc0Left = (q31_t) ((((q63_t) acc0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      acc1Left = (q31_t) ((((q63_t) acc1Left << 32) + ((q63_t) x1Left * c0)) >> 32);

      acc0Right = (q31_t) ((((q63_t) acc0Right << 32) + ((q63_t) x0Right * c0)) >> 32);
      acc1Right = (q31_t) ((((q63_t) acc1Right << 32) + ((q63_t) x1Right * c0)) >> 32);

      /* update state pointers */
      pxLeft0 += 4u;
      pxLeft1 += 4u;

      pxRight0 += 4u;
      pxRight1 += 4u;

      /* Decrement the loop counter */
      tapCnt--;
    }

    /* If the filter length is not a multiple of 4, compute the remaining filter taps */
    tapCnt = numTaps % 0x4u;

    while(tapCnt > 0u)
    {
      /* Read coefficients */
      c0 = *(pb++);

      /* Fetch 1 state variable */
      x0Left = *(pxLeft0++);
      x1Left = *(pxLeft1++);

      x0Right = *(pxRight0++);
      x1Right = *(pxRight1++);

      /* Perform the multiply-accumulate */
      acc0Left = (q31_t) ((((q63_t) acc0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      acc1Left = (q31_t) ((((q63_t) acc1Left << 32) + ((q63_t) x1Left * c0)) >> 32);


      acc0Right = (q31_t) ((((q63_t) acc0Right << 32) + ((q63_t) x0Right * c0)) >> 32);
      acc1Right = (q31_t) ((((q63_t) acc1Right << 32) + ((q63_t) x1Right * c0)) >> 32);
      /* Decrement the loop counter */
      tapCnt--;
    }

    /* Advance the state pointer by the decimation factor       
     * to process the next group of decimation factor number samples */
    pStateLeft = pStateLeft + M * 2;
    pStateRight = pStateRight + M * 2;


    /* The result is in the accumulator, store in the destination buffer. */
    *pDstLeft++ = (q31_t) (acc0Left << 1);
    *pDstLeft++ = (q31_t) (acc1Left << 1);

    *pDstRight++ = (q31_t) (acc0Right << 1);
    *pDstRight++ = (q31_t) (acc1Right << 1);

    /* Decrement the loop counter */
    blkCnt--;
  }

  while(blkCntN2 > 0u)
  {
    /* Copy decimation factor number of new input samples into the state buffer */
    i = M;

    do
    {
      *pStateLeftCurnt++ = *PsrcLeft++;
      *pStateRightCurnt++ = *PsrcRight++;


    } while(--i);

    /* Set accumulator to zero */
    sum0Left = 0;
    sum0Right = 0;
    /* Initialize state pointer */
    pxLeft = pStateLeft;
    pxRight = pStateRight;

    /* Initialize coeff pointer */
    pb = pCoeffs;

    /* Loop unrolling.  Process 4 taps at a time. */
    tapCnt = numTaps >> 2;

    /* Loop over the number of taps.  Unroll by a factor of 4.       
     ** Repeat until we've computed numTaps-4 coefficients. */
    while(tapCnt > 0u)
    {
      /* Read the b[numTaps-1] coefficient */
      c0 = *(pb++);

      /* Read x[n-numTaps-1] sample */
      x0Left = *(pxLeft++);
      x0Right = *(pxRight++);

      /* Perform the multiply-accumulate */
      sum0Left = (q31_t) ((((q63_t) sum0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      sum0Right = (q31_t) ((((q63_t) sum0Right << 32) + ((q63_t) x0Right * c0)) >> 32);


      /* Read the b[numTaps-2] coefficient */
      c0 = *(pb++);

      /* Read x[n-numTaps-2] sample */
      x0Left = *(pxLeft++);
      x0Right = *(pxRight++);


      /* Perform the multiply-accumulate */
      sum0Left = (q31_t) ((((q63_t) sum0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      sum0Right = (q31_t) ((((q63_t) sum0Right << 32) + ((q63_t) x0Right * c0)) >> 32);


      /* Read the b[numTaps-3] coefficient */
      c0 = *(pb++);

      /* Read x[n-numTaps-3] sample */
      x0Left = *(pxLeft++);
      x0Right = *(pxRight++);

      /* Perform the multiply-accumulate */
      sum0Left = (q31_t) ((((q63_t) sum0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      sum0Right = (q31_t) ((((q63_t) sum0Right << 32) + ((q63_t) x0Right * c0)) >> 32);


      /* Read the b[numTaps-4] coefficient */
      c0 = *(pb++);

      /* Read x[n-numTaps-4] sample */
      x0Left = *(pxLeft++);
      x0Right = *(pxRight++);


      /* Perform the multiply-accumulate */
      sum0Left = (q31_t) ((((q63_t) sum0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      sum0Right = (q31_t) ((((q63_t) sum0Right << 32) + ((q63_t) x0Right * c0)) >> 32);


      /* Decrement the loop counter */
      tapCnt--;
    }

    /* If the filter length is not a multiple of 4, compute the remaining filter taps */
    tapCnt = numTaps % 0x4u;

    while(tapCnt > 0u)
    {
      /* Read coefficients */
      c0 = *(pb++);

      /* Fetch 1 state variable */
      x0Left = *(pxLeft++);
      x0Right = *(pxRight++);

      /* Perform the multiply-accumulate */
      sum0Left = (q31_t) ((((q63_t) sum0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      sum0Right = (q31_t) ((((q63_t) sum0Right << 32) + ((q63_t) x0Right * c0)) >> 32);

      /* Decrement the loop counter */
      tapCnt--;
    }

    /* Advance the state pointer by the decimation factor       
     * to process the next group of decimation factor number samples */
    pStateLeft = pStateLeft + M;
    pStateRight = pStateRight + M;

    /* The result is in the accumulator, store in the destination buffer. */
    *pDstLeft++ = (q31_t) (sum0Left << 1);
    *pDstRight++ = (q31_t) (sum0Right << 1);

    /* Decrement the loop counter */
    blkCntN2--;
  }

  /* Processing is complete.       
   ** Now copy the last numTaps - 1 samples to the satrt of the state buffer.       
   ** This prepares the state buffer for the next function call. */

  /* Points to the start of the state buffer */
  pStateLeftCurnt = pStateInitLeft;
  pStateRightCurnt = pStateInitRight;


  i = (numTaps - 1u) >> 2u;

  /* copy data */
  while(i > 0u)
  {
    *pStateLeftCurnt++ = *pStateLeft++;
    *pStateLeftCurnt++ = *pStateLeft++;
    *pStateLeftCurnt++ = *pStateLeft++;
    *pStateLeftCurnt++ = *pStateLeft++;

    *pStateRightCurnt++ = *pStateRight++;
    *pStateRightCurnt++ = *pStateRight++;
    *pStateRightCurnt++ = *pStateRight++;
    *pStateRightCurnt++ = *pStateRight++;

    /* Decrement the loop counter */
    i--;
  }

  i = (numTaps - 1u) % 0x04u;

  /* copy data */
  while(i > 0u)
  {
    *pStateLeftCurnt++ = *pStateLeft++;
    *pStateRightCurnt++ = *pStateRight++;


    /* Decrement the loop counter */
    i--;
  }
}

/**    
 * @} end of FIR_decimate group    
 */
