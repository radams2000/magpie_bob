
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
 * @param[in] *pSrcLeft points to the block of input data.    
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

void arm_fir_decimate_fast_q31_HB_1ch( // ***** RESTRICTIONS, Number of taps must be 4N+3, and the center tap must be 0.5. Not checked
  //arm_fir_decimate_instance_q31_2ch * S,
  uint32_t numTaps,
  q31_t *pCoeffs,
  q31_t *pStateInitLeft,
  q31_t * pSrcLeft,
  q31_t * pDstLeft,
  uint32_t blockSize)
{

// common
  q31_t c0;
  q31_t *pb;
  //q31_t *pCoeffs = S->pCoeffs;                   /* Coefficient pointer */
  //uint32_t numTaps = S->numTaps;                 /* Number of taps, for halfband must be ODD */
  // uint32_t i, tapCnt, blkCnt, outBlockSize = blockSize / S->M;  /* Loop counters */
  uint32_t i, tapCnt, blkCnt, outBlockSize = blockSize / 2;  /* Loop counters , s->M must be 2 for HB*/
  uint32_t blkCntN2;
  uint32_t centerTapIndex = (numTaps-1)/2; // ** numTaps must be of form 4N+3


// left channel
  //q31_t *pStateLeft = S->pStateLeft;                     /* State pointer */
  q31_t *pStateLeft = pStateInitLeft;
  q31_t *pStateLeftCurnt;                            /* Points to the current sample of the state */
  q31_t x0Left;                                 /* Temporary variables to hold state and coefficient values */
  q31_t *pxLeft;                                     /* Temporary pointers for state buffer, not used */                                  /* Temporary pointers for coefficient buffer */
  q31_t sum0Left;                                    /* Accumulator */
  q31_t x1Left;
  q31_t acc0Left, acc1Left;
  q31_t *px0Left, *px1Left;

  


  /* S->pStateLeft buffer contains previous frame (numTaps - 1) samples */
  /* pStateLeftCurnt points to the location where the new input data should be written */
//  pStateLeftCurnt = S->pStateLeft + (numTaps - 1u);
//  pStateRightCurnt = S->pStateRight + (numTaps - 1u);

  pStateLeftCurnt = pStateLeft + (numTaps - 1u);

  /* Total number of output samples to be computed */

  blkCnt = outBlockSize / 2; // == blocksize/4 for HB
  blkCntN2 = outBlockSize - (2 * blkCnt); // == blocksize/2 - outBlockSize = blocksize/2 - blocksize/2 = 0

  while(blkCnt > 0u) // blkCnt = out_size/4, but 2 samples at a time = out_size/2 total output samples
  {
    /* Copy decimation factor number of new input samples into the state buffer */
    // i = 2 * S->M;
    i = 4; // for halfband, S->M must be 2


//************* // copy 4 sample of input buff into state buff offset by numTaps-1 ******/
    do
    {
      *pStateLeftCurnt++ = *pSrcLeft++; // pStateLeft gets incremented by 4 every big loop


    } while(--i);

    /* Set accumulator to zero */
    acc0Left = 0;
    acc1Left = 0;
   
    /* Initialize state pointer */
    px0Left = pStateLeft;
  

    //px1Left = pStateLeft + S->M;
    px1Left = pStateLeft + 2; // HB, only 2 allowed
   



    /* Initialize coeff pointer */
    pb = pCoeffs;

    /* Loop unrolling.  Process 4 taps at a time. */
    tapCnt = numTaps >> 2;

    //  Loop over the number of taps.  Unroll by a factor of 4.
    // Repeat until we've computed numTaps-4 coefficients.
    // orig; 4 taps at a time, but 2 consectutive outputs at a time
    // due to HB, only 2 mults at  time, but 2 consecutive outputs at a time

    while(tapCnt > 0u)
    {
      /* Read the b[numTaps-1] coefficient */
      c0 = *(pb);

      /* Read x[n-numTaps-1] for sample 0 sample 1 */
      x0Left = *(px0Left);
      x1Left = *(px1Left);
      

      /* Perform the multiply-accumulate */
      acc0Left = (q31_t) ((((q63_t) acc0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      acc1Left = (q31_t) ((((q63_t) acc1Left << 32) + ((q63_t) x1Left * c0)) >> 32);

      

      // /* Read the b[numTaps-2] coefficient */
      // c0 = *(pb + 1u);

      // /* Read x[n-numTaps-2]  for sample 0 sample 1  */
      // x0Left = *(pxLeft + 1u);
      // x1Left = *(px1Left + 1u);

      // /* Perform the multiply-accumulate */
      // acc0Left = (q31_t) ((((q63_t) acc0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      // acc1Left = (q31_t) ((((q63_t) acc1Left << 32) + ((q63_t) x1Left * c0)) >> 32);

      /* Read the b[numTaps-3] coefficient */
      c0 = *(pb + 2u);

      /* Read x[n-numTaps-3]  for sample 0 sample 1 */
      x0Left = *(px0Left + 2u);
      x1Left = *(px1Left + 2u);

   

      pb += 4u; // ??

      /* Perform the multiply-accumulate */
      acc0Left = (q31_t) ((((q63_t) acc0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      acc1Left = (q31_t) ((((q63_t) acc1Left << 32) + ((q63_t) x1Left * c0)) >> 32);

    
      /* Read the b[numTaps-4] coefficient */
      // c0 = *(pb - 1u);

      // /* Read x[n-numTaps-4] for sample 0 sample 1 */
      // x0Left = *(pxLeft + 3u);
      // x1Left = *(px1Left + 3u);


      // /* Perform the multiply-accumulate */
      // acc0Left = (q31_t) ((((q63_t) acc0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
      // acc1Left = (q31_t) ((((q63_t) acc1Left << 32) + ((q63_t) x1Left * c0)) >> 32);

      /* update state pointers */
      px0Left += 4u;
      px1Left += 4u;

      /* Decrement the loop counter */
      tapCnt--;
    } // ** if length is multiple of 4, you are finished computing 2 samples here

    // since the filter length is 4N+3, and we have completed the 4N here,
    // we need 3 more, however the middle coeff= 0, so skip it
    /* Read coefficients */
	 c0 = *(pb);
	 /* Fetch 1 state variable */
	 x0Left = *(px0Left);
	 x1Left = *(px1Left);

   
	 /* Perform the multiply-accumulate */
	 acc0Left = (q31_t) ((((q63_t) acc0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
	 acc1Left = (q31_t) ((((q63_t) acc1Left << 32) + ((q63_t) x1Left * c0)) >> 32);

  
	 // advance pointers by 2 because the next coeff is 0 (skip)
	 pb+=2;

	 px0Left+=2;
	 px1Left+=2;


	 /* Read last coefficient */
	 c0 = *(pb);
	 /* Fetch 1 state variable */
	 x0Left = *(px0Left);
	 x1Left = *(px1Left);

	 /* Perform the multiply-accumulate */
	 acc0Left = (q31_t) ((((q63_t) acc0Left << 32) + ((q63_t) x0Left * c0)) >> 32);
	 acc1Left = (q31_t) ((((q63_t) acc1Left << 32) + ((q63_t) x1Left * c0)) >> 32);


// **** haffband edit, add x * center_coeff here , coeff is assumed to be 0.5
    x0Left = *(pStateLeft  + centerTapIndex); // data aligned with center tap for 1st sample
    x1Left = *(pStateLeft + 2 + centerTapIndex); // data aligned with centertap for 2nd sample
    acc0Left = (q31_t) acc0Left + (x0Left >> 2);
    acc1Left = (q31_t) acc1Left + (x1Left >> 2);





    /* Advance the state pointer by the decimation factor       
     * to process the next group of decimation factor number samples */
    //pStateLeft = pStateLeft + S->M * 2;
    pStateLeft = pStateLeft + 4; // HB M=2


    /* The result is in the accumulator, store in the destination buffer. */
    /** write the 2 compted outputs **/
    *pDstLeft++ = (q31_t) (acc0Left << 1);
    *pDstLeft++ = (q31_t) (acc1Left << 1);

    /* Decrement the loop counter */
    blkCnt--;
  } 

  // ************ done entire block **************



  /* Processing is complete.       
   ** Now copy the last numTaps - 1 samples to the start of the state buffer.
   ** This prepares the state buffer for the next function call. */

  /* Points to the start of the state buffer */
//  pStateLeftCurnt = S->pStateLeft;
//  pStateRightCurnt = S->pStateRight;

  pStateLeftCurnt = pStateInitLeft;

  i = (numTaps - 1u) >> 2u;

  /* copy data */
  while(i > 0u)
  {
    *pStateLeftCurnt++ = *pStateLeft++;
    *pStateLeftCurnt++ = *pStateLeft++;
    *pStateLeftCurnt++ = *pStateLeft++;
    *pStateLeftCurnt++ = *pStateLeft++;


    /* Decrement the loop counter */
    i--;
  }

  i = (numTaps - 1u) % 0x04u;

  /* copy data */
  while(i > 0u)
  {
    *pStateLeftCurnt++ = *pStateLeft++;


    /* Decrement the loop counter */
    i--;
  }


} // end of function definition

/**    
 * @} end of FIR_decimate group    
 */
