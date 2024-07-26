void arm_fir_decimate_fast_q31_HB_1ch( // ***** RESTRICTIONS, Number of taps must be 4N+3, and the center tap must be 0.5. Not checked
  uint32_t numTaps,
  q31_t *pCoeffs,
  q31_t *pStateInitLeft,
  q31_t * pSrcLeft,
  q31_t * pDstLeft,
  uint32_t blockSize);
