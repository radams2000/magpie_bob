
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
  uint32_t blockSize);

