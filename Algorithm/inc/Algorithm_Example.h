#ifndef __ALGORITHM_EXAMPLE_H__
#define __ALGORITHM_EXAMPLE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "AK70_Moter.h"
#include <stdbool.h>

HAL_StatusTypeDef Algorithm_Example_Init(void);
void Algorithm_Example_Run(void);
void Algorithm_Example_Stop(void);
bool Algorithm_Example_GetFeedback(AK70_MIT_Feedback *feedback,
                                   uint32_t *feedback_age_ms);

#ifdef __cplusplus
}
#endif

#endif /* __ALGORITHM_EXAMPLE_H__ */
