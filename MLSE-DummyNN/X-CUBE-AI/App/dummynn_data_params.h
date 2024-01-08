/**
  ******************************************************************************
  * @file    dummynn_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Mon Jan  8 17:20:41 2024
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef DUMMYNN_DATA_PARAMS_H
#define DUMMYNN_DATA_PARAMS_H
#pragma once

#include "ai_platform.h"

/*
#define AI_DUMMYNN_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_dummynn_data_weights_params[1]))
*/

#define AI_DUMMYNN_DATA_CONFIG               (NULL)


#define AI_DUMMYNN_DATA_ACTIVATIONS_SIZES \
  { 60000, }
#define AI_DUMMYNN_DATA_ACTIVATIONS_SIZE     (60000)
#define AI_DUMMYNN_DATA_ACTIVATIONS_COUNT    (1)
#define AI_DUMMYNN_DATA_ACTIVATION_1_SIZE    (60000)



#define AI_DUMMYNN_DATA_WEIGHTS_SIZES \
  { 36904, }
#define AI_DUMMYNN_DATA_WEIGHTS_SIZE         (36904)
#define AI_DUMMYNN_DATA_WEIGHTS_COUNT        (1)
#define AI_DUMMYNN_DATA_WEIGHT_1_SIZE        (36904)



#define AI_DUMMYNN_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_dummynn_activations_table[1])

extern ai_handle g_dummynn_activations_table[1 + 2];



#define AI_DUMMYNN_DATA_WEIGHTS_TABLE_GET() \
  (&g_dummynn_weights_table[1])

extern ai_handle g_dummynn_weights_table[1 + 2];


#endif    /* DUMMYNN_DATA_PARAMS_H */
