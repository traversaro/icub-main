/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
*/

#ifndef __IKINVOCABS_H__
#define __IKINVOCABS_H__

#include <yarp/os/Vocab.h>

#define IKINSLV_VOCAB_CMD_GET           VOCAB3('g','e','t')
#define IKINSLV_VOCAB_CMD_SET           VOCAB3('s','e','t')
#define IKINSLV_VOCAB_CMD_ASK           VOCAB3('a','s','k')
#define IKINSLV_VOCAB_CMD_SUSP          VOCAB4('s','u','s','p')
#define IKINSLV_VOCAB_CMD_RUN           VOCAB3('r','u','n')
#define IKINSLV_VOCAB_CMD_STATUS        VOCAB4('s','t','a','t')
#define IKINSLV_VOCAB_CMD_HELP          VOCAB4('h','e','l','p')
#define IKINSLV_VOCAB_CMD_CFG           VOCAB3('c','f','g')
#define IKINSLV_VOCAB_CMD_QUIT          VOCAB4('q','u','i','t')
#define IKINSLV_VOCAB_OPT_MODE          VOCAB4('m','o','d','e')
#define IKINSLV_VOCAB_OPT_POSE          VOCAB4('p','o','s','e')
#define IKINSLV_VOCAB_OPT_PRIO          VOCAB4('p','r','i','o')
#define IKINSLV_VOCAB_OPT_DOF           VOCAB3('d','o','f')
#define IKINSLV_VOCAB_OPT_LIM           VOCAB3('l','i','m')
#define IKINSLV_VOCAB_OPT_XD            VOCAB2('x','d')
#define IKINSLV_VOCAB_OPT_X             VOCAB1('x')
#define IKINSLV_VOCAB_OPT_Q             VOCAB1('q')
#define IKINSLV_VOCAB_OPT_TOKEN         VOCAB3('t','o','k')
#define IKINSLV_VOCAB_OPT_VERB          VOCAB4('v','e','r','b')
#define IKINSLV_VOCAB_OPT_REST_POS      VOCAB4('r','e','s','p')
#define IKINSLV_VOCAB_OPT_REST_WEIGHTS  VOCAB4('r','e','s','w')
#define IKINSLV_VOCAB_OPT_TIP_FRAME     VOCAB3('t','i','p')
#define IKINSLV_VOCAB_OPT_TASK2         VOCAB4('t','s','k','2')
#define IKINSLV_VOCAB_OPT_CONVERGENCE   VOCAB4('c','o','n','v')
#define IKINSLV_VOCAB_VAL_POSE_FULL     VOCAB4('f','u','l','l')
#define IKINSLV_VOCAB_VAL_POSE_XYZ      VOCAB3('x','y','z')
#define IKINSLV_VOCAB_VAL_PRIO_XYZ      VOCAB3('x','y','z')
#define IKINSLV_VOCAB_VAL_PRIO_ANG      VOCAB3('a','n','g')
#define IKINSLV_VOCAB_VAL_MODE_TRACK    VOCAB4('c','o','n','t')
#define IKINSLV_VOCAB_VAL_MODE_SINGLE   VOCAB4('s','h','o','t')
#define IKINSLV_VOCAB_VAL_ON            VOCAB2('o','n')
#define IKINSLV_VOCAB_VAL_OFF           VOCAB3('o','f','f')
#define IKINSLV_VOCAB_REP_ACK           VOCAB3('a','c','k')
#define IKINSLV_VOCAB_REP_NACK          VOCAB4('n','a','c','k')

#endif
