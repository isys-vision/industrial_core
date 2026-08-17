/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024, Industrial Core
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Industrial Core, nor the names
 * 	of its contributors may be used to endorse or promote products derived
 * 	from this software without specific prior written permission.
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

#ifndef MESSAGE_GENERATOR_H
#define MESSAGE_GENERATOR_H

#include "simple_message/simple_message.h"
#include "simple_message/mikado_messages/mik_status_message.h"
#include "simple_message/mikado_classes/mik_status.h"
#include "simple_message/mikado_messages/mik_action_trigger_message.h"
#include "simple_message/mikado_classes/mik_action_trigger.h"
#include "simple_message/mikado_messages/mik_simple_action_reply_message.h"
#include "simple_message/mikado_classes/mik_simple_action_reply.h"

namespace industrial_robot_client
{

void printMsg(industrial::simple_message::SimpleMessage& simpleMsg);
bool createMikMessage(industrial::simple_message::SimpleMessage& simple_message, int msg_type);
void createMikStatusMessage(industrial::simple_message::SimpleMessage& simple_message);
void createActionTriggerMessage(industrial::simple_message::SimpleMessage& simple_message);
void createSimpleActionReplyMessage(industrial::simple_message::SimpleMessage& simple_message);
void createConnectionInfoMessage(industrial::simple_message::SimpleMessage& simple_message);
void createTrajPtMessage(industrial::simple_message::SimpleMessage& simple_message);


} // namespace industrial_robot_client

#endif // MESSAGE_GENERATOR_H