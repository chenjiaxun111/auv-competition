#ifndef __CRPCFUNC_H_
#define __CRPCFUNC_H_

#include "../user/config.h"
#include "jsonrpcServer.h"

cJSON *get_status_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id);
cJSON *get_propeller_power_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id);

cJSON *move_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id);
cJSON *set_gripper_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id);
cJSON *set_auv_target_rpc_handler(jrpc_context*,cJSON*,cJSON*);
cJSON *get_pid_runtime_status_rpc_handler(jrpc_context*,cJSON*,cJSON*);

cJSON *set_propeller_parameters_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id);
cJSON *set_pid_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id);
cJSON *save_propeller_parameters_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id);
cJSON *save_pid_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id);
cJSON *load_parameters_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id);

#endif
