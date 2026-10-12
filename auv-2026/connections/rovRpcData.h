/**
 * @file rovRpcData.h
 * @brief RPC请求参数解析、运行状态管理及响应JSON生成。
 */

#ifndef ROV_RPC_DATA_H
#define ROV_RPC_DATA_H

#include "cjson.h"
#include "rovCommandState.h"

cJSON *build_status_json(void);
cJSON *parse_move_params(cJSON *params);
int parse_pid_params(cJSON *params, const char *str);
int update_propeller_parameters(cJSON *params);
int update_robot_arm_parameters(cJSON *params);
cJSON *propeller_parameters_json_create(void);
cJSON *rov_parameters_response_create(void);
cJSON *pid_runtime_status_json_create(void);

#endif /* ROV_RPC_DATA_H */
