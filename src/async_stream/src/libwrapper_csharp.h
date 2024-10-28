#pragma once

#ifdef __cplusplus
extern "C"{
#endif

typedef struct KetiROSBridgeWrapper KetiROSBridgeWrapper;

void ros_init(const char* node_name);

KetiROSBridgeWrapper* KetiROSBridge_Create(const char* host_name, const char* cmd_host_name);
void KetiROSBridge_Destroy(KetiROSBridgeWrapper* obj);
void KetiROSBridge_StartBridge(KetiROSBridgeWrapper* obj);
void KetiROSBridge_ClientStartListen(KetiROSBridgeWrapper* obj);
void KetiROSBridge_ServerStartStream(KetiROSBridgeWrapper* obj);

#ifdef __cplusplus
}
#endif