// Small diagnostic tool for checking basic LJ-S communication state.
#include <stdio.h>
#include <string.h>

#include "LJS8_IF_Linux.h"
#include "LJS8_ErrorCode.h"
#include "LJS_ACQ.h"

int main() {
    int deviceId = 0;
    LJS8IF_ETHERNET_CONFIG ethernetConfig = {
        {192, 168, 0, 1},
        24691
    };

    int rc = LJS8IF_EthernetOpen(deviceId, &ethernetConfig);
    printf("EthernetOpen: 0x%x\n", rc);
    if (rc != LJS8IF_RC_OK) {
        return 1;
    }

    char model[64] = {};
    char serial[64] = {};
    unsigned char program = 0;
    unsigned short attention = 0;
    unsigned char errCount = 0;
    unsigned short errors[16] = {};
    unsigned char operationMode[8] = {};

    rc = LJS8IF_GetHeadModel(deviceId, model);
    printf("GetHeadModel: 0x%x model='%s'\n", rc, model);

    rc = LJS8IF_GetSerialNumber(deviceId, serial);
    printf("GetSerialNumber: 0x%x serial='%s'\n", rc, serial);

    rc = LJS8IF_GetActiveProgram(deviceId, &program);
    printf("GetActiveProgram: 0x%x program=%u\n", rc, program);

    LJS8IF_TARGET_SETTING commonOperationMode = {};
    commonOperationMode.byType = 0x02;
    commonOperationMode.byCategory = 0x00;
    commonOperationMode.byItem = 0x00;
    rc = LJS8IF_GetSetting(deviceId, LJS8IF_SETTING_DEPTH_RUNNING,
                           commonOperationMode, operationMode,
                           sizeof(operationMode));
    printf("GetSetting common.operation_mode: 0x%x value=", rc);
    for (unsigned char byte : operationMode) {
        printf("%02x ", byte);
    }
    printf("\n");

    rc = LJS8IF_GetAttentionStatus(deviceId, &attention);
    printf("GetAttentionStatus: 0x%x attention=0x%04x trigger_ready=%s\n",
           rc, attention, (attention & TRG_READY) ? "yes" : "no");

    rc = LJS8IF_GetError(deviceId, 16, &errCount, errors);
    printf("GetError: 0x%x count=%u", rc, errCount);
    for (unsigned char i = 0; i < errCount && i < 16; ++i) {
        printf(" 0x%04x", errors[i]);
    }
    printf("\n");

    LJS8IF_CommunicationClose(deviceId);
    return 0;
}
