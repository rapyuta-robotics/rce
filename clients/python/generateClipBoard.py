from ClientDebugMsgs import *
import json

cmds = [cmd_CS_debug,
        cmd_CH_debug,
        cmd_CC_startDebugNodes, 
        cmd_CC_removeDebugNodes,
        cmd_CC_addDebugInterface,
        cmd_CC_removeDebugInterface ,
        cmd_CC_addDebugParameters ,
        cmd_CC_removeDebugParameters ,
        cmd_CC_startNodeaddInterfaceDebug ,
        cmd_CI_registerAtDebugInterface,
        cmd_CM_debugServiceRequest_1 ,
        cmd_CM_debugServiceRequest_2 ,
        cmd_CM_debugServiceRequest_3 ]
        
cmd_info = ["Create Container",
        "Destroy Container",
        "Start Node", 
        "Remove Node",
        "Add Interface",
        "Remove Interface" ,
        "Add Parameter" ,
        "Remove Parameter" ,
        "Start Node/Add Interface together" ,
        "Register Interface",
        "Send Service Request - I" ,
        "Send Service Request - II" ,
        "Send Service Request - III" ]
        
f = open('cmdsClipboard.txt','w')

for idx, cmd in enumerate(cmds):
    f.write('=== '+cmd_info[idx]+' ===\n\n')
    f.write(json.dumps(cmd)+'\n\n')

f.close()