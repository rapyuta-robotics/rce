from ClientDebugMsgs import *
import json

cmds = [cmd_CS_debug,
        cmd_CH_debug,
        cmd_CC_startDebugNodes, 
        cmd_CC_removeDebugNodes,
        cmd_CC_addDebugInterface,
        cmd_CC_removeDebugInterface,
        cmd_CC_addDebugParameters,
        cmd_CC_removeDebugParameters,
        cmd_CC_startNodeaddInterfaceDebug,
        cmd_CI_registerAtDebugInterface,
        cmd_CM_debugServiceRequest_1,
        cmd_CM_debugServiceRequest_2,
        cmd_CM_debugServiceRequest_3]




# .txt clipboard
f = open('cmdsClipboard.txt','w')
for cmd in cmds:
    #print cmd
    f.write('=== '+cmd.info+' ===\n\n')
    f.write(json.dumps(cmd.cmd)+'\n\n')
f.close()



# .js cmd list 

class rceCmdEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj,debugCmd):
            return {'cmd':obj.cmd,
                    'name':obj.name,
                    'info':obj.info}
        else:
            return {}

f = open('rce_cmds.js','w')
##for cmd in cmds:
##    #print cmd
##    f.write(cmd.name+' = '+json.dumps(cmd.cmd)+';\n')
##    f.write(cmd.name+'_info = s'+json.dumps(cmd.info)+';\n\n')
f.write('cmdList = '+json.dumps(cmds, cls=rceCmdEncoder))
f.close()