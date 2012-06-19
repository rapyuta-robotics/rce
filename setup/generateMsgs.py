from ClientDebugMsgs import *
import json

cmds = [cmd_CC_debug,
        cmd_DC_debug,
        cmd_CN_startDebugNodes, 
        cmd_CN_removeDebugNodes,
        cmd_CN_addDebugInterface,
        cmd_CN_removeDebugInterface,
        cmd_CN_addDebugParameters,
        cmd_CN_removeDebugParameters,
        cmd_CN_startNodeaddInterfaceDebug,
        cmd_IR_registerAtDebugInterface,
        cmd_DM_debugJSONServiceRequest_1,
		cmd_DM_debugJSONServiceRequest_2,
		cmd_DM_debugJSONServiceRequest_3,
		cmd_CN_binaryReceiving_N,
		cmd_CN_binaryReceiving_I,				
		cmd_CN_binaryReceiving_NI,
		cmd_IR_binaryReceiving,
		cmd_DM_binaryReceiving,
		cmd_DM_binarySending_N,
		cmd_DM_binarySending_I,
		cmd_DM_binarySending_NI,
		cmd_IR_binarySending,
		cmd_DM_binaryComplexReceiving_N,
		cmd_DM_binaryComplexReceiving_I,
		cmd_DM_binaryComplexReceiving_NI,
		cmd_IR_binaryComplexReceiving,
		cmd_DM_binaryComplexReceiving ]


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