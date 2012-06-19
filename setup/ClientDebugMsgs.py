#!/usr/bin/env python
import ClientMsgTypes

# Configuration Message - Create Container
cmd_CC = {
            "type":ClientMsgTypes.CREATE_CONTAINER,
            "dest":"$$$$$$",
            "orig":"robotUniqueID", 
            "data":{"containerTag":"containerTag01"} #This tag will be used as a reference in the future
            }
           

# Configuration Message - Container Status
cmd_CS = {
            "type":ClientMsgTypes.CONTAINER_STATUS,
            "dest":"robotUniqueID",
            "orig":"$$$$$$",
            "data":{"cTag":"bool: True <-> Connected; False <-> not Connected"} #cTag - tag of the created container
            }
    
    
# Configuration Message - Destroy Container
cmd_DC = {
            "type":ClientMsgTypes.DESTROY_CONTAINER,
            "dest":"$$$$$$",
            "orig":"robotUniqueID",  
            "data":{"containerTag":"containerTag01"}
            }
            
            
# Configuration Message - Configure Components - For a specific container 
nodeConfigs = [{
                "pkg":"name of package",
                "exe":"name of executable",
                "nodeTag":"nn",
                "namespace":"ns",
    }]
cmd_CN_general = {
            "type":ClientMsgTypes.CONFIGURE_COMPONENT,
            "dest":"containerTag",
            "orig":"robotUniqueID",
            "data":{
                    "addNodes":nodeConfigs,
                    "removeNodes":["namespace/exe"],
                    "addInterfaces":[{"name":"inm",
                                    "interfaceType":"type", # Options: Publisher/Subscriber/Service 
                                    "className":"className"} # msgType for Publisher/Subscriber | srvType for Service
                                    ],
                    "removeInterfaces":["inm"],
                    "setParam":[{"paramName":"pname",
                    			"paramValue":"pvalue", 
                    			"paramType":"pType"
                    			}],
                    "deleteParam" : ["paramName"]
                    }
            }

# register/unregister from an Interface
cmd_IR_general = {
            "type":ClientMsgTypes.INTERFACE_REGISTRATION,
            "dest":"containerTag",
            "orig":"robotUniqueID",
            "data":{
                    "itag":"bool: True <-> Actiavte Imterface; False <-> Deactiavate Interface",
                    }
}

# Data Messages
msg = {"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}};
cmd_DM_general_twist = {
    "type":ClientMsgTypes.DATA_MESSAGE,
    "dest":"destination_container/robot",
    "orig":"origin_container/robot",
    "data":{
        "type":"geometry_msgs/Twist", # Service/Msg type # This is actually redundant: Interface has all details
        "msgID":"mid",  # Applicable only to services. Only if you want to maintain correspondence between request and response.
        "interfaceName":'inm', # self explanatory
        "msg":msg} # In case of srv call: _request_class of the srv class
    }
    
    
# Specific commands for the test suite
class debugCmd(object):
    def __init__(self, cmd, name, info):
        self.cmd = cmd
        self.name = name
        self.info = info


cmd_CC_debug = debugCmd(cmd_CC,'cmd_CC_debug','Create Container')

cmd_DC_debug = debugCmd(cmd_DC,'cmd_CC_debug','Destroy Container')

cmd_CN_startDebugNodes = debugCmd({  "type":ClientMsgTypes.CONFIGURE_COMPONENT,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"addNodes":[{"pkg":"Test",
                                                "exe":"TopicTest.py",
                                                "nodeTag":"nodeTag",
                                                "namespace":"Test"}]}
                        },'cmd_CN_startDebugNodes','Start Nodes')
                        
 
cmd_CN_removeDebugNodes = debugCmd({ "type":ClientMsgTypes.CONFIGURE_COMPONENT,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"removeNodes":["nodeTag"]}
                        },'cmd_CN_removeDebugNodes','Remove Nodes')
                        
cmd_CN_addDebugInterface = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"addInterfaces":[{"name":"Test/getSum",
                                                    "interfaceType":"service",
                                                    "className":"Test/TopicService"},
                                                    {"name":"Test/addInt",
                                                    "interfaceType":"publisher",
                                                    "className":"std_msgs/Int32"}]
                                    }
                            },'cmd_CN_removeDebugNodes','Add Interface')
                            
cmd_CN_removeDebugInterface = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"removeInterfaces":["Test/getSum"]}},
                            'cmd_CN_removeDebugInterface','Remove Interface')
                            
cmd_CN_addDebugParameters = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"setParam":[{"paramName":"test","paramValue":3.0,"paramType":"float"}]}
                            },'cmd_CN_addDebugParameters','Add Parameters')

cmd_CN_removeDebugParameters = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"deleteParam":["test"]}
                            },'cmd_CN_removeDebugParameters','Remove Parameters')
                            
cmd_CN_startNodeaddInterfaceDebug = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"addNodes":cmd_CN_startDebugNodes.cmd['data']['addNodes'],
                                    "addInterfaces":cmd_CN_addDebugInterface.cmd['data']['addInterfaces']}
                            },'cmd_CN_startNodeaddInterfaceDebug','Start Node/Add Interface together')
                            
cmd_IR_registerAtDebugInterface = debugCmd({"type":ClientMsgTypes.INTERFACE_REGISTRATION,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"Test/addInt":True,"Test/getSum":True}
                            },'cmd_IR_registerAtDebugInterface','Register Interface')

cmd_DM_debugServiceRequest_1 = debugCmd({"type":ClientMsgTypes.DATA_MESSAGE,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"type":"std_msgs/Int32",
                                    "msgID":"msgID_0",
                                    "interfaceTag":"Test/addInt",
                                    "msg":{"data":3}}
                            },'cmd_DM_debugServiceRequest_1','Send Service Request - I')
                            
cmd_DM_debugServiceRequest_2 = debugCmd({"type":ClientMsgTypes.DATA_MESSAGE,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"type":"std_msgs/Int32",
                                    "msgID":"msgID_1",
                                    "interfaceTag":"Test/addInt",
                                    "msg":{"data":7}}
                            },'cmd_DM_debugServiceRequest_2','Send Service Request - II')

cmd_DM_debugServiceRequest_3 = debugCmd({"type":ClientMsgTypes.DATA_MESSAGE,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"type":"Test/TopicService",
                                "msgID":"msgID_2",
                                "interfaceTag":
                                "Test/getSum","msg":{}}
                            },'cmd_DM_debugServiceRequest_3','Send Service Request - III')