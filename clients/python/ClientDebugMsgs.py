#!/usr/bin/env python

# Configuration Message - Create Container
cmd_CS = {
            "type":"CS",
            "dest":"$$$$$$",
            "orig":"robotUniqueID", 
            "data":{"containerTag":"containerTag01"} #This tag will be used as a reference in the future
            }
           

# Configuration Message - Create Container Response
cmd_CSR = {
            "type":"CSR",
            "dest":"robotUniqueID",
            "orig":"$$$$$$",
            "data":{"cTag":"bool: True <-> Connected; False <-> not Connected"} #cTag - tag of the created container
            }
    
    
# Configuration Message - Distroy Container
cmd_CH = {
            "type":"CH",
            "dest":"$$$$$$",
            "orig":"robotUniqueID",  
            "data":{"containerTag":"containerTag01"}
            }
            
            
# Configuration Message - Change Components - For a specific container 
nodeConfigs = [{
                "pkg":"name of package",
                "exe":"name of executable",
                "nodeTag":"nn",
                "namespace":"ns",
    }]
cmd_CC_general = {
            "type":"CC",
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
cmd_CI_general = {
            "type":"CL",
            "dest":"containerTag",
            "orig":"robotUniqueID",
            "data":{
                    "itag":"bool: True <-> Actiavte Imterface; False <-> Deactiavate Interface",
                    }
}

# Data Messages
msg = {"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}};
cmd_RM_general_twist = {
    "type":"RM",
    "dest":"destination_container/robot",
    "orig":"origin_container/robot",
    "data":{
        "type":"geometry_msgs/Twist", # Service/Msg type # This is actually redundant: Interface has all details
        "msgID":"mid",  # Applicable only to services. Only if you want to maintain correspondence between request and response.
        "interfaceName":'inm', # Corresponds to the receiver in rosbridge
        "msg":msg} # In case of srv call: _request_class of the srv class
    }
    
    
# Specific commands for the test suite
cmd_CS_debug = cmd_CS

cmd_CH_debug = cmd_CH

cmd_CC_startDebugNodes = {  "type":"CC",
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"addNodes":[{"pkg":"Test",
                                                "exe":"TopicTest.py",
                                                "nodeTag":"nodeTag",
                                                "namespace":"Test"}]}
                        }
 
cmd_CC_removeDebugNodes = { "type":"CC",
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"removeNodes":["nodeTag"]}
                        }
                        
cmd_CC_addDebugInterface = {"type":"CC",
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"addInterfaces":[{"name":"Test/getSum",
                                                    "interfaceType":"service",
                                                    "className":"Test/TopicService"},
                                                    {"name":"Test/addInt",
                                                    "interfaceType":"publisher",
                                                    "className":"std_msgs/Int32"}]
                                    }
                            }
                            
cmd_CC_removeDebugInterface = {"type":"CC",
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"removeInterfaces":["Test/getSum"]}}
                            
cmd_CC_addDebugParameters = {"type":"CC",
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"setParam":[{"paramName":"test","paramValue":3.0,"paramType":"float"}]}
                            }

cmd_CC_removeDebugParameters = {"type":"CC",
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"deleteParam":["test"]}
                            }
                            
cmd_CC_startNodeaddInterfaceDebug = {"type":"CC",
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"addNodes":cmd_CC_startDebugNodes['data']['addNodes'],
                                    "addInterfaces":cmd_CC_addDebugInterface['data']['addInterfaces']}
                            }
                            
cmd_CI_registerAtDebugInterface = {"type":"CI",
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"Test/addInt":True,"Test/getSum":True}
                            }

cmd_CM_debugServiceRequest_1 = {"type":"CM",
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"type":"std_msgs/Int32",
                                    "msgID":"msgID_0",
                                    "interfaceTag":"Test/addInt",
                                    "msg":{"data":3}}
                            }
                            
cmd_CM_debugServiceRequest_2 = {"type":"CM",
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"type":"std_msgs/Int32",
                                    "msgID":"msgID_1",
                                    "interfaceTag":"Test/addInt",
                                    "msg":{"data":7}}
                            }

cmd_CM_debugServiceRequest_3 = {"type":"CM",
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"type":"Test/TopicService",
                                "msgID":"msgID_2",
                                "interfaceTag":
                                "Test/getSum","msg":{}}
                            }