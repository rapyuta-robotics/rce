#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ClientDebugMsgs.py
#       
#       This file is part of the RoboEarth Cloud Engine clients.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Gajamohan Mohanarajah
#       
#       

import ClientMsgTypes

# Initialization Message
cmd_IN = {
            "type":ClientMsgTypes.INIT,
            "data":{"userID":"userID",
                    "robotID":"robotID",
                    "key":"32 byte key from master manager"}
            }

# Configuration Message - Create Container
cmd_CC = {
            "type":ClientMsgTypes.CREATE_CONTAINER,
            "data":{"containerTag":"containerTag01"} #This tag will be used as a reference in the future
            }
           

# Configuration Message - Container Status
cmd_CS = {
            "type":ClientMsgTypes.CONTAINER_STATUS,
            "data":{"cTag":"bool: True <-> Connected; False <-> not Connected"} #cTag - tag of the created container
            }
    
    
# Configuration Message - Destroy Container
cmd_DC = {
            "type":ClientMsgTypes.DESTROY_CONTAINER,
            "data":{"containerTag":"containerTag01"}
            }
            

# Configuration Message - Configure Components - For a specific container 
nodeConfigs = [{
                "containerTag":"containerTag01",
                "nodeTag":"nn",
                "pkg":"name of package",
                "exe":"name of executable",
                "namespace":"ns",
    }]
interfaceConfigs = [{
                     "endpoint":"containerTag01",
                     "interfaceTag":"ii",
                     "interfaceType":"type",
                     "className":"className", # msgType for Publisher/Subscriber | srvType for Service
                     "addr":"rosAddr", # only used for ROS interfaces
    }]
paramConfigs = [{
                 "containerTag":"containerTag01",
                 "name":"parameterName/rosAddr",
                 "value":"pValue",
                 "paramType":"pType", # int/str/float/bool/file
    }]
cmd_CN_general = {
            "type":ClientMsgTypes.CONFIGURE_COMPONENT,
            "data":{
                    "addNodes":nodeConfigs,
                    "removeNodes":[{"containerTag":"containerTag01",
                                    "nodeTag":"nn"}],
                    "addInterfaces":interfaceConfigs,
                    "removeInterfaces":["interfaceTags"],
                    "setParam":paramConfigs,
                    "deleteParam":[{"containerTag":"containerTag01",
                                    "name":"parameterName"}]
                    }
            }

cmd_CX_general = {
            "type":ClientMsgTypes.CONFIGURE_CONNECTION,
            "data":{
                    "connect":[{"tagA":"interfaceTag01",
                                "tagB":"interfaceTag02"}],
                    "disconnect":[{"tagA":"interfaceTag01",
                                   "tagB":"interfaceTag02"}]
                    }
            }


# Data Messages
msg = {"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}};
cmd_DM_general_twist = {
    "type":ClientMsgTypes.DATA_MESSAGE,
    "data":{
        "dest":"destination_interfaceTag/robot",
        "orig":"origin_interfaceTag/robot",
        "type":"geometry_msgs/Twist", # Service/Msg type # This is actually redundant: Interface has all details
        "msgID":"mid",  # Applicable only to services. Only if you want to maintain correspondence between request and response.
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
                        },'cmd_CN_startDebugNodes','Simple Test - Start Nodes')
                        
 
cmd_CN_removeDebugNodes = debugCmd({ "type":ClientMsgTypes.CONFIGURE_COMPONENT,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"removeNodes":["nodeTag"]}
                        },'cmd_CN_removeDebugNodes','Simple Test - Remove Nodes')
                        
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
                            },'cmd_CN_removeDebugNodes','Simple Test - Add Interface')
                            
cmd_CN_removeDebugInterface = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"removeInterfaces":["Test/getSum"]}},
                            'cmd_CN_removeDebugInterface','Simple Test - Remove Interface')
                            
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
                            },'cmd_CN_startNodeaddInterfaceDebug','Simple Test - Start Node/Add Interface together')
                            
cmd_IR_registerAtDebugInterface = debugCmd({"type":ClientMsgTypes.INTERFACE_REGISTRATION,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"Test/addInt":True,"Test/getSum":True}
                            },'cmd_IR_registerAtDebugInterface','Simple Test - Register Interface')
                            
cmd_IR_unregisterAtDebugInterface = debugCmd({"type":ClientMsgTypes.INTERFACE_REGISTRATION,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"Test/addInt":False,"Test/getSum":False}
                            },'cmd_IR_unregisterAtDebugInterface','Simple Test - Unregister Interface')

cmd_DM_debugJSONServiceRequest_1 = debugCmd({"type":ClientMsgTypes.DATA_MESSAGE,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"type":"std_msgs/Int32",
                                    "msgID":"msgID_0",
                                    "interfaceTag":"Test/addInt",
                                    "msg":{"data":3}}
                            },'cmd_DM_debugServiceRequest_1','Simple Test - Send addInt - topic I')
                            
cmd_DM_debugJSONServiceRequest_2 = debugCmd({"type":ClientMsgTypes.DATA_MESSAGE,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"type":"std_msgs/Int32",
                                    "msgID":"msgID_1",
                                    "interfaceTag":"Test/addInt",
                                    "msg":{"data":7}}
                            },'cmd_DM_debugServiceRequest_2','Simple Test - Send addInt - topic II')

cmd_DM_debugJSONServiceRequest_3 = debugCmd({"type":ClientMsgTypes.DATA_MESSAGE,
                            "dest":"containerTag01",
                            "orig":"robotUniqueID",
                            "data":{"type":"Test/TopicService",
                                "msgID":"msgID_2",
                                "interfaceTag":
                                "Test/getSum","msg":{}}
                            },'cmd_DM_debugServiceRequest_3','Simple Test - Send addInt Service Request')


cmd_CN_binaryReceiving_N = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT, 							"dest":"containerTag01",
							"orig":"robotUniqueID",
							"data":{"addNodes":	
								[{	"pkg":"rceBinaryMsgDebug",
									"exe":"imageGenerator.py",
									"nodeTag":"imgGen",
									"namespace":"binary"}]
								}},
							'cmd_CN_binaryReceiving_N', 'Binary Receiving - Config Nodes')

cmd_CN_binaryReceiving_I = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT, 							"dest":"containerTag01",
							"orig":"robotUniqueID",
							"data":{"addInterfaces":[
									{	"name":"binary/circleOut",
										"interfaceType":"subscriber",
										"className":"sensor_msgs/Image"},
									{	"name":"binary/ccPos",
										"interfaceType":"publisher",
										"className":"geometry_msgs/Pose2D"}
							]}},
							'cmd_CN_binaryReceiving_I', 'Binary Receiving - Config Interfaces')
							
cmd_CN_binaryReceiving_NI = debugCmd({	"type":ClientMsgTypes.CONFIGURE_COMPONENT, 							    										"dest":"containerTag01",
									"orig":"robotUniqueID",
									"data":{"addNodes":[{
												"pkg":"rceBinaryMsgDebug",
												"exe":"imageGenerator.py",
												"nodeTag":"imgGen",
												"namespace":"binary"}],
											"addInterfaces":[{
												"name":"binary/circleOut",
												"interfaceType":"subscriber",
												"className":"sensor_msgs/Image"},
												{"name":"binary/ccPos",
												"interfaceType":"publisher",
												"className":"geometry_msgs/Pose2D"}]}},
							'cmd_CN_binaryReceiving_NI', 'Binary Receiving - Config Nodes & Interfaces')
					

cmd_IR_binaryReceiving = debugCmd({	"type":ClientMsgTypes.INTERFACE_REGISTRATION,
									"dest":"containerTag01",
									"orig":"robotUniqueID",
									"data":{"binary/circleOut":True,"binary/ccPos":True}}
									,'cmd_IR_binaryReceiving','Binary Receiving - - Register Interface')


cmd_DM_binaryReceiving = debugCmd({	"type":ClientMsgTypes.DATA_MESSAGE,
									"dest":"containerTag01",
									"orig":"robotUniqueID",
									"data":{"type":"geometry_msgs/Pose2D",
											"msgID":"msgID_0",
											"interfaceTag":"binary/ccPos",
											"msg":{"x":0.2,"y":0.6,"theta":0.0}
											}
									},'cmd_DM_binaryReceiving','Binary Receiving - Data Message')



cmd_DM_binarySending_N = debugCmd({	"type":ClientMsgTypes.CONFIGURE_COMPONENT,
									"dest":"containerTag01",
									"orig":"robotUniqueID",
									"data":{"addNodes":[
											{	"pkg":"rceBinaryMsgDebug",
												"exe":"imageAnalyzer.py",
												"nodeTag":"imgAnalyzer",
												"namespace":"binary"
											}]
										}
									},'cmd_DM_binarySending_N','Binary Sending - Config Nodes')


cmd_DM_binarySending_I = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT,
									"dest":"containerTag01",
									"orig":"robotUniqueID",
									"data":{"addInterfaces":[{
										"name":"binary/circleIn",
										"interfaceType":"publisher",
										"className":"sensor_msgs/Image"},
										{"name":"binary/est_ccPos",
										"interfaceType":"subscriber",
										"className":"geometry_msgs/Pose2D"}]
										}
									},'cmd_DM_binarySending_I','Binary Sending - Config Interfaces')

cmd_DM_binarySending_NI = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT,
									"dest":"containerTag01",
									"orig":"robotUniqueID",
									"data":{"addNodes":[
												{"pkg":"rceBinaryMsgDebug",
												"exe":"imageAnalyzer.py",
												"nodeTag":"imgAnalyzer",
												"namespace":"binary"}],
											"addInterfaces":[{
												"name":"binary/circleIn",
												"interfaceType":"publisher",
												"className":"sensor_msgs/Image"},
												{"name":"binary/est_ccPos",
												"interfaceType":"subscriber",
												"className":"geometry_msgs/Pose2D"}]
									}
									},'cmd_DM_binarySending_NI','Binary Sending - Config Nodes and Interfaces')

cmd_IR_binarySending = debugCmd({"type":ClientMsgTypes.INTERFACE_REGISTRATION,
								"dest":"containerTag01",
								"orig":"robotUniqueID",
								"data":{"binary/circleIn":True,"binary/est_ccPos":True}
								},'cmd_IR_binarySending','Binary Sending - Register Interface')

                      
cmd_DM_binaryComplexReceiving_N = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT,
											"dest":"containerTag01",
											"orig":"robotUniqueID",
											"data":{"addNodes":[{
												"pkg":"Test",
												"exe":"Test.py",
												"nodeTag":"testNode",
												"namespace":"TestBin"}]}},
								'cmd_DM_binaryComplexReceiving_N', 'Binary Complex Receiving - Config Nodes')

                      
cmd_DM_binaryComplexReceiving_I = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT,
											"dest":"containerTag01",
											"orig":"robotUniqueID",
											"data":{"addInterfaces":[
														{"name":"TestBin/test",
														"interfaceType":"service",
														"className":"Test/QueryTest"}]}},
								'cmd_DM_binaryComplexReceiving_I', 'Binary Complex Receiving - Config Interfaces')


cmd_DM_binaryComplexReceiving_NI = debugCmd({"type":ClientMsgTypes.CONFIGURE_COMPONENT,
											"dest":"containerTag01",
											"orig":"robotUniqueID",
											"data":{"addNodes":[{
														"pkg":"Test",
														"exe":"Test.py",
														"nodeTag":"testNode",
														"namespace":"TestBin"}],
													"addInterfaces":[
														{"name":"TestBin/test",
														"interfaceType":"service",
														"className":"Test/QueryTest"}]}},
								'cmd_DM_binaryComplexReceiving_NI', 'Binary Complex Receiving -  Config Nodes and Interfaces')

cmd_IR_binaryComplexReceiving = debugCmd({	"type":ClientMsgTypes.INTERFACE_REGISTRATION,
											"dest":"containerTag01",
											"orig":"robotUniqueID",
											"data":{"TestBin/test":True}},
										'cmd_IR_binaryComplexReceiving','Binary Complex Receiving - Register Interface')

cmd_DM_binaryComplexReceiving = debugCmd({"type":ClientMsgTypes.DATA_MESSAGE,
											"dest":"containerTag01",
											"orig":"robotUniqueID",
											"data":{"type":"Test/QueryTest",
													"msgID":"msgID_0",
													"interfaceTag":"TestBin/test",
													"msg":{"a":3,"b":5}}},
											'cmd_DM_binaryComplexReceiving','Binary Complex Receiving - Data Message')

