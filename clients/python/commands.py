#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       commands.py
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

import json

# Configuration Message - Create Container
cmd_CS = {
            "type":"CS",
            "dest":"$$$$$$",
            "orig":"robotUniqueID", # Redundant
            "data":{"containerTag":"cTag"} #This tag will be used as a reference in the future
            }

cmd_CS_js = json.dumps(cmd_CS)

print(cmd_CS_js)            

# Configuration Message - Create Container Response
cmd_CSR = {
            "type":"CSR",
            "dest":"robotUniqueID",
            "orig":"$$$$$$",
            "data":{"cTag":"bool: True <-> Connected; False <-> not Connected"} 
            }

cmd_CSR_js = json.dumps(cmd_CSR)

print(cmd_CSR_js)     

# Configuration Message - Distroy Container
cmd_CH = {
            "type":"CH",
            "dest":"$$$$$$",
            "orig":"robotUniqueID",  
            "data":{"containerTag":"cTag"}
            }

cmd_CH_js = json.dumps(cmd_CH)

print(cmd_CH_js)            

# Configuration Message - Change Components - For a specific container 
nodeConfigs = [{
    "pkg":"name of package",
    "exe":"name of executable",
    "nodeTag":"nn",
    "namespace":"ns",
    }]
cmd_CC = {
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

#TODO: Future we will be able to send a full roslauch file for nodes and parameters.

cmd_CC_js = json.dumps(cmd_CC)
print(cmd_CC_js)

cmd_CI = {
            "type":"CL",
            "dest":"containerTag",
            "orig":"robotUniqueID",
            "data":{
                    "itag":"bool: True <-> Actiavte Imterface; False <-> Deactiavate Interface",
                    }
}

cmd_CI_jsg = json.dumps(cmd_CI)

# Data Messages
msg = {"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}};

cmd_RM = {
    "type":"RM",
    "dest":"destination_container/robot",
    "orig":"origin_container/robot",
    "data":{
        "type":"geometry_msgs/Twist", # Service/Msg type # This is actually redundant: Interface has all details
        "msgID":"mid",  # Applicable only to services. Only if you want to maintain correspondence between request and response.
        "interfaceName":'inm', # Corresponds to the receiver in rosbridge
        "msg":msg} # In case of srv call: _request_class of the srv class
    }

cmd_RM_js = json.dumps(cmd_RM)
print(cmd_RM_js)
