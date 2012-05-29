#!/usr/bin/env python
import json

# Configuration Message - Create Container
cmd_CS = {
            "type":"CS",
            "data":None,
            "dest":"$$$$$$",
            "orig":"robotUniqueID"
            }

cmd_CS_js = json.dumps(cmd_CS)

print(cmd_CS_js)            

# Configuration Message - Create Container Response
cmd_CSR = {
            "type":"CSR",
            "data":{"containerID":"cid"},
            "dest":"robotUniqueID",
            "orig":"$$$$$$"
            }

cmd_CSR_js = json.dumps(cmd_CSR)

print(cmd_CSR_js)     

# Configuration Message - Distroy Container
cmd_CH = {
            "type":"CH",
            "data":{"containerID":"cid"},
            "dest":"$$$$$$",
            "orig":"robotUniqueID"  
            }

cmd_CH_js = json.dumps(cmd_CH)

print(cmd_CH_js)            

# Configuration Message - Change Components
nodeConfigs = [{
    "nodeID":"nid",
    "nodeName":"nn",
    "namespaceID":"ns",
    #"interfaceID":["iid"],
    "parameters":{"parameterName":"parameterValue"}
    }]
cmd_CC = {
            "type":"CC",
            "data":{
                    "add":nodeConfigs,
                    "remove":["nameSpaceID/nodeID"]
                    },
            "dest":"containerID_receivedFrom_cmd_CSR",
            "orig":"robotUniqueID"  
            }

cmd_CC_js = json.dumps(cmd_CC)
print(cmd_CC_js)

# Data Messages
msg = {"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}};
cmd_RM = {
    "type":"RM",
    "data":{
        "interfaceID":"iid", #I dont live having it here
        "msg":msg,
        "msgID":"mid"},
    "dest":"destination_container/robot", #can we have a dict with level1, level2. level2 being interfaceID
    "orig":"origin_container/robot"
    }

cmd_RM_js = json.dumps(cmd_RM)
print(cmd_RM_js)
