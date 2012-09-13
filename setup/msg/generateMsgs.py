#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       generateMsgs.py
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

from ClientDebugMsgs import *
import json

cmds = [cmd_AU_debug,
        cmd_IN_debug,
        cmd_CC_debug,
        cmd_DC_debug ]


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

f = open('../../clients/html/rce_cmds.js','w')
##for cmd in cmds:
##    #print cmd
##    f.write(cmd.name+' = '+json.dumps(cmd.cmd)+';\n')
##    f.write(cmd.name+'_info = s'+json.dumps(cmd.info)+';\n\n')
f.write('cmdList = '+json.dumps(cmds, cls=rceCmdEncoder))
f.close()
