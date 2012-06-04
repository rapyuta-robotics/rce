import json

class pythonRequestProcessor:
    incompleteMsgs = [] #list of dictionaries: Keys: 'string_msg','uid1'...'uidn'
    
    def createContainer(self, robotUniqueID):
        if not isinstance(robotUniqueID,str):
            raise TypeError('robotUniqueID should be a string')
        cmd_CS = {
            "type":"CS",
            "dest":"$$$$$$",
            "orig":robotUniqueID,
            "data":None
            }
        cmd_CS_js = json.dumps(cmd_CS)
        return cmd_CS_js
        
    def handleCreateContainerResponse(self, CSRmsg):
        '''CSRmsg is a dictionary'''
        '''return the ID of the container that was created'''
        return CSRmsg['data']['containerID']
        
    def handleDataMsg(self, RMmsg):
        '''RMmsg is a dictionary'''
        '''Just returning the whole datapart'''
        return RMMsg['data']
    
    def handleIncomingMessage(self, inputMsg, binary):
        if binary:
            uid = inputMag[:32] # uuid4().hex - 32 char hexadecimal string
            
        else:
            msg = json.loads(inputMsg)
            
        