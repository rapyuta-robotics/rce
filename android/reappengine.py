#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       reappengine.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#       
#       

# Library for android
import android

# Standard Python libraries
import os, time

# Custom reappengine API
import ServiceAPI

# path for storing the image temporarily
IMG_PATH = '/sdcard/reappengine_scene.jpg'

# Context for all android related methods
droid = android.Android()

def reportError(msg):
    """ Report an error with the given message and terminate the process.
    """
    droid.dialogCreateAlert('Error', msg)
    droid.dialogSetNeutralButtonText('Close')
    droid.dialogShow()
    droid.dialogGetResponse()
    droid.dialogDismiss()
    exit(1)

def dispSpinnerMsg(title='', msg=''):
    """ Report a message to the user, using the Spinner dialog.
    """
    droid.dialogCreateSpinnerProgress(title, msg)
    droid.dialogShow()

def askRepeat():
    """ Ask the user to replay the audio.
    """
    droid.dialogCreateAlert('Replay Audio File?')
    droid.dialogSetPositiveButtonText('Replay')
    droid.dialogSetNegativeButtonText('Stop')
    droid.dialogShow()
    resp = droid.dialogGetResponse().result['which']
    
    if resp == 'positive':
        return True
    elif resp == 'negative':
        return False
    else:
        raise ValueError('Returned value is neither "positive" nor "negative".')
    

def loop():
    """ One iteration from taking a picture to playing the parsed text.
    """
    # First take a picture
    droid.cameraInteractiveCapturePicture(IMG_PATH)
    
    # Connect to server and setup the environment
    dispSpinnerMsg('Connecting', 'Please wait...')
    env = ServiceAPI.changeEnv(nodesToAdd=[ ('ReadTextService/ReadText', {}),
                                            ('TTSService/TTS.py', None) ])
    
    # Upload image and start process
    dispSpinnerMsg('Uploading', 'Please wait...')
    fh = ServiceAPI.FileHandle(IMG_PATH)
    task = ServiceAPI.addTask(env, 'ReadTextService/ReadText', {'image' : fh})
    
    # Remove temporary image
    os.remove(IMG_PATH)
    
    # Wait for result
    dispSpinnerMsg('Running TextReader', 'Please wait...')
    (status, result) = ServiceAPI.getTask(env, task, 120)

    # Check whether the ReadText node was successful
    if status != 'completed':
        reportError('Image could not be read.')
    
    if not result['text']:
        reportError('Image does not contain readable text.')
    
    # Convert the text to speech
    dispSpinnerMsg('Running TTS', 'Please wait...')
    task = ServiceAPI.addTask(env, 'TTSService/TTSService', {'text' : '. '.join(result['text'])})
    
    # Wait for result
    (status, result) = ServiceAPI.getTask(env, task, 30)
    droid.dialogDismiss()
    
    if status != 'completed':
        reportError('Text could not be converted to speech.')
    
    # Fetch the audio file and save it temporarily
    audio = os.path.join('/sdcard', result['wavFile']['name'])
    wav = ServiceAPI.getFile(env, task, result['wavFile']['content'])
    
    with open(audio, 'w') as f:
        f.write(wav.read())
    
    repeat = True
    
    while repeat:
        # Play audio file
        droid.mediaPlay(audio, 'speech')
        
        while droid.mediaIsPlaying('speech').result:
            time.sleep(0.1)
        
        repeat = askRepeat()
    
    # Remove the audio file
    os.remove(audio)

if __name__ == '__main__':
    try:
        loop()
        droid.dialogDismiss()
    except Exception as e:
        #reportError(str(e))
        droid.dialogDismiss()
        
        import traceback
        traceback.print_exc()
