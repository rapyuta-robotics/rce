#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       TTS.py
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

import roslib; roslib.load_manifest('TTSService')

from TTSService.srv import *
from Administration.msg import File
import rospy

import tempfile
import subprocess

def callback(req):
    txtfile = tempfile.NamedTemporaryFile(prefix='tts', suffix='.txt')
    wavfile = tempfile.NamedTemporaryFile(prefix='tts', suffix='.wav')

    txtfile.write(req.text)
    txtfile.flush()

    try:
        subprocess.check_call('text2wave -eval "(voice_kal_diphone)" {0} -o {1}'.format(txtfile.name, wavfile.name), shell=True)
    except subprocess.CalledProcessError:
        return QueryTTSResponse()

    return QueryTTSResponse(File(content=wavfile.read(), name='speech.wav'))

def ttsServiceServer():
    rospy.init_node('TTSServiceServer')
    s = rospy.Service('TTSService', QueryTTS, callback)
    rospy.spin()

if __name__ == "__main__":
    ttsServiceServer()
