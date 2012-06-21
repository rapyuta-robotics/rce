#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       SSLUtil.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth - http://www.roboearth.org/
#       The research leading to these results has received funding from the European Union 
#       Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
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
#       \author/s: Dominique Hunziker <dominique.hunziker@gmail.com> 
#       
#       

# twisted specific imports
from twisted.internet.ssl import ContextFactory

# Python specific imports
from OpenSSL import crypto, SSL

def createKey(bits=2048):
    """ Create a new key.
        
        @param bits:    The length of the key in bits.
        @type  bits:    int
        
        @return:        Newly created key.
        @rtype:         crypto.PKey
    """
    key = crypto.PKey()
    key.generate_key(crypto.TYPE_RSA, bits)
    return key

def createCertReq(key, name):
    """ Create a X509 certificate request.
        
        @param key:     Private key which should be used together with the certificate.
        @type  key:     crypto.PKey
        
        @param name:    Name which should be given to the certificate.
        @type  name:    str
        
        @return:        Newly created certificate request.
        @rtpye:         crypto.X509Req
    """
    req = crypto.X509Req()
    req.get_subject().CN = name
    req.set_pubkey(key)
    req.sign(key, 'md5')
    return req

def createCert(req, issuerCert, issuerKey, valid=365*24*60*60):
    """ Create a X509 certificate using a certificate request and a certificate/key
        to sign the newly created certificate.
        
        @param req:     Certificate request which should be processes and signed.
        @type  req:     crypto.X509Req
        
        @param issuerCert:  Certificate of the issuer, i.e. the certificate which is
                            used to sign the new certificate.
        @type  issuerCert:  crypto.X509
        
        @param issuerKey:   Key of the issuer, i.e. the key matching the certificate
                            which is used to sign the new certificate.
        @type  issuerKey:   crypto.PKey
        
        @param valid:   Duration which the certificate will be valid in seconds from
                        now.
        @type  valid:   int
        
        @return:        Newly created and signed certificate.
        @rtype:         crypto.X509
    """
    cert = crypto.X509()
    #cert.set_serial_number(serial)
    cert.gmtime_adj_notBefore(0)
    cert.gmtime_adj_notAfter(valid)
    cert.set_issuer(issuerCert.get_subject())
    cert.set_subject(req.get_subject())
    cert.set_pubkey(req.get_pubkey())
    cert.sign(issuerKey, 'md5')
    return cert

def createKeyCertPair(name, caCert=None, caKey=None):
    """ Create a key/certificate pair which can be either self-signed or signed
        using a given certificate.
        
        @param name:    Name which should be given to the certificate.
        @type  name:    str
        
        @param caCert:  Certificate of the issuer, i.e. the certificate which is
                        used to sign the new certificate.
                        None means that the certificate is self-signed.
        @type  caCert:  crypto.X509
        
        @param caKey:   Key of the issuer, i.e. the key matching the certificate
                        which is used to sign the new certificate.
                        None means that the certificate is self-signed.
        @type  caKey:   crypto.PKey
        
        @return:        Tuple consisting of the pair certificate and key.
        @rtype:         (crypto.X509, crypto.PKey)
    """
    key = createKey()
    req = createCertReq(key, name)
    
    if not caCert and not caKey:
        caCert = req
        caKey = key
    
    cert = createCert(req, caCert, caKey)
    
    return (cert, key)

def loadCertFile(fileName):
    """ Load a certificate from a file.
        
        @param fileName:    Path to the file from which the certificate should be loaded.
        @type  fileName:    str
        
        @return:        Loaded certificate.
        @rtype:         crypto.X509
    """
    with open(fileName, 'r') as f:
        return crypto.load_certificate(crypto.FILETYPE_PEM, f.read())

def loadKeyFile(fileName):
    """ Load a key from a file.
        
        @param fileName:    Path to the file from which the key should be loaded.
        @type  fileName:    str
        
        @return:        Loaded key.
        @rtype:         crypto.PKey
    """
    with open(fileName, 'r') as f:
        return crypto.load_privatekey(crypto.FILETYPE_PEM, f.read())

def writeCertToFile(cert, path):
    """ Write a certificate to a file.
        
        @param req:     Certificate which should be written to a file.
        @type  req:     crypto.X509
        
        @param path:    Path of the file where the Certificate should be written.
        @type  path:    str
    """
    with open(path, 'w') as f:
        f.write(crypto.dump_certificate(crypto.FILETYPE_PEM, cert))

def writeKeyToFile(key, path):
    """ Write a key to a file.
        
        @param req:     Key which should be dumped into a string.
        @type  req:     crypto.PKey
        
        @param path:    Path of th file where the key should be written.
        @type  path:    str
    """
    with open(path, 'w') as f:
        f.write(crypto.dump_privatekey(crypto.FILETYPE_PEM, key))

def dumpCert(cert):
    """ Dump a certificate to string.
        
        @param req:     Certificate which should be dumped into a string.
        @type  req:     crypto.X509
        
        @return:        Certificate as a string.
        @rtype:         str
    """
    return crypto.dump_certificate(crypto.FILETYPE_PEM, cert)

def dumpCertReq(req):
    """ Dump a certificate request to string.
        
        @param req:     Certificate request which should be dumped into a string.
        @type  req:     crypto.X509Req
        
        @return:        Certificate request as a string.
        @rtype:         str
    """
    return crypto.dump_certificate_request(crypto.FILETYPE_PEM, req)

def parseCertReqStr(req):
    """ Parse a certificate request which is in string form.
        
        @param cert:    Key in string form, i.e. content of certificate in
                        file form.
        @type  cert:    str
        
        @return:        Parsed certificate request.
        @rtype:         crypto.X509Req
    """
    return crypto.load_certificate_request(crypto.FILETYPE_PEM, req)

def parseCertStr(cert):
    """ Parse a certificate which is in string form.
        
        @param cert:    Certificate in string form, i.e. content of certificate in
                        file form.
        @type  cert:    str
        
        @return:        Parsed certificate.
        @rtype:         crypto.X509
    """
    return crypto.load_certificate(crypto.FILETYPE_PEM, cert)

class RCEServerContext(ContextFactory):
    """ SSL Context which is used for server-side endpoints in the RCE.
    """
    def __init__(self, ca, cert, key):
        """ Initialize the RCEServerContext.
            
            @param ca:      Path to the certificate which has been used to sign the
                            certificate/key of this Context and which is used to identify
                            the other side.
            @type  ca:      str
            
            @param cert:    Either a certificate X509 instance or a path to the
                            certificate of this Context.
            @type  cert:    crypto.X509 / str
            
            @param key:     Eithr a key instance or a path to the key of this Context.
            @type  key:     crypto.PKey / str
        """
        ctx = SSL.Context(SSL.SSLv23_METHOD)
        ctx.set_options(SSL.OP_NO_SSLv2)
        
        ctx.set_verify(SSL.VERIFY_PEER, self._verify)
        
        if isinstance(cert, crypto.X509):
            ctx.use_certificate(cert)
        else:
            ctx.use_certificate_file(cert)
        
        if isinstance(key, crypto.PKey):
            ctx.use_privatekey(key)
        else:
            ctx.use_privatekey_file(key)
        
        ctx.load_verify_locations(ca)
        
        self._context = ctx
    
    def _verify(self, conn, cert, errnum, depth, ok):
        """ Internally used method to verify the peer certificate.
            Does nothing.
        """
        return ok
    
    def getContext(self):
        """ Method has to be overwritten for twisted.
        """
        return self._context

class RCEClientContext(ContextFactory):
    """ SSL Context which is used for client-side endpoints in the RCE.
    """
    isClient = 1

    def __init__(self, ca, cert=None, key=None):
        """ Initialize the RCEClientContext.
            
            @param ca:      Path to the certificate which has been used to sign the
                            certificate/key of this Context and which is used to identify
                            the other side.
            @type  ca:      str
            
            @param cert:    Either a certificate X509 instance or a path to the
                            certificate of this Context.
                            None means that no certificate/key should be used.
            @type  cert:    crypto.X509 / str
            
            @param key:     Eithr a key instance or a path to the key of this Context.
                            None means that no certificate/key should be used.
            @type  key:     crypto.PKey / str
        """
        ctx = SSL.Context(SSL.SSLv23_METHOD)
        ctx.set_options(SSL.OP_NO_SSLv2)
        
        ctx.set_verify(SSL.VERIFY_PEER, self._verify)
        
        if cert or key:
            if isinstance(cert, crypto.X509):
                ctx.use_certificate(cert)
            else:
                ctx.use_certificate_file(cert)
            
            if isinstance(key, crypto.PKey):
                ctx.use_privatekey(key)
            else:
                ctx.use_privatekey_file(key)
        
        ctx.load_verify_locations(ca)
        
        self._context = ctx
    
    def _verify(self, conn, cert, errnum, depth, ok):
        """ Internally used method to verify the peer certificate.
            Does nothing.
        """
        return ok
    
    def getContext(self):
        """ Method has to be overwritten for twisted.
        """
        return self._context
