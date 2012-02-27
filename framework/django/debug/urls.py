from django.conf.urls.defaults import patterns, include, url
from piston.resource import Resource
import debug.Service

class CsrfExemptResource(Resource):
    """ A Custom Resource that is csrf exempt.
    """
    def __init__(self, handler, authentication=None):
        super(CsrfExemptResource, self).__init__(handler, authentication)
        self.csrf_exempt = getattr(self.handler, 'csrf_exempt', True)

LoginHandler = CsrfExemptResource(debug.Service.LoginHandler)

urlpatterns = patterns('',
    url(r'^login/$', LoginHandler)
)
