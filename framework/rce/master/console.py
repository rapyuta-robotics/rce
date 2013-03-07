from collections import defaultdict

#TODO : Modify settings on the fly for both cloud engine and the underlying settings file


class InterfaceConnection2way(dict):
    def __init__(self, d=None):
        dict.__init__(self)
        self.d1 = defaultdict(list)
        self.d2 = defaultdict(list)

    def __getitem__(self, key):
        try:
            return self.d1.get(key)
        except KeyError:
            return self.d2(key)

    def __setitem__(self, key, val):
        self.d1[key].append(val)
        self.d2[val].append(key)

    def key(self, val):
        return self.d2[val]

    def keys(self, val):
        return self.d2.keys()

    def iterkeys(self, val):
        return self.d2.iterkeys()

    def get(self, key):
        return self.d1[key]

    def values(self, key):
        return self.d1.values()

    def itervalues(self, key):
        return self.d1.itervalues()

class UnauthorisedLogon(Exception):
    """ Exception to be raised when the user login fails
    due to incorrect user credentials.
    """
    pass

class Console(object):
    """
    The selective class that helps implement the console abilities
    """

    def __init__(self,root):
        """ Initialize the Console Handler.

            @param root:        Reference to top level of data structure.
            @type  root:        rce.master.core.RoboEarthCloudEngine
        """
        self._root = root


    # The following method is only accessible to the Admin user
    def _admin_list_machines(self):
        """ Gets a list of all machines that are available in the Cloud Engine
        This should be only accessible to the top level admin for security reasons

        @return:            List of machines .
        @rtype:             List(rce.master.machine.Machine)
        """
        return self._root._balancer._machines

    def _admin_list_machines_byIP(self):
        """ Gets a list of all machines that are available in the Cloud Engine
        This should be only accessible to the top level admin for security reasons

        @return:            List of machines by IPv4 addresses
        @rtype:             List(str)
        """
        return [machine.IP for machine in self._list_machines()]

    def _admin_list_machine_containers(self, machine):
        """ Gets a list of all containers available in the Machine
        This should be only accessible to the top level admin for security reasons

        @param machine:        Machine for which the containers need to be listed.
        @type  machine:        rce.master.machine.Machine

        @return:               List of containers .
        @rtype:                List(rce.master.container.Container)
        """
        return machine._containers

    def _admin_list_machine_stats(self, machine):
        """ Gets some useful facts about the Machine
        This should be only accessible to the top level admin for security reasons

        @param machine:        Machine for which the containers need to be listed.
        @type  machine:        rce.master.machine.Machine

        @return:               List of containers .
        @rtype:                List(rce.master.container.Container)
        """
        stat_info = {'active': machine.active,
                     'IP' :  machine.IP,
                     'capacity' : machine.capacity}
        return stat_info

    def _admin_list_machine_users(self, machine):
        """ Gets some useful facts about the Machine
        This should be only accessible to the top level admin for security reasons

        @param machine:        Machine for which the containers need to be listed.
        @type  machine:        rce.master.machine.Machine

        @return:               Counter containing the users and the number of containers they are running .
        @rtype:                List collections.Counter
        """
        return machine._users.keys()

    def _admin_list_userID(self):
        """ Gets a list of all users that are logged into the Cloud Engine
        This should be only accessible to the top level admin for security reasons

        @return:            List of users .
        @rtype:             list(str)
        """
        return self._root._users.keys()

    def _admin_add_user(self, username, password):
        """ Add a user to the RoboEarth Cloud Engine

        @param username:         Username of the user to be added
        @type username:         str

        @param password:        Password of the user to be added
        @type  passowrd:        str

        @return:            Results to True if succeeded  .
        @rtype:             boolean
        """
        self._root._checker.addUser(username, password)

    def _admin_remove_user(self, username, password):
        """ Remove a user to the RoboEarth Cloud Engine

        @param username:         Username of the user to be removed
        @type username:         str

        @return:            Results to True if succeeded  .
        @rtype:             boolean
        """
        self._root._checker.removeUser(username)

    def _admin_user_passwd(self, username, password):
        """ Add a user to the RoboEarth Cloud Engine

        @param username:         Username of the user to be added
        @type username:         str

        @param password:        Password of the user to be added
        @type  passowrd:        str

        @return:            Results to True if succeeded  .
        @rtype:             boolean
        """
        self._root._checker.passwd(username, password)
    #TODO : Please read
    # The rest of the methods to be used are here.
    # It is the responsibility of the implementor to use the right parameters and handle any exceptions
    # most dict methods will return None as the use the standard get method and will not raise TypeErrors
    def _get_user(self, username, password):
        """ This method will return the reference to the user with the passed
        credentials

        @param username:         Username of the user who wishes to login
        @type  username:         str

        @param password:         Password of the user who wishes to login
        @type  password:         str

        @return:                 User logged into the cloud engine.
        @rtype:                  rce.master.user.User
        """
        d = self._root._checker.requestAvatarId(username, password) #TODO , needs to be fixed depending on the implementation decided

        d.addCallback(lambda uID: self._getUser(uID))
        d.addErrback(lambda failure : failure.raiseException(UnauthorisedLogon))
        # It is the responsibility of the method implementing this to handle this exception and relay the appropriate info to the user terminal
        return d

    def _get_user_byID(self, userID):
        """Gets a User given the usedID

        @param userID:          userID of the user you wish to fetch
        @type  userID:          str

        @return:                User logged into the cloud engine.
        @rtype:                 rce.master.user.User

        """
        return self._root._users.get(userID)

    def _list_user_robots(self, user):
        """ Retrieve a list of all the robots owned by the user.

        @param user:         User logged into the cloud engine.
        @type:               rce.master.user.User

        @return:             List of Robot uuids of rce.master.user.Robot.
        @rtype:              list(str)
        """
        return user._robots.keys()

    def _list_user_containers(self, user):
        """ Retrieve a list of all the containers owned by the user.

        @param user:         User logged into the cloud engine.
        @type:               rce.master.user.User

        @return:             List of Container tags to rce.master.user.Container
        @rtype:              list(str)
        """
        return user._containers.keys()

    def _list_user_connections(self, user):
        """ Retrieve a list of all the connections made by the user.

        @param user:         User logged into the cloud engine.
        @rtype:              rce.master.user.User

        @return:             List of hashed connection keys to rce.master.network.Connection
        @rtype:              list(str)
        """
        return user._connections.keys()

    def _retrieve_robot_byUUID(self, user, uuid):
        """Gets a Robot given the key and user

        @param user:         User logged into the cloud engine.
        @type:               rce.master.user.User

        @param uuid:         uuid associated to the robot object to be fetched
        @type:               str

        @return:             Robot object
        @rtype:              rce.master.user.Robot

        """
        return user._robots.get(uuid)

    def _retrieve_container_byTag(self, user, tag):
        """Gets a User given the usedID

        @param user:             User logged into the cloud engine.
        @type:                   rce.master.user.User

        @param uuid:             tag associated to the container object to be fetched
        @type:                   str

        @return:                 Container object
        @rtype:                  rce.master.user.Container

        """
        return user._containers.get(tag)

    def _retrieve_connection_byHash(self, user, hash_key):
        """Gets a User given the usedID

        @param user:             User logged into the cloud engine.
        @type:                   rce.master.user.User

        @param uuid:             hash_key associated to the connection object to be fetched
        @type:                   str

        @return:                 Container object
        @rtype:                  rce.master.user.Container

        """
        return user._connection.get(hash_key)

    # TODO : Please read
    # The above functions expose a bunch of basic objects form the cloud engine , the rest of these methods use the exposed objects in convenient wrappers ,
    # feel free to edit and modify these as required alternatively one could use the above to generate the required behavior.

    def generate_user_graph(self, user):
        """Generates a multidict representation of the users connect graph across the cloud engine.
        Useful for visualizing and debugging a users activities and connections.
        """
        #build the physical container graph
        def get_containmer_tree(tag):
            container = self._retrieve_container_byTag(user, tag)
            nodes = container._nodes.keys()
            parameters = container._parameters.keys()
            interfaces = container._interfaces.keys()
            return {tag:{'nodes':nodes,'parameters':parameters,
                         'interfaces':interfaces}}

        machines = defaultdict(list)
        for tag,container in user._containers.iteritems():
            machines[container._machine.IP].append(get_containmer_tree(tag))

        #build the robots graph
        robots = {}
        for uuid,robot in user._robots.iteritems():
            robots[uuid] =  robot._interfaces.keys()
        # build the connections map
        connections = InterfaceConnection2way()
        for connection in user._connections.itervalues():
            connections[connection._connectionA._interface._uid] = \
                                 connection._connectionB._interface._uid

        return {'userID':user._userID,'network':machines,
                'robots':robots, 'connections': connections}
