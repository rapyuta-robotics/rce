### Setup machine for lxc containers
### http://www.activestate.com/blog/2011/10/virtualization-ec2-cloud-using-lxc

## Setup bridge
brctl addbr br0
brctl setfd br0 0
ifconfig br0 192.168.3.1 up

## Setup iptables
iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sysctl -w net.ipv4.ip_forward=1

## Setup DNS
# Edit file /etc/dnsmasq.conf
	# Add/Uncomment the following:
	domain-needed
	bogus-priv
	interface = br0
	listen-address = 127.0.0.1
	listen-address = 192.168.3.1
	expand-hosts
	domain = containers
	dhcp-range = 192.168.3.50,192.168.3.200,1h

# Edit file /etc/dhcp/dhclient.conf
	# Add at the beginning of the file
	prepend domain-name-servers 127.0.0.1;
	prepend domain-search "containers.";

# Renew DHCP lease
dhclient3 -e IF_METRIC=100 -pf /var/run/dhclient.eth0.pid -lf /var/lib/dhcp3/dhclient.eth0.leases eth0

# Restart dnsmasq
service dnsmasq restart
