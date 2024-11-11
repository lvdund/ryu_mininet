from mininet.topo import Topo
from mininet.net import Mininet
from mininet.node import OVSSwitch, RemoteController
from mininet.link import TCLink
from mininet.log import setLogLevel
from mininet.cli import CLI

def customNet():
    net = Mininet(topo=None, build=False)
    net.addController('c0', controller=RemoteController, ip = '127.0.0.1', port=6653)

    h1, h2, h3 = [net.addHost(h) for h in ('h1', 'h2', 'h3')]
    s1, s2, s3 = [net.addSwitch(s) for s in ('s1', 's2', 's3')]

    for sa, sb in [ (s1, s2), (s2, s3) ]:
        net.addLink( sa , sb )

    for h, s in [ (h1, s1), (h2, s2), (h3, s3) ]:
        net.addLink( h , s )
    
    net.start()
    CLI(net)
    net.stop()

if __name__ == '__main__':
    setLogLevel('info')
    customNet()

exit(0)