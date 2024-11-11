from mininet.topo import Topo
from mininet.net import Mininet
from mininet.node import OVSSwitch, RemoteController
from mininet.link import TCLink
from mininet.log import setLogLevel
from mininet.cli import CLI

def build():

    net = Mininet(topo=None, build=False)
    net.addController('c0', controller=RemoteController, ip = '127.0.0.1', port=6653)
    
    # Ping thử giữa các node để kiểm tra kết nối
    # net.pingAll()
    
    # Tạo các switch (node)
    switches = {}
    for i in range(1, 16):
        switches[f'N{i}'] = net.addSwitch(f'N{i}')

    # Create hosts and link them to their corresponding switch
    for i in range(1, 16):
        host = net.addHost(f'h{i}')
        net.addLink(host, switches[f'N{i}'])
    
    # Danh sách các liên kết với độ trễ truyền tải là 67.80 ms và băng thông
    links = [
        ('N1', 'N6', 1000),
        ('N1', 'N7', 700),
        ('N1', 'N8', 400),
        ('N2', 'N3', 800),
        ('N2', 'N5', 100),
        ('N2', 'N6', 1000),
        ('N3', 'N5', 100),
        ('N3', 'N8', 800),
        ('N4', 'N5', 100),
        ('N4', 'N6', 400),
        ('N6', 'N13', 200),
        ('N7', 'N10', 500),
        ('N7', 'N14', 500),
        ('N8', 'N9', 300),
        ('N8', 'N15', 300),
        ('N9', 'N10', 200),
        ('N9', 'N12', 300),
        ('N9', 'N15', 100),
        ('N10', 'N12', 100),
        ('N11', 'N13', 100),
        ('N11', 'N14', 100),
        ('N13', 'N14', 100)
    ]
    
    # Tạo các liên kết với băng thông và độ trễ
    for src, dst, bw in links:
        net.addLink(
            switches[src],
            switches[dst],
            cls=TCLink,
            bw=bw if bw > 0 else None,  # Chỉ định băng thông nếu không phải là 0
            delay='67.8ms'
            )
    
    net.start()
    CLI(net)
    net.stop()


if __name__ == '__main__':
    setLogLevel('info')
    build()

exit(0)