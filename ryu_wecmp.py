from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.ofproto import ofproto_v1_3
from ryu.lib.packet import packet
from ryu.lib.packet import ethernet
from ryu.lib.packet import arp
from ryu.lib.packet import ipv6
from ryu.topology.api import get_switch, get_link
from ryu.topology import event

from collections import defaultdict
import random
import time

REFERENCE_BW = 10000000
DEFAULT_BW = 10000000
MAX_PATHS = 4  # Tối đa số lượng đường đi có thể chọn để cân bằng tải

class WeightedECMPController(app_manager.RyuApp):
    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(WeightedECMPController, self).__init__(*args, **kwargs)
        self.datapath_list = {}
        self.arp_table = {}
        self.switches = []
        self.hosts = {}
        self.multipath_group_ids = {}
        self.group_ids = []
        self.adjacency = defaultdict(dict)
        self.bandwidths = defaultdict(lambda: defaultdict(lambda: DEFAULT_BW))

    def get_paths(self, src, dst):
        ''' Tìm tất cả các đường đi từ src đến dst bằng thuật toán DFS '''
        if src == dst:
            return [[src]]
        paths = []
        stack = [(src, [src])]
        while stack:
            (node, path) = stack.pop()
            for next_node in set(self.adjacency[node].keys()) - set(path):
                if next_node == dst:
                    paths.append(path + [next_node])
                else:
                    stack.append((next_node, path + [next_node]))
        return paths

    def get_link_cost(self, s1, s2):
        ''' Tính chi phí của liên kết dựa trên băng thông '''
        e1 = self.adjacency[s1][s2]
        e2 = self.adjacency[s2][s1]
        link_bandwidth = min(self.bandwidths[s1][e1], self.bandwidths[s2][e2])
        return REFERENCE_BW / link_bandwidth

    def get_path_cost(self, path):
        ''' Tính chi phí của đường đi '''
        cost = 0
        for i in range(len(path) - 1):
            cost += self.get_link_cost(path[i], path[i + 1])
        return cost

    def get_optimal_paths(self, src, dst):
        ''' Lấy MAX_PATHS đường đi tối ưu nhất '''
        paths = self.get_paths(src, dst)
        paths = sorted(paths, key=lambda x: self.get_path_cost(x))
        return paths[:MAX_PATHS]

    def add_ports_to_paths(self, paths, first_port, last_port):
        ''' Thêm các cổng nối các switch trong mỗi đường đi '''
        paths_p = []
        for path in paths:
            path_ports = {}
            in_port = first_port
            for s1, s2 in zip(path[:-1], path[1:]):
                out_port = self.adjacency[s1][s2]
                path_ports[s1] = (in_port, out_port)
                in_port = self.adjacency[s2][s1]
            path_ports[path[-1]] = (in_port, last_port)
            paths_p.append(path_ports)
        return paths_p

    def generate_openflow_gid(self):
        ''' Tạo group ID ngẫu nhiên cho OpenFlow '''
        gid = random.randint(0, 2**32)
        while gid in self.group_ids:
            gid = random.randint(0, 2**32)
        return gid

    def install_paths(self, src, first_port, dst, last_port, ip_src, ip_dst):
        ''' Cài đặt các đường đi và tạo các group OpenFlow cho Weighted ECMP '''
        paths = self.get_optimal_paths(src, dst)
        path_weights = [self.get_path_cost(path) for path in paths]
        sum_of_weights = sum(path_weights)
        paths_with_ports = self.add_ports_to_paths(paths, first_port, last_port)
        switches_in_paths = set().union(*paths)

        for node in switches_in_paths:
            dp = self.datapath_list[node]
            ofp = dp.ofproto
            parser = dp.ofproto_parser
            ports = defaultdict(list)

            for i, path in enumerate(paths_with_ports):
                if node in path:
                    in_port = path[node][0]
                    out_port = path[node][1]
                    weight = path_weights[i]
                    ports[in_port].append((out_port, weight))

            for in_port, out_ports in ports.items():
                match_ip = parser.OFPMatch(eth_type=0x0800, ipv4_src=ip_src, ipv4_dst=ip_dst)
                match_arp = parser.OFPMatch(eth_type=0x0806, arp_spa=ip_src, arp_tpa=ip_dst)
                actions = []

                if len(out_ports) > 1:
                    group_id = self.multipath_group_ids.get((node, src, dst))
                    if group_id is None:
                        group_id = self.generate_openflow_gid()
                        self.multipath_group_ids[(node, src, dst)] = group_id

                    buckets = []
                    for port, weight in out_ports:
                        bucket_weight = int(round((1 - weight / sum_of_weights) * 10))
                        bucket_action = [parser.OFPActionOutput(port)]
                        buckets.append(
                            parser.OFPBucket(
                                weight=bucket_weight,
                                watch_port=port,
                                watch_group=ofp.OFPG_ANY,
                                actions=bucket_action
                            )
                        )

                    req = parser.OFPGroupMod(dp, ofp.OFPGC_ADD, ofp.OFPGT_SELECT, group_id, buckets)
                    dp.send_msg(req)
                    actions = [parser.OFPActionGroup(group_id)]
                else:
                    actions = [parser.OFPActionOutput(out_ports[0][0])]

                self.add_flow(dp, 32768, match_ip, actions)
                self.add_flow(dp, 1, match_arp, actions)

    def add_flow(self, datapath, priority, match, actions, buffer_id=None):
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]
        if buffer_id:
            mod = parser.OFPFlowMod(datapath=datapath, buffer_id=buffer_id, priority=priority, match=match, instructions=inst)
        else:
            mod = parser.OFPFlowMod(datapath=datapath, priority=priority, match=match, instructions=inst)
        datapath.send_msg(mod)

    @set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
    def _switch_features_handler(self, ev):
        datapath = ev.msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        match = parser.OFPMatch()
        actions = [parser.OFPActionOutput(ofproto.OFPP_CONTROLLER, ofproto.OFPCML_NO_BUFFER)]
        self.add_flow(datapath, 0, match, actions)

    @set_ev_cls(event.EventSwitchEnter)
    def switch_enter_handler(self, ev):
        switch = ev.switch.dp
        ofp_parser = switch.ofproto_parser
        if switch.id not in self.switches:
            self.switches.append(switch.id)
            self.datapath_list[switch.id] = switch
            req = ofp_parser.OFPPortDescStatsRequest(switch)
            switch.send_msg(req)

    @set_ev_cls(ofp_event.EventOFPPortDescStatsReply, MAIN_DISPATCHER)
    def port_desc_stats_reply_handler(self, ev):
        switch = ev.msg.datapath
        for p in ev.msg.body:
            self.bandwidths[switch.id][p.port_no] = p.curr_speed

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def _packet_in_handler(self, ev):
        msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']

        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)
        arp_pkt = pkt.get_protocol(arp.arp)

        if eth.ethertype == 35020:
            return

        if pkt.get_protocol(ipv6.ipv6):
            match = parser.OFPMatch(eth_type=eth.ethertype)
            actions = []
            self.add_flow(datapath, 1, match, actions)
            return None

        dst = eth.dst
        src = eth.src
        dpid = datapath.id

        if src not in self.hosts:
            self.hosts[src] = (dpid, in_port)

        out_port = ofproto.OFPP_FLOOD

        if arp_pkt:
            src_ip = arp_pkt.src_ip
            dst_ip = arp_pkt.dst_ip
            if arp_pkt.opcode == arp.ARP_REPLY:
                self.arp_table[src_ip] = src
                h1 = self.hosts[src]
                h2 = self.hosts[dst]
                out_port = self.install_paths(h1[0], h1[1], h2[0], h2[1], src_ip, dst_ip)
                self.install_paths(h2[0], h2[1], h1[0], h1[1], dst_ip, src_ip)
            elif arp_pkt.opcode == arp.ARP_REQUEST:
                if dst_ip in self.arp_table:
                    self.arp_table[src_ip] = src
                    dst_mac = self.arp_table[dst_ip]
                    h1 = self.hosts[src]
                    h2 = self.hosts[dst_mac]
                    out_port = self.install_paths(h1[0], h1[1], h2[0], h2[1], src_ip, dst_ip)
                    self.install_paths(h2[0], h2[1], h1[0], h1[1], dst_ip, src_ip)

        actions = [parser.OFPActionOutput(out_port)]
        data = None
        if msg.buffer_id == ofproto.OFP_NO_BUFFER:
            data = msg.data
        out = parser.OFPPacketOut(datapath=datapath, buffer_id=msg.buffer_id, in_port=in_port, actions=actions, data=data)
        datapath.send_msg(out)
