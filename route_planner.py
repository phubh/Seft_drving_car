import math
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib import pylab 
import carla
from enum import Enum
import astar  
class RoadOption(Enum):
    """
    RoadOption đại diện cho các cấu hình topology có thể có khi di chuyển từ một đoạn làn đường sang một đoạn đường khác.
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6


def vector(location_1, location_2):
    """
    Trả về vectơ đơn vị từ vị trí_1 đến vị trí_2

        :param location_1, location_2: carla.Location objects
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]

class RoutePlanner(object):
    """
    Lớp này cung cấp một kế hoạch lộ trình ở mức rất cao.
     Khởi tạo lớp bằng cách chuyển một tham chiếu tới
     Một đối tượng RoutePlannerDAO.
    """

    def __init__(self, dao):
        """
        Constructor
        """
        self._dao = dao
        self._topology = None
        self._graph = None
        self._id_map = None
        self._road_id_to_edge = None
        self._intersection_end_node = -1
        self._previous_decision = RoadOption.VOID

    def setup(self):
        """
        Thực hiện tra cứu dữ liệu máy chủ ban đầu cho cấu trúc liên kết chi tiết
        và xây dựng biểu đồ biểu diễn bản đồ thế giới.
        """
        self._topology = self._dao.get_topology()
        self._graph, self._id_map, self._road_id_to_edge = self._build_graph()
        
        self._find_loose_ends() 
        self._lane_change_link()        

    def _build_graph(self):
        """
        Hàm này xây dựng một biểu diễn đồ thị networkx của cấu trúc liên kết.
        Cấu trúc liên kết được đọc từ self._topology.
        thuộc tính nút đồ thị:
            vertex   -   (x,y,z) vị trí trên bản đồ thế giới
        thuộc tính cạnh đồ thị:
             entry_vector - vector đơn vị dọc theo tiếp tuyến tại điểm vào
             exit_vector - vector đơn vị dọc theo tiếp tuyến tại điểm thoát
             net_vector - vector đơn vị của hợp âm từ đầu vào đến đầu ra
             giao điểm - boolean cho biết nếu cạnh thuộc về một ngã tư
         return: graph -> networkx graph biểu thị bản đồ thế giới,
                         id_map-> ánh xạ từ (x, y, z) tới id nút
                         road_id_to_edge-> ánh xạ từ id đường đến cạnh trong biểu đồ
        """
        graph = nx.DiGraph()
        id_map = dict()  # Map with structure {(x,y,z): id, ... }
        road_id_to_edge = dict()  # Map with structure {road_id: {lane_id: edge, ... }, ... }

        for segment in self._topology:

            entry_xyz, exit_xyz = segment['entryxyz'], segment['exitxyz']
            path = segment['path']
            entry_wp, exit_wp = segment['entry'], segment['exit']
            intersection = entry_wp.is_junction
            road_id, section_id, lane_id = entry_wp.road_id, entry_wp.section_id, entry_wp.lane_id
            for vertex in entry_xyz, exit_xyz:
                # Adding unique nodes and populating id_map
                if vertex not in id_map:
                    new_id = len(id_map)
                    id_map[vertex] = new_id
                    graph.add_node(new_id, vertex=vertex)
            n1 = id_map[entry_xyz]
            n2 = id_map[exit_xyz]
            if road_id not in road_id_to_edge:
                road_id_to_edge[road_id] = dict()
            if section_id not in road_id_to_edge[road_id]:
                road_id_to_edge[road_id][section_id] = dict()
            road_id_to_edge[road_id][section_id][lane_id] = (n1, n2)

            entry_carla_vector = entry_wp.transform.rotation.get_forward_vector()
            exit_carla_vector = exit_wp.transform.rotation.get_forward_vector()

            # Adding edge with attributes
            graph.add_edge(
                n1, n2,
                length=len(path) + 1, path=path,
                entry_waypoint=entry_wp, exit_waypoint=exit_wp,
                entry_vector=np.array(
                    [entry_carla_vector.x, entry_carla_vector.y, entry_carla_vector.z]),
                exit_vector=np.array(
                    [exit_carla_vector.x, exit_carla_vector.y, exit_carla_vector.z]),
                net_vector=vector(entry_wp.transform.location, exit_wp.transform.location),
                intersection=intersection, type=RoadOption.LANEFOLLOW)
        
        return graph, id_map, road_id_to_edge

    def _find_loose_ends(self):
        """
        Phương pháp này tìm các đoạn đường có phần cuối không được kết nối và thêm chúng vào bên trong đồ thị 
        """
        count_loose_ends = 0
        hop_resolution = self._dao.get_resolution()
        for segment in self._topology:
            end_wp = segment['exit']
            exit_xyz = segment['exitxyz']
            road_id, section_id, lane_id = end_wp.road_id, end_wp.section_id, end_wp.lane_id
            if road_id in self._road_id_to_edge and section_id in self._road_id_to_edge[road_id] and lane_id in self._road_id_to_edge[road_id][section_id]:
                pass
            else:
                count_loose_ends += 1
                if road_id not in self._road_id_to_edge:
                    self._road_id_to_edge[road_id] = dict()
                if section_id not in self._road_id_to_edge[road_id]:
                    self._road_id_to_edge[road_id][section_id] = dict()
                n1 = self._id_map[exit_xyz]
                n2 = -1*count_loose_ends
                self._road_id_to_edge[road_id][section_id][lane_id] = (n1, n2)
                next_wp = end_wp.next(hop_resolution)
                path = []
                while next_wp is not None and next_wp and next_wp[0].road_id == road_id and next_wp[0].section_id == section_id and next_wp[0].lane_id == lane_id:
                    path.append(next_wp[0])
                    next_wp = next_wp[0].next(hop_resolution)
                if path:
                    n2_xyz = (path[-1].transform.location.x,
                              path[-1].transform.location.y,
                              path[-1].transform.location.z)
                    self._graph.add_node(n2, vertex=n2_xyz)
                    self._graph.add_edge(
                        n1, n2,
                        length=len(path) + 1, path=path,
                        entry_waypoint=end_wp, exit_waypoint=path[-1],
                        entry_vector=None, exit_vector=None, net_vector=None,
                        intersection=end_wp.is_junction, type=RoadOption.LANEFOLLOW)
       
    def _localize(self, location):
        """
        Hàm này tìm đoạn đường gần nhất với vị trí nhất định
        location        :   carla. Vị trí được bản địa hóa trong biểu đồ
        return          :   cặp id nút đại diện cho một cạnh trong biểu đồ
        """
        waypoint = self._dao.get_waypoint(location)
        edge = None
        try:
            edge = self._road_id_to_edge[waypoint.road_id][waypoint.section_id][waypoint.lane_id]
        except KeyError:
            print(
                "Failed to localize! : ",
                "Road id : ", waypoint.road_id,
                "Section id : ", waypoint.section_id,
                "Lane id : ", waypoint.lane_id,
                "Location : ", waypoint.transform.location.x,
                waypoint.transform.location.y)
        return edge

    def _lane_change_link(self):
        """
        Phương pháp này đặt các liên kết chi phí bằng không trong biểu đồ cấu trúc liên kết
        thể hiện khả năng thay đổi làn đường.
        """

        for segment in self._topology:
            left_found, right_found = False, False

            for waypoint in segment['path']:
                if not segment['entry'].is_junction:
                    next_waypoint, next_road_option, next_segment = None, None, None

                    if waypoint.right_lane_marking.lane_change & carla.LaneChange.Right and not right_found:
                        next_waypoint = waypoint.get_right_lane()
                        if next_waypoint is not None and next_waypoint.lane_type == carla.LaneType.Driving and waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANERIGHT
                            next_segment = self._localize(next_waypoint.transform.location)
                            if next_segment is not None:
                                self._graph.add_edge(
                                    self._id_map[segment['entryxyz']], next_segment[0], entry_waypoint=waypoint,
                                    exit_waypoint=next_waypoint, intersection=False, exit_vector=None,
                                    path=[], length=0, type=next_road_option, change_waypoint=next_waypoint)
                                right_found = True
                    if waypoint.left_lane_marking.lane_change & carla.LaneChange.Left and not left_found:
                        next_waypoint = waypoint.get_left_lane()
                        if next_waypoint is not None and next_waypoint.lane_type == carla.LaneType.Driving and waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANELEFT
                            next_segment = self._localize(next_waypoint.transform.location)
                            if next_segment is not None:
                                self._graph.add_edge(
                                    self._id_map[segment['entryxyz']], next_segment[0], entry_waypoint=waypoint,
                                    exit_waypoint=next_waypoint, intersection=False, exit_vector=None,
                                    path=[], length=0, type=next_road_option, change_waypoint=next_waypoint)
                                left_found = True
                if left_found and right_found:
                    break
       
    def _distance_heuristic(self, n1, n2):
        """
        Tính heuristic khoảng cách để tìm kiếm đường đi
        in self._graph
        """
        l1 = np.array(self._graph.nodes[n1]['vertex'])
        l2 = np.array(self._graph.nodes[n2]['vertex'])
        return np.linalg.norm(l1-l2)

    def _path_search(self, origin, destination):
        """
        Hàm này tìm đường đi ngắn nhất kết nối điểm đi và điểm đến
         bằng cách sử dụng tìm kiếm A * với tính toán khoảng cách.
         origin: carla. đối tượng định vị của vị trí bắt đầu
         destination: carla.Location đối tượng của vị trí kết thúc
         return: đường dẫn dưới dạng danh sách id nút (dưới dạng int) của biểu đồ self._graph
         kết nối điểm xuất phát và điểm đến
        """
        start, end = self._localize(origin), self._localize(destination)
       
        route = astar.astar_path(
            self._graph, source=start[0], target=end[0],
            heuristic=self._distance_heuristic, weight='length')
        route.append(end[1])
        return route

    def _successive_last_intersection_edge(self, index, route):
        """
        Phương thức này trả về cạnh giao nhau liên tiếp cuối cùng
         từ một chỉ mục bắt đầu trên tuyến đường.
         Điều này giúp di chuyển qua các cạnh giao lộ nhỏ để tính toán
         quyết định rẽ thích hợp.
        """

        last_intersection_edge = None
        last_node = None
        for node1, node2 in [(route[i], route[i+1]) for i in range(index, len(route)-1)]:
            candidate_edge = self._graph.edges[node1, node2]
            if node1 == route[index]:
                last_intersection_edge = candidate_edge
            if candidate_edge['type'] == RoadOption.LANEFOLLOW and candidate_edge['intersection']:
                last_intersection_edge = candidate_edge
                last_node = node2
            else:
                break

        return last_node, last_intersection_edge

    def _turn_decision(self, index, route, threshold=math.radians(35)):
        """
        Phương thức này trả về quyết định rẽ (RoadOption) cho cặp cạnh
        xung quanh chỉ mục hiện tại của danh sách tuyến đường
        """

        decision = None
        previous_node = route[index-1]
        current_node = route[index]
        next_node = route[index+1]
        next_edge = self._graph.edges[current_node, next_node]
        if index > 0:
            if self._previous_decision != RoadOption.VOID and self._intersection_end_node > 0 and self._intersection_end_node != previous_node and next_edge['type'] == RoadOption.LANEFOLLOW and next_edge['intersection']:
                decision = self._previous_decision
            else:
                self._intersection_end_node = -1
                current_edge = self._graph.edges[previous_node, current_node]
                calculate_turn = current_edge['type'] == RoadOption.LANEFOLLOW and not current_edge[
                    'intersection'] and next_edge['type'] == RoadOption.LANEFOLLOW and next_edge['intersection']
                if calculate_turn:
                    last_node, tail_edge = self._successive_last_intersection_edge(index, route)
                    self._intersection_end_node = last_node
                    if tail_edge is not None:
                        next_edge = tail_edge
                    cv, nv = current_edge['exit_vector'], next_edge['exit_vector']
                    if cv is None or nv is None:
                        return next_edge['type']
                    cross_list = []
                    for neighbor in self._graph.successors(current_node):
                        select_edge = self._graph.edges[current_node, neighbor]
                        if select_edge['type'] == RoadOption.LANEFOLLOW:
                            if neighbor != route[index+1]:
                                sv = select_edge['net_vector']
                                cross_list.append(np.cross(cv, sv)[2])
                    next_cross = np.cross(cv, nv)[2]
                    deviation = math.acos(np.clip(
                        np.dot(cv, nv)/(np.linalg.norm(cv)*np.linalg.norm(nv)), -1.0, 1.0))
                    if not cross_list:
                        cross_list.append(0)
                    if deviation < threshold:
                        decision = RoadOption.STRAIGHT
                    elif cross_list and next_cross < min(cross_list):
                        decision = RoadOption.LEFT
                    elif cross_list and next_cross > max(cross_list):
                        decision = RoadOption.RIGHT
                    elif next_cross < 0:
                        decision = RoadOption.LEFT
                    elif next_cross > 0:
                        decision = RoadOption.RIGHT
                else:
                    decision = next_edge['type']

        else:
            decision = next_edge['type']

        self._previous_decision = decision
        return decision

    def abstract_route_plan(self, origin, destination):
        """
        Hàm sau tạo kế hoạch tuyến đường dựa trên
         origin: carla.Location object của vị trí bắt đầu của tuyến đường
         destination: carla.Location đối tượng của vị trí kết thúc của tuyến đường
         return: danh sách các quyết định điều hướng lần lượt như
         Phần tử RoadOption 
        Possible values are STRAIGHT, LEFT, RIGHT, LANEFOLLOW, VOID
        CHANGELANELEFT, CHANGELANERIGHT
        """

        route = self._path_search(origin, destination)
        plan = []

        for i in range(len(route) - 1):
            road_option = self._turn_decision(i, route)
            plan.append(road_option)

        return plan

    def _find_closest_in_list(self, current_waypoint, waypoint_list):
        min_distance = float('inf')
        closest_index = -1
        for i, waypoint in enumerate(waypoint_list):
            distance = waypoint.transform.location.distance(
                current_waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def trace_route(self, origin, destination):
        """
        Phương thức này trả về danh sách (carla.Waypoint, RoadOption)
        từ điểm xuất phát đến điểm đến
        """
        
        route_trace = []
        route = self._path_search(origin, destination) 

        current_waypoint = self._dao.get_waypoint(origin)
        destination_waypoint = self._dao.get_waypoint(destination)
        resolution = self._dao.get_resolution()

        for i in range(len(route) - 1):
            road_option = self._turn_decision(i, route)
            edge = self._graph.edges[route[i], route[i+1]]
            path = []

            if edge['type'] != RoadOption.LANEFOLLOW and edge['type'] != RoadOption.VOID:
                route_trace.append((current_waypoint, road_option))
                exit_wp = edge['exit_waypoint']
                n1, n2 = self._road_id_to_edge[exit_wp.road_id][exit_wp.section_id][exit_wp.lane_id]
                next_edge = self._graph.edges[n1, n2]
                if next_edge['path']:
                    closest_index = self._find_closest_in_list(current_waypoint, next_edge['path'])
                    closest_index = min(len(next_edge['path'])-1, closest_index+5)
                    current_waypoint = next_edge['path'][closest_index]
                else:
                    current_waypoint = next_edge['exit_waypoint']
                route_trace.append((current_waypoint, road_option))

            else:
                path = path + [edge['entry_waypoint']] + edge['path'] + [edge['exit_waypoint']]
                closest_index = self._find_closest_in_list(current_waypoint, path)
                for waypoint in path[closest_index:]:
                    current_waypoint = waypoint
                    route_trace.append((current_waypoint, road_option))
                    if len(route)-i <= 2 and waypoint.transform.location.distance(destination) < 2*resolution:
                        break
                    elif len(route)-i <= 2 and current_waypoint.road_id == destination_waypoint.road_id and current_waypoint.section_id == destination_waypoint.section_id and current_waypoint.lane_id == destination_waypoint.lane_id:
                        destination_index = self._find_closest_in_list(destination_waypoint, path)
                        if closest_index > destination_index:
                            break       
        # nx.draw_networkx(self._graph,pos=nx.spring_layout(self._graph))         
        # plt.show()
        return route_trace
