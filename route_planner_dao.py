import numpy as np


class RoutePlannerDAO(object):
    """
    Lớp này là lớp truy cập dữ liệu để tìm nạp dữ liệu
    từ  máy chủ  cho  RoutePlanner
    """

    def __init__(self, wmap, sampling_resolution):
        """
        Constructor.

            :param wmap: carla.world object
            :param sampling_resolution: khoảng cách lấy mẫu giữa các điểm tham chiếu 
        """
        self._sampling_resolution = sampling_resolution
        self._wmap = wmap

    def get_topology(self):
        """
        Bộ truy cập cho cấu trúc liên kết.
            :return topology: danh sách các đối tượng từ điển với các thuộc tính sau 
                entry   -   điểm tham chiếu của điểm lối vào của đoạn đường
                entryxyz-   tọa độ của điểm lối vào
                exit    -   điểm tham chiếu của điểm lối ra của đoạn đường
                exitxyz -   tọa độ điểm lối ra
                path    -   danh sách các điểm cách nhau 1m từ lối vào đến lối ra 
        """
        topology = []
        
        # Retrieving waypoints to construct a detailed topology
        for segment in self._wmap.get_topology():
            wp1, wp2 = segment[0], segment[1]
            l1, l2 = wp1.transform.location, wp2.transform.location
            # Rounding off to avoid floating point imprecision
            x1, y1, z1, x2, y2, z2 = np.round([l1.x, l1.y, l1.z, l2.x, l2.y, l2.z], 0)
            wp1.transform.location, wp2.transform.location = l1, l2
            seg_dict = dict()
            seg_dict['entry'], seg_dict['exit'] = wp1, wp2
            seg_dict['entryxyz'], seg_dict['exitxyz'] = (x1, y1, z1), (x2, y2, z2)
            seg_dict['path'] = []
            endloc = wp2.transform.location
            if wp1.transform.location.distance(endloc) > self._sampling_resolution:
                w = wp1.next(self._sampling_resolution)[0]
                while w.transform.location.distance(endloc) > self._sampling_resolution:
                    seg_dict['path'].append(w)
                    w = w.next(self._sampling_resolution)[0]
            else:
                seg_dict['path'].append(wp1.next(self._sampling_resolution)[0])
            topology.append(seg_dict)
        return topology

    def get_waypoint(self, location):
        """
        Phương thức trả về điểm tham chiếu tại vị trí đã cho

            :param location: vị trí của xe
            :return waypoint: điểm tham chiếu được tạo gần với vị trí
        """
        waypoint = self._wmap.get_waypoint(location)
        return waypoint

    def get_resolution(self):
        """ Truy cập lấy thông tin khoảng cách các điểm """
        return self._sampling_resolution
