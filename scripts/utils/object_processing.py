import numpy as np
from collections import defaultdict, namedtuple
import matplotlib.pyplot as plt
from scipy.cluster.vq import whiten, kmeans


def objects_clustering(fileName):
    logs = np.load(fileName)
    object_pos = defaultdict(list)
    for log in logs:
        object_pos[log[0]].append((np.float(log[1]),\
                                   np.float(log[2]),\
                                   np.float(log[3]),\
                                   int(log[4]),\
                                   np.float(log[5]),\
                                   np.float(log[6]),\
                                   np.float(log[7]),\
                                   np.float(log[8]),\
                                   np.float(log[9])))
    objects = []
    Point = namedtuple('Point', ['x', 'y', 'rob', 'name'])
    candidates = ['cat', 'banana', 'hot_dog' , 'stop_sign', 'person', 'horse']
    for key in candidates:
        if key not in object_pos:
            continue
        log = np.array(object_pos[key])
        filter_idx = (np.argwhere(np.logical_and(log[:,4] < 1, log[:,2] > 0.6))).flatten()
        if len(filter_idx) > 5:
            x = log[:,0][filter_idx]
            y = log[:,1][filter_idx]
            dist = log[:,5][filter_idx]
            closest_idx = np.argmin(dist)
            closest_pos = [np.float(coord) for coord in log[:,6:][filter_idx][closest_idx]]
            location_matrix = np.array([x, y]).T
            whitened = location_matrix #scipy.cluster.vq.whiten(location_matrix)
            cluster_centroid, _ = kmeans(whitened, 1)
            print(cluster_centroid[0][0], cluster_centroid[0][1], closest_pos , key)
            p = Point(cluster_centroid[0][0], cluster_centroid[0][1], closest_pos , key)
#             print(key)
#             plt.scatter(x, y)
#             plt.scatter([p.x], [p.y], c ='r')
#             plt.show()
            objects.append(p)
    return objects
if __name__ == '__main__':
    print(objects_clustering())