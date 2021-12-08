from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from utils.grids import StochOccupancyGrid2D
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import rospy
class testAstar:
    def __init__(self) -> None:
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0, 0]
        self.map_probs = [] 
        self.occupancy = None
        print("Creating Subscribers")
        # rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        # rospy.Subscriber("/map_metadata", MapMetaData, self.map_md_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
    def scan_callback(self, msg):
        print(msg.range)
    def map_md_callback(self, msg):
        """
        receives maps meta data and stores it
        """
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x, msg.origin.position.y)

    def map_callback(self, msg):
        """
        receives new map info and updates the map
        """
        self.map_probs = msg.data
        print(self.map_probs)
        # if we've received the map metadata and have a way to update it:
        if (
            self.map_width > 0
            and self.map_height > 0
            and len(self.map_probs) > 0
        ):
            self.occupancy = StochOccupancyGrid2D(
                self.map_resolution,
                self.map_width,
                self.map_height,
                self.map_origin[0],
                self.map_origin[1],
                2,
                self.map_probs,
            )
    def run(self):
        while not rospy.is_shutdown():
            # if not self.occupancy is None:
            #     self.occupancy.plot()
            #     plt.savefig("Occupan.png")
            #     break

if __name__ == "__main__":
        test = testAstar()
        test.run()