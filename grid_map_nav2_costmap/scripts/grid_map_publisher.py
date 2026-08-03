#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
import numpy as np

class MockGridMapPublisher(Node):
    def __init__(self):
        super().__init__('mock_grid_map_publisher')
        self.pub = self.create_publisher(GridMap, '/elevation_mapping/elevation_map', 10)
        self.timer = self.create_timer(0.5, self.publish_map)

    def publish_map(self):
        msg = GridMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = 0.05
        msg.layers = ['traversability']

        msg.info.length_x = 20.0
        msg.info.length_y = 20.0
        self.get_logger().info(f'length_x={msg.info.length_x} length_y={msg.info.length_y} resolution={msg.info.resolution}')
        size = int(msg.info.length_x / msg.info.resolution)
        # Low value = low cost (traversable), high value = high cost (difficult)
        data = np.zeros((size, size), dtype=np.float32)  # fully traversable everywhere

        # Thick vertical band through the middle, full y extent
        # dim1 is column_index (x-axis), dim2 is row_index (y-axis)
        # Middle 10 columns (1.0m wide) centered on the map, all rows
        mid = size // 2
        band_half_width = 5  # 5 cells each side = 10 cells = 1.0m wide
        data[:, mid - band_half_width:mid + band_half_width] = 1.0  # non-traversable band

        layer = Float32MultiArray()
        layout = MultiArrayLayout()
        dim1 = MultiArrayDimension()
        dim1.label = 'column_index'
        dim1.size = size
        dim1.stride = size * size
        dim2 = MultiArrayDimension()
        dim2.label = 'row_index'
        dim2.size = size
        dim2.stride = size
        layout.dim = [dim1, dim2]
        layer.layout = layout
        layer.data = data.flatten().tolist()
        msg.data = [layer]

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MockGridMapPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()