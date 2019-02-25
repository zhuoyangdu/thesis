#! /usr/bin/python3
import path_planner_debug_pb2
import sys
import google.protobuf.text_format as text_format

import numpy as np
import matplotlib.pyplot as plt
plt.style.use('ggplot')
debug = path_planner_debug_pb2.PathPlannerDebug()

f = open("/home/zy/thesis/tmp_data/path_.txt", 'r')
text_format.Merge(f.read(), debug)
f.close()
#print(text_format.MessageToString(debug))

tree = debug.tree
plt.figure(1)

for i in range(1, len(tree.nodes)):
    node1 = tree.nodes[i]
    node2 = tree.nodes[node1.parent_index]
    plt.plot([node1.col, node2.col], [511-node1.row, 511-node2.row], 'b', linewidth=1)

plt.axis('equal')
plt.axis([0, 512, 0, 512])

plt.show()

