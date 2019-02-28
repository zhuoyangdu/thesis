#! /usr/local/bin/python3
import path_planner_debug_pb2
import speed_profile_debug_pb2
import sys
import google.protobuf.text_format as text_format
import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection

length = 3.5
width = 1.5

def get_vehicle_vertex(x, y, ang):
    pos_f = [x + length/2 * math.cos(ang), y + length/2 * math.sin(ang)];
    pos_r = [x - length/2 * math.cos(ang), y - length/2 * math.sin(ang)];
    pos_rl = [pos_r[0] - width/2 * math.sin(ang), pos_r[1] + width/2 * math.cos(ang)];
    pos_rr = [pos_r[0] + width/2 * math.sin(ang), pos_r[1] - width/2 * math.cos(ang)];
    v = [[pos_f[0], pos_f[1]], [pos_rr[0],pos_rr[1]], [pos_rl[0], pos_rl[1]]]
    return Polygon(v)

def plot_roads(axes):
    axes.plot([-200,-20],[0,0],'k',linewidth=1, alpha=1)
    axes.plot([-200,-20],[3.75,3.75],'w--',linewidth=1)
    axes.plot([-200,-20],[-3.75,-3.75],'w--',linewidth=1)
    axes.plot([-200,-20],[3.75*2,3.75*2],'w--',linewidth=1)
    axes.plot([-200,-20],[-3.75*2,-3.75*2],'w--',linewidth=1)
    patches = []
    color = []
    v = [[-500,-11.25], [-500, 11.25], [500, 11.25], [500,-11.25]]
    polygon = Polygon(v)
    color.append([0.5,0.5,0.5])
    patches.append(polygon)
    p = PatchCollection(patches, facecolors=color, alpha=0.6)
    axes.add_collection(p)
    axes.set_xlim([-160,-80])
    axes.set_ylim([-12,12])
    axes.set_aspect(1.0)


def parse_file(path_file_name, speed_file_name):
    debug_path = path_planner_debug_pb2.PathPlannerDebug()

    f = open(path_file_name, 'r')
    text_format.Merge(f.read(), debug_path)
    f.close()

    debug_speed = speed_profile_debug_pb2.SpeedProfileDebug()

    f = open(speed_file_name, 'r')
    text_format.Merge(f.read(), debug_speed)
    f.close()
    return debug_path, debug_speed


def plots_allocation():
    f = plt.figure(figsize=(12, 6),tight_layout=True)
    gs0 = gridspec.GridSpec(1, 3, figure=f)
    gs00 = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=gs0[0], hspace=0.5, wspace=0.5)

    ax_path_tree = plt.Subplot(f, gs00[1:,:])
    f.add_subplot(ax_path_tree)
    ax_path_tree.set_title("RRT tree of path searching.", fontdict={'fontsize': 10})
    ax_path_tree.set_xlabel("s(m)",  fontdict={'fontsize': 10})
    ax_path_tree.set_ylabel("d(m)",  fontdict={'fontsize': 10})

    ax_path_scene = plt.Subplot(f, gs00[0,0])
    f.add_subplot(ax_path_scene)
    ax_path_scene.set_title("Path Planning")
    ax_path_scene.set_xlabel("x(m)",  fontdict={'fontsize': 10})
    ax_path_scene.set_ylabel("y(m)",  fontdict={'fontsize': 10})

    gs01 = gridspec.GridSpecFromSubplotSpec(3, 4, subplot_spec=gs0[1:], hspace=0.5, wspace=0.5)

    ax_speed_scene = plt.Subplot(f, gs01[0, :])
    f.add_subplot(ax_speed_scene)
    ax_speed_scene.set_title("Speed Profile")
    ax_speed_scene.set_xlabel("x(m)",  fontdict={'fontsize': 10})
    ax_speed_scene.set_ylabel("y(m)",  fontdict={'fontsize': 10})

    ax_speed_tree = plt.Subplot(f, gs01[1:, :-2])
    f.add_subplot(ax_speed_tree)
    ax_speed_tree.set_title("RRT tree of speed profile", fontdict={'fontsize': 10})
    ax_speed_tree.set_xlabel("t(s)",  fontdict={'fontsize': 10})
    ax_speed_tree.set_ylabel("s(m)",  fontdict={'fontsize': 10})

    ax_speed_vel = plt.Subplot(f, gs01[1:, 2:])
    f.add_subplot(ax_speed_vel)
    ax_speed_vel.set_title("Speed profile result", fontdict={'fontsize': 10})
    ax_speed_vel.set_xlabel("t(s)",  fontdict={'fontsize': 10})
    ax_speed_vel.set_ylabel("v(m/s)",  fontdict={'fontsize': 10})

    return ax_path_scene, ax_path_tree, ax_speed_scene, ax_speed_tree, ax_speed_vel


def plot_path_tree(ax1, debug):
    tree = debug.tree
    for i in range(1, len(tree.nodes)):
        node1 = tree.nodes[i]
        node2 = tree.nodes[node1.parent_index]
        ax1.plot([node1.col, node2.col], [511-node1.row, 511-node2.row], 'b', linewidth=1)

    path = debug.path
    path_x = []
    path_y = []
    for node in path.nodes:
        path_x.append(node.col)
        path_y.append(511-node.row)
    ax1.plot(path_x, path_y, 'r', linewidth=2)

    spline_path = debug.spline_path
    path_x = []
    path_y = []
    for node in spline_path.nodes:
        path_x.append(node.col)
        path_y.append(511-node.row)
    ax1.plot(path_x, path_y, 'g', linewidth=1)

    obstacle_image_polygons = debug.obstacle_image_polygons
    print(len(debug.obstacle_image_polygons.obstacles))
    patches = []
    for obstacle in obstacle_image_polygons.obstacles:
        nv = len(obstacle.vertexes)
        v = np.zeros(shape=(nv, 2))
        for i in range(0, nv):
            v[i][0] = obstacle.vertexes[i].row
            v[i][1] = 511-obstacle.vertexes[i].col
        polygon = Polygon(v)
        patches.append(polygon)
    p = PatchCollection(patches)
    ax1.add_collection(p)

    ax1.set_xlim([0,512])
    ax1.set_ylim([0,512])
    ax1.set_aspect(1.0)


def plot_path_scene(ax2, debug):
    plot_roads(ax2)

    global_path = debug.global_path
    path_x = []
    path_y = []
    for point in global_path.points:
        path_x.append(point.x)
        path_y.append(point.y)
    ax2.plot(path_x, path_y, 'r', linewidth=2, alpha=0.5)

    patches = []
    color = []
    vehicle_state = debug.vehicle_state
    vehicle_state.theta = vehicle_state.theta
    polygon = get_vehicle_vertex(vehicle_state.x, vehicle_state.y, vehicle_state.theta)
    patches.append(polygon)
    color.append('w')
    print(vehicle_state.theta)
    for obstacle in debug.obstacles:
        polygon = get_vehicle_vertex(obstacle.x, obstacle.y, obstacle.theta)
        print(obstacle.theta)
        patches.append(polygon)
        color.append('g')

    p = PatchCollection(patches, facecolors=color, alpha=0.6)
    ax2.add_collection(p)

def plot_speed_scene(ax1, debug):
    plot_roads(ax1)
    polygons = []
    colors = []
    alphas = []
    vehicle_states = debug.prediction_veh.states
    for i in range(0, len(vehicle_states)):
        x = vehicle_states[i].x
        y = vehicle_states[i].y
        theta = math.pi / 2 - vehicle_states[i].theta
        polygons.append(get_vehicle_vertex(x, y, theta))
        colors.append('w')
        alphas.append(1 - 0.2 * i)

    obstacles = debug.prediction_obs
    for i in range(0, len(obstacles)):
        obs = obstacles[i]
        if obs.states[0].x == obs.states[1].x and obs.states[0].y == obs.states[1].y:
            x = obs.states[0].x
            y = obs.states[0].y
            theta = math.pi / 2 - obs.states[0].theta
            polygons.append(get_vehicle_vertex(x, y, theta))
            colors.append('g')
        else:
            for i in range(0, len(obs.states)):
                x = obs.states[i].x
                y = obs.states[i].y
                theta = math.pi / 2 - obs.states[i].theta
                polygons.append(get_vehicle_vertex(x, y, theta))
                colors.append('r')
                alphas.append(1 - 0.2 * i)

    p = PatchCollection(polygons, facecolors=colors)
    ax1.add_collection(p)

def plot_speed_tree(ax2, debug):
    t = []
    s = []
    for point in debug.distance_map.points:
        t.append(point.t)
        s.append(point.s)
    ax2.scatter(t, s, s=(1,1))

    tree = debug.tree
    for i in range(1, len(tree)):
        node1 = tree[i]
        node2 = tree[node1.parent_index]
        ax2.plot([node1.t, node2.t], [node1.s, node2.s], 'b', linewidth=1)

    ax2.plot([0,5], [0,12*5], 'g', linewidth=1)
    ax2.set_xlim([0,5])
    ax2.set_ylim([0,60])
    # ax2.set_aspect(1.0)

    path = debug.st_path
    t = []
    v = []
    s = []
    for node in path:
        t.append(node.t)
        v.append(node.v)
        s.append(node.s)
    ax2.plot(t, s, 'r', linewidth=1)


def plot_speed_vel(ax3, debug):
    path = debug.st_path
    t = []
    v = []
    s = []
    for node in path:
        t.append(node.t)
        v.append(node.v)
        s.append(node.s)
    ax3.plot(t, v, 'g', linewidth=1)
    ax3.set_xlim([0,5])
    ax3.set_ylim([0,12])


if __name__ == '__main__':
    last_vehicle_state = []

    for i in range(1,37):
        path_file_name = "/Users/zhuoyang/workspace/thesis/for_vis/path_"
        speed_file_name = "/Users/zhuoyang/workspace/thesis/for_vis/speed_"

        str_i = str(i)
        if i < 10:
            str_i = "0" + str_i
        path_file_name = path_file_name + str_i+ ".txt"
        speed_file_name = speed_file_name + str_i + ".txt"
        debug_path, debug_speed = parse_file(path_file_name, speed_file_name)

        plt.style.use('ggplot')

        ax_path_scene, ax_path_tree, ax_speed_scene, ax_speed_tree, ax_speed_vel = plots_allocation()
        plot_path_tree(ax_path_tree, debug_path)
        plot_path_scene(ax_path_scene, debug_path)

        plot_speed_scene(ax_speed_scene, debug_speed)
        plot_speed_tree(ax_speed_tree, debug_speed)
        plot_speed_vel(ax_speed_vel, debug_speed)

        plt.savefig("pic/" + str_i + ".png")
        # plt.show()
        plt.clf()
