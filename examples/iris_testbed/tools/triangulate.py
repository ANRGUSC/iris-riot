import math
import numpy as np
import itertools as it

def tdoa_to_dist(tdoa):
    return ((tdoa - 19628.977) / 885.274) * 2.54 * 12; #this equation can be modified

def get_mid_dist(d1, d2, sep):
    x = (d1**2 / 2) + (d2**2 / 2) - (sep**2 / 4)
    if(x >= 0):
        return math.sqrt(x)
    else:
        return -1

def get_angle(d1, d2, sep):
    x = get_mid_dist(d1,d2, sep);
    
    if x == -1:
        return -1
    elif x == 0:
        return 0

    ratio = (d2**2 - d1**2) / (2 * sep * x)
    
    if ratio > 1 or ratio < -1:
        return 361
    else:
        return math.asin(ratio)

def triangulate(node_data, a_nodelist):
    anchor_coor = [None]*10
    anchor_id = [None]*10
    num_nodes = 0
    estimates = []

    for node_id in sorted(node_data.keys()):
        if str(node_id) in a_nodelist.keys():
            # print("added")
            anchor_id[num_nodes] = node_id
            anchor_coor[num_nodes] = a_nodelist[str(node_id)]
        num_nodes = num_nodes + 1

    print(anchor_id)

    permutelist = list(it.permutations(range(num_nodes), r=3))
    print(permutelist)
    i = 0
    for item in permutelist:
        i=i+1
        print(item)
        a = anchor_coor[item[0]]
        b = anchor_coor[item[1]]
        c = anchor_coor[item[2]]

        d1 = tdoa_to_dist(node_data[anchor_id[item[0]]])
        d2 = tdoa_to_dist(node_data[anchor_id[item[1]]])
        d3 = tdoa_to_dist(node_data[anchor_id[item[2]]])
        # d1 = node_data[anchor_id[item[0]]]
        # d2 = node_data[anchor_id[item[1]]]
        # d3 = node_data[anchor_id[item[2]]]

        ab_vec = np.subtract(a,b)
        print(ab_vec)
        ab_mag = np.linalg.norm(ab_vec)

        print('d1: ' + str(d1))
        print('d2: ' + str(d2))
        print('d3: ' + str(d3))
        print('ab_mag: ' + str(ab_mag))
        if((d1 + d2) < ab_mag):
            continue


        ab_mid = np.add(np.divide(ab_vec, 2), b)

        node_mag = get_mid_dist(d1, d2, ab_mag)
        node_ang = 0

        ab_ang = math.atan(ab_vec[1]/ab_vec[0])

        if ab_vec[0] < 0 and ab_vec[1] < 0:
            ab_ang += math.pi
        if ab_vec[0] < 0 and ab_vec[1] >= 0:
            ab_ang += math.pi
        
        node_ang = get_angle(d1, d2, ab_mag)
            

        node_ang1 = ab_ang + math.pi/2 - node_ang
        node_ang2 = ab_ang - math.pi/2 + node_ang

        node_vec1 = [node_mag * math.cos(node_ang1), node_mag * math.sin(node_ang1)]
        node_vec2 = [node_mag * math.cos(node_ang2), node_mag * math.sin(node_ang2)]

        node_vec1 = np.add(node_vec1, ab_mid)
        node_vec2 = np.add(node_vec2, ab_mid)

        print(str(i)+"***********************")
        print("AB_ang: "+str(math.degrees(ab_ang)))
        print("node_ang: " +str(math.degrees(node_ang)))
        # print(math.degrees(node_ang))
        print(math.degrees(node_ang1))
        print(math.degrees(node_ang2))
        print("AB_mid: "+str(ab_mid))
        print("C: "+str(c))
        print(node_vec1)
        print(node_vec2)
        

        node_dist1 = abs(np.linalg.norm(np.subtract(c, node_vec1)) - d3)
        node_dist2 = abs(np.linalg.norm(np.subtract(c, node_vec2)) - d3)

        if(node_dist1 < node_dist2):
            estimates.append(node_vec1)
            print("node1 selected")
        else:
            estimates.append(node_vec2)
            print("node2 selected")

        print("***********************")

    return estimates