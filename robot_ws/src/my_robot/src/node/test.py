#!/usr/bin/env python
import cv2 
import numpy as np 



def  draw_circle(img, vitri, color, size):
    return cv2.circle(img,vitri, size, color, -1)

def  prep_img(xuatphat, dich):
    img = cv2.imread("/home/tran/github/bag_file/map.pgm")
    img = cv2.resize(img, (1500,1500))
    img = img[600:900, 600:900]
    print(img.shape)
    (w,h) = (img.shape[0], img.shape[1])
    for row in range(w):
        for column in range(h):
            if np.sum(img[row,column]) != 254 *3 :
                img[row,column] = [0,0,0]
    return img

def mov_around(map, node):
    mov = []
    index_wall =[]
    for index in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1,  -1), (-1, 1), ( -1 , - 1)):
        mov.append((node[0] + index[0], node[1] + index[1]))
    for index,m in enumerate(mov):
        if sum(map[m[1], m[0]] == [0, 0 , 0]) > 0:
           index_wall.append(index) 
    return [slices for slices in mov if mov.index(slices) not in index_wall]

def tinh_f(xuatphat, node, dich):
    g = abs(node[0] - xuatphat[0]) + abs(node[1] - xuatphat[1])
    h = (node[0] - dich[0])**2 + (node[1] - dich[1]) ** 2
    f = g +h
    return g, h, f
def a_star(xuatphat, dich):
    img = prep_img(xuatphat, dich)
    opened_list =[]
    closed_list =[]
    opened_list.append(xuatphat)
    current_node = opened_list[0]
    current_index = 0
    i = 0
    while len(opened_list) > 0:
        i +=1
        draw_circle(img,current_node,(0,100,255), 0)
        if current_node == dich :
            print("u done")
            break
        
        opened_list.pop(current_index)
        closed_list.append(current_node)

        if len(closed_list) > 5:
            if closed_list[-1] == closed_list[-3]:
                current_node = opened_list[0]
        mov_list = mov_around(img, current_node)
        list_fchild = []
        list_g = []
        for child in mov_list:
            for child_closed in closed_list:
                if child == child_closed:
                    continue
            g_child, h_child, f_child = tinh_f(current_node, child, dich)
            list_fchild.append(f_child)
            list_g.append(g_child)
            for child_open in opened_list:
                if child == child_open:
                    continue

            opened_list.append(child)

        index = [i for i,j  in enumerate(list_fchild) if j == min(list_fchild)]
        current_node = mov_list[index[0]]
        if i % 10000 == 0:
            cv2.imshow("name", img)
            cv2.waitKey()
            cv2.destroyAllWindows()
        

    draw_circle(img,xuatphat,(0,100,255), 0)
    draw_circle(img,dich,(0,100,255), 0)

    for node in closed_list:
        draw_circle(img, node,(100, 100 ,1), 0)
    
    cv2.imshow("name", img)
    cv2.waitKey()
    cv2.destroyAllWindows()

    

if __name__ == "__main__":
    start = (68, 183)
    end = (202, 192)
    
    a_star(start, end)