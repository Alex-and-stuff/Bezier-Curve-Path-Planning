import math
from typing import no_type_check
import matplotlib.pyplot as plt
import csv


# Define class to store position data (x, y, theta)
class Position:
    def __init__(self, x, y, theta, v = 0, w = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.w = w

# Set up variables
dt = 0.1
K = 0.3#(2+1/pow(2,0.5)-0.5)/8#0.3
T_RESOLUTION = 100
mapl = 1.5
radius = 0.1
MAX_V, MAX_W = 0.3, 0.9

def correct_angle(angle):
    while angle <= -math.pi:
        angle += math.pi*2
    while angle > math.pi:
        angle -= math.pi*2
    return angle

def predict(current, p_T):
    next_T     = round(p_T + dt,3)
    next_theta = current.theta + current.w*dt
    next_x     = round(current.x + (current.v*dt) * math.cos(next_theta),5)
    next_y     = round(current.y + (current.v*dt) * math.sin(next_theta),5)
    next_theta = round(correct_angle(next_theta),5)
    next_point = Position(next_x, next_y, next_theta)
    return next_point, next_T

def generate_path(start, goal, middle = Position(0,0,-1,0)):
    Ki, Kf = K, K
    T = T_RESOLUTION
    path = []
    x, y = start.x, start.y
    total_length, total_vs, s = 0, 0, 0
    path_data = [[start.x, start.y, start.theta, 0, 0, 0, start.v]] #[x,y,theta,length,,average_vs,speed]

    ax = Ki * math.cos(start.theta) + start.x
    ay = Ki * math.sin(start.theta) + start.y
    bx = -Kf * math.cos(goal.theta) + goal.x
    by = -Kf * math.sin(goal.theta) + goal.y
    if middle.theta == -1:
        # no designated middle point
        xm = (ax + bx)/2
        ym = (ay + by)/2
    else:
        # with designated middle point
        xm = middle.x
        ym = middle.y
    # put in starting points to path
    path.append([start.x, start.y, start.theta])

    for i in range(T):
        s_ = s # s of former timestep
        s  = (i + 1)/T
        x_ = x # x of former timestep
        y_ = y # y of former timestep
        x = pow(1-s,4)*start.x + 4*s*pow(1-s,3)*ax + 6*pow(s,2)*pow(1-s,2)*xm + 4*pow(s,3)*(1-s)*bx + pow(s,4)*goal.x
        y = pow(1-s,4)*start.y + 4*s*pow(1-s,3)*ay + 6*pow(s,2)*pow(1-s,2)*ym + 4*pow(s,3)*(1-s)*by + pow(s,4)*goal.y
        theta = math.atan2((y-y_)/(s-s_),(x-x_)/(s-s_))
        path.append([x, y, theta])
        total_length += math.sqrt(pow((x-x_),2)+pow((y-y_),2)) # accumulating the length at every timestep

        dx = 4*start.x*pow(s-1,3) - 4*pow(s,3)*bx - 4*ax*pow(s-1,3) + 4*pow(s,3)*goal.x - 12*pow(s,2)*bx*(s-1) - 12*s*ax*pow(s-1,2) + 12*s*xm*pow(s-1,2) + 6*pow(s,2)*xm*(2*s-2)
        dy = 4*start.y*pow(s-1,3) - 4*pow(s,3)*by - 4*ay*pow(s-1,3) + 4*pow(s,3)*goal.y - 12*pow(s,2)*by*(s-1) - 12*s*ay*pow(s-1,2) + 12*s*ym*pow(s-1,2) + 6*pow(s,2)*ym*(2*s-2)
        dv = math.sqrt(pow((dx),2)+pow((dy),2))
        total_vs += dv # accumulating the speed at every timestep

    # calculate average speed "in s domain?"
    average_vs = total_vs / T
    path_data.append([x,y,theta,total_length,0,average_vs,goal.v])

    # print(path)

    # Plot path
    # plt.figure(figsize=(6, 6))#600x600
    # # plt.xlim((-mapl,mapl))
    # # plt.ylim((-mapl,mapl))
    # plt.xlabel('x(m)')
    # plt.ylabel('y(m)')
    # plt.title("just check path plan via cartesian polynomial, no practical use [1]")
    # color = 'blue'
    # for i in range(len(path)):
    #     if i != 0:
    #         plt.plot([path[i-1][0],path[i][0]],
    #                   [path[i-1][1],path[i][1]], color,'-')
    #     if i % 1 == 0 or i == len(path)-1:
    #         path_x = path[i][0]
    #         path_y = path[i][1]
    #         path_theta = path[i][2]
    #         plt.scatter(path_x, path_y, c = color)
    #         plt.arrow(path_x, path_y, radius*math.cos(path_theta),radius*math.sin(path_theta),color=[1,0.5,0.5,0.5])
   
    mid_point = Position(xm, ym, 0)

    return path_data, mid_point

def process_data_speed(path_data, vi, vf): # v_initial, v_final
    path_data_w_speed = []
    total_length = path_data[1][3]
    slope = ()

def generate_path_subpoint(path_data, middle_point):
    f = open('log_dataqq.csv', 'w')
    writer = csv.writer(f)

    ITERATING_INCREMENTAL = 0.0001
    # generate the middle subpoints between the start and goal
    track = []
    p_point = Position(path_data[0][0], path_data[0][1], correct_angle(path_data[0][2]), path_data[0][6])
    # Tracking Gain Obtained From LMI
    k1 = -1.0154 
    k2 = -1.0914
    k3 = -0.5746
    start_x, start_y, start_theta, start_v, goal_x, goal_y, goal_theta, goal_v = 0,0,0,0,0,0,0,0
    iterating_p_x, iterating_p_y, iterating_p_x_, iterating_p_y_ = 0,0,0,0
    command_path = []
    TpT = []
    apT = 0
    

    for i in range(len(path_data) - 1):
        print(i)
        
        start_x, start_y, start_theta, start_v = path_data[i][0], path_data[i][1], path_data[i][2], path_data[i][6]
        goal_x, goal_y, goal_theta, goal_v     = path_data[i+1][0], path_data[i+1][1], path_data[i+1][2], path_data[i+1][6]
        # start = Position(path_data[i][0], path_data[i][1], path_data[i][2], path_data[i][6])
        # goal  = Position(path_data[i+1][0], path_data[i+1][1], path_data[i+1][2], path_data[i+1][6])
        average_vs  = path_data[i+1][5] # stored at the 2nd [] of path data
        v_initial   = path_data[i][6]
        v_final     = path_data[i+1][6]
        length      = path_data[i+1][3]
        T           = 2*length/(v_initial + v_final)
        v_initial_s = v_initial/length # in s domain?
        v_final_s   = v_final/length
        p, p_integral, p_T = 0, 0, 0 # a searching point to iterate for a designated length
        # if i != 0:
        #     p_T = dt
        xd, yd, theta_d = start_x, start_y, start_theta
        s_step, s_step_ = 0, 0
        xdd, ydd =0,0

        while p_T <= T: # meaning that the search time should be smaller than the appointed moving time
            # track.append([p_point.x, p_point.y, p_point.theta])
            track.append(p_point)
            xd_ = xd
            yd_ = yd
            displacement_s   = v_initial_s*p_T + (v_final_s - v_initial_s)/(2*T)*pow(p_T, 2) # v0*t + 0.5*a*t^2
            # print('dis: ', displacement_s)
            Ki, Kf = K, K
            xm = middle_point[i].x
            ym = middle_point[i].y
            ax = Ki * math.cos(start_theta) + start_x
            ay = Ki * math.sin(start_theta) + start_y
            bx = -Kf * math.cos(goal_theta) + goal_x
            by = -Kf * math.sin(goal_theta) + goal_y

            s_step_ = s_step      # record the former value of s step
            s_step = displacement_s # incrementing s means incrementing through displacement
            stuff = ['->']
            count = 0 

            # print(p_integral, displacement_s*length)
            while p_integral < displacement_s * length: #????????????????????
                # former iterating point
                iterating_p_x_ = pow(1-p,4)*start_x + 4*p*pow(1-p,3)*ax + 6*pow(p,2)*pow(1-p,2)*xm + 4*pow(p,3)*(1-p)*bx + pow(p,4)*goal_x
                iterating_p_y_ = pow(1-p,4)*start_y + 4*p*pow(1-p,3)*ay + 6*pow(p,2)*pow(1-p,2)*ym + 4*pow(p,3)*(1-p)*by + pow(p,4)*goal_y
                p += ITERATING_INCREMENTAL
                # new iterating point
                iterating_p_x = pow(1-p,4)*start_x + 4*p*pow(1-p,3)*ax + 6*pow(p,2)*pow(1-p,2)*xm + 4*pow(p,3)*(1-p)*bx + pow(p,4)*goal_x
                iterating_p_y = pow(1-p,4)*start_y + 4*p*pow(1-p,3)*ay + 6*pow(p,2)*pow(1-p,2)*ym + 4*pow(p,3)*(1-p)*by + pow(p,4)*goal_y
                p_integral += math.sqrt(pow(iterating_p_x - iterating_p_x_,2)+pow(iterating_p_y - iterating_p_y_,2))
                stuff.append([p_integral, displacement_s*length])
                count += 1
                
                # print(p_integral, displacement_s*length)
            writer.writerow(stuff)
            print('====',round(displacement_s*length - p_integral, 5), '||', p_integral, displacement_s*length, count)
            # out of while loop. meaning that p_integrat == s * length (New command point is found)
            s = p # record the found point as s

            


            xd = pow(1-s,4)*start_x + 4*s*pow(1-s,3)*ax + 6*pow(s,2)*pow(1-s,2)*xm + 4*pow(s,3)*(1-s)*bx + pow(s,4)*goal_x
            yd = pow(1-s,4)*start_y + 4*s*pow(1-s,3)*ay + 6*pow(s,2)*pow(1-s,2)*ym + 4*pow(s,3)*(1-s)*by + pow(s,4)*goal_y
            print('====>', pow(xd- xdd,2)+ pow(yd-ydd,2))
            xdd = xd
            ydd = yd
            if p_T != 0:
                theta_d = round(math.atan2((yd-yd_)/(s_step-s_step_),(xd-xd_)/(s_step-s_step_)),5)

            s = s_step
            # s = p seems to both work?
            dx =4*start_x*pow(s-1,3) - 4*pow(s,3)*bx - 4*ax*pow(s-1,3) + 4*pow(s,3)*goal_x - 12*pow(s,2)*bx*(s-1) - 12*s*ax*pow(s-1,2) + 12*s*xm*pow(s-1,2) + 6*pow(s,2)*xm*(2*s-2)
            ddx=12*start_x*pow(s-1,2) + 12*xm*pow(s-1,2) - 24*bx*pow(s,2) + 12*pow(s,2)*goal_x + 12*pow(s,2)*xm - 24*ax*pow(s-1,2) - 24*bx*s*(s-1) - 12*ax*s*(2*s-2) + 24*s*xm*(2*s-2)
            dy =4*start_y*pow(s-1,3) - 4*pow(s,3)*by - 4*ay*pow(s-1,3) + 4*pow(s,3)*goal_y - 12*pow(s,2)*by*(s-1) - 12*s*ay*pow(s-1,2) + 12*s*ym*pow(s-1,2) + 6*pow(s,2)*ym*(2*s-2)
            ddy=12*start_y*pow(s-1,2) + 12*ym*pow(s-1,2) - 24*by*pow(s,2) + 12*pow(s,2)*goal_y + 12*pow(s,2)*ym - 24*ay*pow(s-1,2) - 24*by*s*(s-1) - 12*ay*s*(2*s-2) + 24*s*ym*(2*s-2)
            ds = v_initial_s + (v_final_s-v_initial_s)*p_T/T
            
            ws = (ddy*dx-ddx*dy)/(pow(dx,2)+pow(dy,2))
            vs = average_vs
            
            vd = round(vs*ds,4) # in real scale
            wd = round(ws*ds,4)
            command_path.append(Position(xd,yd,theta_d,vd,wd))

            if abs(xd - p_point.x) > 0.015 or abs(yd - p_point.y) > 0.015:
                 p_T += dt
            else:
                #=======================================================
                #===================Tracking Control====================
                #=======================================================
                e1 = round(math.cos(p_point.theta)*(xd - p_point.x)+math.sin(p_point.theta)*(yd - p_point.y),5)
                e2 = round(-math.sin(p_point.theta)*(xd - p_point.x)+math.cos(p_point.theta)*(yd - p_point.y),5)
                e3 = round(correct_angle(theta_d - p_point.theta),5) # important to add "correct angle(){}"
                
                u1 = k1 * e1
                u2 = k2 * e2 + k3 * e3
                v = vd * math.cos(e3) - u1
                w = wd - u2
                print(apT, v, '||', e1, e2, e3, '||',xd, p_point.x, yd, p_point.y, '||', p_point.theta)
                # print(apT, v, e1, e2, e3, vd, wd)
                # print(apT, xd,yd)
                # set limits to command inputs
            v = max(min(v,MAX_V),-MAX_V) 
            w = max(min(w,MAX_W),-MAX_W)
            p_point.v = v
            p_point.w = w
            p_point, p_T = predict(p_point, p_T)
            
            apT += dt### for debug
            TpT += [apT]### for debug
    
    # track.append([p_point.x, p_point.y, p_point.theta])
    track.append(p_point)
    # plot figures
    plt.figure(figsize=(6, 6))#600x600
    plt.xlim((-mapl,mapl))
    plt.ylim((-mapl,mapl))
    plt.xlabel('x(m)')
    plt.ylabel('y(m)')
    plt.title("Check xd yd thd")
    color = 'blue'
    for w in range(len(command_path)):
        if w != 0:
            plt.plot([command_path[w-1].x,command_path[w].x],
                     [command_path[w-1].y,command_path[w].y], color,'-')
        if w % 1 == 0 or w == len(command_path)-1:
            arrow_x = command_path[w].x
            arrow_y = command_path[w].y
            arrow_theta = command_path[w].theta
            plt.scatter(arrow_x, arrow_y, c = color)
            plt.arrow(arrow_x, arrow_y, radius*math.cos(arrow_theta),radius*math.sin(arrow_theta),color=[1,0.5,0.5,0.5])

    plt.figure(figsize=(6, 6))#600x600
    plt.xlabel('t(s)')
    plt.ylabel('v(m/s)')
    plt.title("A")
    for i in range(len(track)-1):
        #plt.scatter(TpT[iii], Tv[iii],c = 'b')
        plt.scatter(TpT[i], track[i].v, c = 'b')
        
    plt.figure(figsize=(6, 6))#600x600
    plt.xlabel('t(s)')
    plt.ylabel('w(m/s)')
    plt.title("B")
    for i in range(len(track)-1):
        #plt.scatter(TpT[iii], Tv[iii],c = 'b')
        plt.scatter(TpT[i], track[i].w, c = 'b')
        
    plt.figure(figsize=(6, 6))#600x600
    plt.xlabel('t(s)')
    plt.ylabel('vd(m/s)')
    plt.title("C")
    for i in range(len(track)-1):
        #plt.scatter(TpT[iii], Tv[iii],c = 'b')
        plt.scatter(TpT[i], command_path[i].v, c = 'b')

    plt.figure(figsize=(6, 6))#600x600
    plt.xlabel('t(s)')
    plt.ylabel('wd(rad/s)')
    plt.title("D")
    for i in range(len(track)-1):
        #plt.scatter(TpT[iii], Tv[iii],c = 'b')
        plt.scatter(TpT[i], command_path[i].w, c = 'b')


    plt.figure(figsize=(6, 6))#600x600
    plt.xlabel('t(s)')
    plt.ylabel('v(m/s)')
    plt.title("E")
    for i in range(len(track)-1):
        if i != 0:
            plt.plot([TpT[i-1],TpT[i]],[command_path[i-1].v,command_path[i].v], 'b','-')
    for i in range(len(track)-1):
        if i != 0:
            plt.plot([TpT[i-1],TpT[i]],[track[i-1].v,track[i].v], 'r','-')
    plt.text(7, 0.35, "vd(m/s)", bbox=dict(color='b', alpha=0.3))
    plt.text(7, 0.32, "v(m/s)", bbox=dict(color='r', alpha=0.3))

    return track
             

def generate_mid_point(former_midpoint, new_starting_point):
    new_mx = former_midpoint.x + 2*math.cos(new_starting_point.theta)*(K+K)
    new_my = former_midpoint.y + 2*math.sin(new_starting_point.theta)*(K+K)
    new_midpoint = Position(new_mx, new_my, 0) # theta unimportant

    return new_midpoint

def plot_track(route, path_points, midpoint_list):
    arrow_length, robot_radius = radius, radius
    plt.figure(figsize=(6, 10))#600x600
    # plt.xlim((-mapl,mapl))
    # plt.ylim((-mapl,mapl))
    plt.xlim((-1,2))
    plt.ylim((0,5))
    plt.xlabel('x(m)')
    plt.ylabel('y(m)')
    color = 'blue'
    # draw the smooth blue path and label the seleceted points by number
    for w in range(len(route)):
        if w != 0:
            plt.plot([route[w-1].x,route[w].x],[route[w-1].y,route[w].y], color,'-') 
        if w*dt % 1 == 0 or w == len(route)-1: #since dt is 0.1, a circle is drawn every 10 steps
            track_x     = route[w].x
            track_y     = route[w].y
            track_theta = route[w].theta
            plt.scatter(track_x, track_y, c = color)
            plt.arrow(track_x, track_y, arrow_length*math.cos(track_theta),arrow_length*math.sin(track_theta), color = 'pink', linewidth = 2)
            draw_circle = plt.Circle((track_x, track_y),robot_radius, color = color, fill=False)
            plt.text(track_x-0.05, track_y, str(w), bbox=dict(color=color, alpha=0.3))
            plt.gcf().gca().add_artist(draw_circle)
            


   
    
    # plot the calculated bezier curves in red (Reference)
    # ========= loop ============
    # 1. iterate through the path points to build start and goal
    # 2. get the midpoints from midpoint_list
    # 3. generate bezier curve 
    # 4. iterate through every timestep
    # 5. add to path_trace[]
    # ===========================
    # 6. draw out path on plot

    T = 10
    bezier_path = []
    for i in range(len(path_points) - 1):
        start = path_points[i]
        goal  = path_points[i+1]
        midpoint = midpoint_list[i]
        Ki, Kf = K, K
        ax = Ki * math.cos(start.theta) + start.x
        ay = Ki * math.sin(start.theta) + start.y
        bx = -Kf * math.cos(goal.theta) + goal.x
        by = -Kf * math.sin(goal.theta) + goal.y
        x, y, theta, s  = 0, 0, 0, 0
        bezier_path.append(start)
        for j in range(T):
            s_ = s
            s = (j + 1)/T
            x_ = x
            y_ = y
            x = pow(1-s,4)*start.x + 4*s*pow(1-s,3)*ax + 6*pow(s,2)*pow(1-s,2)*midpoint.x + 4*pow(s,3)*(1-s)*bx + pow(s,4)*goal.x
            y = pow(1-s,4)*start.y + 4*s*pow(1-s,3)*ay + 6*pow(s,2)*pow(1-s,2)*midpoint.y + 4*pow(s,3)*(1-s)*by + pow(s,4)*goal.y
            theta = math.atan2((y-y_)/(s-s_),(x-x_)/(s-s_))
            bezier_path.append(Position(x,y,theta))
    for i in range(len(bezier_path)):
        if i != 0:
            plt.plot([bezier_path[i-1].x,bezier_path[i].x],
                      [bezier_path[i-1].y,bezier_path[i].y], 'red','-')

     # plot starting point in green
    start_point = path_points[0]
    plt.scatter(start_point.x, start_point.y, s = 300, c = 'g')
    plt.arrow(start_point.x,start_point.y,
              arrow_length*math.cos(start_point.theta),
              arrow_length*math.sin(start_point.theta))
    # plot other goal points in red
    for i in range(len(path_points) - 1):
        goal_point = path_points[i+1]
        plt.scatter(goal_point.x, goal_point.y, s = 300, c = 'r')
        plt.arrow(goal_point.x,goal_point.y,
              arrow_length*math.cos(goal_point.theta),
              arrow_length*math.sin(goal_point.theta))
    # plot the midpoints in 
    for i in range(len(midpoint_list)):
        plt.scatter(midpoint_list[i].x, midpoint_list[i].y, s = 500, c = 'purple')
        print('midpoint:', midpoint_list[i].x, midpoint_list[i].y)
    


if __name__ == '__main__':
    
    # === set list of designated points ===
    # 1. create first path 
    # 2. designate the first two points 
    # 3. generate path
    # ====== loop =============
    # 4. give next point
    # 5. calculate new middle point 
    # 6. generate path
    # 7. append to former path
    # ====== end loop =========
    # 8. generate path subpoint

    final_track = []
    midpoint_list = []
    # point_ = Position(-1,1,math.radians(0),0.3)
    # point_a = Position(0,1,math.radians(0),0.3)
    # point_b = Position(1,1,math.radians(0),0.2)
    # point_c = Position(1.84,1.42,math.radians(45),0.2)
    # point_d = Position(2.27,2.27,math.radians(90),0.2)
    # point_e = Position(2.27,3.27,math.radians(90),0.2)
    # path_points = [point_, point_a, point_b, point_c, point_d, point_e]

    # ======== Relatively GOOD =========
    point_ = Position(-1,1,math.radians(0),0.3)#
    point_a = Position(0,1,math.radians(0),0.3)#
    point_b = Position(1,1,math.radians(0),0.2)
    point_c = Position(1.7,1.7,math.radians(90),0.2)
    point_d = Position(1.7,2.7,math.radians(90),0.2)
    point_e = Position(1.7,3.7,math.radians(90),0.2)
    point_f = Position(1.0,4.6,math.radians(180),0.2)
    point_g = Position(0 ,4.6,math.radians(180),0.2)
    point_h = Position(-0.7 ,3.7,math.radians(270),0.2)
    point_i = Position(-0.7 ,2.7,math.radians(270),0.2)
    point_j = Position(-0.7 ,1.7,math.radians(270),0.2)
    point_k = Position(0,1,math.radians(0),0.2)

    path_points = [point_a, point_b, point_c, point_d, point_e, point_f, point_g]
    # path_points = [point_a, point_b, point_c, point_d, point_e, point_f, point_g,
    #                point_h, point_i, point_j, point_k]

    path_points = [point_a, point_b, point_c, point_d, point_e]
    
    
    
    # point_a = Position(0,1,math.radians(0),0.2)#
    # point_b = Position(1,1,math.radians(0),0.2)
    # point_c = Position(1.7,1.7,math.radians(90),0.2)
    # point_d = Position(1.7,2.7,math.radians(90),0.2)
    # point_e = Position(1.7,3.7,math.radians(90),0.2)
    # path_points = [point_a, point_b, point_c, point_d, point_e]

    # point_  = Position(0,0,math.radians(0),0.3)#
    # point_a = Position(1,0,math.radians(0),0.2)#
    # point_b = Position(2,0,math.radians(0),0.1)
    # point_c = Position(3,0,math.radians(0),0.2)
    # point_d = Position(4,0,math.radians(0),0.2)
    # point_e = Position(5.5,0,math.radians(0),0.2)
    # point_f = Position(6,0,math.radians(0),0.2)
    # point_g = Position(6.5,0,math.radians(0),0.2)

    # point_ = Position(-1,1,math.radians(0),0.3)
    # point_a = Position(0,1,math.radians(0),0.3)
    # point_b = Position(1,1,math.radians(0),0.2)
    # point_c = Position(1.6366,1.6366,math.radians(90),0.2)
    # point_d = Position(1.6366,2.6366,math.radians(90),0.2)
    # point_e = Position(1.6366,3.6366,math.radians(90),0.2)
    # path_points = [point_, point_a, point_b, point_c, point_d, point_e, point_f, point_g, point_h, point_i]
    # path_points = [point_, point_a, point_b, point_c, point_d, point_e, point_f, point_g]

    # path_points = [point_a, point_b, point_c, point_d, point_e]
    # path_points = [point_d, point_e, point_f, point_g]
    # path_points = [point_c, point_d, point_e]
    # path_points = [point_, point_a, point_b, point_c]

    # initial_data, initial_midpoint = generate_path(path_points[0], path_points[1])
    # midpoint_list.append(initial_midpoint)
    # former_mid_point = initial_midpoint
    # final_track+=initial_data
    # print(final_track)
    # for i in range(len(path_points) - 2):
    #     next_start_point = path_points[i + 1]
    #     next_goal_point  = path_points[i + 2]
    #     next_mid_point   = generate_mid_point(former_mid_point, next_start_point)
    #     new_data, new_midpoint = generate_path(next_start_point, next_goal_point, next_mid_point)
    #     final_track.append(new_data[1])
    #     midpoint_list.append(next_mid_point)
    #     former_mid_point = next_mid_point
    #     print(len(midpoint_list))
    #     print(final_track)
    # track_result = generate_path_subpoint(final_track, midpoint_list)

    for i in range(len(path_points)-1):
        new_start_point = path_points[i]
        new_goal_point  = path_points[i+1]
        if i == 0: # starting path where the middld point doesn't need to be calculated
            new_data, new_mid_point = generate_path(new_start_point, new_goal_point)
            final_track += new_data
        else:      # middle point needs to be caculated in order to keep continuous 
            # if i == 2:
            #     K = 0.35
            # if i == 3 or i == 3:
            #     K = 0.15
            new_mid_point = generate_mid_point(former_mid_point, new_start_point)
            new_data, new_mid_point = generate_path(new_start_point, new_goal_point, new_mid_point)
            final_track.append(new_data[1])
        midpoint_list.append(new_mid_point)
        former_mid_point = new_mid_point
    track_result = generate_path_subpoint(final_track, midpoint_list)
    plot_track(track_result, path_points, midpoint_list)

    # print(data)
    plt.show()
