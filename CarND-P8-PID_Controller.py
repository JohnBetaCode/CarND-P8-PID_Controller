#!/usr/bin/env python
# =============================================================================
"""
Code Information:
    Date: 04/08/2019
	Programmer: John A. Betancourt G.
	Mail: john.betancourt93@gmail.com
    Web: www.linkedin.com/in/jhon-alberto-betancourt-gonzalez-345557129

Description: Project 8 - Udacity - self driving cars Nanodegree
    PID Controller

Tested on: 
    python 3.5
    UBUNTU 16.04
"""

# =============================================================================
# LIBRARIES AND DEPENDENCIES - LIBRARIES AND DEPENDENCIES - LIBRARIES AND DEPEN
# =============================================================================
import os

from multiprocessing import Pool 

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import subprocess
import csv

# =============================================================================
def run_process(process):                                                             
    os.system('{}'.format(process)) 

def getClipboardData():
    p = subprocess.Popen(['xclip','-selection', 'clipboard', '-o'], 
        stdout=subprocess.PIPE)
    retcode = p.wait()
    data = p.stdout.read()
    return data

def setClipboardData(data):
    p = subprocess.Popen(['xclip','-selection','clipboard'], 
        stdin=subprocess.PIPE)
    p.stdin.write(data)
    p.stdin.close()
    retcode = p.wait()

def main():

    # Copy command to clipboard to Create Video from desktop
    video_name = "PID_controller_test{}.mp4".format(1)
    command = "ffmpeg -video_size {}x{} -framerate {} -f x11grab -i :0.0+100,200 {}".format(
        1000, 650, 30, os.path.join(os.getcwd(), "video_results", video_name))
    setClipboardData(command.encode())

    # Run subprocess
    try:

        if not os.path.isdir("build"):
            os.mkdir("build")
            os.system('cd build && cmake .. && make') 
        else:
            os.system('clear && cd build && make') 

        processes = (
            "{}".format(os.path.join(os.getcwd(), "term2_sim_linux", "term2_sim.x86_64")),
            "termdown {} && {}".format(0, os.path.join(os.getcwd(), "build", "pid"))
        )
        # Run simulator and socket
        pool = Pool(processes=len(processes))                                                        
        pool.map(run_process, processes)

    except Exception as e: 
        print(str(e))

    print("Process has finished")

def plot_controller_graph(steering_file_path, speed_file_path, save_file=None):

    # -------------------------------------------------------------------------
    # Variables inizialitation
    json_steer_angle = []
    json_steer_cte = []
    steer_value = [] # OK
    steer_p_error = [] # OK
    steer_i_error = [] # OK
    steer_d_error = [] # OK
    
    # Read csv file for steering pid controller
    with open(steering_file_path) as csvfile:
        reader = csv.reader(csvfile)
        for idx, line in enumerate(reader):
            if idx:
                json_steer_angle.append(float(line[0])) # json_angle 
                json_steer_cte.append(float(line[1]))   # json_cte
                steer_value.append(float(line[2]))      # steer_pid
                steer_p_error.append(float(line[3]))    # steer_p_error
                steer_i_error.append(float(line[4]))    # steer_i_error
                steer_d_error.append(float(line[5]))    # steer_d_error
    
    # -------------------------------------------------------------------------
    # Variables inizialitation
    json_speed = [] # OK
    speed_ref = []  # OK
    speed_throttle = [] # OK
    speed_p_error = [] # OK
    speed_i_error = [] # OK
    speed_d_error = [] # OK
    
    # Read csv file for speed pid controller
    with open(speed_file_path) as csvfile:
        reader = csv.reader(csvfile)
        for idx, line in enumerate(reader):
            if idx:
                json_speed.append(float(line[0]))       # json_speed 
                speed_ref.append(float(line[1]))        # speed_ref
                speed_throttle.append(float(line[2]))   # speed_pid
                speed_p_error.append(float(line[3]))    # speed_p_error
                speed_i_error.append(float(line[4]))    # speed_i_error
                speed_d_error.append(float(line[5]))    # speed_d_error
    speed_error = [round(ref - speed, 2) for ref, speed in zip(speed_ref, json_speed)]

    # -------------------------------------------------------------------------
    plt.figure(figsize=(30,25))

    plt.subplot(3, 2, 1)
    plt.plot(speed_ref, '#0000FF')
    plt.plot(json_speed, '#008000')
    plt.plot(speed_error, 'r')
    speed_ref_patch = mpatches.Patch(color='#0000FF', label='speed_reference')
    json_speed_patch = mpatches.Patch(color='#008000', label='cars_speed') 
    speed_error_patch = mpatches.Patch(color='r', label='speed_error')    
    plt.legend(handles=[speed_ref_patch, json_speed_patch, speed_error_patch])
    plt.title('Cars Speed')
    plt.xlabel('Iteration (t)')
    plt.ylabel('Speed')
    plt.grid(True)

    plt.subplot(3, 2, 3)
    plt.plot(speed_p_error, '#0000FF')
    plt.plot(speed_i_error, '#008000')
    plt.plot(speed_d_error, '#FF00FF')
    speed_p_error_patch = mpatches.Patch(color='#0000FF', label='speed_p_error')
    speed_i_error_patch = mpatches.Patch(color='#008000', label='speed_i_error') 
    speed_d_error_patch = mpatches.Patch(color='#FF00FF', label='speed_d_error')    
    plt.legend(handles=[speed_p_error_patch, speed_i_error_patch, speed_d_error_patch])
    plt.title('Throttle PID Controller errors')
    plt.xlabel('Iteration (t)')
    plt.ylabel('Error')
    plt.grid(True)

    plt.subplot(3, 2, 5)
    plt.plot(speed_throttle, '#0000FF')
    speed_throttle_patch = mpatches.Patch(color='#0000FF', label='Throttle')    
    plt.legend(handles=[speed_throttle_patch])
    plt.title('Cars Throttle')
    plt.xlabel('Iteration (t)')
    plt.ylabel('Throttle Value')
    plt.grid(True)

    # -------------------------------------------------------------------------
    plt.subplot(3, 2, 2)
    plt.plot(json_steer_angle, '#0000FF')
    plt.plot(json_steer_cte, '#008000')
    json_steer_angle_patch = mpatches.Patch(color='#0000FF', label='Cars Angle')
    json_steer_cte_patch = mpatches.Patch(color='#008000', label='Cross Track')   
    plt.legend(handles=[json_steer_angle_patch, json_steer_cte_patch])
    plt.title('Cars Steering Angle')
    plt.xlabel('Iteration (t)')
    plt.ylabel('Steering Angle')
    plt.grid(True)
    
    plt.subplot(3, 2, 4)
    plt.plot(steer_p_error, '#0000FF')
    plt.plot(steer_i_error, '#008000')
    plt.plot(steer_d_error, '#FF00FF')
    steer_p_error_patch = mpatches.Patch(color='#0000FF', label='steer_p_error')
    steer_i_error_patch = mpatches.Patch(color='#008000', label='steer_i_error') 
    steer_d_error_patch = mpatches.Patch(color='#FF00FF', label='steer_d_error')    
    plt.legend(handles=[steer_p_error_patch, steer_i_error_patch, steer_d_error_patch])
    plt.title('Steering PID Controller errors')
    plt.xlabel('Iteration (t)')
    plt.ylabel('Error')
    plt.grid(True)

    plt.subplot(3, 2, 6)
    plt.plot(steer_value, '#0000FF')
    steer_value_patch = mpatches.Patch(color='#0000FF', label='steering value')    
    plt.legend(handles=[steer_value_patch])
    plt.title('Cars Steering')
    plt.xlabel('Iteration (t)')
    plt.ylabel('Steering Value')
    plt.grid(True)

    if save_file is not None:
        plt.savefig(save_file)

    # Plot graphs
    plt.show()

# =============================================================================
if __name__=="__main__":

    # Run all stuff
    main()

    # Plot steering controller graph
    plot_controller_graph(
        steering_file_path="pid_steering.csv", 
        speed_file_path="pid_speed.csv", 
        save_file="writeup_files/steering_pid_graph.png"
        )

# =============================================================================