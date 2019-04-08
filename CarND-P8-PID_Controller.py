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

    # Run make in build fodler
    os.system('clear && cd build && make') 

    # Run subprocess
    try:
        processes = (
            "{}".format(os.path.join(os.getcwd(), "term2_sim_linux", "term2_sim.x86_64")),
            "{}".format(os.path.join(os.getcwd(), "build", "pid"))
        )
        # Run simulator and socket
        pool = Pool(processes=len(processes))                                                        
        pool.map(run_process, processes)
    except Exception as e: 
        print(str(e))

    print("Process has finished")

def plot_controller_graph(file_path, save_file=None):

    # Variables inizialitation
    json_speed = []
    angle = []
    cte = []
    throttle = []
    steer_value = []
    p_error = []
    i_error = []
    d_error = []
    
    # Read csv file
    with open(file_path) as csvfile:
        reader = csv.reader(csvfile)
        for idx, line in enumerate(reader):
            if idx:
                json_speed.append(float(line[0]))
                angle.append(float(line[1]))
                cte.append(float(line[2]))
                throttle.append(float(line[3]))
                steer_value.append(float(line[4]))
                p_error.append(float(line[5]))
                i_error.append(float(line[6]))
                d_error.append(float(line[7]))
    
    # Plot cross-track error
    plt.subplot(2, 2, 1)
    plt.plot(cte, 'r')
    cte_patch = mpatches.Patch(color='r', label='p_error')     
    plt.legend(handles=[cte_patch])
    plt.title('cross-track')
    plt.xlabel('Iteration (t)')
    plt.ylabel('Error')
    plt.grid(True)

    # Plot PID errors
    plt.subplot(2, 2, 2)
    plt.plot(p_error, 'r') 
    plt.plot(i_error, 'g') 
    plt.plot(d_error, 'b')
    p_error_patch = mpatches.Patch(color='r', label='p_error')   
    i_error_patch = mpatches.Patch(color='g', label='i_error')   
    d_error_patch = mpatches.Patch(color='b', label='d_error')    
    plt.legend(handles=[p_error_patch, i_error_patch, d_error_patch])
    plt.title('Steering PID Controller Errors')
    plt.xlabel('Iteration (t)')
    plt.ylabel('PID Errors')
    plt.grid(True)

    # Plot cross-track error
    plt.subplot(2, 2, 3)
    max_json_speed = max(json_speed)
    plt.plot([x / max_json_speed for x in json_speed], 'r')
    plt.plot(throttle, 'b')
    json_speed_patch = mpatches.Patch(color='r', label='speed [mph] max:{}'.format(round(max_speed, 2)))  
    throttle_patch = mpatches.Patch(color='b', label='throttle')      
    plt.legend(handles=[json_speed_patch])
    plt.title('Cars speed')
    plt.xlabel('Iteration (t)')
    plt.ylabel('Speed')
    plt.grid(True)

    # Plot cross-track error
    plt.subplot(2, 2, 4)
    plt.plot(steer_value, 'r')
    steer_value_patch = mpatches.Patch(color='r', label='pid_steering')    
    plt.legend(handles=[steer_value_patch])
    plt.title('Cars Steering')
    plt.xlabel('Iteration (t)')
    plt.ylabel('Steering Angle')
    plt.grid(True)

    # Plot graphs
    plt.show()

# =============================================================================
if __name__=="__main__":

    # Run all stuff
    main()

    # Plot steering controller graph
    plot_controller_graph(file_path="pid_steering.csv", save_file="writeup_files/steering_pid_graph.png")

# =============================================================================