import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import customtkinter
import socket
import json
import atexit


# Link lengths
a1 = 120.0
a2 = 151.0
a3 = 100.0
a4 = 60.0
a5 = 60.0
a6 = 20.0

# Toolframe specifications array
#                     X    Y    Z    Rz   Ry   Rx   
toolframe = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Create the links of the robot for 3D visualization
links = [
    RevoluteDH(d=a1, a=0, alpha=np.pi/2),
    RevoluteDH(d=0, a=a2, alpha=0),
    RevoluteDH(d=0, a=0, alpha=np.pi/2),
    RevoluteDH(d=a3+a4, a=0, alpha=-np.pi/2),
    RevoluteDH(d=0, a=0, alpha=np.pi/2),
    RevoluteDH(d=a5+a6, a=0, alpha=0)
]

# Create the robot visualization object
robot_3D = DHRobot(links, name='6-DOF Arm')

# Socket cleanup function to execute upon interpreter termination
def close_socket():
    s.close()

atexit.register(close_socket)


class Robot:
    def __init__(self, a1, a2, a3, a4, a5, a6, toolframe):
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4
        self.a5 = a5
        self.a6 = a6
        self.theta_1 = 0.0
        self.theta_2 = 0.0
        self.theta_3 = 0.0
        self.theta_4 = 0.0
        self.theta_5 = 0.0
        self.theta_6 = 0.0
        # DH parameters matrix
        self.dh_matrix = np.array([[self.theta_1, (np.pi)/2, 0.0, a1],
                            [self.theta_2+np.pi/2, 0.0, a2, 0.0],
                            [self.theta_3, (np.pi)/2, 0.0, 0.0],
                            [self.theta_4, -(np.pi)/2, 0.0, a3+a4],
                            [self.theta_5, (np.pi)/2, 0.0, 0.0],
                            [self.theta_6, 0.0, 0.0, a5+a6]])
        
        # Manually creating the transformation matrix for the attached toolframe
        self.toolframe_matrix = np.array([[np.cos(toolframe[3])*np.cos(toolframe[4]), np.cos(toolframe[3])*np.sin(toolframe[4])*np.sin(toolframe[5])-np.sin(toolframe[3])*np.cos(toolframe[5]), np.cos(toolframe[3])*np.sin(toolframe[4])*np.cos(toolframe[5])+np.sin(toolframe[3])*np.sin(toolframe[5]), toolframe[0]],
                                        [np.sin(toolframe[3])*np.cos(toolframe[4]), np.sin(toolframe[3])*np.sin(toolframe[4])*np.sin(toolframe[5])+np.cos(toolframe[3])*np.cos(toolframe[5]), np.sin(toolframe[3])*np.sin(toolframe[4])*np.cos(toolframe[5])-np.cos(toolframe[3])*np.sin(toolframe[5]), toolframe[1]],
                                        [-np.sin(toolframe[4]), np.cos(toolframe[4])*np.sin(toolframe[5]), np.cos(toolframe[4])*np.cos(toolframe[5]), toolframe[2]],
                                        [0.0, 0.0, 0.0, 1.0]])

    def update_dh_matrix(self, new_theta, joint_index):
        match joint_index:
            case 0:
                self.theta_1 = new_theta
                self.dh_matrix[0, 0] = new_theta
            case 1:
                self.theta_2 = new_theta
                self.dh_matrix[1, 0] = new_theta + np.pi/2
            case 2:
                self.theta_3 = new_theta
                self.dh_matrix[2, 0] = new_theta
            case 3:
                self.theta_4 = new_theta
                self.dh_matrix[3, 0] = new_theta
            case 4:
                self.theta_5 = new_theta
                self.dh_matrix[4, 0] = new_theta
            case 5:
                self.theta_6 = new_theta
                self.dh_matrix[5, 0] = new_theta
            case _:
                pass

    def forward_kinematics(self):
        # Initialize the transform matrix using DH parameters of J1
        fk_transformMatrix = np.array([[np.cos(self.dh_matrix[0, 0]), -np.sin(self.dh_matrix[0, 0])*np.cos(self.dh_matrix[0, 1]), np.sin(self.dh_matrix[0, 0])*np.sin(self.dh_matrix[0, 1]), self.dh_matrix[0, 2]*np.cos(self.dh_matrix[0, 0])],
                                       [np.sin(self.dh_matrix[0, 0]), np.cos(self.dh_matrix[0, 0])*np.cos(self.dh_matrix[0, 1]), -np.cos(self.dh_matrix[0, 0])*np.sin(self.dh_matrix[0, 1]), self.dh_matrix[0, 2]*np.sin(self.dh_matrix[0, 0])],
                                       [0.0, np.sin(self.dh_matrix[0, 1]), np.cos(self.dh_matrix[0, 1]), self.dh_matrix[0, 3]],
                                       [0.0, 0.0, 0.0, 1.0]])

        # Multiply by the transform matrix of J2 to J6
        for i in range(1,6):
            fk_loopMatrix = np.array([[np.cos(self.dh_matrix[i, 0]), -np.sin(self.dh_matrix[i, 0])*np.cos(self.dh_matrix[i, 1]), np.sin(self.dh_matrix[i, 0])*np.sin(self.dh_matrix[i, 1]), self.dh_matrix[i, 2]*np.cos(self.dh_matrix[i, 0])],
                                       [np.sin(self.dh_matrix[i, 0]), np.cos(self.dh_matrix[i, 0])*np.cos(self.dh_matrix[i, 1]), -np.cos(self.dh_matrix[i, 0])*np.sin(self.dh_matrix[i, 1]), self.dh_matrix[i, 2]*np.sin(self.dh_matrix[i, 0])],
                                       [0.0, np.sin(self.dh_matrix[i, 1]), np.cos(self.dh_matrix[i, 1]), self.dh_matrix[i, 3]],
                                       [0.0, 0.0, 0.0, 1.0]])     
            fk_transformMatrix = np.matmul(fk_transformMatrix, fk_loopMatrix)

        # Multiply by the transform matrix of the toolframe to get the final frame's matrix
        fk_transformMatrix = np.matmul(fk_transformMatrix, self.toolframe_matrix)

        # Calculate the cartesian coordiantes (X,Y,Z) and Euler angles (Rx,Ry,Rz) from the final matrix
        fk_x = fk_transformMatrix[0, 3]
        fk_y = fk_transformMatrix[1, 3]
        fk_z = fk_transformMatrix[2, 3]
        fk_Rz = np.arctan2(fk_transformMatrix[1, 0], fk_transformMatrix[0, 0])  # yaw
        fk_Ry = np.arcsin(-fk_transformMatrix[2, 0])  # pitch
        fk_Rx = np.arctan2(fk_transformMatrix[2, 1], fk_transformMatrix[2, 2])  # roll

        # Return an output array containing the coordinates(in mm) and angles(in degrees)
        fk_output = np.array([fk_x, fk_y, fk_z, fk_Rz, fk_Ry, fk_Rx])
        return fk_output


    def inverse_kinematics(self, ik_target):
        # Calculating the spherical wrist's centre
        # Manually creating the transformation matrix for the robot's end effector target position
        ik_R06T = np.array([[np.cos(ik_target[3])*np.cos(ik_target[4]), np.cos(ik_target[3])*np.sin(ik_target[4])*np.sin(ik_target[5])-np.sin(ik_target[3])*np.cos(ik_target[5]), np.cos(ik_target[3])*np.sin(ik_target[4])*np.cos(ik_target[5])+np.sin(ik_target[3])*np.sin(ik_target[5]), ik_target[0]],
                                        [np.sin(ik_target[3])*np.cos(ik_target[4]), np.sin(ik_target[3])*np.sin(ik_target[4])*np.sin(ik_target[5])+np.cos(ik_target[3])*np.cos(ik_target[5]), np.sin(ik_target[3])*np.sin(ik_target[4])*np.cos(ik_target[5])-np.cos(ik_target[3])*np.sin(ik_target[5]), ik_target[1]],
                                        [-np.sin(ik_target[4]), np.cos(ik_target[4])*np.sin(ik_target[5]), np.cos(ik_target[4])*np.cos(ik_target[5]), ik_target[2]],
                                        [0.0, 0.0, 0.0, 1.0]])
        
        # Invert the toolframe and calculate joint 6's frame
        ik_invertToolframe = np.linalg.inv(self.toolframe_matrix)
        ik_R06 = np.matmul(ik_R06T, ik_invertToolframe)

        # Matrix with negated a5+a6
        ik_R06Negate = np.array([[1.0, 0.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0, 0.0],
                                 [0.0, 0.0, 1.0, -self.dh_matrix[5, 3]],
                                 [0.0, 0.0, 0.0, 1.0]])
        
        # Transformation matrix for the center of the spherical wrist (J5)
        ik_R05 = np.matmul(ik_R06, ik_R06Negate)

        # Top view geometric calculation
        # Calculating J1 angle (radians), only works for positive x coordinates
        ik_thetaJ1 = np.arctan2(ik_R05[1, 3], ik_R05[0, 3])

        ik_l1 = np.sqrt(ik_R05[0, 3]**2 + ik_R05[1, 3]**2)

        # Side view geometric calculations for calculating J2 and J3 angles (radians)
        # Calculate the x coordinate if j1 was at 0 degrees, x' = xcos(theta) - ysin(theta) = L1 where theta = ik_thetaJ1
#        ik_l1 = ik_R05[0, 3]*np.cos(ik_thetaJ1) - ik_R05[1, 3]*np.sin(ik_thetaJ1)

        ik_l2 = ik_R05[2, 3] - self.dh_matrix[0, 3]

        ik_l3 = np.sqrt((ik_l1**2)+(ik_l2**2))

        ik_thetaB = np.arctan2(ik_l2, ik_l1)

        ik_thetaC = np.arccos((self.dh_matrix[1, 2]**2 + ik_l3**2 - self.dh_matrix[3, 3]**2)/(2 * self.dh_matrix[1, 2] * ik_l3))

        ik_thetaA = np.arccos((self.dh_matrix[3, 3]**2 + self.dh_matrix[1, 2]**2 - ik_l3**2)/(2 * self.dh_matrix[3, 3] * self.dh_matrix[1, 2]))

        if ik_R05[0, 3] > 0.0:
            if ik_R05[2, 3] >= self.dh_matrix[0, 3]:
                ik_thetaJ2 = ik_thetaC + ik_thetaB
            else:
                ik_thetaJ2 = ik_thetaC - ik_thetaB
        else:
            ik_thetaJ2 = -(ik_thetaC + ik_thetaB)

        ik_thetaJ3 = ik_thetaA + np.pi/2
        
        if np.abs(3.14-np.abs(ik_thetaJ3)) <= 0.003:
            ik_thetaJ3 = 0.0

        # Calculating the wrist angles by going back to frame R03, and multipling its inverse by R06 to get R36 (R03*R36=R06, therefore, R36=R03'*R06)
        # Initialize the transform matrix using DH parameters of J1
        ik_J1 = np.array([[np.cos(ik_thetaJ1), -np.sin(ik_thetaJ1)*np.cos(self.dh_matrix[0, 1]), np.sin(ik_thetaJ1)*np.sin(self.dh_matrix[0, 1]), self.dh_matrix[0, 2]*np.cos(ik_thetaJ1)],
                                       [np.sin(ik_thetaJ1), np.cos(ik_thetaJ1)*np.cos(self.dh_matrix[0, 1]), -np.cos(ik_thetaJ1)*np.sin(self.dh_matrix[0, 1]), self.dh_matrix[0, 2]*np.sin(ik_thetaJ1)],
                                       [0.0, np.sin(self.dh_matrix[0, 1]), np.cos(self.dh_matrix[0, 1]), self.dh_matrix[0, 3]],
                                       [0.0, 0.0, 0.0, 1.0]])
        
        ik_J2 = np.array([[np.cos(ik_thetaJ2), -np.sin(ik_thetaJ2)*np.cos(self.dh_matrix[1, 1]), np.sin(ik_thetaJ2)*np.sin(self.dh_matrix[1, 1]), self.dh_matrix[1, 2]*np.cos(ik_thetaJ2)],
                                       [np.sin(ik_thetaJ2), np.cos(ik_thetaJ2)*np.cos(self.dh_matrix[1, 1]), -np.cos(ik_thetaJ2)*np.sin(self.dh_matrix[1, 1]), self.dh_matrix[1, 2]*np.sin(ik_thetaJ2)],
                                       [0.0, np.sin(self.dh_matrix[0, 1]), np.cos(self.dh_matrix[0, 1]), self.dh_matrix[0, 3]],
                                       [0.0, 0.0, 0.0, 1.0]])
        
        ik_J3 = np.array([[np.cos(ik_thetaJ3), -np.sin(ik_thetaJ3)*np.cos(self.dh_matrix[2, 1]), np.sin(ik_thetaJ3)*np.sin(self.dh_matrix[2, 1]), self.dh_matrix[2, 2]*np.cos(ik_thetaJ3)],
                                       [np.sin(ik_thetaJ3), np.cos(ik_thetaJ3)*np.cos(self.dh_matrix[2, 1]), -np.cos(ik_thetaJ3)*np.sin(self.dh_matrix[2, 1]), self.dh_matrix[2, 2]*np.sin(ik_thetaJ3)],
                                       [0.0, np.sin(self.dh_matrix[2, 1]), np.cos(self.dh_matrix[2, 1]), self.dh_matrix[2, 3]],
                                       [0.0, 0.0, 0.0, 1.0]])

        ik_R02 = np.matmul(ik_J1, ik_J2)

        ik_R03 = np.matmul(ik_R02, ik_J3)

        ik_R03Inverted = np.linalg.inv(ik_R03)

        ik_R36 = np.matmul(ik_R03Inverted, ik_R06)

        # Extracting J4-J6 angles from R36
        ik_thetaJ4 = np.arctan2(ik_R36[3, 2], ik_R36[3, 3])                                  # Roll
        ik_thetaJ5 = np.arctan2(ik_R36[2, 2], np.sqrt(1-ik_R36[2, 2]**2))                    # Pitch
        ik_thetaJ6 = np.arctan2(ik_R36[2, 1], ik_R36[1, 1])                                  # Yaw

        # Return an output array containing J1-J6 angles
        fk_output = np.array([ik_thetaJ1*(180/np.pi), ik_thetaJ2*(180/np.pi), ik_thetaJ3*(180/np.pi), ik_thetaJ4*(180/np.pi), ik_thetaJ5*(180/np.pi), ik_thetaJ6*(180/np.pi)])
        return fk_output 


# GUI Functions
def update_slider_from_entry(event, slider, robot, joint_index, canvas, ax, inverse_entries):
    try:
        value = float(event.widget.get())
        if -180 <= value <= 180:
            slider.set(value)
            # Convert from degrees to radians and update joint angle
            match joint_index:
                case 0:
                    new_theta = np.radians(value)
                    robot.update_dh_matrix(new_theta, 0)
                case 1:
                    new_theta = np.radians(value)
                    robot.update_dh_matrix(new_theta, 1)
                case 2:
                    new_theta = np.radians(value)
                    robot.update_dh_matrix(new_theta, 2)
                case 3:
                    new_theta = np.radians(value)
                    robot.update_dh_matrix(new_theta, 3)
                case 4:
                    new_theta = np.radians(value)
                    robot.update_dh_matrix(new_theta, 4)
                case 5:
                    new_theta = np.radians(value)
                    robot.update_dh_matrix(new_theta, 5)
                case _:
                    pass
                  
            update_robot_plot(robot, canvas, ax)
            update_ik_entries(robot, inverse_entries)
        else:
            # Out-of-range values
            pass
    except ValueError:
        # Non-numeric input
        pass

def update_entry_from_slider(value, entry, robot, joint_index, canvas, ax, inverse_entries):
    #Update the entry text with new slider number
    entry.delete(0, customtkinter.END)
    entry.insert(0, str(value))
    # Convert from degrees to radians and update joint angle
    match joint_index:
        case 0:
            new_theta = np.radians(value)
            robot.update_dh_matrix(new_theta, 0)
        case 1:
            new_theta = np.radians(value)
            robot.update_dh_matrix(new_theta, 1)
        case 2:
            new_theta = np.radians(value)
            robot.update_dh_matrix(new_theta, 2)
        case 3:
            new_theta = np.radians(value)
            robot.update_dh_matrix(new_theta, 3)
        case 4:
            new_theta = np.radians(value)
            robot.update_dh_matrix(new_theta, 4)
        case 5:
            new_theta = np.radians(value)
            robot.update_dh_matrix(new_theta, 5)
        case _:
            pass

    update_robot_plot(robot, canvas, ax)
    update_ik_entries(robot, inverse_entries)

def update_ik_entries(robot, inverse_entries):
    # Perform forward kinematics to get the current end effector position and orientation
    fk_output = robot.forward_kinematics()  # Assuming this returns [x, y, z, Rz, Ry, Rx]

    # Update inverse kinematics entries
    for i, value in enumerate(fk_output):
        inverse_entries[i].delete(0, customtkinter.END)
        inverse_entries[i].insert(0, f"{value:.2f}")

def home_robot(robot, sliders, entries, canvas, ax):
    # Reset all joint angles to 0 radians and update sliders and entries to reflect the reset
    for i in range(6):
        robot.update_dh_matrix(0.0, i)
        sliders[i].set(0)
        entries[i].delete(0, customtkinter.END)
        entries[i].insert(0, "0")
    
    update_robot_plot(robot, canvas, ax)

# Wifi socket TCP communication when "Send FK" button is pressed
def send_fk(robot):
    try:
        # Convert FK angles to JSON
        data = {
            'theta_1': robot.theta_1 * (180.0/np.pi),
            'theta_2': robot.theta_2 * (180.0/np.pi),
            'theta_3': robot.theta_3 * (180.0/np.pi),
            'theta_4': robot.theta_4 * (180.0/np.pi),
            'theta_5': robot.theta_5 * (180.0/np.pi),
            'theta_6': robot.theta_6 * (180.0/np.pi)
        }
        json_data = json.dumps(data)
        
        # Send JSON data to ESP32
        s.sendall(json_data.encode()) 
    
        # Wait for ESP32 to confirm receipt
        response = s.recv(1024).decode('utf-8')  # Receive data from ESP32
        if response:
            print("Received from ESP32:", response)

    except Exception as e:
        print(f"Error sending FK data:{e}")


def create_gui(robot):
    customtkinter.set_appearance_mode("Dark")
    customtkinter.set_default_color_theme("blue")
    root = customtkinter.CTk()
    root.title("Robotic Arm Controller")
    root.geometry("1280x720")

    # Forward Kinematics Section
    fk_label = customtkinter.CTkLabel(master=root, text="Forward Kinematics:")
    fk_label.place(x=935, y=10)
    forward_frame = customtkinter.CTkFrame(master=root)
    forward_frame.place(x=825, y=50)
    fk_button = customtkinter.CTkButton(master=root, text="Send FK", command=lambda: send_fk(robot))
    fk_button.place(x=925, y=300)

    forward_sliders = []
    forward_entries = []

    # Create Matplotlib Figure and Canvas
    fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().place(x=50, y=50)

    for i in range(6):
        customtkinter.CTkLabel(forward_frame, text=f"Joint {i+1}").grid(row=i, column=0, padx=10, pady=5)
        slider = customtkinter.CTkSlider(forward_frame, from_=-180, to=180, orientation="horizontal")
        slider.grid(row=i, column=1, padx=10, pady=5)
        entry = customtkinter.CTkEntry(forward_frame, width=60)
        entry.grid(row=i, column=2, padx=10, pady=5)
        slider.configure(command=lambda value, e=entry, j=i: update_entry_from_slider(value, e, robot, j, canvas, ax, inverse_entries))
        entry.bind("<Return>", lambda event, s=slider, j=i: update_slider_from_entry(event, s, robot, j, canvas, ax, inverse_entries))
        forward_sliders.append(slider)
        forward_entries.append(entry)

    # Inverse Kinematics Section
    ik_label = customtkinter.CTkLabel(master=root, text="Inverse Kinematics:")
    ik_label.place(x=935, y=400)
    inverse_frame = customtkinter.CTkFrame(master=root)
    inverse_frame.place(x=750, y=440)
    ik_button = customtkinter.CTkButton(master=root, text="Send FK")
    ik_button.place(x=925, y=540)

    inverse_entries = []
    inverse_labels = ["X", "Y", "Z", "Rz", "Ry", "Rx"]

    for i, label in enumerate(inverse_labels):
        customtkinter.CTkLabel(inverse_frame, text=f"{label}").grid(row=0, column=i, padx=10, pady=5)
        entry = customtkinter.CTkEntry(inverse_frame, width=60)
        entry.grid(row=1, column=i, padx=10, pady=5)
        inverse_entries.append(entry)

    # Homing button (returns robot to home position)
    home_button = customtkinter.CTkButton(master=root, text="Home Robot")
    home_button.place(x=925, y=625)

    update_robot_plot(robot, canvas, ax)  # Initial plot
    root.mainloop()

# Matplotlib 3D simumlation
def plot_robot_arm(robot, ax):
    ax.clear()  # Clear the current axes to update the plot

    # List to store positions of each joint
    joint_positions = [[0,0,0]]

    # Initial transformation matrix (base frame)
    T = np.eye(4)

    # Calculate the transformation matrix for each joint and extract the position
    for i in range(6):
        theta = robot.dh_matrix[i, 0]
        alpha = robot.dh_matrix[i, 1]
        r = robot.dh_matrix[i, 2]
        d = robot.dh_matrix[i, 3]

        # Transformation matrix for the current joint
        T_joint = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), r*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), r*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

        # Update the overall transformation matrix
        T = np.matmul(T, T_joint)

        # Extract the position of the current joint
        joint_positions.append(T[:3, 3])

    # Add the end effector position by multiplying with the tool frame matrix
    T_end_effector = np.matmul(T, robot.toolframe_matrix)
    joint_positions.append(T_end_effector[:3, 3])

    # Convert positions to numpy array for plotting
    joint_positions = np.array(joint_positions)

    # Plot the links (x,y,z)
    ax.plot(joint_positions[:, 0], joint_positions[:, 1], joint_positions[:, 2], '-o', label='Robot Arm')    
    
    # Setting plot limits
    ax.set_xlim([-500, 500])
    ax.set_ylim([-400, 500])
    ax.set_zlim([0, 500])

    # Labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('6-DOF Robot Arm Visualization')

    ax.legend()
    return ax

def update_robot_plot(robot, canvas, ax):
    plot_robot_arm(robot, ax)
    canvas.draw()



if __name__ == "__main__":
    # Create a Robot object for FK and IK calculations
    robot = Robot(a1, a2, a3, a4, a5, a6, toolframe)

    # Wifi communication with ESP32 initialization
    host = "0.0.0.0"      #ESP32's IP Address
    port = 100
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.connect((host, port))

    create_gui(robot)
