import serial
import open3d
import numpy as np


s = serial.Serial('COM4', 115200, timeout=10)
                            
print("Opening: " + s.name)


# wait for user's signal to start the program
input("Press Enter to start communication...")

coords = []
scans = []

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# This will signal MCU to start the transmission
s.write('s'.encode())

while 1:
    # read the data from UART one line at a time
    x = s.readline()
    print(x.decode())
    if(x.decode() == 'mapping...\r\n'):
        break

    # split the line into words seperated by a comma
    words = x.decode().split(',')
    # convert the words into numbers
    nums = [float(word) for word in words]
    
    coords.append([nums[0], nums[1], nums[2]])

       
# write to file
f = open("file.xyz", "w")

for coord in coords:
    for point in coord:
        f.write(str(point) + ' ')
        if point != coord[2]:
            f.write(' ')
    f.write('\n')

f.close()

pcd = open3d.io.read_point_cloud("file.xyz", format="xyz")

plane = []
for i in range(len(np.asarray(pcd.points))):
    plane.append([i])

lines = []  
for i in range(len(np.asarray(pcd.points)) - 1):
    lines.append([plane[i], plane[i+1]])

#Define coordinates to connect lines between current and next yz slice        
for i in range(len(np.asarray(pcd.points)) - 32):
    lines.append([plane[i], plane[i+32]])

if len(np.asarray(pcd.points)) - 1 >= 32:
    lines.append([plane[0], plane[32]])
for i in range(0, len(np.asarray(pcd.points)) - 32, 32):
    if i % 32 == 0 and (i + 32) <= len(np.asarray(pcd.points)):
        lines.append([plane[i], plane[i+32]])

line_set = open3d.geometry.LineSet(points=open3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=open3d.utility.Vector2iVector(lines))

open3d.visualization.draw_geometries([line_set])

#close the port
print("Closing: " + s.name)
s.close()
