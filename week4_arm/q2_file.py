import math
l1 = 2
l2 = 1.5
theta1 = 0.0
theta2 = 0.0
target_x = float(input("Enter x coordinate of the target point: "))
target_y = float(input("Enter y coordinate of the target point: "))
flag = True
dist = math.sqrt(target_x**2 + target_y**2)
max_reach = 3.5
min_reach = 0.5
if dist < min_reach:
    flag = False
    print("Target too close.")
elif dist > max_reach:
    flag = False
    print("Target too far.")

if flag == True:
    theta2 = math.acos((target_x**2 + target_y**2 - 4 - 2.25)/(2*l1*l2))
    theta1 = math.atan2(target_y, target_x) - math.atan((l2*math.sin(theta2))/(l1+l2*math.cos(theta2)))
    print(f"The angles to reach the target point: {theta1}, {theta2}")
