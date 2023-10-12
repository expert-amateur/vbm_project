import cv2
import numpy as np

image = cv2.imread('src/pcl_sampling/scripts/object.jpeg', cv2.IMREAD_GRAYSCALE)
def dist(p1,p2):
    return ((p1[0]-p2[0])**2 +(p1[1]-p2[1])**2)**0.5
#Blur
blurred = cv2.GaussianBlur(image, (17, 17), 0)

# Edges
edges = cv2.Canny(blurred, 100, 200)

cv2.imshow('Detected Edges', edges)
cv2.waitKey(0)

#Computing dx 
dx = cv2.Sobel(edges, cv2.CV_64F, 1, 0, ksize=3)
dy = cv2.Sobel(edges, cv2.CV_64F, 0, 1, ksize=3)

# Compute the magnitude and angle (this can be useful for visualization)
magnitude, angle = cv2.cartToPolar(dx, dy, angleInDegrees=True)

# Normalize to get unit vectors (for visualization)
normalized_dx = cv2.normalize(dx, None, 0, 10, cv2.NORM_MINMAX)
normalized_dy = cv2.normalize(dy, None, 0, 10, cv2.NORM_MINMAX)


contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
largest_contour = max(contours, key=cv2.contourArea)

# Find the CM
moments = cv2.moments(edges, binaryImage=True)
cx = int(moments['m10'] / moments['m00'])
cy = int(moments['m01'] / moments['m00'])
cm=(cx,cy)

distance = cv2.pointPolygonTest(largest_contour, (cx, cy), False)
if distance < 0:
    print("The center of mass is outside the object.")
else:
    print("The center of mass is inside the object.")

inside_object = cv2.pointPolygonTest(largest_contour, (cx, cy), False) >= 0

color_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)  # Center of mass in red
cv2.imshow('Center of Mass', color_image)
cv2.waitKey(0)


# Find the closest points to CM
y, x = np.where(edges > 200) # white points on the image
white_points=points = list(zip(x, y))
distances = np.sqrt((x - cx)**2 + (y - cy)**2) # distance 
closest_idx = np.argmin(distances) #min value
closest_point = (x[closest_idx], y[closest_idx])
print(closest_point)

pairs = []
seen_pairs = set()

def perp_dist(point1, point2, point3): #Dist of pt3 from
    delta_x=point1[0]-point2[0]
    delta_y=point1[1]-point2[1]
    if delta_x!=0:
        m=(delta_y/delta_x)
        c=point1[1]-m*point1[0]
        p_dist=abs(point3[1]-m*point3[0]-c)/(1+m**2)**0.5
    else:
        p_dist=abs(point3[0]-point1[0])
    return p_dist


for key1 in white_points:
    for key2 in white_points:
        if key1 != key2 and perp_dist(key1,cm,key2) <=1.1 and dist(key1, key2)>10:
            pair = [key1, key2]
            pair.sort()  # Sort the keys to avoid repetitions [key1, key2] == [key2, key1]
            pair_tuple = tuple(pair)
            if pair_tuple not in seen_pairs:
                pairs.append(pair)
                seen_pairs.add(pair_tuple)

ran_idx=np.random.randint(0,len(x))

target_key=(x[ran_idx],y[ran_idx])
print("Target key:", target_key)
result = []
for pair in pairs:
    if target_key in pair:
        result.append(pair)
print("Result",result)

k=9

cv2.circle(color_image, (target_key[0], target_key[1]), 5, (0, 255, 0), -1)  
for i in range(len(result)):
    if result[i][0]!=target_key:
        opposite_point=result[i][0]
    else:
        opposite_point=result[i][1]
    cv2.circle(color_image,(opposite_point[0],opposite_point[1]),5,(255,0,0),-1)
cv2.imshow('Closest point', color_image)
cv2.waitKey(0)

