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

def draw_normals(image, points, dx, dy, scale=5):
    """Draw normals on the image for given points using dx, dy gradients."""
    for point in [points[650]]:  # Select every 2nd point
        x, y = point
        end_x = int(x + dx[y, x] * scale)
        end_y = int(y + dy[y, x] * scale)
        cv2.arrowedLine(image, (x, y), (end_x, end_y), (255, 255, 0), 1, tipLength=0.5)
    return image

def fit_line_normals(image, neighborhood_size=5):
    """
    Compute surface normals using line fitting.
    :param image: The input image with white object outline.
    :param neighborhood_size: Size of the neighborhood to consider around each white pixel.
    :return: Normals' dx and dy components.
    """
    height, width = image.shape
    dx = np.zeros_like(image, dtype=np.float64)
    dy = np.zeros_like(image, dtype=np.float64)

    y, x = np.where(image > 200)  # white points on the image

    # Half of the neighborhood size
    half_n = neighborhood_size // 2

    for xi, yi in zip(x, y):
        # Get neighborhood
        x1 = max(xi - half_n, 0)
        x2 = min(xi + half_n, width-1)
        y1 = max(yi - half_n, 0)
        y2 = min(yi + half_n, height-1)
        
        nx, ny = np.where(image[y1:y2, x1:x2] > 200)

        # Correcting coordinates to global image
        nx += x1
        ny += y1

        if len(nx) < 2:  # Not enough points
            continue
        
        # Fit line using numpy's polyfit
        fit_params = np.polyfit(nx, ny, 1)  # Degree 1 (linear fit)
        
        # The slope of the fitted line
        slope = fit_params[0]
        
        # Direction vector of the line
        direction = np.array([1, slope])
        direction /= np.linalg.norm(direction)

        # Normal (perpendicular to direction)
        normal = np.array([-direction[1], direction[0]])

        
        # # Compute the normal
        # magnitude = np.sqrt(1 + slope**2)
        # dx[yi, xi] = -1 / magnitude
        # dy[yi, xi] = slope / magnitude
        dx[yi, xi] = normal[0]
        dy[yi, xi] = normal[1]


    return dx, dy



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

#Fitted Line
fit_dx, fit_dy = fit_line_normals(edges)
# Visualize normals using line fitting method
fit_normals_visualized = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
draw_normals(fit_normals_visualized, white_points, fit_dx, fit_dy)
fit_resize = cv2.resize(fit_normals_visualized,(1280,960))
cv2.imshow('Line Fit Normals Visualization', fit_resize)
cv2.waitKey(0)


normals_visualized = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
draw_normals(normals_visualized, white_points, normalized_dx, normalized_dy)
resized = cv2.resize(normals_visualized,(640,480))
cv2.imshow('Normals Visualization', resized)
cv2.waitKey(0)
cv2.circle(color_image, (target_key[0], target_key[1]), 5, (0, 255, 0), -1)  
for i in range(len(result)):
    if result[i][0]!=target_key:
        opposite_point=result[i][0]
    else:
        opposite_point=result[i][1]
    cv2.circle(color_image,(opposite_point[0],opposite_point[1]),5,(255,0,0),-1)
cv2.imshow('Closest point', color_image)
cv2.waitKey(0)
cv2.destroyAllWindows()


