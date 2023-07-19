# VIO (Visual Odometry)

## Coding ORB features from scratch.

ORB features are also composed of two parts: *ORB key points* and *ORB descriptor*. Its key point is called **oriented FAST**, which is an improved version of the FAST. Its descriptor is called **BRIEF (Binary Robust Independent Elementary Feature)**. 

### FAST corner point extraction

Use Fast algorithm from OpenCV to extract fast corner points

### Compute Rotation

In terms of rotation, we calculate the gray centroid of the image near the feature point. The so-called centroid refers to the gray value of the image block as the center of weight.

1. In a small image block **B**, define the moment of the image block as:

$$
m_{pq}=\sum_{x,y \in B}x^{p}y^{q}I(x,y), \quad p, q = \{0,1\}.
$$

2. Calculate the centroid of the image block by the moment:

$$
C=\left(\frac{m_{10}}{m_{00}},\frac{m_{01}}{m_{00}}\right).
$$

3. Connect the geometric center $O$ and the centroid $C$ of the image block to get a direction vector $ \overrightarrow{OC}$ , so the direction of the feature point can be defined as:

$$
\theta = \arctan(m_{01}/m_{10}).
$$

FAST corner points have a description of scale and rotation, which significantly improves the robustness of their representation between different images. This improved FAST is called oriented FAST in ORB.

### Feature Matching

The simplest feature matching method is the brute-force matcher, which measures the distance between each pair of the features $x_{t}^{m}$ and all $x_{t+1}^{n}$ descriptors. Then sort the matching distance, and take the closest one as the matching point. The descriptor distance indicates the degree of similarity of two features. In practice, different distance metric norms can be used. For descriptors of floating-point type, using Euclidean distance to measure is a good choice. For binary descriptors (such as BRIEF), we often use Hamming distance as a metric. The Hamming distance between two binary vectors refers to the number of different digits.

### Result

Features:

![](./features.png)

Match result:

![](./matches.png)
