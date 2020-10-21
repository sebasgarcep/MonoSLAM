# MonoSLAM

FIXME: Number equations like:

$$
x = 1
$$ (1)

FIXME: Change and explain camera noise

For prototyping we will be performing SLAM on a 30 FPS monochrome set of images. The main idea is to capture features from the robot's surroundings and track them in order to localize the robot in world coordinates.

## State model

The system model can be described by:

$$
x = \begin{pmatrix} x_v \\ y_1 \\ \vdots \\ y_n \end{pmatrix}
$$
$$
P = \begin{pmatrix}
    P_{x,x} & P_{x,y_1} & \dots & P_{x,y_n} \\
    P_{y_1,x} & P_{y_1,y_1} & \dots & P_{y_1,y_n} \\
    \vdots & \vdots & \ddots & \vdots \\
    P_{y_n,x} & P_{y_n,y_1} & \dots & P_{y_n,y_n}
    \end{pmatrix}
$$

where $x$ is the state, $P$ is the state uncertainty, $x_v$ is the robot state and each $y_i$ is the 3D location in world coordinates of a map feature. Furthermore:

$$
x_v = \begin{pmatrix} r^W \\ q^{WR} \\ v^W \\ w^R \end{pmatrix}
$$

where $r^W$ is the robot coordinates in world coordinates, $q^{WR}$ is the orientation unit quaternion of the robot frame with respect to the world frame, $v^W$ is the robot velocity in the world frame, and $w^R$ is the angular velocity of the robot in the robot frame.

## State model initialization

For the first image we have a set of four features for which we already know their $y_i$ values. We will start them with zero uncertainty, that is:

$$
P = \begin{pmatrix} P_{x,x} & 0 \\ 0 & 0 \end{pmatrix}
$$

Furthermore, we will assume zero uncertainty in all components of $P_{x,x}$ except in the main diagonal of $P_{x_v,x_v}$, which we will initialize with a small degree of uncertainty.

## Motion model

We will assume that between each pair of frames an accelaration impulse

$$
\bold{n} = \begin{pmatrix} V^W \\ \Omega^W \end{pmatrix}
$$

is applied to the robot, where $V^W$ is a linear acceleration impulse vector and $\Omega^W$ is an angular acceleration impulse vector. This impulse vector is assumed to come from a zero-mean gaussian distribution.

Therefore, the motion model $f$, given a $\Delta t$ between the two frames, will be:

$$
x_v \rightarrow_{f_v}
    \begin{pmatrix}
    r^W + (v^W + V^W) \Delta t \\
    q^{WR} \times q((w^W + \Omega^W) \Delta t) \\
    v^W + V^W \\
    w^W + \Omega^W
    \end{pmatrix}
$$

$$
y_i \rightarrow_f y_i
$$

## Camera model

We will use a wide angle camera model to project from 3D space to a 2D image and back. Given a 3D vector $\begin{pmatrix} x & y & z \end{pmatrix}$ in the robot frame, the wide angle camera model outputs a 2D vector $\begin{pmatrix} u & d \end{pmatrix}$ as follows:

$$
\begin{pmatrix} u_d & v_d \end{pmatrix} = \begin{pmatrix} -f_x (x / z) & -f_y (y / z) \end{pmatrix}
$$

$$
\begin{pmatrix} u & v \end{pmatrix} = \frac{\begin{pmatrix} u_d & v_d \end{pmatrix}}{\sqrt{1 + 2 R_d |\begin{pmatrix} u_d & v_d \end{pmatrix}|^2}} + \begin{pmatrix} u_0 & v_0 \end{pmatrix}
$$

where $f_x$ is the focal length along the x-axis, $f_y$ is the focal length along the y-axis, $(u_0, v_0)$ is the principal point and $R_d$ is the radial distortion coming from the wide angle camera.

## Shi-Tomasi feature

A Shi-Tomasi feature is a corner detection algorithm that generates features that are designed to be good for tracking purposes. The algorithm requires calculating the x-axis and y-axis derivatives of the image. This can be done by convoluting the image with the Sobel operators. To elaborate, the Sobel operators are:

$$
S_x = \begin{pmatrix}
      1 & 0 & -1 \\
      2 & 0 & -2 \\
      1 & 0 & -1 \\
      \end{pmatrix}
$$

$$
S_y = \begin{pmatrix}
      1 & 2 & 1 \\
      0 & 0 & 0 \\
      -1 & -2 & -1 \\
      \end{pmatrix}
$$

The convolutions $G_x = S_x * I$, $G_y = S_y * I$ give the x-axis and y-axis derivatives and can be calculates using the formulas:

$$
G_x(i, j) = \sum_{-1 \leq u,v \leq 1} S_x(u, v) \: I(i + u, j + v)
$$

$$
G_y(i, j) = \sum_{-1 \leq u,v \leq 1} S_y(u, v) \: I(i + u, j + v)
$$

where $S_x$ and $S_y$ have their center component at $(0, 0)$ in order to make the formulas clear.

Each Shi-Tomasi feature is a $N \times N$ patch of the image, in our case $11 \times 11$. A patch $P$ classifies to be a feature if the smallest eigenvalue of the following matrix is larger than a given threshold:

$$
Z_P = \sum_{(i, j) \in P} \begin{pmatrix}
         G_x(i, j)^2 & G_x(i, j) G_y(i, j) \\
         G_x(i, j) G_y(i, j) & G_y(i, j)^2
         \end{pmatrix}
$$

## Normalized Cross-Correlation

Given a template window $T$ and an image window $I$ with equal dimensions, the normalized cross correlation is given by:

$$
NCC(T, I) = \frac{\sum_{x,y} (T_{x,y} - \mu_T) (I_{x,y} - \mu_I)}{\sqrt{(\sum_{x,y} (T_{x,y} - \mu_T)^2)(\sum_{x,y} (I_{x,y} - \mu_I)^2)}}
$$

where $\mu_T$ and $\mu_I$ are the means of each respective patch.

## Ellipse search

Given a 2D Gaussian defined by a mean $\mu$ and covariance $\Sigma$, the probability density function is given by:

$$
Z^2 = (x - \mu)^T \Sigma^{-1}(x - \mu)
$$

$$
p(x) = \frac{1}{2 \pi |\Sigma|^{1/2}} e^{-\frac{1}{2}Z^2}
$$

An ellipse search consists of a search inside the ellipsoid of all points withint $N$ standard deviations from the mean. These points satisfy $Z^2 < N^2$, and therefore we can perform the search by generating a rectangular bound which covers all points that satisfy this condition and then testing within this space whether a point is inside the ellipsoid or not. To find the formula for these bounds consider the following:

$$
Z^2 = \Sigma^{-1}_{0,0} (x_x - \mu_x)^2 + 2 \Sigma^{-1}_{0,1} (x_x - \mu_x) (x_y - \mu_y) + \Sigma^{-1}_{1,1} (x_y - \mu_y)^2 = N^2
$$

Now set $A = \Sigma^{-1}_{0,0}$, $B = \Sigma^{-1}_{0,1}$, $C = \Sigma^{-1}_{1,1}$, $u = (x_x - \mu_x)$, $v = (x_y - \mu_y)$. Then the equation can be rewritten as:

$$
A u^2 + 2 B u v + C v^2 = N^2
$$

To find the maximum value of $u$ we will use Lagrange multipliers. First set $f(u, v) = u$ as the function we want to maximize, subject to $g(u, v) = A u^2 + 2 B u v + C v^2 = N^2$. Then let us solve the following system of equations:

$$
\begin{cases}
\nabla f(u, v) = \lambda \nabla g(u, v) \\
g(u, v) = N^2
\end{cases}
$$

$$
\begin{cases}
1 = 2 \lambda A u + 2 \lambda B v \\
0 = 2 \lambda B u + 2 \lambda C v \\
A u^2 + 2 B u v + C v^2 = N^2
\end{cases}
$$

$$
\begin{cases}
\frac{1}{2 \lambda} = A u + B v \\
0 = B u + C v \\
A u^2 + 2 B u v + C v^2 = N^2
\end{cases}
$$

$$
\begin{cases}
\frac{1}{2 \lambda} = A u + B v \\
0 = B u + C v \\
A u^2 + B u v + v (B u + C v) = N^2
\end{cases}
$$

$$
\begin{cases}
\frac{1}{2 \lambda} = A u + B v \\
0 = B u + C v \\
u (A u + B v) = N^2
\end{cases}
$$

$$
\begin{cases}
\frac{1}{2 \lambda} = A u + B v \\
0 = B u + C v \\
u \frac{1}{2 \lambda} = N^2
\end{cases}
$$

$$
\begin{cases}
\frac{1}{2 \lambda} = A u + B v \\
0 = B^2 \frac{1}{C} u + B v \\
u \frac{1}{2 \lambda} = N^2
\end{cases}
$$

$$
\begin{cases}
\frac{1}{2 \lambda} = (A - B^2 \frac{1}{C}) u \\
u \frac{1}{2 \lambda} = N^2
\end{cases}
$$

$$
(A - \frac{B^2}{C}) u^2 = N^2
$$

$$
(AC - B^2) u^2 = N^2 C
$$

$$
|\Sigma^{-1}| u^2 = N^2 C
$$

$$
u^2 = N^2 C / |\Sigma^{-1}|
$$

And by symmetry, solving for $f(u, v) = v$ we get:

$$
v^2 = N^2 A / |\Sigma^{-1}|
$$

Therefore the bounding rectangle for the ellipse is given by the center $\mu$, the x-axis side length $2 N \sqrt{C} / |\Sigma^{-1}|^{0.5}$ and the y-axis side length $2 N \sqrt{A} / |\Sigma^{-1}|^{0.5}$.

## Kalman Filter

The Kalman filter is a procedure that allows us to combine the information from a model and a set of observations to predict a better possible state, with less uncertainty, than any of those two would have told us on their own. The Kalman Filter is comprised of two steps: prediction and update. Given a state $x$, uncertainty $P$, model $F$ and model uncertainty $Q$, the prediction step is:

$$
x^* = F x
$$

$$
P^* = F P F^T + Q
$$

Assume an observation $z$, observation uncertainty $R$ and an observation operator $H$ that maps from state space to observation space. Then the measurement step involves calculating the innovation vector $y$ and innovation covariance $S$:

$$
y = z - H x^*
$$

$$
S = H P H^T + R
$$

With these we can calculate the Kalman Gain:

$$
K = P H^T S^{-1}
$$

which will allow us to obtain the final model state and uncertainty.

$$
\hat{x} = x^* + Ky
$$

$$
\hat{P} = (I - KH) P^*
$$

## Extended Kalman Filter

In a non-linear model like ours, we do not have linear $F$ and $H$. Instead we have non-linear $f$ and $h$. The Extended Kalman Filter tells us we can solve this problem by changing some of the previous equations to be:

$$
x^* = f(x)
$$

$$
y = z - h(x^*)
$$

and setting $F$ and $H$ for the rest of the equations to be:

$$
F = \frac{\partial f}{\partial x}
$$

$$
H = \frac{\partial h}{\partial x}
$$

Finally, due to numerical instability, the $\hat{P}$ matrix usually loses symmetry. To solve this, we replace it with the following after filtering:

$$
\frac{1}{2} (\hat{P} + \hat{P}^T)
$$

# Execution Loop

## Prediction step

For the prediction step we use the $\Delta t$ value between frames to project the state $x$ and uncertainty $P$. The state is projected setting $V^W = \Omega^W = 0$:

$$
x_v \rightarrow_{f_v}
    \begin{pmatrix}
    r^W_{\text{new}} \\
    q^{WR}_{\text{new}} \\
    v^W_{\text{new}} \\
    w^R_{\text{new}} \\
    \end{pmatrix}
    = \begin{pmatrix}
    r^W + v^W \Delta t \\
    q^{WR} \times q(w^W \Delta t) \\
    v^W \\
    w^W
    \end{pmatrix}
$$

$$
y_i \rightarrow_f y_i
$$

The uncertainty $P$ is projected using:

$$
P \rightarrow F P F^T + Q
$$

where $F$ is the jacobian of the state projection (also setting $V^W = \Omega^W = 0$ after deriving):

$$
F = \frac{\partial f}{\partial x} =
    \begin{pmatrix}
    I_{3 \times 3} & 0 & (\Delta t) I_{3 \times 3} & 0 & 0 \\
    0 & \frac{\partial q^{WR}_{\text{new}}}{\partial q^{WR}} & 0 & \frac{\partial q^{WR}_{\text{new}}}{\partial w^W} & 0 \\
    0 & 0 & I_{3 \times 3} & 0 & 0 \\
    0 & 0 & 0 & I_{3 \times 3} & 0 \\
    0 & 0 & 0 & 0 & I_{3n \times 3n}
    \end{pmatrix}
$$

and $Q$ is the noise in the model evolution given by the random acceleration impulse when projected along the model:

$$
Q = \begin{pmatrix}
    Q_v & 0 \\
    0 & 0
    \end{pmatrix}
$$

Notice that each feature is unaffected by the random acceleration impulse and therefore the noise related to each feature is zero. The noise related to the robot state $Q_v$ is then calculated as:

$$
Q_v = \frac{\partial f_v}{\partial \bold{n}} P_n \frac{\partial f_v}{\partial \bold{n}}^T
$$

$$
P_n = \begin{pmatrix}
      \sigma^2_{V^W} I_{3 \times 3} & 0 \\
      0 & \sigma^2_{\Omega^W} I_{3 \times 3}
      \end{pmatrix}
$$

$$
\frac{\partial f_v}{\partial \bold{n}} = \begin{pmatrix}
                                         (\Delta t) I_{3 \times 3} & 0 \\
                                         0 & \frac{\partial q^{WR}_{\text{new}}}{\partial \Omega^W} \\
                                         I_{3 \times 3} & 0 \\
                                         0 & I_{3 \times 3}
                                         \end{pmatrix}
$$

and it can be proven that:

$$
\frac{\partial q^{WR}_{\text{new}}}{\partial \Omega^W}|_x = \frac{\partial q^{WR}_{\text{new}}}{\partial w^W}|_x
$$

## Measurement step

For the measurement step we use the frame captured by the robot. First, we need to use the wide angle camera model to project map features in 3D space to the 2D image. Let $\begin{pmatrix} u_i & v_i \end{pmatrix}$ be the projection into 2D image space of the map feature $y_i$. Then:

$$
(x_v, y_i) \rightarrow y^R_i = q^{RW} \times (y_i - r^W) \rightarrow \begin{pmatrix} u_i & v_i \end{pmatrix}
$$

Therefore the Jacobian of the observation operator $H$ is:

$$
H = \frac{\partial h}{\partial x} = \begin{pmatrix}
                                    \frac{\partial \begin{pmatrix} u_1 & v_1 \end{pmatrix}}{\partial x_v} & \frac{\partial \begin{pmatrix} u_1 & v_1 \end{pmatrix}}{\partial y_1} & \dots & 0 \\
                                    \vdots & \vdots & \ddots & \vdots \\
                                    \frac{\partial \begin{pmatrix} u_n & v_n \end{pmatrix}}{\partial x_v} & 0 & \dots & \frac{\partial \begin{pmatrix} u_n & v_n \end{pmatrix}}{\partial y_n} \\
                                    \end{pmatrix}
$$

where:

$$
\frac{\partial \begin{pmatrix} u_i & v_i \end{pmatrix}}{\partial x_v} = \frac{\partial \begin{pmatrix} u_i & v_i \end{pmatrix}}{\partial y^R_i} \frac{\partial y^R_i}{\partial x_v}
$$

$$
\frac{\partial \begin{pmatrix} u_i & v_i \end{pmatrix}}{\partial y_i} = \frac{\partial \begin{pmatrix} u_i & v_i \end{pmatrix}}{\partial y^R_i} \frac{\partial y^R_i}{\partial y_i}
$$

$$
\frac{\partial y^R_i}{\partial x_v} = \begin{pmatrix}
                                      \frac{\partial y^R_i}{\partial r^W} & \frac{\partial y^R_i}{\partial q^{WR}} & 0 & 0
                                      \end{pmatrix}
$$

$$
\frac{\partial y^R_i}{\partial q^{WR}} = \frac{\partial y^R_i}{\partial q^{RW}} \frac{\partial q^{RW}}{\partial q^{WR}}
$$

$$
\frac{\partial q^{RW}}{\partial q^{WR}} =
\begin{pmatrix}
1 & 0 & 0 & 0 \\
0 & -1 & 0 & 0 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & -1
\end{pmatrix}
$$

Notice that if $RM(q)$ is the rotation matrix associated with the unit quaternion $q$ then:

$$
\frac{\partial y^R_i}{\partial y_i} = RM(q^{RW})
$$

$$
\frac{\partial y^R_i}{\partial r^W} = -RM(q^{RW})
$$

Second, we calculate the measurement noise covariance. We will assume that the errors in one measurement are uncorrelated with the errors in another, and that errors come mostly from discretization, and therefore $\sigma^2_R = 1$ and the measurement noise covariance matrix is $R = (\sigma^2_R) I_{2n \times 2n}$.

Finally we need to calculate the observations from the image. For this we perform an ellipse search using $3 \sigma$ to bound the search space. The ellipse will be centered on $\begin{pmatrix} u_i & v_i \end{pmatrix}$ and use the $2 \times 2$ symmetric innovation covariance matrix $S_i$ to define its shape, where $S_i$ is defined by $S$:

$$
S = H P H^T + R = \begin{pmatrix}
                  S_1 & \dots & \dots & \dots \\
                  \vdots & S_2 & \dots & \dots \\
                  \vdots & \vdots & \ddots & \dots \\
                  \vdots & \vdots & \vdots & S_n \\
                  \end{pmatrix}
$$

The ellipse search will try to find the position $z_i$ in the image whose surrounding patch maximizes the normalized cross correlation with respect to the template image. Then let:

$$
z = \begin{pmatrix}
    z_1 \\
    \vdots \\
    z_n
    \end{pmatrix}
$$

With all of this we can finally apply the Extended Kalman Filter:

$$
\phi = z - h(x)
$$

$$
K = P H^T S^{-1}
$$

$$
\hat{P} = (I - KH) P
$$

and project the state $x$ and uncertainty $P$:

$$
x \rightarrow x + K \phi
$$

$$
P \rightarrow \frac{1}{2} (\hat{P} + \hat{P}^T)
$$


## Feature detection step

FIXME:

## Redetection of partially initialized features

FIXME:

## Depth detection of partially initialized features

When a feature $\gamma$ is detected there is a high amount of uncertainty in its depth. This uncertainty can be reduced by using parallax in consecutives frames to better estimate the feature's depth. MonoSLAM does this by initializing the feature with a set of different depth hypothesis with equal probability (specifically there are 100 depth hypothesis between 0.5m and 5m).

Each feature is also initialized with state and uncertainty given by:

$$
y_\gamma = \begin{pmatrix}
      r^W_\gamma \\
      \hat{h}^W_\gamma
      \end{pmatrix}
$$

$$
P_{x,\gamma} = P_{x,x} \frac{\partial y_\gamma}{\partial x_v}^T
$$

$$
R_\gamma = \sigma^2_R I_{2 \times 2}
$$

$$
P_{\gamma,\gamma} = \frac{\partial y_\gamma}{\partial x_v} P_{x, x} \frac{\partial y_\gamma}{\partial x_v}^T + \frac{\partial y_\gamma}{\partial \gamma} R_\gamma \frac{\partial y_\gamma}{\partial \gamma}^T
$$

where $r^W_\gamma$ is the position of the robot in the world frame in which the feature was captured and $\hat{h}^W_\gamma$ is the unit vector describing the direction in which the feature was detected, given by the inversion of the camera model.

Given the transformation:

$$
(x_v, \gamma) \rightarrow h^R_\gamma = h^{-1}(\gamma) \rightarrow h^W_\gamma = q^{WR} \times h^R_\gamma \rightarrow \hat{h}^W_\gamma = \frac{h^W_\gamma}{|h^W_\gamma|}
$$

We can obtain the following derivatives:

$$
\frac{\partial y_\gamma}{\partial x_v} = \begin{pmatrix}
                                         I_{3 \times 3} & 0 & 0 & 0 \\
                                         0 & \frac{\partial \hat{h}^W_\gamma}{\partial q^{WR}} & 0 & 0
                                         \end{pmatrix}

$$

$$
\frac{\partial \hat{h}^W_\gamma}{\partial q^{WR}} = \frac{\partial \hat{h}^W_\gamma}{\partial h^W_\gamma} \frac{\partial h^W_\gamma}{\partial q^{WR}}
$$

$$
\frac{\partial y_\gamma}{\partial \gamma} = \begin{pmatrix}
                                            0 \\
                                            \frac{\partial \hat{h}^W_\gamma}{\partial \gamma}
                                            \end{pmatrix}
$$

$$
\frac{\partial \hat{h}^W_\gamma}{\partial \gamma} = \frac{\partial \hat{h}^W_\gamma}{\partial h^W_\gamma} \frac{\partial h^W_\gamma}{\partial h^R_\gamma} \frac{\partial h^R_\gamma}{\partial \gamma}
$$

The quaternion-times-vector derivative with respect to the vector gives:

$$
\frac{\partial h^W_\gamma}{\partial h^R_\gamma} = RM(q^{WR})
$$

And the jacobian of the normalization $\frac{\partial \hat{h}^W_\gamma}{\partial h^W_\gamma}$ is given by:

$$
\frac{\partial \hat{v}}{\partial v} = \frac{1}{|v|^3}
\begin{pmatrix}
y^2 + z^2 & -xy & -xz \\
-xy & x^2 + z^2 & -yz \\
-xz & -yz & x^2 + y^2
\end{pmatrix}
$$

Finally, $\frac{\partial h^R_\gamma}{\partial \gamma}$ is given by the jacobian of the inversion of the camera model and $\frac{\partial h^W_\gamma}{\partial q^{WR}}$ is given by the quaternion-times-vector derivative with respect to the quaternion.

In every consecutive frame each depth hypothesis $\lambda$ is projected unto the image to generate $\mu_\lambda$. For each depth hypothesis an innovation covariance matrix $S_\lambda$ is calculated:

$$
\begin{aligned}
S_\lambda &= \frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial x_v} P_{x,x} \frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial x_v}^T \\
&+ \frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial x_v} P_{x,\gamma} \frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial y_\gamma}^T \\
&+ \frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial y_\gamma} P_{\gamma,x} \frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial x_v}^T \\
&+ \frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial y_\gamma} P_{\gamma, \gamma} \frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial y_\gamma}^T \\
&+ R_\lambda
\end{aligned}
$$

where $R_\lambda = \sigma^2_R I_{2 \times 2}$ and the transformation:

$$
(x_v, y_\gamma) \rightarrow (x_v, y_\lambda) = (x_v, r^W_\gamma + \lambda \hat{h}^W_\gamma) \rightarrow y^R_\lambda = q^{RW} \times (y_\lambda - r^W) \rightarrow \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix} = h(y^R_\lambda)
$$

gives the following derivatives:

$$
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial x_v} =
\begin{pmatrix}
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial r^W} &
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial q^{WR}} &
0 &
0
\end{pmatrix}
$$

$$
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial y_\gamma} =
\begin{pmatrix}
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial r^W_\gamma} &
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial \hat{h}^W_\gamma}
\end{pmatrix}
$$

$$
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial r^W} =
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial y^R_\lambda}
\frac{\partial y^R_\lambda}{\partial r^W}
$$

$$
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial q^{WR}} =
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial y^R_\lambda}
\frac{\partial y^R_\lambda}{\partial q^{RW}}
\frac{\partial q^{RW}}{\partial q^{WR}}
$$

$$
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial r^W_\gamma} =
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial y^R_\lambda}
\frac{\partial y^R_\lambda}{\partial y_\lambda}
\frac{\partial y_\lambda}{\partial r^W_\gamma}
$$

$$
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial \hat{h}^W_\gamma} =
\frac{\partial \begin{pmatrix} u_\lambda & v_\lambda \end{pmatrix}}{\partial y^R_\lambda}
\frac{\partial y^R_\lambda}{\partial y_\lambda}
\frac{\partial y_\lambda}{\partial \hat{h}^W_\gamma}
$$

And the following can be simplified further to:

$$
\frac{\partial y^R_\lambda}{\partial r^W} = -RM(q^{RW})
$$

$$
\frac{\partial y^R_\lambda}{\partial y_\lambda} = RM(q^{RW})
$$

$$
\frac{\partial y_\lambda}{\partial r^W_\gamma} = I_{3 \times 3}
$$

$$
\frac{\partial y_\lambda}{\partial \hat{h}^W_\gamma} = \lambda I_{3 \times 3}
$$

Both the projection $\mu_\lambda$ and the covariance $S_\lambda$ are used to perform an ellipse search for the most correlated patch in the incoming frame, with respect to the patch in the first image where the feature was detected. Suppose the feature is detected at position $x_\lambda$. Then the likelihood of this feature is:

$$
\mathcal{L}(x_\lambda|\mu_\lambda, S_\lambda) = \frac{1}{2 \pi |S_\lambda|^{1/2}} e^{-\frac{1}{2} (x_\lambda - \mu_\lambda)^T S^{-1}_\lambda (x_\lambda - \mu_\lambda)}
$$

The depth hypothesis then has it's probability updated by:

$$
P(\lambda) \rightarrow P(\lambda) * \mathcal{L}(x_\lambda|\mu_\lambda, S_\lambda)
$$

Finally, the depth hypotheses with very low probability are dropped to speed up convergence and the probabilities of the remaining depth hypotheses are normalized. Once the ratio of the standard deviation with respect to the mean of all these particles is below a certain threshold we can initialize the feature.

## Converting partially initialized features to fully initialized features

FIXME:
