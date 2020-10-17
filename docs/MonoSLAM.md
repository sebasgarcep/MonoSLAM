# MonoSLAM

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

## Normalized Cross-Correlation

Given a template window $T$ and an image window $I$ with equal dimensions, the normalized cross correlation is given by:

$$
NCC(T, I) = \frac{\sum_{x,y} (T_{x,y} - \mu_T) (I_{x,y} - \mu_I)}{\sqrt{(\sum_{x,y} (T_{x,y} - \mu_T)^2)(\sum_{x,y} (I_{x,y} - \mu_I)^2)}}
$$

where $\mu_T$ and $\mu_I$ are the means of each respective patch.

## Ellipse search

FIXME: talk about this

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

The ellipse search will try to maximize the normalized cross correlation of the template image associated with the feature with respect to all image patches in the search area.

## Feature selection step

FIXME: Write about the Shi-Tomasi Operator, partially initialized features, etc...
