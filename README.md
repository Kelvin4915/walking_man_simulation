# humanoidBot

The purpose of this code is to simulate and validate the Linear Quadratic Regulator (LQR), an optimal control algorithm.

https://github.com/Harshit0803/humanoidBot/assets/29999543/0bf24bb9-a7f4-479d-97bb-67f0d2c5bd8c




## Objectives:
* The task will involve utilizing the walking action of a human.
* Control of the motion of the Center of Mass (CoM) will be achieved through the use of the Center of Pressure (CoP) and a regulator.
* The model will be subject to simulation under conditions of both fixed and variable velocities for the center of mass (CoM).
* External forces will be applied to the system to validate the precision and accuracy of the Linear Quadratic Regulator (LQR).

## Assumptions made:
* The first assumption stipulates that the lengths of the legs remain constant and exhibit no variation, resulting in a consistent g/L ratio.
* The second assumption involves the observation that the angle formed by the line intersecting the Centers of Pressure (CoP) and Center of Mass (CoM) with the vertical axis is exceedingly small. Consequently, it is appropriate to consider the sine of this angle as approximately equal to the angle itself.

## Linear Inverted Pendulum Model (LIPM)

![image](https://github.com/Harshit0803/humanoidBot/assets/29999543/b4a713e2-5cce-49ef-bcbc-46d9200d098d)


$$
a = \omega * (x_{com} - x_{cop})
$$

$$\frac{d}{dt} \begin{bmatrix}
    x_{n+1}\\
    \dot{x_{n+1}}\\
\end{bmatrix} = \begin{bmatrix}
    x_{n}\\
    \dot{x_{n}}\\
\end{bmatrix} + \begin{bmatrix}
    \dot{x_{n}}\\
    \omega(x_{com} - x_{cop})\\
\end{bmatrix} \Delta t $$

$$ \begin{bmatrix}
    \dot{x_{n+1}}\\
    \ddot{x_{n+1}}\\
    \end{bmatrix} = \begin{bmatrix}
                    1\  \quad \Delta t\\
                    \omega\Delta t\  \quad   1\\
                    \end{bmatrix} \begin{bmatrix}
                    x_n\\
                    \dot{x_n}
                    \end{bmatrix} + \begin{bmatrix}
                    0\\
                    \-\omega \Delta t\\
                    \end{bmatrix} x_{cop}
                    $$

$$ x_{n+1} = A_nx_n + B_nu_n $$

## Linear Quadratic Regulator (LQR)

$$
P_N = Q_N 
$$

$$
p_N = q_N 
$$

$$
K_n = -(R_n + B_n^TP_{n+1}B_n)^{-1}B_n^TP_{n+1}A_n 
$$

$$
P_n = Q_n + A_n^TP_{n+1}A_n + A_n^TP_{n+1}B_nK_n 
$$

$$
k_n = -(R_n + B_n^TP_{n+1}B_n)^{-1}B_n^Tp_{n+1}
$$

$$
p_n = q_n + A_n^Tp_{n+1} + A_n^TP_{n+1}B_nk_n
$$

$$
\mu_n^*(x_n) = K_nx_n + k_n
$$

$$
min½x_N^TQ_Nx_N + q_N^Tx_N + \sum_{n=0}^{N-1}½x_N^TQ_Nx_N + q_N^Tx_N + ½u_nR_nu_n
$$
