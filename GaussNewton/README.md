# Gauss-Newton Curve Fitting
## The Gauss-Newton Method

Use a simple example to demonstrate how to solve the least-square problem. We will demonstrate how to write the Gauss-Newton method by hand and then introduce how to use the optimization library to solve this problem. 

Consider a curve that satisfies the following equation:


$$
y = \exp( ax^2 + bx + c ) + w
$$

Suppose we have N observation data points about x and y and want to find the parameters of the curve based on these data points.

Then, we solve the following least-square problem to estimate the curve parameters:

$$
    \min \limits_{a,b,c} \frac{1}{2}\sum\limits_{i = 1}^N {{{\left\| {{y_i} - \exp \left( {ax_i^2 + bx_i + c} \right)} \right\|}^2}} .
$$

Define the error as:

$$
e_i = y_i - \exp \left( {ax_i^2 + bx_i + c} \right),
$$

Then we can find the derivative of each error term with respect to the state variable:

$$
    \begin{aligned}
        \frac{{\partial {e_i}}}{{\partial a}} &=  - x_i^2\exp \left( {ax_i^2 + b{x_i} + c} \right)\\
        \frac{{\partial e_i}}{{\partial b}} &=  - {x_i}\exp \left( {ax_i^2 + b{x_i} + c} \right)\\
        \frac{{\partial {e_i}}}{{\partial c}} &=  - \exp \left( {ax_i^2 + b{x_i} + c} \right)
    \end{aligned}
$$

So 
$` \mathbf{J}_i = \left[\frac{{\partial {e_i}}}{{\partial a}},\frac{{\partial {e_i}}}{{\partial b}},\frac{{\partial {e_i}}}{{\partial c}} \right]^T `$, 

and the normal equation of the Gauss-Newton method is:

$$
    \left(\sum\limits_{i = 1}^{100} {\mathbf{J}_i{(\sigma^2)^{ - 1}}{\mathbf{J}_i}}^T \right)  \Delta \mathbf{x}_k = \sum\limits_{i=1}^{100} { - {\mathbf{J}_i}{(\sigma^2)^{ - 1}}{e_i}}
$$

Result:

```
total cost: 3.19575e+06, 		update: 0.0455771  0.078164 -0.985329		estimated params: 2.04558,-0.921836,4.01467
total cost: 376785, 		update:  0.065762  0.224972 -0.962521		estimated params: 2.11134,-0.696864,3.05215
total cost: 35673.6, 		update: -0.0670241   0.617616  -0.907497		estimated params: 2.04432,-0.0792484,2.14465
total cost: 2195.01, 		update: -0.522767   1.19192 -0.756452		estimated params: 1.52155,1.11267,1.3882
total cost: 174.853, 		update: -0.537502  0.909933 -0.386395		estimated params: 0.984045,2.0226,1.00181
total cost: 102.78, 		update: -0.0919666   0.147331 -0.0573675		estimated params: 0.892079,2.16994,0.944438
total cost: 101.937, 		update: -0.00117081  0.00196749 -0.00081055		estimated params: 0.890908,2.1719,0.943628
total cost: 101.937, 		update:   3.4312e-06 -4.28555e-06  1.08348e-06		estimated params: 0.890912,2.1719,0.943629
total cost: 101.937, 		update: -2.01204e-08  2.68928e-08 -7.86602e-09		estimated params: 0.890912,2.1719,0.943629
cost: 101.937>= last cost: 101.937, break.
solve time cost = 0.000212903 seconds.
estimated abc = 0.890912, 2.1719, 0.943629
```

