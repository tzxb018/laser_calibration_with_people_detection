# from sklearn.linear_model import LinearRegression
# import numpy
#
# if __name__ == '__main__':
#     time = [21.1029, 21.1029, 21.8945, 21.8945, 22.5883, 22.5883, 22.5883, 23.3749, 24.7016, 25.407, 26.1125, 26.1125, 26.6596, 26.6596,27.519]
#     pos_x = [-0.640490, -1.023339, -1.285145, -1.387116, -1.987243, -2.462033, -2.011330, -2.607346, -4.212203, -4.229677, -4.888209, -4.826655, -5.112056, -5.094664, -5.297697]
#
#     pos_x, time = numpy.array(pos_x), numpy.array(time).reshape(-1,1)
#     # pos_x = numpy.array(pos_x).reshape((-1, 1))
#     # pos_y = numpy.array(pos_y).reshape((-1, 1))
#     # time = numpy.array(time).reshape((-1,1))
#     # print(pos_x, pos_y, time)
#
#     model_x = LinearRegression().fit(time, pos_x)
#     x_pred = []
#     intercept = model_x.intercept_
#     coef = model_x.coef_
#     for extra in range(0,10):
#         x_pred.append(intercept + coef * (max(time) + extra/5.0))
#     print(x_pred)
#
#     # print('before send', linear_fit)


import operator

import numpy as np
import matplotlib.pyplot as plt

from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.preprocessing import PolynomialFeatures

x = [1564098026.82,1564098026.82, 1564098027.29, 1564098027.29, 1564098028.3, 1564098028.3, 1564098028.93,1564098029.47,1564098029.47,1564098031.98,1564098031.98,1564098033.31,1564098033.31, 1564098033.31, 1564098033.31]
# x = [26.82,26.82, 27.29, 27.29, 28.3, 28.3, 28.93, 29.47, 29.47,31.98,31.98,33.31,33.31, 33.31, 33.31]

y = [-5.30037327242, -5.0595223615, -4.90047014838, -4.82562474019, -4.72777952235, -4.2429237249, -4.23223193991, -2.88752080565, -2.89226034559,
-1.03743786879,
-0.446104663531,
-0.496198200126,
-0.467476159668,
0.203822559591,
0.364401182103]

a = min(x)
for i in range(0,len(x)):
    x[i] = (x[i] - a)
print(x)
print(y)

x, y = np.array(x).reshape((-1,1)), np.array(y)
polynomial_features= PolynomialFeatures(degree=3)
x_poly = polynomial_features.fit_transform(x)

model = LinearRegression()
model.fit(x_poly, y)
y_poly_pred = model.predict(x_poly)

print(model.coef_)
print(model.intercept_)
print(model.predict(polynomial_features.fit_transform(11.0)))
print(model.predict(polynomial_features.fit_transform(5.0)))
plt.scatter(x, y, s=10)
# sort the values of x before line plot
sort_axis = operator.itemgetter(0)
sorted_zip = sorted(zip(x,y_poly_pred), key=sort_axis)
x, y_poly_pred = zip(*sorted_zip)
plt.plot(x, y_poly_pred, color='m')
plt.show()