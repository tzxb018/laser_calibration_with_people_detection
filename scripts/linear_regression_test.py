from sklearn.linear_model import LinearRegression
import numpy

if __name__ == '__main__':
    time = [21.1029, 21.1029, 21.8945, 21.8945, 22.5883, 22.5883, 22.5883, 23.3749, 24.7016, 25.407, 26.1125, 26.1125, 26.6596, 26.6596,27.519]
    pos_x = [-0.640490, -1.023339, -1.285145, -1.387116, -1.987243, -2.462033, -2.011330, -2.607346, -4.212203, -4.229677, -4.888209, -4.826655, -5.112056, -5.094664, -5.297697]

    pos_x, time = numpy.array(pos_x), numpy.array(time).reshape(-1,1)
    # pos_x = numpy.array(pos_x).reshape((-1, 1))
    # pos_y = numpy.array(pos_y).reshape((-1, 1))
    # time = numpy.array(time).reshape((-1,1))
    # print(pos_x, pos_y, time)

    model_x = LinearRegression().fit(time, pos_x)
    x_pred = []
    intercept = model_x.intercept_
    coef = model_x.coef_
    for extra in range(0,10):
        x_pred.append(intercept + coef * (max(time) + extra/5.0))
    print(x_pred)

    # print('before send', linear_fit)