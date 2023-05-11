import math
import matplotlib.pyplot as plt
import numpy as np

def plotTrajAndBound(data):
  x, y, phi, delta, v, left_bound, right_bound, front_bound, back_bound = data
  indexes = list(range(1, len(x) + 1))
  
  fig = plt.figure()
  ax1 = fig.add_subplot(2,2,1)
  ax1.plot(x, y)
  ax1.set_title("xy")
  ax2 = fig.add_subplot(2,4,3)
  ax2.plot(indexes, phi)
  ax2.set_title("phi")
  ax3 = fig.add_subplot(2,4,4)
  ax3.plot(indexes, v)
  ax3.set_title("v")
  
  ax4 = fig.add_subplot(2,4,5)
  ax4.plot(indexes, left_bound)
  ax4.set_title("left_bound")
  ax5 = fig.add_subplot(2,4,6)
  ax5.plot(indexes, right_bound)
  ax5.set_title("right_bound")
  ax6 = fig.add_subplot(2,4,7)
  ax6.plot(indexes, front_bound)
  ax6.set_title("front_bound")
  ax7 = fig.add_subplot(2,4,8)
  ax7.plot(indexes, back_bound)
  ax7.set_title("back_bound")
  
  plt.show()
  return 0

def GetSideCollisionCoeff(xn, yn, x0, y0, phi, slack_max, d_safe, L):
    c1 = xn
    c2 = yn
    c3 = yn * L * math.cos(phi) - xn * L * math.sin(phi)
    d = -slack_max - d_safe + x0 * xn + y0 * yn - L * (math.cos(phi) + phi * math.sin(phi)) * xn - \
        L * (math.sin(phi) - phi * math.cos(phi)) * yn
    return (c1, c2, c3, 0.0, 0.0,   0.0, 0.0, -1.0, 0.0,   d)

def GetFrontCollisionCoeff(xn, yn, x0, y0, phi, slack_max, d_safe, L, W):
    c1 = xn
    c2 = yn
    c3 = yn * L * math.cos(phi) - xn * L * math.sin(phi) - W / 2.0
    d = -slack_max - d_safe + x0 * xn + y0 * yn - L * (math.cos(phi) + phi * math.sin(phi)) * xn - \
        L * (math.sin(phi) - phi * math.cos(phi)) * yn - phi * W / 2.0
    return (c1, c2, c3, 0.0, 0.0,   0.0, 0.0, 0.0, -1.0,   d)

def GetBackCollisionCoeff(xn, yn, x0, y0, phi, slack_max, d_safe, L, W):
    c1 = xn
    c2 = yn
    c3 = yn * L * math.cos(phi) - xn * L * math.sin(phi) + W / 2.0
    d = -slack_max - d_safe + x0 * xn + y0 * yn - L * (math.cos(phi) + phi * math.sin(phi)) * xn - \
        L * (math.sin(phi) - phi * math.cos(phi)) * yn + phi * W / 2.0
    return (c1, c2, c3, 0.0, 0.0,   0.0, 0.0, 0.0, -1.0,   d)

def GetCollisionCoeff(params, x, y, phi, left_bound, right_bound, front_bound, back_bound):
  vehicleLength = params.vehicleLength
  vehicleWidth = params.vehicleWidth
  wheelBase = params.wheelBase
  Lf = params.Lf
  Lr = params.Lr
  slack_max = params.slack_max
  d_safe_side = params.d_safe_side
  d_safe_front = params.d_safe_front

  dataLength = len(x)
  collisionCoeff = np.zeros((8 * dataLength, 10))
  for i in range(dataLength):
    # for left side
    xn = math.cos(phi[i] + math.pi / 2.0)
    yn = math.sin(phi[i] + math.pi / 2.0) 
    xv =  0
    yv = left_bound[i]   
    x0 = x[i] + xv * math.cos(phi[i]) - yv * math.sin(phi[i])
    y0 = y[i] + xv * math.sin(phi[i]) + yv * math.cos(phi[i])


    collisionCoeff[8 * i + 0, 0:10] = GetSideCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_side, Lf)
    collisionCoeff[8 * i + 1, 0:10] = GetSideCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_side, -Lr)

    # for right side
    xn = math.cos(phi[i] - math.pi / 2.0)
    yn = math.sin(phi[i] - math.pi / 2.0)
    xv =  0
    yv = -right_bound[i]   
    x0 = x[i] + xv * math.cos(phi[i]) - yv * math.sin(phi[i])
    y0 = y[i] + xv * math.sin(phi[i]) + yv * math.cos(phi[i])
    collisionCoeff[8 * i + 2, 0:10] = GetSideCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_side, Lf)
    collisionCoeff[8 * i + 3, 0:10] = GetSideCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_side, -Lr)

    # for front side
    xn = math.cos(phi[i])
    yn = math.sin(phi[i])
    xv = front_bound[i]
    yv = 0   
    x0 = x[i] + xv * math.cos(phi[i]) - yv * math.sin(phi[i])
    y0 = y[i] + xv * math.sin(phi[i]) + yv * math.cos(phi[i])
    collisionCoeff[8 * i + 4, 0:10] = GetFrontCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_front, Lf, vehicleWidth)
    collisionCoeff[8 * i + 5, 0:10] = GetBackCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_front, Lf, vehicleWidth)

    # for rear side
    xn = math.cos(phi[i] - math.pi)
    yn = math.sin(phi[i] - math.pi)
    xv = -back_bound[i]
    yv = 0   
    x0 = x[i] + xv * math.cos(phi[i]) - yv * math.sin(phi[i])
    y0 = y[i] + xv * math.sin(phi[i]) + yv * math.cos(phi[i])
    collisionCoeff[8 * i + 6, 0:10] = GetFrontCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_front, -Lr, vehicleWidth)
    collisionCoeff[8 * i + 7, 0:10] = GetBackCollisionCoeff(xn, yn, x0, y0, phi[i], slack_max, d_safe_front, -Lr, vehicleWidth)

  return collisionCoeff


if __name__ == "__main__":
  GetCollisionCoeff()

  data = loadData()
  plotTrajAndBound(data)
  
  print("done")
