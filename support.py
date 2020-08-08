# -*- coding: utf-8 -*-
"""
Created on Sat Apr  4 11:11:39 2020

@author: 34086
"""

import numpy as np
pi=np.pi

#高斯投影正算函数，返回高斯坐标:x,y
def LatLon2XY(latitude, longitude):
    a = 6378137.0
    # b = 6356752.3142
    # c = 6399593.6258
    # alpha = 1 / 298.257223563
    e2 = 0.0066943799013
    # epep = 0.00673949674227


    #将经纬度转换为弧度
    latitude2Rad = (pi / 180.0) * latitude

    beltNo = int((longitude + 1.5) / 3.0) #计算3度带投影度带号
    
    L = beltNo * 3 #计算中央经线
    
    l0 = longitude - L #经差
    tsin = np.sin(latitude2Rad)
    tcos = np.cos(latitude2Rad)
    t = np.tan(latitude2Rad)
    m = (np.pi / 180.0) * l0 * tcos
    et2 = e2 * pow(tcos, 2)
    et3 = e2 * pow(tsin, 2)
    X = 111132.9558 * latitude - 16038.6496 * np.sin(2 * latitude2Rad) + 16.8607 * np.sin(
        4 * latitude2Rad) - 0.0220 * np.sin(6 * latitude2Rad)
    N = a / np.sqrt(1 - et3)

    x = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (
    61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0)
    y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (
    5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0)

    return x, y

#高斯投影反算函数，返回大地坐标B,L
def XY2LatLon(X, Y, L0):

    iPI = 0.0174532925199433
    a = 6378137.0
    f= 0.00335281006247
    ZoneWide = 3 #按3度带进行投影

    ProjNo = int(Y / 1000000)
    L0 = L0 * iPI
    Y0 = ProjNo * 1000000 + 500000
    X0 = 0
    xval = X - X0
    yval = Y - Y0

    e2 = 2 * f - f * f #第一偏心率平方
    e1 = (1.0 - np.sqrt(1 - e2)) / (1.0 + np.sqrt(1 - e2))
    ee = e2 / (1 - e2) #第二偏心率平方

    M = xval
    u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256))

    fai = u \
          + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * np.sin(2 * u) \
          + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * np.sin(4 * u) \
          + (151 * e1 * e1 * e1 / 96) * np.sin(6 * u)\
          + (1097 * e1 * e1 * e1 * e1 / 512) * np.sin(8 * u)
    C = ee * np.cos(fai) * np.cos(fai)
    T = np.tan(fai) * np.tan(fai)
    NN = a / np.sqrt(1.0 - e2 * np.sin(fai) * np.sin(fai))
    R = a * (1 - e2) / np.sqrt(
        (1 - e2 * np.sin(fai) * np.sin(fai)) * (1 - e2 * np.sin(fai) * np.sin(fai)) * (1 - e2 * np.sin(fai) * np.sin(fai)))
    D = yval / NN

    #计算经纬度（弧度单位的经纬度）
    longitude1 = L0 + (D - (1 + 2 * T + C) * D * D * D / 6 + (
    5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D * D * D * D * D / 120) / np.cos(fai)
    latitude1 = fai - (NN * np.tan(fai) / R) * (
    D * D / 2 - (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24 + (
    61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) * D * D * D * D * D * D / 720)

    #换换为deg
    longitude = longitude1 / iPI
    latitude = latitude1 / iPI

    return latitude, longitude


if __name__=='__main__':
    print('this is support!')
