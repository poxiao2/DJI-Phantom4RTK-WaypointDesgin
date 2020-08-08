# -*- coding: utf-8 -*-
"""
Created on Fri Apr  3 18:11:03 2020

@author: 34086
"""

import os
import numpy as np
import support
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import xml.dom.minidom
import sys  #引入模块

dataPath='./data'         #存放桥墩顶面中心点坐标的文件路径
wayPointsPath='./KML'     #生成的KML航点文件存放路径
objectDistance=5          #无人机相机与桥墩顶面中心点的距离
homeAltitude=0            #无人机起飞点绝对高度，用于后面计算目标与起飞点之间的相对高度
wayPointsNumber=19        #所要规划的航点数量
pi=np.pi


def init():
    global objectDistance
    global homeAltitude
    global wayPointsNumber
    objectDistance=np.float32(sys.argv[1])   #无人机相机与桥墩顶面中心点的距离
    homeAltitude=np.float64(sys.argv[2])     #无人机起飞点绝对高度，用于后面计算目标与起飞点之间的相对高度
    wayPointsNumber=np.int(sys.argv[3])      #无人机规划的航点数量
    print("init done!")
    
#绘制3d航点及航线的函数
def draw3dWayPoints(wayPointsCoords):
    ax = plt.axes(projection='3d')    
    data=np.array(wayPointsCoords)
    # 三维散点的数据    
    xdata = data[:,0]
    ydata = data[:,1]
    zdata = data[:,2]
    ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Reds')
    ax.plot3D(xdata,ydata,zdata,'gray')
#    plt.show()
    
#读取桥墩顶面中心点坐标的函数，返回一个存放坐标点的列表
def readObjCoords(path):
    coords=[]
    reader=open(path,'r')
    for line in reader.readlines():
        item=line.split(',')
        coords.append([item[0],np.float64(item[1]),np.float64(item[2]),np.float64(item[3])-homeAltitude])
    reader.close()
    return coords

#计算航点坐标及相机姿态的函数，返回一个存放19个杭点坐标及姿态的列表
def computeWayPointsCoords_19(coord):
    L=coord[1]
    B=coord[2]
    x0,y0=support.LatLon2XY(B,L)
    z0=coord[3]
    wayPointsCoords=[]
    sita_v=0
    sita_h=0
    L0=0
    if abs(L-int(L/3)*3)<=1.5:
        L0=int(L/3)*3
    else: 
        L0=int(L/3+1)*3
    
    for i in range(8):
        sita_h=pi/4*i
        x=x0+objectDistance*np.cos(sita_v)*np.cos(sita_h)
        y=y0+objectDistance*np.cos(sita_v)*np.sin(sita_h)
        z=z0+objectDistance*np.sin(sita_v)
        
        heading=0
        if sita_h>pi:
            heading=sita_h*180.0/pi-360
        else:
            heading=sita_h*180.0/pi
            
        if heading>0:
            heading=heading-180
        else:
            heading=heading+180
        
        gimbalPitch=-sita_v*180.0/pi
        B,L=support.XY2LatLon(x,y,L0)
        wayPointsCoords.append([L,B,z,heading,gimbalPitch])
            
    sita_v=pi/6
    sita_h=0
    
    for i in range(6):
        sita_h=pi/3*i
        x=x0+objectDistance*np.cos(sita_v)*np.cos(sita_h)
        y=y0+objectDistance*np.cos(sita_v)*np.sin(sita_h)
        z=z0+objectDistance*np.sin(sita_v)
        
        heading=0
        if sita_h>pi:
            heading=sita_h*180.0/pi-360
        else:
            heading=sita_h*180.0/pi
        
        if heading>0:
            heading=heading-180
        else:
            heading=heading+180
        
        gimbalPitch=-sita_v*180.0/pi
        B,L=support.XY2LatLon(x,y,L0)
        wayPointsCoords.append([L,B,z,heading,gimbalPitch])
    
    sita_v=pi/3
    sita_h=0
    
    for i in range(4):
        sita_h=pi/2*i
        x=x0+objectDistance*np.cos(sita_v)*np.cos(sita_h)
        y=y0+objectDistance*np.cos(sita_v)*np.sin(sita_h)
        z=z0+objectDistance*np.sin(sita_v)
        
        heading=0
        if sita_h>pi:
            heading=sita_h*180.0/pi-360
        else:
            heading=sita_h*180.0/pi
        
        if heading>0:
            heading=heading-180
        else:
            heading=heading+180
        
        gimbalPitch=-sita_v*180.0/pi
        B,L=support.XY2LatLon(x,y,L0)
        wayPointsCoords.append([L,B,z,heading,gimbalPitch])
    
    sita_v=pi/2
    sita_h=0
       
    x=x0+objectDistance*np.cos(sita_v)*np.cos(sita_h)
    y=y0+objectDistance*np.cos(sita_v)*np.sin(sita_h)
    z=z0+objectDistance*np.sin(sita_v)
    gimbalPitch=-sita_v*180.0/pi
    B,L=support.XY2LatLon(x,y,L0)
    wayPointsCoords.append([L,B,z,heading,gimbalPitch])
    print("----------waypoints coords:--------------")
    for p in wayPointsCoords:
        print(p)	
    print("-----------------------------------------")
    return wayPointsCoords

#计算航点坐标及相机姿态的函数，返回一个存放27个杭点坐标及姿态的列表
def computeWayPointsCoords_27(coord):
    L=coord[1]
    B=coord[2]
    x0,y0=support.LatLon2XY(B,L)
    z0=coord[3]
    wayPointsCoords=[]
    sita_v=0
    sita_h=0
    L0=0
    if abs(L-int(L/3)*3)<=1.5:
        L0=int(L/3)*3
    else: 
        L0=int(L/3+1)*3
    
    for i in range(12):
        sita_h=pi/6*i
        x=x0+objectDistance*np.cos(sita_v)*np.cos(sita_h)
        y=y0+objectDistance*np.cos(sita_v)*np.sin(sita_h)
        z=z0+objectDistance*np.sin(sita_v)
        
        heading=0
        if sita_h>pi:
            heading=sita_h*180.0/pi-360
        else:
            heading=sita_h*180.0/pi
            
        if heading>0:
            heading=heading-180
        else:
            heading=heading+180
        
        gimbalPitch=-sita_v*180.0/pi
        B,L=support.XY2LatLon(x,y,L0)
        wayPointsCoords.append([L,B,z,heading,gimbalPitch])
            
    sita_v=pi/6
    sita_h=0
    
    for i in range(8):
        sita_h=pi/4*i
        x=x0+objectDistance*np.cos(sita_v)*np.cos(sita_h)
        y=y0+objectDistance*np.cos(sita_v)*np.sin(sita_h)
        z=z0+objectDistance*np.sin(sita_v)
        
        heading=0
        if sita_h>pi:
            heading=sita_h*180.0/pi-360
        else:
            heading=sita_h*180.0/pi
        
        if heading>0:
            heading=heading-180
        else:
            heading=heading+180
        
        gimbalPitch=-sita_v*180.0/pi
        B,L=support.XY2LatLon(x,y,L0)
        wayPointsCoords.append([L,B,z,heading,gimbalPitch])
    
    sita_v=pi/3
    sita_h=0
    
    for i in range(6):
        sita_h=pi/3*i
        x=x0+objectDistance*np.cos(sita_v)*np.cos(sita_h)
        y=y0+objectDistance*np.cos(sita_v)*np.sin(sita_h)
        z=z0+objectDistance*np.sin(sita_v)
        
        heading=0
        if sita_h>pi:
            heading=sita_h*180.0/pi-360
        else:
            heading=sita_h*180.0/pi
        
        if heading>0:
            heading=heading-180
        else:
            heading=heading+180
        
        gimbalPitch=-sita_v*180.0/pi
        B,L=support.XY2LatLon(x,y,L0)
        wayPointsCoords.append([L,B,z,heading,gimbalPitch])
    
    sita_v=pi/2
    sita_h=0
       
    x=x0+objectDistance*np.cos(sita_v)*np.cos(sita_h)
    y=y0+objectDistance*np.cos(sita_v)*np.sin(sita_h)
    z=z0+objectDistance*np.sin(sita_v)
    gimbalPitch=-sita_v*180.0/pi
    B,L=support.XY2LatLon(x,y,L0)
    wayPointsCoords.append([L,B,z,heading,gimbalPitch])

    print("----------waypoints coords:--------------")
    for p in wayPointsCoords:
        print(p)	
    print("-----------------------------------------")
    return wayPointsCoords

#将规划好的航点写入到KML文件中
def makeKmlFile(wayPointsCoords,filename):
    #在内存中创建一个空的文档
    doc = xml.dom.minidom.Document() 
    #创建一个根节点Managers对象
    kml = doc.createElement('kml') 
    #设置根节点的属性
    kml.setAttribute('xmlns', 'http://www.opengis.net/kml/2.2') 
    #将根节点添加到文档对象中 
    Document=doc.createElement('Document')
    Name=doc.createElement('name')
    Name.appendChild(doc.createTextNode(filename))
    Open=doc.createElement('open')
    Open.appendChild(doc.createTextNode('1'))
    ExtendedData=doc.createElement('ExtendedData')
    ExtendedData.setAttribute('xmlns:mis','www.dji.com')
    mis_type=doc.createElement('mis:type')
    mis_type.appendChild(doc.createTextNode('Waypoint'))
    ExtendedData.appendChild(mis_type)
    
    StyleLine=doc.createElement('Style')
    StyleLine.setAttribute('id','waylineGreenPoly')
    LineStyle=doc.createElement('LineStyle')
    Color=doc.createElement('color')
    Color.appendChild(doc.createTextNode('FF0AEE8B'))
    Width=doc.createElement('width')
    Width.appendChild(doc.createTextNode('6'))
    LineStyle.appendChild(Color)
    LineStyle.appendChild(Width)
    StyleLine.appendChild(LineStyle)
    
    StylePoint=doc.createElement('Style')
    StylePoint.setAttribute('id','waypointStyle')
    IconStyle=doc.createElement('IconStyle')
    Icon=doc.createElement('Icon')
    Href=doc.createElement('href')
    Href.appendChild(doc.createTextNode('https://cdnen.dji-flighthub.com/static/app/images/point.png'))
    Icon.appendChild(Href)
    IconStyle.appendChild(Icon)
    StylePoint.appendChild(IconStyle)
    
    Document.appendChild(Name)
    Document.appendChild(Open)
    Document.appendChild(ExtendedData)
    Document.appendChild(StyleLine)
    Document.appendChild(StylePoint)
    
    Folder=doc.createElement('Folder')
    Name=doc.createElement('name')
    Name.appendChild(doc.createTextNode('Waypoints'))
    Description=doc.createElement('description')
    Description.appendChild(doc.createTextNode('Waypoints in the Mission'))
    Folder.appendChild(Name)
    Folder.appendChild(Description)    
    
    counter=0
    for point in wayPointsCoords:      
        counter=counter+1
        Placemark=doc.createElement('Placemark')
        Name=doc.createElement('name')
        Name.appendChild(doc.createTextNode('Waypoint'+str(counter)))
        Visibility=doc.createElement('visibility')
        Visibility.appendChild(doc.createTextNode('1'))
        Description=doc.createElement('description')
        Description.appendChild(doc.createTextNode('Waypoint'))
        StyleUrl=doc.createElement('styleUrl')
        StyleUrl.appendChild(doc.createTextNode('#waypointStyle'))
        ExtendedData=doc.createElement('ExtendedData')
        ExtendedData.setAttribute('xmlns:mis','www.dji.com')
        mis_useWaylineAltitude=doc.createElement('mis:useWaylineAltitude')
        mis_useWaylineAltitude.appendChild(doc.createTextNode('false'))
        mis_turnMode=doc.createElement('mis:turnMode')
        mis_turnMode.appendChild(doc.createTextNode('Auto'))
        #-----**********----#
        mis_heading=doc.createElement('mis:heading')
        mis_heading.appendChild(doc.createTextNode(str(int(point[3]))))
        mis_gimbalPitch=doc.createElement('mis:gimbalPitch')
        mis_gimbalPitch.appendChild(doc.createTextNode(str(int(point[4]))))
        #-----**********----#
        mis_actions=doc.createElement('mis:actions')
        mis_actions.appendChild(doc.createTextNode('ShootPhoto'))
        ExtendedData.appendChild(mis_useWaylineAltitude)
        ExtendedData.appendChild(mis_turnMode)
        ExtendedData.appendChild(mis_heading)
        ExtendedData.appendChild(mis_gimbalPitch)
        ExtendedData.appendChild(mis_actions)
        Point=doc.createElement('Point')
        AltitudeMode=doc.createElement('altitudeMode')
        AltitudeMode.appendChild(doc.createTextNode('relativeToGround'))
        Coordinates=doc.createElement('coordinates')
        Coordinates.appendChild(doc.createTextNode(str(point[0])+','+str(point[1])+','+str(point[2])))
        Point.appendChild(AltitudeMode)
        Point.appendChild(Coordinates)
        Placemark.appendChild(Name)
        Placemark.appendChild(Visibility)
        Placemark.appendChild(Description)
        Placemark.appendChild(StyleUrl)
        Placemark.appendChild(ExtendedData)
        Placemark.appendChild(Point)
        Folder.appendChild(Placemark)
        
    #----------------------------------------
    Placemark=doc.createElement('Placemark')
    
    Name=doc.createElement('name')
    Name.appendChild(doc.createTextNode('Wayline'))
    
    Description=doc.createElement('description')
    Description.appendChild(doc.createTextNode('Wayline'))
    
    Visibility=doc.createElement('visibility')
    Visibility.appendChild(doc.createTextNode('1'))  
    
    ExtendedData=doc.createElement('ExtendedData')
    ExtendedData.setAttribute('xmlns:mis','www.dji.com')
    mis_altitude=doc.createElement('mis:altitude')
    mis_altitude.appendChild(doc.createTextNode('50'))
    mis_autoFlightSpeed=doc.createElement('mis:autoFlightSpeed')
    mis_autoFlightSpeed.appendChild(doc.createTextNode('1'))
    mis_actionOnFinish=doc.createElement('mis:actionOnFinish')
    mis_actionOnFinish.appendChild(doc.createTextNode('Hover'))
    mis_headingMode=doc.createElement('mis:headingMode')
    mis_headingMode.appendChild(doc.createTextNode('UsePointSetting'))
    mis_gimbalPitchMode=doc.createElement('mis:gimbalPitchMode')
    mis_gimbalPitchMode.appendChild(doc.createTextNode('UsePointSetting'))
    ExtendedData.appendChild(mis_altitude)
    ExtendedData.appendChild(mis_autoFlightSpeed)
    ExtendedData.appendChild(mis_actionOnFinish)
    ExtendedData.appendChild(mis_headingMode)
    ExtendedData.appendChild(mis_gimbalPitchMode)
    
    StyleUrl=doc.createElement('styleUrl')
    StyleUrl.appendChild(doc.createTextNode('#waylineGreenPoly'))
    
    LineString=doc.createElement('LineString')
    Tessellate=doc.createElement('tessellate')
    Tessellate.appendChild(doc.createTextNode('1'))
    AltitudeMode=doc.createElement('altitudeMode')
    AltitudeMode.appendChild(doc.createTextNode('relativeToGround'))
    Coordinates=doc.createElement('coordinates')
    coordsText=''
    for point in wayPointsCoords:
        coordsText=coordsText+str(point[0])+','+str(point[1])+','+str(point[2])+'\n'
    Coordinates.appendChild(doc.createTextNode(coordsText))
    LineString.appendChild(Tessellate)
    LineString.appendChild(AltitudeMode)
    LineString.appendChild(Coordinates)
    
    Placemark.appendChild(Name)
    Placemark.appendChild(Description)
    Placemark.appendChild(Visibility)
    Placemark.appendChild(ExtendedData)
    Placemark.appendChild(StyleUrl)
    Placemark.appendChild(LineString)
    
    Document.appendChild(Folder)
    Document.appendChild(Placemark)
    
    kml.appendChild(Document)
    
    doc.appendChild(kml)
    
    #开始写xml文档
    fp = open(os.path.join(wayPointsPath,filename), 'w')
    doc.writexml(fp, indent='\t', addindent='\t', newl='\n', encoding="utf-8")
    fp.close()
    
    print(filename+'写入完毕!')
        
if __name__=="__main__":
	#初始化全局变量
    init()
    print("object distance: ",objectDistance)
    print("home altitude: ",homeAltitude)
	#读取桥墩顶面中心点坐标的文件，得到一个存放桥墩顶面中心点坐标的列表
    a=readObjCoords(os.path.join(dataPath,'objCoords.txt'))
    print(a)    
    for i in a:
		#计算航点坐标及相机姿态，返回一个存放杭点坐标及姿态的列表
        if(wayPointsNumber==19):
            b=computeWayPointsCoords_19(i)
            #将b中的航点信息写入KML文件
            makeKmlFile(b,i[0]+'.kml')
        if(wayPointsNumber==27):
            b=computeWayPointsCoords_27(i)
            #将b中的航点信息写入KML文件
            makeKmlFile(b,i[0]+'.kml')
        