#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 10 11:11:55 2025

@author: riesna r. audh (riesnaaudh@gmail.com)
map with inset
"""
from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
import numpy as np
import pandas as pd
import xarray as xr
import matplotlib.gridspec as gridspec

# load all files here
#datadir = '/home/riesna/Desktop/PhD2022-FinalYear/SCALE22/Agi-Map/' #assign filepath
#datadir = 'C:\Users\agori\Downloads\Agi-Map'
datadir = ''
a1 = (datadir + 'Ant_20220720_res3.125_pyres.nc') #.nc file (sea ice conc) 
file1= (datadir + 'SCALE22_Stations.xlsm') #excel sheet with coordinates
file2= (datadir + 'BuoyLocations.xlsm') #excel sheet with coordinates

#%%
###### leave this alone if you are just plotting one map on the figure #######
#setting up the figure and basemap - you would change here to add subplots
gs = gridspec.GridSpec(1, 1)
fig = plt.figure(1,figsize=(19.20,10.80))
ax = fig.add_subplot(gs[0, 0]) # row, col
################## change here to adjust map size etc. #######################
#m = Basemap(width=150000,height=175000,projection='lcc',
            #resolution='c',lat_1=-46.,lat_2=-60,lat_0=-58.8,lon_0=-0.5)
m = Basemap(width=150000,height=175000,projection='lcc',
            resolution='c',lat_1=-46.,lat_2=-60,lat_0=-58.8,lon_0=-0.2)
##############################################################################

#reading SIC file
sia = xr.open_dataset(a1)               
lat = sia.latitude.values
lon = sia.longitude.values

#mapping SIC data to basemap
mx, my = m(lon, lat)

#map specs here
parallels = np.arange(-90,0,0.2)
m.drawparallels(parallels,labels=[False,True,False,False],fontsize=8,color='#808080') # labels = [left,right,top,bottom]
meridians = np.arange(0.,360.,0.2)
m.drawmeridians(meridians,labels=[False,False,False,True],fontsize=6,color='#808080')
m.fillcontinents(color='thistle',lake_color='grey')
#m.pcolormesh(mx, my, sia.sea_ice_concentration[0].to_masked_array(), cmap=plt.cm.get_cmap('cubehelix', 10))#change 'cubehelix' for different colormap
m.pcolormesh(mx, my, sia.sea_ice_concentration[0].to_masked_array(), cmap=plt.cm.get_cmap('bone', 10))#change 'cubehelix' for different colormap
m.drawmapboundary(fill_color='navy') #ocean colour
cb = m.colorbar(location='bottom',pad=0.35)
cb.set_label('AMSR2 Sea Ice Concentration [%] on 20.07.2022', fontsize=8)

#shiptrack
dfShip = pd.read_excel(file1, sheet_name='Ship')
dfShip = dfShip.dropna()
lonShip = dfShip['LON_DEC'].tolist()
latShip = dfShip['LAT_DEC'].tolist()
xShip,yShip = m(lonShip, latShip)
#m.plot(xShip, yShip, color='purple',linestyle='-', linewidth=3,label='ship track') 
m.plot(xShip, yShip, color='purple',linestyle='--', linewidth=2,label='Ship Track') 
#stations
#dfStations = pd.read_excel(file1, sheet_name='StationListAgi')
dfStations = pd.read_excel(file1, sheet_name='FieldLiDAR')
lonStations = dfStations['LON_DEC_OPEN'].tolist()
latStations = dfStations['LAT_DEC_OPEN'].tolist()
xStations,yStations = m(lonStations, latStations)
m.plot(xStations, yStations, 'r*', markersize=10,label='Deck 7 LiDAR Scans') 

dfStations = pd.read_excel(file1, sheet_name='FieldLiDARHighlight')
lonStations = dfStations['LON_DEC_OPEN'].tolist()
latStations = dfStations['LAT_DEC_OPEN'].tolist()
xStations,yStations = m(lonStations, latStations)
m.plot(xStations, yStations, 'b*', markersize=10) 

dfPOI = pd.read_excel(file1, sheet_name='POI')
lonPOI = dfPOI['LON_DEC_OPEN'].tolist()
latPOI = dfPOI['LAT_DEC_OPEN'].tolist()
xPOI,yPOI = m(lonPOI, latPOI)
#m.plot(xPOI, yPOI, '*', markersize=14,label='Pancake Stations',color='g',markeredgecolor='g') 
#m.plot(xPOI, yPOI, '*', markersize=12,label='Pancake Stations',color='#316C3C',markeredgecolor='k') 

fnt=10
#ax.annotate('P1&P2', xy=(xPOI[0], yPOI[0]), xycoords='data', xytext=(xPOI[0]-6000, yPOI[0]-6000), 
#            textcoords='data',fontsize=fnt,color='y')
#ax.annotate('P3', xy=(xPOI[1], yPOI[1]), xycoords='data', xytext=(xPOI[1]+6000, yPOI[1]+6000), 
#            textcoords='data',fontsize=fnt,color='y')
#ax.annotate('P4', xy=(xPOI[2], yPOI[2]), xycoords='data', xytext=(xPOI[2]-6000, yPOI[2]-6000), 
#            textcoords='data',fontsize=fnt,color='y')
#ax.annotate('SB06', xy=(xPOI[3], yPOI[3]), xycoords='data', xytext=(xPOI[3]-6000, yPOI[3]-6000), 
#            textcoords='data',fontsize=fnt,color='y')
#labels for stations - this is a bitch. adjusting location of the text is in the xytext argument
#duplicate and edit for more station labels


#ax.annotate('station-name2', xy=(xStations[1], yStations[1]), xycoords='data', xytext=(xStations[1]-6000, yStations[1]-6000), 
 #           textcoords='data',fontsize=15,color='y')

#buoy tracks - add more for more tracks
dfA = pd.read_excel(file2, sheet_name='SB6new')
#dfA = dfA.dropna()
lonA = dfA['Lon'].tolist()
latA = dfA['Lat'].tolist()
xA,yA = m(lonA, latA)
#m.plot(xA, yA, 'c--', linewidth=2, label='SHARC Buoy 06') #'mo-' indicates m = magenta(change for different colours), o=dot, -=line joining points
#m.plot(xA, yA, 'c-', linewidth=2, label='SHARC Buoy 06') #'mo-' indicates m = magenta(change for different colours), o=dot, -=line joining points
#m.plot(xA[0], yA[0], 'cs', markersize=7,markeredgecolor='k')
#m.plot(xA[-1], yA[-1], 'co', markersize=7,markeredgecolor='k')


#inset map - leave alone if your're not adding to the map
axins = zoomed_inset_axes(ax, 0.007, loc=3)
m2 = Basemap(width=5500000,height=4000000,projection='lcc',
            resolution='c',lat_1=-8.,lat_2=-65,lat_0=-60,lon_0=0.,ax=axins)

sia = xr.open_dataset(a1)               
lat = sia.latitude.values
lon = sia.longitude.values

mx2, my2 = m2(lon, lat)

parallels2 = np.arange(-90,0,10.)
m2.drawparallels(parallels2,labels=[False,False,False,False],color='#808080') # labels = [left,right,top,bottom]
meridians2 = np.arange(0.,351.,20.)
m2.drawmeridians(meridians2,labels=[False,False,False,False],color='#808080')
m2.fillcontinents(color='thistle',lake_color='grey')
m2.pcolormesh(mx2, my2, sia.sea_ice_concentration[0].to_masked_array(), cmap=plt.cm.get_cmap('cubehelix', 10))#GnBu_r, 
m2.drawmapboundary(fill_color='black') #ocean colour

#red box indicating the target area
df5 = pd.read_excel(file2, sheet_name='Box')
lonBOX = df5['Lon'].tolist()
latBOX = df5['Lat'].tolist()
xBOX,yBOX = m2(lonBOX, latBOX)
m2.plot(xBOX, yBOX, 'r-', linewidth=1) #'mo-' indicates m = magenta(change for different colours), o=dot, -=line joining points
m2.plot(xBOX, yBOX, 'r-', linewidth=1)

#shading the  big map border
ax.spines['bottom'].set_color('r')
ax.spines['top'].set_color('r') 
ax.spines['right'].set_color('red')
ax.spines['left'].set_color('red')

#legend -> to edit legend items, change the label argument when plotting a point
#leg =ax.legend(loc='upper center',bbox_to_anchor=(0.5,-0.2),frameon=True, ncol=5,prop={'size':8})
leg =ax.legend(loc='lower center',bbox_to_anchor=(0.5,1.02),frameon=True, ncol=5,prop={'size':8})

#small map border colour
axins.spines['bottom'].set_color('w')
axins.spines['top'].set_color('w') 
axins.spines['right'].set_color('w')
axins.spines['left'].set_color('w')

plt.show()
plt.savefig(datadir+'your-figure-name.png',bbox_inches='tight')
