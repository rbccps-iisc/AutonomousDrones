#Script to compare the data in two different csv files and outputs the comparision results in Plot_Comparisions directory

import matplotlib.pyplot as plt
import pandas as pd
import csv
import numpy as np
import argparse
import os


def main(f1,f2):


	data_1 =  pd.read_csv('./'+f1)
	data_2 =  pd.read_csv('./'+f2)


	save_path = 'Plot_Comparisions/' + f1.replace('.csv','') + 'and' + f2.replace('.csv','') + '/'

	if not os.path.exists(save_path):
		os.makedirs(save_path)


	cart_x_1 = data_1['cart_x']
	cart_y_1 = data_1['cart_y']
	cart_z_1 = data_1['cart_z']
	vel_x_1 = data_1['vel_x']
	vel_y_1 = data_1['vel_y']
	vel_z_1 = data_1['vel_z']
	des_x_1 = data_1['desired_x']
	des_y_1 = data_1['desired_y']
	des_z_1 = data_1['desired_z']

	cart_x_2 = data_2['cart_x']
	cart_y_2 = data_2['cart_y']
	cart_z_2 = data_2['cart_z']
	vel_x_2 = data_2['vel_x']
	vel_y_2 = data_2['vel_y']
	vel_z_2 = data_2['vel_z']
	des_x_2 = data_2['desired_x']
	des_y_2 = data_2['desired_y']
	des_z_2 = data_2['desired_z']

	x_data_1 = data_1.shape[0]
	x_data_2 = data_2.shape[0]
	
	if x_data_1>x_data_2:
		for i in range(x_data_2,x_data_1):
			cart_x_2[i]=(float('NAN'))
			cart_y_2[i]=(float('NAN'))
			cart_z_2[i]=(float('NAN'))
			vel_x_2[i]=(float('NAN'))
			vel_y_2[i]=(float('NAN'))
			vel_z_2[i]=(float('NAN'))
			des_x_2[i]=(float('NAN'))
			des_y_2[i]=(float('NAN'))
			des_z_2[i]=(float('NAN'))

	if x_data_2>x_data_1:
		for i in range(x_data_1,x_data_2):
			cart_x_1[i]=(float('NAN'))
			cart_y_1[i]=(float('NAN'))
			cart_z_1[i]=(float('NAN'))
			vel_x_1[i]=(float('NAN'))
			vel_y_1[i]=(float('NAN'))
			vel_z_1[i]=(float('NAN'))
			des_x_1[i]=(float('NAN'))
			des_y_1[i]=(float('NAN'))
			des_z_1[i]=(float('NAN'))

	x_data = max(x_data_1,x_data_2)
	
	l = ['file = ' + f1 , 'file = ' + f2]

	
	#plotting comparision of cart_x
	fig1 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(cart_x_1),'b-')
	ax.plot(range(x_data), np.array(cart_x_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='cartesian coordinate in x direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(cart_x_1,cart_x_2),max(cart_x_1,cart_x_2))
	plt.legend(l)
	plt.title('Comparision of cartision coordinate in x direction')
	save_file = 'cartisean_x.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig2 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(cart_y_1),'b-')
	ax.plot(range(x_data), np.array(cart_y_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='cartesian coordinate in y direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(cart_y_1,cart_y_2),max(cart_y_1,cart_y_2))
	plt.legend(l)
	plt.title('Comparision of cartision coordinate in y direction')
	save_file = 'cartisean_y.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig3 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(cart_z_1),'b-')
	ax.plot(range(x_data), np.array(cart_z_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='cartesian coordinate in downward direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(cart_z_1,cart_z_2),max(cart_z_1,cart_z_2))
	plt.legend(l)
	plt.title('Comparision of cartision coordinate in downward direction')
	save_file = 'cartisean_z.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig4 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(vel_x_1),'b-')
	ax.plot(range(x_data), np.array(vel_x_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='sensed velocity in x direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(vel_x_1,vel_x_2),max(vel_x_1,vel_x_2))
	plt.legend(l)
	plt.title('Comparision of sensed velocity in x direction')
	save_file = 'sensed_velocity_x.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig5 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(vel_y_1),'b-')
	ax.plot(range(x_data), np.array(vel_y_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='sensed velocity in y direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(vel_y_1,vel_y_2),max(vel_y_1,vel_y_2))
	plt.legend(l)
	plt.title('Comparision of sensed velocity in y direction')
	save_file = 'sensed_velocity_y.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig6 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(vel_z_1),'b-')
	ax.plot(range(x_data), np.array(vel_z_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='sensed velocity in downward direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(vel_z_1,vel_z_2),max(vel_z_1,vel_z_2))
	plt.legend(l)
	plt.title('Comparision of sensed velocity in downward direction')
	save_file = 'sensed_velocity_z.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig7 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_x_1),'b-')
	ax.plot(range(x_data), np.array(des_x_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='desired velocity in x direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_x_1,des_x_2),max(des_x_1,des_x_2))
	plt.legend(l)
	plt.title('Comparision of desired velocity in x direction')
	save_file = 'desired_velocity_x.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig8 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_y_1),'b-')
	ax.plot(range(x_data), np.array(des_y_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='desired velocity in y direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_y_1,des_y_2),max(des_y_1,des_y_2))
	plt.legend(l)
	plt.title('Comparision of desired velocity in y direction')
	save_file = 'desired_velocity_y.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig9 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_z_1),'b-')
	ax.plot(range(x_data), np.array(des_z_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='desired velocity in downward direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_z_1,des_z_2),max(des_z_1,des_z_2))
	plt.legend(l)
	plt.title('Comparision of desired velocity in downward direction')
	save_file = 'desired_velocity_z.png'
	plt.savefig(os.path.join(save_path,save_file))

	leg1 = ['desired velocity of ' + f1 , 'sensor velocity of ' + f1]
	leg2 = ['desired velocity of ' + f2 , 'sensor velocity of ' + f2]

	fig10 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_x_1),'b-')
	ax.plot(range(x_data), np.array(vel_x_1),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in x direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_x_1,vel_x_1),max(des_x_1,vel_x_1))
	plt.legend(leg1)
	plt.title('Comparision of desired and senses velocity in x direction for file ' + f1)
	save_file = 'velocity_compare_x' + f1 + '.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig11 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_y_1),'b-')
	ax.plot(range(x_data), np.array(vel_y_1),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in y direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_y_1,vel_y_1),max(des_y_1,vel_y_1))
	plt.legend(leg1)
	plt.title('Comparision of desired and senses velocity in y direction for file ' + f1)
	save_file = 'velocity_compare_y' + f1 + '.png'
	plt.savefig(os.path.join(save_path,save_file))
	
	fig12= plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_z_1),'b-')
	ax.plot(range(x_data), np.array(vel_z_1),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in upward direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_z_1,vel_z_1),max(des_z_1,vel_z_1))
	plt.legend(leg1)
	plt.title('Comparision of desired and senses velocity in upward direction for file ' + f1)
	save_file = 'velocity_compare_up' + f1 + '.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig13 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_x_2),'b-')
	ax.plot(range(x_data), np.array(vel_x_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in x direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_x_2,vel_x_2),max(des_x_2,vel_x_2))
	plt.legend(leg2)
	plt.title('Comparision of desired and senses velocity in x direction for file ' + f2)
	save_file = 'velocity_compare_x' + f2 + '.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig14 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_y_2),'b-')
	ax.plot(range(x_data), np.array(vel_y_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in y direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_y_2,vel_y_2),max(des_y_2,vel_y_2))
	plt.legend(leg1)
	plt.title('Comparision of desired and senses velocity in y direction for file ' + f2)
	save_file = 'velocity_compare_y' + f2 + '.png'
	plt.savefig(os.path.join(save_path,save_file))
	
	fig15= plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_z_2),'b-')
	ax.plot(range(x_data), np.array(vel_z_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in upward direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_z_2,vel_z_2),max(des_z_2,vel_z_2))
	plt.legend(leg2)
	plt.title('Comparision of desired and senses velocity in upward direction for file ' + f2)
	save_file = 'velocity_compare_up' + f2 + '.png'
	plt.savefig(os.path.join(save_path,save_file))

	#plt.show()


if __name__ == '__main__':

	parser = argparse.ArgumentParser(description='params')
	#parser.add_argument('--file_1', default='_2_8_13_33.csv', type=str)
	#parser.add_argument('--file_2', default='_2_8_13_31.csv', type=str)
	parser.add_argument('--file_1', type=str)
	parser.add_argument('--file_2', type=str)

	args = parser.parse_args()
	f1 = args.file_1
	f2 = args.file_2

	if f1 == None or f2==None:
		print('Failed to enter file names. Run \"\033[1mpython plot_csv_compare.py -h\033[0m\" for more information')
		exit()

	main(f1,f2)
