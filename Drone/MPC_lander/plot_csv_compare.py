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


	cart_e_1 = data_1['cart_e']
	cart_n_1 = data_1['cart_n']
	cart_u_1 = data_1['cart_u']
	vel_e_1 = data_1['vel_e']
	vel_n_1 = data_1['vel_n']
	vel_u_1 = data_1['vel_u']
	des_e_1 = data_1['desired_e']
	des_n_1 = data_1['desired_n']
	des_u_1 = data_1['desired_u']

	cart_e_2 = data_2['cart_e']
	cart_n_2 = data_2['cart_n']
	cart_u_2 = data_2['cart_u']
	vel_e_2 = data_2['vel_e']
	vel_n_2 = data_2['vel_n']
	vel_u_2 = data_2['vel_u']
	des_e_2 = data_2['desired_e']
	des_n_2 = data_2['desired_n']
	des_u_2 = data_2['desired_u']

	x_data_1 = data_1.shape[0]
	x_data_2 = data_2.shape[0]
	
	if x_data_1>x_data_2:
		for i in range(x_data_2,x_data_1):
			cart_e_2[i]=(float('NAN'))
			cart_n_2[i]=(float('NAN'))
			cart_u_2[i]=(float('NAN'))
			vel_e_2[i]=(float('NAN'))
			vel_n_2[i]=(float('NAN'))
			vel_u_2[i]=(float('NAN'))
			des_e_2[i]=(float('NAN'))
			des_n_2[i]=(float('NAN'))
			des_u_2[i]=(float('NAN'))

	if x_data_2>x_data_1:
		for i in range(x_data_1,x_data_2):
			cart_e_1[i]=(float('NAN'))
			cart_n_1[i]=(float('NAN'))
			cart_u_1[i]=(float('NAN'))
			vel_e_1[i]=(float('NAN'))
			vel_n_1[i]=(float('NAN'))
			vel_u_1[i]=(float('NAN'))
			des_e_1[i]=(float('NAN'))
			des_n_1[i]=(float('NAN'))
			des_u_1[i]=(float('NAN'))

	x_data = max(x_data_1,x_data_2)
	
	l = ['file = ' + f1 , 'file = ' + f2]

	
	#plotting comparision of cart_e
	fig1 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(cart_e_1),'b-')
	ax.plot(range(x_data), np.array(cart_e_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='cartesian coordinate in east direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(cart_e_1,cart_e_2),max(cart_e_1,cart_e_2))
	plt.legend(l)
	plt.title('Comparision of cartision coordinate in east direction')
	save_file = 'cartisean_east.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig2 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(cart_n_1),'b-')
	ax.plot(range(x_data), np.array(cart_n_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='cartesian coordinate in north direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(cart_n_1,cart_n_2),max(cart_n_1,cart_n_2))
	plt.legend(l)
	plt.title('Comparision of cartision coordinate in north direction')
	save_file = 'cartisean_north.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig3 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(cart_u_1),'b-')
	ax.plot(range(x_data), np.array(cart_u_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='cartesian coordinate in downward direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(cart_u_1,cart_u_2),max(cart_u_1,cart_u_2))
	plt.legend(l)
	plt.title('Comparision of cartision coordinate in downward direction')
	save_file = 'cartisean_down.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig4 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(vel_e_1),'b-')
	ax.plot(range(x_data), np.array(vel_e_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='sensed velocity in east direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(vel_e_1,vel_e_2),max(vel_e_1,vel_e_2))
	plt.legend(l)
	plt.title('Comparision of sensed velocity in east direction')
	save_file = 'sensed_velocity_east.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig5 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(vel_n_1),'b-')
	ax.plot(range(x_data), np.array(vel_n_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='sensed velocity in north direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(vel_n_1,vel_n_2),max(vel_n_1,vel_n_2))
	plt.legend(l)
	plt.title('Comparision of sensed velocity in north direction')
	save_file = 'sensed_velocity_north.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig6 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(vel_u_1),'b-')
	ax.plot(range(x_data), np.array(vel_u_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='sensed velocity in downward direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(vel_u_1,vel_u_2),max(vel_u_1,vel_u_2))
	plt.legend(l)
	plt.title('Comparision of sensed velocity in downward direction')
	save_file = 'sensed_velocity_down.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig7 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_e_1),'b-')
	ax.plot(range(x_data), np.array(des_e_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='desired velocity in east direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_e_1,des_e_2),max(des_e_1,des_e_2))
	plt.legend(l)
	plt.title('Comparision of desired velocity in east direction')
	save_file = 'desired_velocity_east.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig8 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_n_1),'b-')
	ax.plot(range(x_data), np.array(des_n_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='desired velocity in north direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_n_1,des_n_2),max(des_n_1,des_n_2))
	plt.legend(l)
	plt.title('Comparision of desired velocity in north direction')
	save_file = 'desired_velocity_north.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig9 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_u_1),'b-')
	ax.plot(range(x_data), np.array(des_u_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='desired velocity in downward direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_u_1,des_u_2),max(des_u_1,des_u_2))
	plt.legend(l)
	plt.title('Comparision of desired velocity in downward direction')
	save_file = 'desired_velocity_down.png'
	plt.savefig(os.path.join(save_path,save_file))

	leg1 = ['desired velocity of ' + f1 , 'sensor velocity of ' + f1]
	leg2 = ['desired velocity of ' + f2 , 'sensor velocity of ' + f2]

	fig10 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_e_1),'b-')
	ax.plot(range(x_data), np.array(vel_e_1),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in east direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_e_1,vel_e_1),max(des_e_1,vel_e_1))
	plt.legend(leg1)
	plt.title('Comparision of desired and senses velocity in east direction for file ' + f1)
	save_file = 'velocity_compare_east' + f1 + '.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig11 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_n_1),'b-')
	ax.plot(range(x_data), np.array(vel_n_1),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in north direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_n_1,vel_n_1),max(des_n_1,vel_n_1))
	plt.legend(leg1)
	plt.title('Comparision of desired and senses velocity in north direction for file ' + f1)
	save_file = 'velocity_compare_north' + f1 + '.png'
	plt.savefig(os.path.join(save_path,save_file))
	
	fig12= plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_u_1),'b-')
	ax.plot(range(x_data), np.array(vel_u_1),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in upward direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_u_1,vel_u_1),max(des_u_1,vel_u_1))
	plt.legend(leg1)
	plt.title('Comparision of desired and senses velocity in upward direction for file ' + f1)
	save_file = 'velocity_compare_up' + f1 + '.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig13 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_e_2),'b-')
	ax.plot(range(x_data), np.array(vel_e_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in east direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_e_2,vel_e_2),max(des_e_2,vel_e_2))
	plt.legend(leg2)
	plt.title('Comparision of desired and senses velocity in east direction for file ' + f2)
	save_file = 'velocity_compare_east' + f2 + '.png'
	plt.savefig(os.path.join(save_path,save_file))

	fig14 = plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_n_2),'b-')
	ax.plot(range(x_data), np.array(vel_n_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in north direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_n_2,vel_n_2),max(des_n_2,vel_n_2))
	plt.legend(leg1)
	plt.title('Comparision of desired and senses velocity in north direction for file ' + f2)
	save_file = 'velocity_compare_north' + f2 + '.png'
	plt.savefig(os.path.join(save_path,save_file))
	
	fig15= plt.figure()
	ax = plt.gca()
	ax.plot(range(x_data), np.array(des_u_2),'b-')
	ax.plot(range(x_data), np.array(vel_u_2),'r-')
	ax.grid(True)
	ax.set(xlabel='data points', ylabel='velocity in upward direction')
	plt.xlim(0, x_data+5)
	#plt.ylim(min(des_u_2,vel_u_2),max(des_u_2,vel_u_2))
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
