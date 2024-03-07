# -*- coding: utf-8 -*-

import sys
import csv
import os
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET

flight_obj_dir = '/home/santy/source/SantyPilot/shared/uavobjectdefinition'
Flight = 1
pnt_cnt = 10
show_obj_fields = {
	'GPSPOSITIONSENSOR': ['Altitude']
}

def read_csv_log(filename):
	with open(filename, 'r') as file:
		reader = csv.reader(file, delimiter='\t')
		for row in reader:
		    yield row

def parse_obj_data(raw, info):
	# print('parsing line {}'.format(raw))
	ret = {}
	raw = ' '.join(raw.strip().split())
	str_list = raw.split(' ')
	name = str_list[0]
	if name not in info:
		return 'INVALID OBJ NAME', ret
	refer = info[name]
	if str_list[1][0] != '(':
		return name, ret
	idi = -1
	for i in range(1, len(str_list)):
		if str_list[i][-1] == ')':
			idi = i + 2
			if str_list[i + 1] != 'Data:':
				return name, ret 
			break
	if idi == -1:
		return name, ret 
	while idi < len(str_list):
		k = str_list[idi][0: -1]
		v = str_list[idi + 2]
		ret[k] = v
		idi = idi + 4
		if k not in refer:
			continue
		step = len(refer[k].strip().split())
		# print('{0}, {1}, {2}'.format(k, step, str_list))
		idi = idi + step
	return name, ret
	

# Element: <tag attrib>text<child/>...</tag>tail
def read_obj_xml(filename):
    # print('processing obj file: {}'.format(filename))
	tree = ET.parse(filename)
	root = tree.getroot()
	return root[0]

def print_obj_detail(obj_list, key = '', num = 0):
	print('total uavobjects num: {}'.format(len(obj_list)))
	idx = 0
	for obj in obj_list:
		if num != 0 and idx >= num:
		    break
		if key != '' and obj.attrib['name'] != key:
		    continue
		desc_str = obj[0].text.strip()
		fields_str = ''
		units_str = ''
		for field in obj:
			if field.tag != 'field':
				continue
			fields_str = fields_str + field.attrib['name'] + ' '
			units_str = units_str + field.attrib['units'] + ' '
		fields_str.strip()
		print('index: {0} flight object name {1}:\n\t' \
				'description: {2}\n\t'
				'fields: {3}\n\t' \
				'units: {4}\n\t' \
				.format(idx, obj.attrib['name'], desc_str, 
					fields_str, units_str))
		idx = idx + 1

def get_obj_name_unit(obj_list, key = ''):
	info = {}
	for obj in obj_list:
		if key != '' and obj.attrib['name'] != key:
		    continue
		kv = {}
		for field in obj:
			if field.tag != 'field':
				continue
			if 'name' not in field.attrib or 'units' not in field.attrib:
				continue
			k = field.attrib['name']
			v = field.attrib['units']
			kv[k] = v
		ukey = obj.attrib['name'].upper()
		info[ukey] = kv
	return info

def main(args):
    # read all supported xml obj files
	obj_list = []
	for root,dirs,files in os.walk(flight_obj_dir):
		for f in files:
		    full_path = os.path.join(root, f)
		    if full_path.split('.')[-1] != 'xml':
		        continue
		    obj_elem = read_obj_xml(full_path)
		    obj_list.append(obj_elem)

	'''
	for key in ['GPSPositionSensor']:
		print_obj_detail(obj_list, key)
	'''
	info = get_obj_name_unit(obj_list)

    # print 5 lines
	idx = 0
	detail = {}
	for row in read_csv_log(args[1]):
		if idx == 0: # skip header
			idx += 1
			continue
		if len(row) != 4:
			continue # skip non-data line
		if row[3].strip() == '':
			continue
		name, fields = parse_obj_data(row[3], info)
		if name == 'INVALID OBJ NAME' or len(fields) == 0:
			continue
		'''
		print('Flight Time: {0}, Obj Name: {1}\n\r' \
				'fields {2}' \
				.format(row[1], name, fields))
		'''
		fields['FlightTime'] = row[1]
		if name not in detail:
			detail[name] = [fields]
		else:
			detail[name].append(fields)

	if len(show_obj_fields) == 0:
		for key in detail.keys():
			print('set show_obj_fields in flight_log_parser.py file!\n')
			print('obj name: {0}, obj fields: {1}'.format(name, detail[name][0]))

	# draw 
	x = []
	y = []
	for name in show_obj_fields.keys():
		for field in show_obj_fields[name]:
			temp = []
			for i in range(len(detail[name])):
				x.append(detail[name][i]['FlightTime'])
				temp.append(detail[name][i][field])
			y.append(temp)
	subrate = int(len(x) / pnt_cnt)
	# print(len(x), len(y[0]))
	plt.figure()
	plt.plot(x[1:len(x):subrate], y[0][1:len(x):subrate], color='red', linestyle='dashed')
	plt.title('Flight Data')
	plt.xlabel('Flight Time')
	plt.xticks(rotation=45)
	plt.show()

if __name__ == "__main__":
    main(sys.argv)
