import json
import os, sys
 
json_path = 'MultiViewTank.json'
txt_path = 'list.txt'
dict = {}
 
def get_json_data(json_path):
 
 
    

    
    # add '/color/'to the path
    file = open("list.txt")
    while 1:
        lines = file.readlines(1000)
        if not lines:
            break
        for line in lines:
            print(line)

    file.close()
    # print("params",params)
    dict = params
    f.close()
    return dict
 
def write_json_data(dict):
    with open(json_path1,'w') as r:
        json.dump(dict,r)
    r.close()
 
if __name__ == "__main__":
    the_revised_dict = get_json_data(json_path)
    
    write_json_data(the_revised_dict)