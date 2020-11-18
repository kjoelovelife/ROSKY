#!/usr/bin/env python3

import os , sys , argparse ,errno


class mkdir(object):
    
    def __init__(self):
        self.parser = argparse.ArgumentParser(description="Make a directory in ~/ROSKY/catkin_ws/src/object/image",epilog="save your image")
        self.parser.add_argument("--name" , "-n" , type=str,required=True,help="Please type you want the name of dirctory.")
        self.args = self.parser.parse_args()
        self.path = os.path.abspath(os.path.join(os.path.dirname(__file__),os.pardir))
        action = self.try_make(self.args.name)

    def try_make(self,name):
        try:
            save_path = self.path + "/image/" + name
            os.makedirs(save_path)
            print("Done! Make a directory : {}".format(save_path))
        except OSError as e:
            if e.errno == errno.EEXIST:
                print("Note! Directory not created because it already exit. ")
            else:
                raise

if __name__ == "__main__" :
    make = mkdir()
    
    
