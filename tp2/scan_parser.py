import math
import argparse

class Parser:
    # Function to parse the log file
    def parse_log_file(self,file_path):
        with open(file_path, 'r') as file:
            line = file.readline()
            string_values = line.split(',')
            # Convert each string value to float
            return list(map(float, string_values))

class Processor:
    def __init__(self, args):
        self.x = args.x
        self.y = args.y
        self.orientation = args.orientation
        self.min_range = args.min_range
        self.max_range = args.max_range
        self.angle_min = args.angle_min
        self.angle_increment = args.angle_increment

    def process(self, ranges):
        clean_ranges = self.__clean(ranges)
        return self.__split_by_cylinder(clean_ranges)
        
    def __clean(self, ranges):
        clean_data = []
        for i in range(0, len(ranges)):
            length = ranges[i]
            if length > self.max_range or length < self.min_range:
                length = None
                
            angle = self.orientation + self.angle_min + self.angle_increment * i
            clean_data.append({
                'angle': angle,
                'range': length
            })
        return clean_data
    
    def __polar_to_xy(self, polar):
        x = self.x + polar['range'] * math.cos(polar['angle'])
        y = self.y + polar['range'] * math.sin(polar['angle'])
        return {'x': x, 'y': y}
    
    def __split_by_cylinder(self, ranges):
        cylinders = []
        rigth_side = None
        left_side = None
        for range in ranges:
            if range['range']:
                if not rigth_side:
                    rigth_side = self.__polar_to_xy(range)
                else: 
                    left_side = self.__polar_to_xy(range)
                continue
            
            if left_side:
                cylinders.append({
                    'x': (rigth_side['x'] + left_side['x'])/2,
                    'y': (rigth_side['y'] + left_side['y'])/2,
                })
                rigth_side = None
                left_side = None
        return cylinders

def parse_args():
    parser = argparse.ArgumentParser(
        prog='DumpParser',
        description='This script parse a dump of odom'
    )

    parser.add_argument('filepath')
    parser.add_argument('-x', type=float, default=0.0)
    parser.add_argument('-y', type=float, default=0.0)
    parser.add_argument('-o', '--orientation', type=float, default=0.0)
    parser.add_argument('-m', '--min-range', type=float, default=0.0)
    parser.add_argument('-M', '--max-range', type=float, default=11.0)
    parser.add_argument('-a', '--angle-min', type=float, default=0.0)
    parser.add_argument('-i', '--angle-increment', type=float, default=0.01749303564429283)

    return parser.parse_args()

def main():
    args = parse_args()
    parsed_data = Parser().parse_log_file(args.filepath)
    # x = 0.38356395178922886
    # y = -7.093319960833665
    # orientation = -1.7107137005077189
    # angle_increment = 0.01749303564429283
    processed = Processor(args).process(parsed_data)
    print(processed)
    return 0

if __name__ == "__main__":
    main()
