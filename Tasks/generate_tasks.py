#!/bin/python3
import xml.etree.ElementTree as ET
import argparse
import numpy as np

def main(args):
    #Load in map
    map = ET.parse(args.map)
    map_root = map.getroot()

    # Extract grid dimensions
    width = int(map_root.find('.//width').text)
    height = int(map_root.find('.//height').text)

    # Initialize a numpy array to store the grid
    grid = np.zeros((height, width), dtype=int)

    # Find all row elements and populate the numpy array
    row_elements = map_root.findall('.//row')
    for i, row_element in enumerate(row_elements):
        row_data = row_element.text.split()
        for j, value in enumerate(row_data):
            grid[i, j] = int(value)

    # Create a boolean mask to track the location of zeros
    zero_locations = (grid == 0)
    zero_indices = np.transpose(np.where(zero_locations))

    for i in range(args.num_tasks):
        root = ET.Element("root")
        root.set("width", str(width))
        root.set("height", str(height))

        locations = zero_indices[np.random.choice(zero_indices.shape[0], args.agents*2, replace=False)]
        for j in range(0, len(locations), 2):
            agent = ET.Element("agent")
            agent.set("start_i", str(locations[j][0]))
            agent.set("start_j", str(locations[j][1]))
            agent.set("goal_i", str(locations[j+1][0]))
            agent.set("goal_j", str(locations[j+1][1]))
            root.append(agent)
        tree = ET.ElementTree(root)
        ET.indent(tree)
        tree.write(f'{args.name}_{i}.xml', encoding="utf-8", xml_declaration=True)

def create_parser():
    parser = argparse.ArgumentParser(description='Generate a bunch of random scenarios based on the given map')
    parser.add_argument('--map', '-m', default='Examples/grid_map.xml', type=str, help='The map to generate tasks for. Only supports grid type maps')
    parser.add_argument('--agents', '-a', type=int, default=10, help='Number of agents')
    parser.add_argument('--name', '-n', default='Examples/grid_task', help='First part of output file\'s name')
    parser.add_argument('--num_tasks', type=int, default = 10, help='number of tasks to generate')
    return parser

if __name__ == "__main__":
    args = create_parser().parse_args()
    main(args)