import argparse
import subprocess
import json

# python client.py -prev longdress_1051.ply -next longdress_1055.ply -out build/output.ply -pos 0.5

parser = argparse.ArgumentParser()
parser.add_argument("-prev", dest="prev", type=str, help="Set previous frame path", required=True)
parser.add_argument("-next", dest="next", type=str, help="Set next frame path", required=True)
parser.add_argument("-out", dest="output", type=str, help="Set output frame path", required=True)
parser.add_argument("-pos", dest="position", type=float, help="Relative position between prev and next frame", required=True)

args = parser.parse_args()

command = [
        "./build/Pipeline",
        f'{args.prev}',
        f'{args.next}',
        f'{args.output}',
        f'{args.position}',
    ]

subprocess.run(command)
