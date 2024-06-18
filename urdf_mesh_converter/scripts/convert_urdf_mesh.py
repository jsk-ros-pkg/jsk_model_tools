#!/usr/bin/env python

import argparse
import os.path as osp
from pathlib import Path
import shutil

from urdf_mesh_converter.urdf import export_mesh_format
from urdf_mesh_converter.urdf import enable_mesh_cache
from urdf_mesh_converter.urdf import URDF


def main():
    parser = argparse.ArgumentParser(description='Convert URDF Mesh.')
    parser.add_argument('urdf',
                        help='Path to the input URDF file')
    parser.add_argument('--format', '-f',
                        default='dae',
                        choices=['dae', 'stl'],
                        help='Mesh format for export. Default is dae.')
    parser.add_argument('--output', '-o', help='Path for the output URDF file. If not specified, a filename is automatically generated based on the input URDF file.')  # NOQA
    parser.add_argument('--inplace', '-i', action='store_true',
                        help='Modify the input URDF file inplace. If not specified, a new file is created.')  # NOQA

    args = parser.parse_args()

    base_path = Path(args.urdf).parent
    urdf_path = Path(args.urdf)

    if args.output is None:
        fn, _ = osp.splitext(args.urdf)
        index = 0
        pattern = fn + "_%i.urdf"
        outfile = pattern % index
        while osp.exists(outfile):
            index += 1
            outfile = pattern % index
        args.output = outfile
    output_path = Path(args.output)
    output_path = base_path / output_path

    with enable_mesh_cache():
        with open(base_path / urdf_path) as file_obj:
            urdf_robot_model = URDF.load(file_obj=file_obj)

    with export_mesh_format('.' + args.format):
        urdf_robot_model.save(str(output_path))
    if args.inplace:
        shutil.move(output_path, base_path / urdf_path)
        print('=> Saved to {}'.format(base_path / urdf_path))
    else:
        print('=> Saved to {}'.format(output_path))


if __name__ == '__main__':
    main()
