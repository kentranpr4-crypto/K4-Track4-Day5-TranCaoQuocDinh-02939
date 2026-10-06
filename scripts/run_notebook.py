"""Execute every Python cell in a fresh process, capture prints and PNG figures.

Requires only the numerical dependencies, not a Jupyter server. Stops on errors.
"""
import base64
import contextlib
import io
import json
import os
from pathlib import Path
import sys
import traceback

os.environ.setdefault('MPLBACKEND', 'Agg')
import matplotlib.pyplot as plt


def run(path):
    path = Path(path).resolve()
    notebook = json.loads(path.read_text())
    namespace = {'__name__': '__main__'}
    old_cwd = Path.cwd()
    os.chdir(path.parent)
    try:
        count = 0
        for i, cell in enumerate(notebook['cells']):
            if cell['cell_type'] != 'code':
                continue
            count += 1
            cell['outputs'] = []
            cell['execution_count'] = count
            def show(*args, **kwargs):
                for number in plt.get_fignums():
                    buffer = io.BytesIO()
                    plt.figure(number).savefig(buffer, format='png', bbox_inches='tight')
                    cell['outputs'].append({'output_type': 'display_data', 'metadata': {},
                        'data': {'image/png': base64.b64encode(buffer.getvalue()).decode()}})
                plt.close('all')
            plt.show = show
            output = io.StringIO()
            try:
                with contextlib.redirect_stdout(output):
                    exec(compile(''.join(cell['source']), f'{path.name}:cell_{i}', 'exec'), namespace)
            except Exception:
                print(f'FAILED cell {i}\n{output.getvalue()}', file=sys.stderr)
                traceback.print_exc()
                raise
            if output.getvalue():
                cell['outputs'].append({'output_type': 'stream', 'name': 'stdout', 'text': output.getvalue().splitlines(True)})
                print(f'Cell {i}: {output.getvalue().strip()}')
        path.write_text(json.dumps(notebook, ensure_ascii=False, indent=1)+'\n')
        print(f'Executed {count} code cells: {path}')
    finally:
        os.chdir(old_cwd)


if __name__ == '__main__':
    run(sys.argv[1] if len(sys.argv)>1 else 'Lab/kalman_fusion_lab_STUDENT.ipynb')
