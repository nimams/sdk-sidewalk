#!/usr/bin/env bash
set -e

python3 -m venv venv
source ./venv/bin/activate

pip install -r $(pwd)/requirements-doc.txt

pip install sphinx --upgrade
pip install sphinx_rtd_theme --upgrade
sphinx-build -M html . build
