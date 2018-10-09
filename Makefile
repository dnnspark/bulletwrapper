venv:
	virtualenv venv -p `which python3.6` 

install_dev:
	source ./venv/bin/activate; \
	pip install -e . && pip install pytest && pip install flake8

ci:
	pytest

flake8:
	flake8 --ignore=E501,F401,E128,E402,E731,F821 pybullet
	flake8 --ignore=E501,F401,E128,E402,E731,F821 tests

clean:
	rm -rf *.egg-info
	rm -rf build/
	rm -rf dist/
	rm -rf junit-py*.xml
	rm -rf venv/
	rm `find . -name '*.pyc'`
	rm `find . -name __pycache__`

.PHONY: venv install_dev ci flake8 clean
