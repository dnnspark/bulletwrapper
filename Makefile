SHELL := /bin/bash

PACKAGE_NAME = bulletwrapper

error:
	@echo "Empty target is not allowed. Choose one of the targets in Makefile."
	@exit 2

venv:
	python3 -m venv ./venv
	ln -s venv/bin/activate activate
	. ./venv/bin/activate && \
	pip3 install -U pip setuptools wheel

install_package:
	. ./venv/bin/activate && \
	python setup.py build develop

install_test:
	. ./venv/bin/activate && \
	pip3 install -U pytest flake8

install_tools:
	. ./venv/bin/activate && \
	pip3 install -U seaborn scikit-image imageio

install: venv install_package install_test

dev: venv install_package install_test install_tools

test:
	pytest tests -s

ci:
	pytest tests

flake8:
	. ./venv/bin/activate && \
	python -m flake8 --config flake8.config

clean:
	rm -rf `find $(PACKAGE_NAME) -name '*.pyc'`
	rm -rf `find $(PACKAGE_NAME) -name __pycache__`
	rm -rf `find tests -name '*.pyc'`
	rm -rf `find tests -name __pycache__`

clean_all: clean
	rm -rf build/
	rm -rf *.egg-info
	rm -rf venv/
	rm -rf activate

.PHONY: venv install_package install_test install_tools install dev test ci flake8 clean clean_all

# Tools for setting up / syncing remote ubuntu server.

setup:
	scp ${HOME}/projects/scripts/_dircolors ${REMOTE_IP}:~/.dircolors
	scp ${HOME}/projects/scripts/_bash_custom ${REMOTE_IP}:~/.bash_custom
	ssh ${REMOTE_IP} "mkdir -p projects"
	ssh ${REMOTE_IP} "sudo apt-get install python3-venv"

dry_sync: clean
	rsync -anv ${PWD} ${REMOTE_IP}:~/projects/ --exclude-from='${HOME}/projects/scripts/rsync_exclude.txt'

sync: clean
	rsync -azP ${PWD} ${REMOTE_IP}:~/projects/ --exclude-from='${HOME}/projects/scripts/rsync_exclude.txt'

