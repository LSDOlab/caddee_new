from setuptools import setup, find_packages, Extension
import sys
import numpy as np


compile_args = []

if sys.platform.startswith('darwin'):
    compile_args=['-std=c++17', '-stdlib=libc++']
else:
    compile_args=['-std=c++17']

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name='caddee_new',
    version='0.1.0',
    author='Marius Ruh',
    author_email='mruh@ucsd.edu',
    license='LGPLv3+',
    keywords='caddee aircraft design framework',
    url='http://github.com/LSDOlab/lsdo_project_template',
    download_url='http://pypi.python.org/pypi/lsdo_project_template',
    description='A comprehensive aircraft design framework',
    long_description=long_description,
    long_description_content_type='text/markdown',
    packages=find_packages(),
    python_requires='>=3.7',
    platforms=['any'],
    install_requires=[
        'numpy',
        'pytest',
        'csdl @ git+https://github.com/LSDOlab/csdl.git', # 'sphinx-collections',
        'setuptools',
        'wheel',
        'twine',
        'myst-nb',
        'sphinx',
        'sphinx_rtd_theme',
        'sphinx-copybutton',
        'sphinx-autoapi',
        'numpydoc',
        'gitpython',
        # 'sphinxcontrib-collections @ git+https://github.com/anugrahjo/sphinx-collections.git', # 'sphinx-collections',
        'sphinxcontrib-bibtex',
    ],
    classifiers=[
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'License :: OSI Approved :: GNU Lesser General Public License v3 or later (LGPLv3+)',
        'Operating System :: OS Independent',
        'Intended Audience :: Developers',
        'Natural Language :: English',
    ],
)
