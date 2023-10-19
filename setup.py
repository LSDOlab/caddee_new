from setuptools import setup, find_packages

import codecs
import os.path

def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), 'r') as fp:
        return fp.read()

def get_version(rel_path):
    for line in read(rel_path).splitlines():
        if line.startswith('__version__'):
            delim = '"' if '"' in line else "'"
            return line.split(delim)[1]
    else:
        raise RuntimeError("Unable to find version string.")

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name='caddee',
    version=get_version('caddee/__init__.py'),
    author='Marius Ruh',
    author_email='mruh@ucsd.edu',
    license='LGPLv3+',
    keywords='aircraft design framework',
    url='http://github.com/LSDOlab/caddee',
    download_url='http://pypi.python.org/pypi/caddee',
    description='aircraft design framework',
    long_description=long_description,
    long_description_content_type='text/markdown',
    packages=find_packages(),
    python_requires='>=3.7',
    platforms=['any'],
    install_requires=[
        'numpy',
        'pytest',
        'myst-nb',
        'csdl @ git+https://github.com/LSDOlab/csdl.git',
        'python_csdl_backend @ git+https://github.com/LSDOlab/python_csdl_backend.git',
        'lsdo_geo @ git+https://github.com/LSDOlab/lsdo_geo.git',
        'sphinx',
        'sphinx_rtd_theme',
        'sphinx-copybutton',
        'sphinx-autoapi',
        'numpydoc',
        'gitpython',
        'sphinxcontrib-collections @ git+https://github.com/anugrahjo/sphinx-collections.git', # 'sphinx-collections',
        'sphinxcontrib-bibtex',
        'setuptools',
        'wheel',
        'twine',
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
        'Topic :: Documentation',
        'Topic :: Documentation :: Sphinx',
        'Topic :: Software Development',
        'Topic :: Software Development :: Documentation',
        'Topic :: Software Development :: Testing',
        'Topic :: Software Development :: Libraries',
    ],
)



# from setuptools import setup, find_packages, Extension
# import sys
# import numpy as np


# compile_args = []

# if sys.platform.startswith('darwin'):
#     compile_args=['-std=c++17', '-stdlib=libc++']
# else:
#     compile_args=['-std=c++17']

# with open("README.md", "r") as fh:
#     long_description = fh.read()

# setup(
#     name='caddee_new',
#     version='0.1.0',
#     author='Marius Ruh',
#     author_email='mruh@ucsd.edu',
#     license='LGPLv3+',
#     keywords='caddee aircraft design framework',
#     url='http://github.com/LSDOlab/caddee_new',
#     download_url='http://pypi.python.org/pypi/caddee_new',
#     description='A comprehensive aircraft design framework',
#     long_description=long_description,
#     long_description_content_type='text/markdown',
#     packages=find_packages(),
#     python_requires='>=3.7',
#     platforms=['any'],
#     install_requires=[
#         'numpy',
#         'pytest',
#         'csdl @ git+https://github.com/LSDOlab/csdl.git',
#         # 'python_csdl_backend @ git+https://github.com/LSDOlab/python_csdl_backend.git',
#         # # 'lsdo_geo @ git+https://github.com/LSDOlab/lsdo_geo.git',
#         # 'm3l @ git+https://github.com/LSDOlab/m3l.git',
#         'setuptools',
#         'wheel',
#         'twine',
#         'myst-nb',
#         'sphinx', # Mostly useful for developer but doesn't need to be installed for a user
#         'sphinx_rtd_theme',
#         'sphinx-copybutton',
#         'sphinx-autoapi',
#         'numpydoc',
#         'gitpython',
#         'sphinxcontrib-collections @ git+https://github.com/anugrahjo/sphinx-collections.git', # 'sphinx-collections',
#         'sphinxcontrib-bibtex',
#     ],
#     classifiers=[
#         'Programming Language :: Python',
#         'Programming Language :: Python :: 3.7',
#         'Programming Language :: Python :: 3.8',
#         'Programming Language :: Python :: 3.9',
#         'License :: OSI Approved :: GNU Lesser General Public License v3 or later (LGPLv3+)',
#         'Operating System :: OS Independent',
#         'Intended Audience :: Developers',
#         'Natural Language :: English',
#     ],
# )
