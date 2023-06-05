from setuptools import setup, find_packages, Extension
import sys
import numpy as np

from setuptools import setup, Extension
from Cython.Build import cythonize

compile_args = []

if sys.platform.startswith('darwin'):
    compile_args=['-std=c++17', '-stdlib=libc++']
else:
    compile_args=['-std=c++17']

# list_of_pyx_names = [
#     ('cython', 'get_open_uniform'),
#     ('cython', 'get_open_uniform_py'),
#     ('cython', 'basis0'),
#     ('cython', 'basis1'),
#     ('cython', 'basis2'),
#     ('cython', 'basis_matrix_curve'),
#     ('cython', 'basis_matrix_curve_py'),
#     ('cython', 'basis_matrix_surface'),
#     ('cython', 'basis_matrix_surface_py'),
#     ('cython', 'basis_matrix_volume'),
#     ('cython', 'basis_matrix_volume_py'),
#     ('cython', 'surface_projection'),
#     ('cython', 'surface_projection_py'),
#     ('cython', 'volume_projection'),
#     ('cython', 'volume_projection_py'),
#     ('cython', 'curve_projection'),
#     ('cython', 'curve_projection_py'),
# ]

# ext_modules = []
# packages = []
# for name_list in list_of_pyx_names:
#     ext_name = 'caddee/core'
#     source_name = 'caddee/core'
#     packages.append('{}.{}'.format('caddee', name_list[0]))
#     for name_part in name_list:
#         ext_name = '{}.{}'.format(ext_name, name_part)
#         source_name = '{}/{}'.format(source_name, name_part)
#     source_name = source_name + '.pyx'
#     ext_modules = ext_modules + cythonize(
#         Extension(
#             name=ext_name,
#             sources=[source_name],
#             language='c++',
#             extra_compile_args=compile_args,
#             include_dirs=[np.get_include()],
#         ),
#         annotate=True,
#         build_dir='build',
#     )

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
