# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'assignment_2_2023'
copyright = '2024, Daniele Rialdi'
author = 'Daniele Rialdi'
release = '1.0'

# for c++
import os
import subprocess 
import sys
sys.path.insert(0, os.path.abspath('../'))

html_context = {
    'author': author,
    'version': release,
}
show_authors = True
# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['sphinx.ext.autodoc', 
'sphinx.ext.doctest', 
'sphinx.ext.intersphinx', 
'sphinx.ext.todo', 
'sphinx.ext.coverage', 
'sphinx.ext.mathjax', 
'sphinx.ext.ifconfig', 
'sphinx.ext.viewcode', 
'sphinx.ext.githubpages', 
"sphinx.ext.napoleon",
'sphinx.ext.inheritance_diagram',
'breathe'
]

autodoc_mock_imports = ["rospy", "actionlib", "actionlib_msgs", "assignment_2_2023", "nav_msgs"]
templates_path = ['_templates']
exclude_patterns = []

source_suffix = '.rst' 
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_static_path = ['_static']
