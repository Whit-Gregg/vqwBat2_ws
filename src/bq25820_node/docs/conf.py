import os

project = "bq25820_node"

extensions = ["breathe", "exhale"]
templates_path = ["_templates"]
exclude_patterns = []
html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]

# Doxygen XML (generated into docs/_build/doxygen/xml)
breathe_projects = {
    project: os.path.abspath("_build/doxygen/xml"),
}
breathe_default_project = project

# Exhale auto-generates API pages from Doxygen XML
exhale_args = {
    "containmentFolder": "./api",
    "rootFileName": "library_root.rst",
    "rootFileTitle": "API Reference",
    "doxygenStripFromPath": "../",
    "createTreeView": True,
    "generateBreatheFileDirectives": True,
}
