# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.

import os
import sys

sys.path.insert(0, os.path.abspath(".."))

import sphinx_rtd_theme


# -- Project information -----------------------------------------------------

project = "Autonomy Stack"
copyright = "2026, Rumarino Team"
author = "Rumarino Team"

version = "0.1"
release = "0.1.0"


# -- General configuration ---------------------------------------------------

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.coverage",
    "sphinx.ext.githubpages",
    "sphinx.ext.napoleon",
    "sphinx.ext.todo",
    "sphinxcontrib.mermaid",
    "sphinx_rtd_theme",
]

templates_path = ["_templates"]
source_suffix = ".rst"
master_doc = "index"
root_doc = "index"
language = "en"

exclude_patterns = [
    "_build",
    "Thumbs.db",
    ".DS_Store",
    ".venv-docs",
    ".venv-docs/**",
    "**/.venv-docs/**",
]
pygments_style = None

todo_include_todos = True


# -- Options for HTML output -------------------------------------------------

html_theme = "sphinx_rtd_theme"
html_logo = "images/logo.svg"
html_static_path = ["_static"]

html_theme_options = {
    "logo_only": True,
    "prev_next_buttons_location": "bottom",
    "style_external_links": True,
    "style_nav_header_background": "#016A91",
    "collapse_navigation": True,
    "sticky_navigation": True,
    "navigation_depth": 4,
    "includehidden": True,
    "titles_only": False,
}

htmlhelp_basename = "autonomy_stack_doc"
html_css_files = ["custom.css"]


# -- Options for LaTeX output ------------------------------------------------

latex_elements = {}

latex_documents = [
    (
        master_doc,
        "autonomy_stack.tex",
        "Autonomy Stack Documentation",
        author,
        "manual",
    ),
]


# -- Options for manual page output ------------------------------------------

man_pages = [
    (master_doc, "autonomy_stack", "Autonomy Stack Documentation", [author], 1)
]


# -- Options for Texinfo output ----------------------------------------------

texinfo_documents = [
    (
        master_doc,
        "autonomy_stack",
        "Autonomy Stack Documentation",
        author,
        "autonomy_stack",
        "ROS 2 autonomy stack documentation template.",
        "Miscellaneous",
    ),
]


# -- Options for Epub output -------------------------------------------------

epub_title = project
epub_exclude_files = ["search.html"]
