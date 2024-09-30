# System Requirement:

This document may only works for you if your os is: 

OS: Ubuntu 22.04

# Prerequistes:

Install Isaac-sim-4.0.0

Install MkDocs and Necessary Plugins(pip)
```
pip install mkdocs
pip install mkdocs-material
```

Change site_url in mkdocs.yml
```
site_url: https://<Your github username>.github.io/isaac-ros/
```

# Build and Serve Your Website Locally
```
cd /isaac-ros
mkdocs serve
```

# Deplot Your Website on Github

```
cd /isaac-ros
mkdocs gh-deploy
```
# Made changes

```
mkdocs build
mkdocs gh-deploy
```