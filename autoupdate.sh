#!/bin/bash

read -p "Please Make sure your current directory is isaac-ros. Proceed with commit? (y/n) " confirm
if [[ $confirm != [yY] ]]; then exit; fi

git add .
read -p "Proceed with commit? (y/n) " confirm
if [[ $confirm != [yY] ]]; then exit; fi

git commit -m "Update Document"
read -p "Proceed with push? (y/n) " confirm
if [[ $confirm != [yY] ]]; then exit; fi

git push
read -p "Proceed with mkdocs build? (y/n) " confirm
if [[ $confirm != [yY] ]]; then exit; fi

mkdocs build
read -p "Proceed with mkdocs gh-deploy? (y/n) " confirm
if [[ $confirm != [yY] ]]; then exit; fi

mkdocs gh-deploy