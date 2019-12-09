#!/bin/bash

# Make sure you have the latest version of the repo
echo
git pull
echo

# Ask the user for login details
read -p 'https://github.com/chaliburton/Udacity-ND-Self-Driving-Car-Project-5-Extended-Kalman-Filters-Project-.git' upstreamVar
read -p 'chaliburton: ' userVar
read -p 'chris.haliburton@gmail.com' emailVar

echo
echo Thank you $userVar!, we now have your credentials
echo for upstream $upstreamVar. You must supply your password for each push.
echo

echo setting up git

git config --global user.name $userVar
git config --global user.email $emailVar
git remote set-url origin $upstreamVar
echo

echo Please verify remote:
git remote -v
echo

echo Please verify your credentials:
echo username: `git config user.name`
echo email: `git config user.email`
