#!/bin/bash

# Git Repository Setup Script for Error Estimation Project
# This script initializes a Git repository and sets up GitLab integration

set -e  # Exit on any error

echo "🚀 Setting up Git Repository for Error Estimation Project"
echo "============================================================"

# Check if we're in the right directory
if [[ ! -d "uwb_error_estimation" ]] || [[ ! -d "ble_error_estimation" ]]; then
    echo "❌ Error: Please run this script from the error_estimation root directory"
    echo "   Expected directories: uwb_error_estimation/ and ble_error_estimation/"
    exit 1
fi

# Initialize Git repository if not already initialized
if [[ ! -d ".git" ]]; then
    echo "📦 Initializing Git repository..."
    git init
    echo "✅ Git repository initialized"
else
    echo "📦 Git repository already exists"
fi

# Configure Git user (you can change these)
echo "👤 Configuring Git user..."
read -p "Enter your name: " GIT_NAME
read -p "Enter your email: " GIT_EMAIL

git config user.name "$GIT_NAME"
git config user.email "$GIT_EMAIL"
echo "✅ Git user configured"

# Add GitLab remote
echo "🌐 Setting up GitLab remote..."
echo ""
echo "Please create a new project on GitLab named 'error_estimation'"
echo "GitLab URL: [redacted]"
echo ""
read -p "Have you created the GitLab project? (y/n): " CREATED_PROJECT

if [[ "$CREATED_PROJECT" == "y" || "$CREATED_PROJECT" == "Y" ]]; then
    # Add remote
    read -p "Enter GitLab repository URL: " GITLAB_URL
    
    if git remote | grep -q "origin"; then
        echo "📍 Updating existing origin remote..."
        git remote set-url origin "$GITLAB_URL"
    else
        echo "📍 Adding GitLab remote..."
        git remote add origin "$GITLAB_URL"
    fi
    
    echo "✅ GitLab remote configured: $GITLAB_URL"
else
    echo "⚠️  Skipping remote setup. You can add it later with:"
    echo "   git remote add origin <your_gitlab_url>"
fi

# Add all files to Git
echo "📁 Adding files to Git..."
git add .gitignore
git add README.md
git add setup_git_repo.sh

# Add UWB project files
echo "📡 Adding UWB error estimation files..."
git add uwb_error_estimation/

# Add BLE project files  
echo "📱 Adding BLE error estimation files..."
git add ble_error_estimation/

# Check status
echo "📋 Git status:"
git status

# Create initial commit
echo "💾 Creating initial commit..."
read -p "Enter commit message (or press Enter for default): " COMMIT_MSG
if [[ -z "$COMMIT_MSG" ]]; then
    COMMIT_MSG="Initial commit: UWB and BLE error estimation projects"
fi

git commit -m "$COMMIT_MSG"
echo "✅ Initial commit created"

# Set up main branch
echo "🌿 Setting up main branch..."
git branch -M main

# Push to GitLab (if remote is configured)
if git remote | grep -q "origin"; then
    echo "🚀 Pushing to GitLab..."
    read -p "Push to GitLab now? (y/n): " PUSH_NOW
    
    if [[ "$PUSH_NOW" == "y" || "$PUSH_NOW" == "Y" ]]; then
        git push -u origin main
        echo "✅ Pushed to GitLab successfully!"
    else
        echo "⚠️  Skipping push. You can push later with:"
        echo "   git push -u origin main"
    fi
fi

echo ""
echo "🎉 Git repository setup complete!"
echo "=============================================="
echo ""
echo "📁 Repository structure:"
echo "   error_estimation/"
echo "   ├── uwb_error_estimation/  (UWB positioning)"
echo "   ├── ble_error_estimation/  (BLE positioning)"
echo "   ├── README.md              (Main documentation)"
echo "   └── .gitignore             (Git ignore rules)"
echo ""
echo "🔗 GitLab URL: [configure your repository URL]"
echo ""
echo "📋 Next steps:"
echo "   1. Make changes to your code"
echo "   2. git add ."
echo "   3. git commit -m 'Your commit message'"
echo "   4. git push"
echo ""
echo "🛠️  Useful Git commands:"
echo "   git status           # Check current status"
echo "   git log --oneline    # View commit history"
echo "   git branch           # List branches"
echo "   git remote -v        # View remotes"

