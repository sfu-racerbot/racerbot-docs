# Racerbot - Intro to Git/GitHub

## What is Version Control
Version control is a system that records changes to files over time so that you can recall and revert to specific versions later on, similar to an undo button for your project. It also allows users to work on the same file simultaneously mostly without errors (We will get to that later). Users can write code, share it, review each other's work and roll back to previous versions of code.

## What is Git?
Git is a free, open-source distributed version control system created by Linus Torvalds (The guy who made Linux) in 2005. It runs entirely on your local machine and tracks changes to files in a project folder called a repository (or repo for short).

### Key Features
- **Snapshots:** Git stores a snapshot of your project with every commit, allowing you to roll back to a previous version if you break the latest one.
- **Local Operation:** All Git operations are performed locally on your machine, no internet needed.
- **Branching:** With Git you can easily make isolated branches to experiment without affecting your main repo.
- **Data Integrity:** Git maintains data integrity through the use of checksums.

## What is GitHub?
GitHub is a cloud-based hosting platform for your Git repositories. While Git is the tool you run on your computer, GitHub is the website where you store and share your code online. GitHub has all the features of Git and more.

### Key Features
- **Remote repo hosting:** Your repo now lives in the cloud and is accessible from anywhere as long as you have an internet connection.
- **Pull Requests:** A way to propose changes and get them reviewed by others on the team before merging.
- **Issue:** A built in bug tracker and task list for your project.
- **Actions:** Automated workflows for CI/CD testing.
- **Organizations and Teams:** Allows you to manage access permissions for users.

## Git vs GitHub

| Feature | Git | GitHub |
|---|---:|---:|
| What is it? | Software on your computer | A cloud based version control platform |
| Works offline? | Yes | No |
| Where is Code Stored? | Locally on your machine | Remotely in the cloud |
| Main Use? | Track local changes | Track changes, share code and collaborate online |
| Creators/Owners | Created by Linus Torvalds, now an Open Source Software | Currently owned by Microsoft |

## Core Concepts
- **Repository (Repo)** - A repository is a folder that Git is tracking. It contains your project files plus a hidden `.git` folder that stores all the history and configuration.
  - Local repository - Lives on your computer.
  - Remote repository - Lives on GitHub (or another server).
- **Commit** - A commit is a saved snapshot of your project at a point in time. Each commit has a unique ID (called a hash), a timestamp, the author's name, and a message describing what changed.
- **Branch** - A branch is an independent line of development. The default branch is usually called `main`, `master`, or `dev`.
- **Merge** - Merging takes the changes from one branch and integrates them into another.
- **Clone** - Cloning creates a full copy of a remote repository on your local machine, including all its history.
- **Push** - Sends your local commits to the remote repository on GitHub.
- **Pull** - Fetches changes from the remote repository and merges them into your local branch.
- **Staging Area (Index)** - Before you commit, you add files to the staging area. This lets you choose exactly which changes to include in the next commit.

## Common Git Commands

### Setup & Configuration
| Command | Description |
|---|---|
| `git config --global user.name "Name"` | Set your name (shown in commits) |
| `git config --global user.email "yourname@email.com"` | Set your email (must match GitHub) |
| `git config --list` | View all your current Git settings |

### Starting a Project
| Command | Description |
|---|---|
| `git init` | Initialize a new Git repo in the current folder |
| `git clone <url>` | Clone a remote repo to your machine |

### Common Commands
| Command | Description |
|---|---|
| `git status` | Show which files have changed, staged, or are untracked |
| `git add <file>` | Stage a specific file for the next commit |
| `git add .` | Stage ALL changed files in the current directory |
| `git commit -m "message"` | Save staged changes with a descriptive message |
| `git log` | Show the commit history (press Q to exit) |
| `git log --oneline` | Show a compact one-line commit history |
| `git diff` | Show unstaged changes in your working files |

### Branching & Merging
| Command | Description |
|---|---|
| `git branch` | List all local branches |
| `git branch <name>` | Create a new branch |
| `git checkout <name>` | Switch to an existing branch |
| `git checkout -b <name>` | Create a new branch AND switch to it |
| `git merge <branch>` | Merge the specified branch into the current branch |
| `git branch -d <name>` | Delete a branch (destructive) |

### Working with GitHub (Remote)
| Command | Description |
|---|---|
| `git remote -v` | Show connected remote repositories |
| `git remote add origin <url>` | Link your local repo to a GitHub repo |
| `git push origin <branch>` | Push your branch to GitHub |
| `git push -u origin main` | Push and set upstream (first push shortcut) |
| `git pull` | Fetch and merge changes from GitHub |
| `git fetch` | Download changes from GitHub without merging |

### Undoing Things
| Command | Description |
|---|---|
| `git restore <file>` | Discard unstaged changes to a file |
| `git restore --staged <file>` | Unstage a file (keep changes in working dir) |
| `git revert <commit-hash>` | Create a new commit that undoes a past commit |
| `git reset --hard HEAD~1` | Permanently undo last commit (destructive) |
| `git stash` | Temporarily save uncommitted changes |
| `git stash pop` | Re-apply stashed changes |

## Writing Good Commit Messages
It's essential to write a clear commit message that makes it easy to understand the project's history at a glance. Here are some examples:

**Good Messages:**
```
git commit -m "Add obstacle avoidance to LiDAR module"
git commit -m "Fix motor calibration off-by-one error"
git commit -m "Refactor motor driver for cleaner abstraction"
```

**Bad Messages:**
```
git commit -m "stuff"
git commit -m "fix"
git commit -m "asdfghjkl"
```

## Typical Workflow
Here is the standard workflow you will use when contributing to a Raverbot project on GitHub:

```
# Clone the repo (first time only)
git clone https://github.com/sfu-racerbot/racerbot_ws.git

# Create a feature branch
git checkout -b feature/motor-controller

# Make changes, then stage and commit
git add . && git commit -m "Add PID tuning for motor controller"

# Push to GitHub
git push origin feature/motor-controller

# Open a Pull Request on GitHub and request a review
```

## The .gitignore File
A `.gitignore` file tells Git which files and folders to ignore. These are things you never want to commit, like build artifacts, secrets, or IDE config files.

## Resources
- [Git Official Documentation](https://git-scm.com/docs)
- [GitHub Docs](https://docs.github.com/en)
- [Learn Git Branching](https://learngitbranching.js.org/)
- [GitHub Skills](https://learn.github.com/skills)
- [Git Cheat Sheet (PDF)](https://education.github.com/git-cheat-sheet-education.pdf)
