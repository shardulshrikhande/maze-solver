# Using main_pull_reset.sh
This script pulls the latest version on the main branch and makes all python scripts in the subdirectories executable. This is incredibly useful when pulling changes onto the robot which cause file to loose their executable status. To use the script, follow the below process where make_executable.sh is located.
```
chmod +x make_executable.sh
./make_executable.sh
```

# Using make_executable.sh
This script makes all python scripts in the subdirectories executable. This is incredibly useful when pulling changes onto the robot which cause file to loose their executable status. To use the script, follow the below process where make_executable.sh is located.
```
chmod +x make_executable.sh
./make_executable.sh
```

# GitHub Tips
To get the most recent version of a branch onto the Pi, the following line can be used within the catkin workspace once configured:
```
git fetch origin
git checkout <branch_name>
```

This process can fail if there are conflicts between the local version and the version on Bitbucket.
In the case where this occurs and any of the code (tracked or untracked) on the Pi is not desired to be kept, run the following:
```
git fetch origin
git reset --hard origin/<branch_name>
```

An IDE with source control built in (such as [Visual Studio Code](https://code.visualstudio.com/)) makes interacting with git easier for beginners but some of the common git commands are below for reference.

Change the branch (Changes all files to the versions on that branch):
```
git checkout <branch_name>
```

Create a branch:
```
git branch <branch_name>
```

Show file changes since last commit:
```
git diff <file>
```