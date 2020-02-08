# Installation

Linux support only at the moment

## Summary:
Sentinet ros is designed in both python and c++ centered project. To get a feel for the project, visit nodes. These are were all executables are stored. 

### Include
C++ header files. To include a c++ header file, follow the prefix kermit

``` c++
#include "kermit/path.hpp"
```

### Nodes
A List of ros nodes 


# Build instructions

### Installing dependencies
```bash
$ sudo apt install cmake make curl libcurl4-gnutls-dev autoconf automake libtool g++ unzip libzmq3-dev

# Build the ros project
$ cd path/to/catkin_ws
$ git clone https://github.com/curmc/sentinet_ros ./src/kermit
$ catkin_make
```
  
# How to checkout / create / merge a branch

This project is restricted on master and release. You will not be able to commit to these branches. In order to make changes to the project, create a new branch

```bash
  $ git checkout -b my_new_branch
  $ git push -u origin my_new_branch

  ... Do stuff - add changes

  $ git add <specific files>
  $ git status
  $ git commit -m "my_new_branch: added changes"
  $ git push
```

## Updating your branch - rebase it, don't merge 
Every time you make changes, you need to rebase from master. To see why, remember master is recieving new changes all the time. If you push with an old master, this might cause merge conflicts
``` bash
  $ git fetch # not necessary, but I'm showing this here because it's useful to update your branches
  $ git pull --rebase
  $ git rebase origin/master
```
