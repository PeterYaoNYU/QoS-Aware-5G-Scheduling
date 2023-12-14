# QoS-aware 5G scheduling
In this project, we identify the potential problems of RadioSaber, and propose a simple solution. 

## Problem
To verify the problems with experiments on your own machine, go to /ran-sched-experiments, you will find more instructions under the folder /score_exp (this name does not have any implication. Originally the experiment design used score, but now it has nothing to do with score)

## Solution
To verify the effectiveness of the solution, please go to /ran-sched-experiments, you will find more instructions under the folder /improve_v1

## Key Modifications
We changed rather dramatically the original RadioSaber implementaiton. The most important modifications can be found in the folloing files:
* /src/device/ENodeB.cpp
* /src/protocalStack/mac/packetScheduler/downlink-transport-scheudler.cpp
* /src/LTE-Sim.cpp


## Instructions (to group members)

Always check at least that there are no compilation errors. Simply remove the `LTE-Sim` executable and do `make -j8` and see if the compilation succeeds.

### Git operations

For small changes that do not need review, directly modify the `master` branch. However, always pull before pushing. Do `git fetch origin` and `git merge origin/master` and solve any potential conflicts, then do `git push -u origin`.

For large changes that need review, do it on a different branch.

```bash
git fetch origin
git merge origin/master  # Update to the latest master branch
git checkout -b [feature-branch]  # Create and checkout to a new branch
```

Replace `[feature-branch]` with a customized branch name that is instructive. Make your changes in that branch. Fetch and merge the master branch frequently to keep your branch up-to-date and avoid too much conflicts. When you are done with your modifications, do `git push -u origin [feature-branch]`. Then go to GitHub and make a pull request.

### Style guide

Regardless of what format the simluator previously follows, we will stick to [Google C++ format](https://google.github.io/styleguide/cppguide.html) for any further changes made. One should install `clang-format` and format the code prior to any code commit.

```bash
sudo apt install clang-format  # Install clang formatting tool
git clang-format --style=file  # Do this prior to any commit
```

