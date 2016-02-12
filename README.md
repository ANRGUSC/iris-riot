# USC ANRG 

This repository holds RIOT-OS that ANRG is using for the wireless robotic 
testbed. Please do NOT ever work on the `master` and `develop` branches. 
Create feature branches from the `develop` branch and merge your feature branch
into `develop` when it has been reviewed by another member in the project 
(please try to use a Gitlab Merge Request). `master` will serve as a mirror 
of the RIOT-OS public repo (advanced: this can also be used to manage your
GitHub fork of RIOT-OS in the same local directory).

## Managing Updates from RIOT's Github Repository

This repository's maintainer (currently Jason Tran) will periodically `rebase`
updates from [RIOT's master branch on github] (https://github.com/RIOT-OS/RIOT) 
into this repo's master branch instead of `merge` becuase a `merge` will show up as a 
`commit` which will pollute the history of our repo. The maintainer shall then `rebase` the 
`develop` branch onto `master`. If things get messy, the updates in `master` can be 
merged into `develop` to avoid messy conflict resolutions. To update your 
branch, jump to the next section for instructions on how to rebase your branch 
with respect to origin/master. This workflow has been adopted from [here] 
(http://stackoverflow.com/questions/7244321/how-to-update-a-github-forked-repository).

`git remote add upstream https://github.com/RIOT-OS/RIOT`

`git fetch upstream`

`git checkout master`

`git rebase upstream/master`

`git checkout develop`

`git rebase master`

The last line can be `git merge master` if conflict resolution is too messy 
(note this will add a commit). Once any conflicts are resolved from a `rebase`
or `merge` resolve any conflicts and push the changes to the Gitlab repo.

## Updating Feature Branches with Changes to `develop`
#### (REQUIRED before pushing new features into the master branch)

If you are ready to push a new feature to the master branch or simply update 
your feature branch, please use this git workflow guideline which uses `rebase`.
To understand the difference between a merging-based workflow vs. rebasing-based
workflow, read [this] 
(https://www.atlassian.com/git/tutorials/merging-vs-rebasing/conceptual-overview)

`git checkout your_working_branch`

`git rebase origin/master`

You may run into conflicts. If so, please [resolve] 
(https://help.github.com/articles/resolving-a-merge-conflict-from-the-command-line/)
them. When you are done, you can push your changes to the Gitlab repository. 
Note that you might have to `git push --force` (make sure you know what you are
doing). Ask around if you are having troubles.

## Merging Your Feature Branch to the Master Branch

When you have rebased your branch with updates to `master` and tested to see if
your features work with the updates, you are ready to push your changes to
`master`. First, it's best to always run your code by another member of the 
team before pushing your new features. Always do this locally first.

`git checkout master`

`git pull #update your local clone of origin/master`

`git merge your_working_branch`

Resolve any conflicts. If everything looks good, go ahead and `git push` to 
update `origin/master`.

## (END OF USC ANRG)
                          ZZZZZZ
                        ZZZZZZZZZZZZ
                      ZZZZZZZZZZZZZZZZ
                     ZZZZZZZ     ZZZZZZ
                    ZZZZZZ        ZZZZZ
                    ZZZZZ          ZZZZ
                    ZZZZ           ZZZZZ
                    ZZZZ           ZZZZ
                    ZZZZ          ZZZZZ
                    ZZZZ        ZZZZZZ
                    ZZZZ     ZZZZZZZZ       777        7777       7777777777
              ZZ    ZZZZ   ZZZZZZZZ         777      77777777    77777777777
          ZZZZZZZ   ZZZZ  ZZZZZZZ           777     7777  7777       777
        ZZZZZZZZZ   ZZZZ    Z               777     777    777       777
       ZZZZZZ       ZZZZ                    777     777    777       777
      ZZZZZ         ZZZZ                    777     777    777       777
     ZZZZZ          ZZZZZ    ZZZZ           777     777    777       777
     ZZZZ           ZZZZZ    ZZZZZ          777     777    777       777
     ZZZZ           ZZZZZ     ZZZZZ         777     777    777       777
     ZZZZ           ZZZZ       ZZZZZ        777     777    777       777
     ZZZZZ         ZZZZZ        ZZZZZ       777     777    777       777
      ZZZZZZ     ZZZZZZ          ZZZZZ      777     7777777777       777
       ZZZZZZZZZZZZZZZ            ZZZZ      777      77777777        777
         ZZZZZZZZZZZ               Z
            ZZZZZ

The friendly Operating System for IoT!

RIOT is a real-time multi-threading operating system that supports a range of
devices that are typically found in the Internet of Things (IoT): 
8-bit, 16-bit and 32-bit microcontrollers.

RIOT is based on the following design principles: energy-efficiency, real-time
capabilities, small memory footprint, modularity, and uniform API access,
independent of the underlying hardware (this API offers partial POSIX
compliance).

RIOT is developed by an international open source community which is
independent of specific vendors (e.g. similarly to the Linux community).
RIOT is licensed with LGPLv2.1, a copyleft license which fosters
indirect business models around the free open-source software platform
provided by RIOT, e.g. it is possible to link closed-source code with the
LGPL code.

## FEATURES

RIOT is based on a microkernel architecture, and provides features including,
but not limited to:

* a preemptive, tickless scheduler with priorities
* flexible memory management
* high resolution, long-term timers
* support for AVR, MSP430, MIPS, ARM7, and ARM Cortex-M on over 80 boards
* the native port allows to run RIOT as-is on Linux, BSD, and MacOS. Multiple
  instances of RIOT running on a single machine can also be interconnected via
  a simple virtual Ethernet bridge
* IPv6
* 6LoWPAN (RFC4944, RFC6282, and RFC6775)
* UDP
* RPL (storing mode, P2P mode)
* CoAP
* CCN-Lite


## GETTING STARTED
* You want to start the RIOT? Just follow our [quickstart guide](http://doc.riot-os.org/index.html#the-quickest-start) or the [getting started documentation](http://doc.riot-os.org/getting-started.html).
* The RIOT API itself can be built from the code using doxygen. The latest
  version is uploaded daily to http://riot-os.org/api.

## KNOWN ISSUES
* With latest GCC version (>= 6) platforms based on some ARM platforms will
  raise some warnings, leading to a failing build
  (see https://github.com/RIOT-OS/RIOT/issues/5519).
  As a workaround, you can compile with warnings not being treated as errors:
  `WERROR=0 make`

### USING THE NATIVE PORT WITH NETWORKING
If you compile RIOT for the native cpu and include the `netdev_tap` module,
you can specify a network interface like this: `PORT=tap0 make term`

#### SETTING UP A TAP NETWORK
There is a shellscript in `RIOT/dist/tools/tapsetup` called `tapsetup` which
you can use to create a network of tap interfaces.

*USAGE*
To create a bridge and two (or count at your option) tap interfaces:

    ./dist/tools/tapsetup/tapsetup [-c [<count>]]

## CONTRIBUTE

To contribute something to RIOT, please refer to the [development
procedures](https://github.com/RIOT-OS/RIOT/wiki/Development-procedures) and
read all notes for best practice.

## MAILING LISTS
* RIOT OS kernel developers list
 * devel@riot-os.org (http://lists.riot-os.org/mailman/listinfo/devel)
* RIOT OS users list
 * users@riot-os.org (http://lists.riot-os.org/mailman/listinfo/users)
* RIOT commits
 * commits@riot-os.org (http://lists.riot-os.org/mailman/listinfo/commits)
* Github notifications
 * notifications@riot-os.org
   (http://lists.riot-os.org/mailman/listinfo/notifications)

## LICENSE
* Most of the code developed by the RIOT community is licensed under the GNU
  Lesser General Public License (LGPL) version 2.1 as published by the Free
  Software Foundation.
* Some external sources, especially files developed by SICS are published under
  a separate license.

All code files contain licensing information.

For more information, see the RIOT website:

http://www.riot-os.org
