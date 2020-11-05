# Releasing DART from Open Robotics fork

The idea is to use [Debian git-buildpackage tool](https://wiki.debian.org/PackagingWithGit)
to maintain a fork of DART that imports changes from azeey DART and merge the
Debian metadata.

Once that is ready, a source package can be generated and uploaded to Ubuntu's
PPA to generate the Debian binaries. Afterwards, they are imported into
packages.osrfoundation.org

## Prerequisites

### Credentials

  * Upload to Ubuntu's PPA: 
     * Account in Ubuntu launchpad
     * Permissions on https://launchpad.net/~j-rivero/dart

### Configurations

  * gbp configurations for changelogs (optionally in .bashrc):
    ```
    export DEBEMAIL="user@openrobotics.org
    export DEBFULLNAME="Jose Luis Rivero"
    ```

### Tools installation

  * `apt-get install -y git-buildpackage dput`

## Releasing 

### Update changelog

 1. `gbp dch --ignore-branch --no-git-author -D bionic --force-distribution --new-version=6.10.0~osrf6~$(date +%Y-%m-%d)~$(git rev-parse HEAD) --commit-msg 'New OSRF testing release' --commit` (check changelog by running `git diff HEAD~1`)

### Upload changes in releasing
 1. `git push origin azeey/friction_per_shape_more_params`

### Releasing in Ubuntu PPA
 
#### Generate source package file

 1. `gbp buildpackage -S`

#### Upload source package to Ubuntu's PPA

 1. `dput ppa:j-rivero/dartsim-openrobotics-testing ../dart6_*_source.changes`
