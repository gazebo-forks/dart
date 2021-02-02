# Releasing DART from Open Robotics fork

The idea is to use [Debian git-buildpackage tool](https://wiki.debian.org/PackagingWithGit)
to maintain a fork of DART that imports changes from azeey DART and merge the
Debian metadata.

Once that is ready, a [source
package](https://wiki.debian.org/Packaging/SourcePackage) can be generated and
uploaded to Ubuntu's PPA (Personal Package Archive) to generate the Debian
binaries. Afterwards, they are imported into packages.osrfoundation.org

## Prerequisites

### Credentials

  * Upload to Ubuntu's PPA:
     * Account in [Ubuntu One](https://login.ubuntu.com/)
     * Access to https://launchpad.net/~openrobotics
       * New members can apply after login in launchpad in that same page

### System Configurations

  * gbp configurations for changelogs (optionally in .bashrc):
    ```bash
    export DEBEMAIL="user@openrobotics.org
    export DEBFULLNAME="Your name"
    ```

  * Software installation
    ```bash
    apt-get install -y git-buildpackage dput
    ```

## Step 1: Releasing into PPA

The first step will create a source package from DART git checkout in the local
system and upload it to Open Robotics PPA.

**One Ubuntu distribution needs to be released at a time**. The information
about target distribution (i.e Focal) goes only in the Changelog entry. To
release multiple distribution, repeat this step 1 changing the changelog entry
and uploading the new source package to the PPA.

A copy of [this fork](https://github.com/ignition-forks/dart) is required to be in the system.

```bash
cd dart
git checkout azeey/friction_per_shape_more_params
```
`
### Update changelog, push changes

 1. `gbp dch --ignore-branch --no-git-author -D <UBUNTU_DISTRO> --force-distribution --new-version=6.10.0~osrf6~$(date +%Y-%m-%d)~$(git rev-parse HEAD) --commit-msg 'New OSRF testing release' --commit`
    (change UBUNTU_DISTRO by the target distribution name, i.e: focal. Check changelog by running `git diff HEAD~1`)
 1. `git push origin azeey/friction_per_shape_more_params`

### Releasing in Ubuntu PPA

After updating the changelog, the directory is ready to generate the source package.

#### Generate source package file

 1. `gbp buildpackage -S`

#### Upload source package to Ubuntu's PPA

 1. `dput ppa:openrobotics/dartsim-openrobotics-testing ../dart6_*_source.changes`


The last command will upload the source package to openrobotics PPA and will create the binaries
for the selected Ubuntu distribution in all arches (supported architectures can be configured in PPA)

## Step 2: Copy packages to packages.osrfoundation.org

TODO
