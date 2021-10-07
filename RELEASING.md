# Releasing DART from Open Robotics fork

Ubuntu Launchpad PPA (Personal Package Archive) system is being used to release
the Open Robotics fork of DART. The debian metadata is being hosted the same
development branch `release-6.10`.

The procedure consists in two steps:

  1. Produce a local tarball of the sources and upload the tarball to Ubuntu's
     servers to built binaries
  1. Imported binaries from Ubuntu PPA into packages.osrfoundation.org

## Prerequisites

### Credentials

  * For being able to upload to Open Robotics Ubuntu PPA:
     * Account in [Ubuntu One](https://login.ubuntu.com/)
     * Access to https://launchpad.net/~openrobotics
       * New members can apply after login in launchpad in that same page

### Step 0: System Configurations

  * Enviroment variables to use as gbp configurations for changelogs
    (optionally in .bashrc):
    ```
    export DEBEMAIL="user@openrobotics.org
    export DEBFULLNAME="Your name"
    ```

  * Software installation: [debian git-buildpackage
    tool](https://wiki.debian.org/PackagingWithGit) and dput (to upload
    packages to PPA)
    ```
    apt-get install -y git-buildpackage dput
    ```

## Step 1: Releasing into PPA

The first step will create a source package from DART git checkout in the local
system and upload it to Open Robotics PPA.

**One Ubuntu distribution needs to be released at a time**. The information
about target distribution (bionic) goes only in the Changelog entry.

**Focal is not supported to be imported into packages.osrfoundation.org since we 
are using Ubuntu's packages**

A copy of [this fork](https://github.com/ignition-forks/dart) is required to be in the system.

```
cd dart
git checkout release-6.10
```

### Update changelog

 1. ```
     gbp dch --ignore-branch --no-git-author -D bionic --force-distribution \
             --new-version=6.10.0-osrf<new-open-robotics-release-version>~$(date +%Y%m%d)~bionic~$(git rev-parse HEAD)  \
             --commit-msg 'New OSRF testing release' --commit
    ```

### Releasing in Ubuntu PPA

After updating the changelog, the directory is ready to generate the source package.

#### Generate source package

 1. Build source package files using gbp
    ```
    gbp buildpackage -S
    ```

#### Upload source package to Ubuntu's PPA

 1. Upload source package (source package files are located in the parent directory `..`)
    ```
    dput ppa:openrobotics/dart ../dart6_*_source.changes
    ```

 1. If everything is correct, update the repository
    ```
    git push origin release-6.10
    ```

The last command will upload the source package to openrobotics PPA and will create the binaries
for the selected Ubuntu distribution in all arches (supported architectures can be configured in PPA)

## Step 2: Copy packages to packages.osrfoundation.org

### Update reprepro importer config

Create a PR against the `refactor_osrfbuild` branch in reprepro-updater repository and change the 
[SourceVersion field to the new one released in the PPA](https://github.com/ros-infrastructure/reprepro-updater/blob/refactor_osrfbuild/config/packages.osrfoundation.org/openrobotics_dart_packages_from_ppa.yaml#L8).

### Run reprepro-importer Jenkins test

Launch a build of [reprepro_importer](https://build.osrfoundation.org/job/reprepro_importer) with the
following parameters:

 * `RTOOLS_BRANCH` : `master`
 * `UPLOAD_TO_REPO` : `ubuntu_testing` (playground repository)
 * `REPREPRO_IMPORT_YAML_FILE` : `openrobotics_dart_packages_from_ppa.yaml`
 * `COMMIT` : checked (to perform real import on the playground repo)

### Run reprepro-importer real import

Run the same Jenkins job changing the parameter`UPLOAD_TO_REPO`to *ubuntu_stable*.
Run without `COMMIT` set to check that simulation is fine. If everything is good, 
re-run with `COMMIT` checked.
