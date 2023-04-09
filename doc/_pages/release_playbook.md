---
title: Release Playbook
---

This document explains the steps related to publishing a version-numbered Drake
binary release.  It is intended only for use by the few Drake Developer experts
who regularly handle the release process.

We publish a minor release approximately once per month in the middle of the
calendar month, with version number is ``v1.N.0`` where ``N`` is monotonically
increasing.

# Minor releases

Begin this process around 1 week prior to the intended release date.

## Prior to release

1. Choose the next version number.
2. Create a local Drake branch named ``release_notes-v1.N.0`` (so that others
   can easily find and push to it after the PR is opened).
3. As the first commit on the branch, mimic the commit
   ([link](https://github.com/RobotLocomotion/drake/pull/14208/commits/674b84877bc08448b59a2243f3b910a7b6dbab43)
   from [PR 14208](https://github.com/RobotLocomotion/drake/pull/14208)
   in order to disable CI.  A quick way to do this might be:
   ```
   git fetch upstream pull/14208/head
   git cherry-pick 674b84877bc08448b59a2243f3b910a7b6dbab43
   ```
4. Push that branch and then open a new pull request titled:
   ```
   [doc] Add release notes v1.N.0
   ```
   Make sure that "Allow edits by maintainers" on the GitHub PR page is
   enabled (the checkbox is checked). Set label `release notes: none`.
5. For release notes, on an ongoing basis, add recent commit messages to the
   release notes draft using the ``tools/release_engineering/relnotes`` tooling.
   (Instructions can be found atop its source code: [``relnotes.py``](https://github.com/RobotLocomotion/drake/blob/master/tools/release_engineering/relnotes.py))
    1. On the first run, use ``--action=create`` to bootstrap the file.
       * The output is draft release notes in ``doc/_release-notes/v1.N.0.md``.
    2. On the subsequent runs, use ``--action=update`` to refresh the file.
       * Try to avoid updating the release notes to refer to changes newer than
       the likely release, i.e., if you run ``--update`` on the morning you're
       actually doing the release, be sure to pass the ``--target_commit=``
       argument to avoid including commits that will not be part of the tag.
6. For release notes, on an ongoing basis, clean up and relocate the commit
   notes to properly organized and wordsmithed bullet points. See [Polishing
   the release notes](#polishing-the-release-notes).
7. From time to time, merge ``upstream/master`` into your
   ``origin/release_notes-v1.N.0`` branch (so that it doesn't go stale).
   Never rebase or force-push to the branch.  We expect that several people
   will clone and push to it concurrently.
8. As the release is nearly ready, post a call for action for feature teams to
   look at the draft document and provide suggestions (in reviewable) or fixes
   (as pushes).
    1. To help ensure that the "newly deprecated APIs" section is accurate, grep
       the code for ``YYYY-MM-01`` deprecation notations, for the ``MM`` values
       that would have been associated with our +3 months typical period.

## Polishing the release notes

Here are some guidelines for bringing commit notes from the relnotes tool into
the main body of the document:

* Many commit messages can be cut down to their summary strings and used as-is.
* File geometry/optimization changes under the "Mathematical Program" heading,
  not the "Multibody" heading.
* Expand all acronyms (eg, MBP -> MultibodyPlant, SG -> SceneGraph).
* Commits can be omitted if they only affect tests or non-installed examples. {% comment %}TODO(jwnimmer-tri) Explain how to check if something is installed.{% endcomment %}
* In general you should mention deprecated/removed classes and methods using
  their exact name (for easier searching).
  * In the deprecation section you can provide the fully-qualified name as the
    whole line item; the meaning is clear from context.
  * This may mean having a long list of items for a single commit.  That is
    fine.

* We have four common grammatical forms for our commit messages:
  * Past tense ("Added new method foo") is acceptable
  * Noun phrase ("Ability to foo the bar") is acceptable
  * Imperative ("Add new method foo", i.e. PEP-8 style) is acceptable
  * Present tense ("Adds new method foo", i.e. Google styleguide style) is
    discouraged

* Use exactly the same wording for the boilerplate items:
  * Every dependency upgrade line should be "Upgrade libfoobar to latest
    release 1.2.3" or "Upgrade funrepo to latest commit".
  * Dependencies should be referred to by their workspace name.
  * Only one dependency change per line. Even if both meshcat and meshcat-python
    were upgraded in the same pull request, they each should get their own
    line in the release notes.

* Some features under development (eg, deformables as of this writing) may
  have no-release-notes policies, as their APIs although public are not yet
  fully supported.  Be sure to take note of which these are, or ask on
  `#platform_review` slack.
* Keep all bullet points to one line.
  * Using hard linebreaks to stay under 80 columns makes the bullet lists hard
    to maintain over time.

* Say "macOS" not "Mac" or "Apple" or etc.
* Say "SDFormat" not "SDF" nor "sdf".

## Cutting the release

<<<<<<< HEAD
1. Find a plausible nightly build to use:
=======
1. Find a plausible build to use
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
   1. Make sure <https://drake-jenkins.csail.mit.edu/view/Production/> is clean
   2. Make sure <https://drake-jenkins.csail.mit.edu/view/Nightly%20Production/>
      has nothing still running (modulo the ``*-coverage`` builds, which we can
      ignore)
   3. Open the latest builds from the following builds:
      1. <https://drake-jenkins.csail.mit.edu/view/Packaging/job/linux-focal-unprovisioned-gcc-bazel-nightly-packaging/>
      2. <https://drake-jenkins.csail.mit.edu/view/Packaging/job/linux-jammy-unprovisioned-gcc-bazel-nightly-packaging/>
      3. <https://drake-jenkins.csail.mit.edu/view/Packaging/job/mac-x86-monterey-unprovisioned-clang-bazel-nightly-packaging/>
      4. <https://drake-jenkins.csail.mit.edu/view/Packaging/job/mac-arm-monterey-unprovisioned-clang-bazel-nightly-packaging/>
   4. Check the logs for those packaging builds and find the URLs they posted
      to (open the latest build, go to "View as plain text", and search for
      ``drake/nightly/drake-20``), and find the date.  It will be ``YYYYMMDD``
      with today's date (they kick off after midnight).  All of the builds
      should have the same date. If not, wait until the following night.
   5. Use the
      ``tools/release_engineering/download_release_candidate`` tool with the
      ``--find-git-sha`` option to download and verify that all the nightlies
      are built from the same commit.  (It's usage instructions are atop its
      source code:
      [download_release_candidate.py](https://github.com/RobotLocomotion/drake/blob/master/tools/release_engineering/download_release_candidate.py).)
2. Launch the staging builds for that git commit sha:
   1. Open the following five Jenkins jobs (e.g., each in its own
      new browser tab):
      - [Linux Jenkins Wheel Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-focal-unprovisioned-gcc-wheel-staging-release/)
      - [macOS x86 Jenkins Wheel Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/mac-x86-monterey-unprovisioned-clang-wheel-staging-release/)
      - [macOS arm Jenkins Wheel Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/mac-arm-monterey-unprovisioned-clang-wheel-staging-release/)
      - [Focal Packaging Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-focal-unprovisioned-gcc-bazel-staging-packaging/)
      - [Jammy Packaging Staging](https://drake-jenkins.csail.mit.edu/view/Staging/job/linux-jammy-unprovisioned-gcc-bazel-staging-packaging/)
   2. In the upper right, click "log in" (unless you're already logged in). This
      will use your GitHub credentials.
   3. Click "Build with Parameters".
   4. Change "sha1" to the full **git sha** corresponding to ``v1.N.0`` and
      "release_version" to ``1.N.0`` (no "v").
   5. Click "Build"; each build will take around an hour, give or take.
   6. Note: The macOS wheel jobs will produce one `.whl` file, whereas the linux
      job will produce multiple `.whl` files (in the same job).
   7. Wait for all staging jobs to succeed.  It's OK to work on release notes
      finishing touches in the meantime, but do not merge the release notes nor
      tag the release until all five builds have succeeded.
3. Update the release notes to have the ``YYYYMMDD`` we choose, and to make
   sure that the nightly build git sha from the prior steps matches the
   ``newest_commit`` whose changes are enumerated in the notes.  Some dates
   are YYYYMMDD format, some are YYYY-MM-DD format; be sure to manually fix
   them all.
   1. Update the github links within ``doc/_pages/from_binary.md`` to reflect
      the upcoming v1.N.0 and YYYYMMDD.
4. Re-enable CI by reverting the commit you added way up above in step 3 of **Prior to release**.
5. Wait for the wheel builds to complete, and then download release artifacts:
   1. Use the
      ``tools/release_engineering/download_release_candidate`` tool with the
      ``--version`` option to download and verify all binaries.  (It's usage
      instructions are atop its source code:
      [download_release_candidate.py](https://github.com/RobotLocomotion/drake/blob/master/tools/release_engineering/download_release_candidate.py).)
<<<<<<< HEAD
6. Merge the release notes PR
=======

2. Update the release notes to have the ``YYYYMMDD`` we choose, and to make
   sure that the nightly build git sha from the prior step matches the
   ``newest_commit`` whose changes are enumerated in the notes.  Some dates
   are YYYYMMDD format, some are YYYY-MM-DD format; be sure to manually fix
   them all.
   1. Update the github links within doc/_pages/from_binary.md to reflect the
      upcoming v1.N.0 and YYYYMMDD.
3. Re-enable CI by reverting the commit you added in step 3.
4. Merge the release notes PR
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
   1. Take care when squashing not to accept github's auto-generated commit message if it is not appropriate.
   2. After merge, go to <https://drake-jenkins.csail.mit.edu/view/Documentation/job/linux-jammy-unprovisioned-gcc-bazel-nightly-documentation/> and push "Build now".
      * If you don't have "Build now" click "Log in" first in upper right.
<<<<<<< HEAD
7. Open <https://github.com/RobotLocomotion/drake/releases> and choose "Draft
   a new release".  Note that this page has neither history nor undo.  Be
   slow and careful!
   1. Tag version is: v1.N.0
   2. Target is: [the git sha from the `--find-git-sha` in step 1.v]
=======
5. Open <https://github.com/RobotLocomotion/drake/releases> and choose "Draft
   a new release".  Note that this page does has neither history nor undo.  Be
   slow and careful!
   1. Tag version is: v1.N.0
   2. Target is: [the git sha from above]
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
     *  You should select the commit from Target > Recent Commits. The search
        via commit does not work if you don't use the correct length.
   3. Release title is: Drake v1.N.0
   4. The body of the release should be forked from the prior release (open the
      prior release's web page and click "Edit" to get the markdown), with
      appropriate edits as follows:
      * The version number
   5. Into the box labeled "Attach binaries by dropping them here or selecting
<<<<<<< HEAD
      them.", drag and drop the 36 release files from
      ``/tmp/drake-release/v1.N.0``:
      - 12: 4 `.tar.gz` + 8 checksums
      - 6: 2 `.deb` + 4 checksums
      - 12: 4 linux `.whl` + 8 checksums
      - 3: 1 macOS x86 `.whl` + 2 checksums
      - 3: 1 macOS arm `.whl` + 2 checksums
   6. Choose "Save draft" and take a deep breath.
8. Once the documentation build finishes, release!
   1. Check that the link to drake.mit.edu docs from the GitHub release draft
      page actually works.
   2. Click "Publish release"
   3. Notify `@BetsyMcPhail` by creating a GitHub issue asking her to manually 
      tag docker images and upload the releases to S3. Be sure to provide her 
      with the binary date and release tag in the same ping.
=======
      them.", drag and drop the 6 release binary artifacts from above (the 2
      tarballs, and their 4 checksums).
   6. Choose "Save draft" and take a deep breath.
6. Once the documentation build finishes, release!
   1. Check that the link to drake.mit.edu docs from the GitHub release draft
      page actually works.
   2. Click "Publish release"
   3. Notify `@BetsyMcPhail` via a GitHub comment to manually tag docker images
      and upload the releases to S3. Be sure to provide her with the binary
      date, commit SHA, and release tag in the same ping.
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
   4. Announce on Drake Slack, ``#general``.
   5. Party on, Wayne.

## Post-release wheel upload

<<<<<<< HEAD
After tagging the release, you must manually upload a PyPI release.

If you haven't done so already, follow Drake's PyPI
[account setup](https://docs.google.com/document/d/17D0yzyr0kGH44eWpiNY7E33A8hW1aiJRmADaoAlVISE/edit#)
instructions to obtain a username and password.

1. Run ``twine`` to upload the wheel release, as follows:

   1. You will need your PyPI username and password for this. (Do not use drake-robot.)
   2. Run ``twine upload /tmp/drake-release/v1.N.0/*.whl``.
   3. Confirm that all of the uploads succeeded without any error messages in
      your terminal.
=======
1. Open the [tagged workspace](https://github.com/RobotLocomotion/drake/tree/v1.N.0/tools/workspace)
   (editing that URL to have the correct value for ``N``) and ensure that
   certain Drake-owned externals have sufficient tags:
   1. Open ``common_robotics_utilities/repository.bzl`` and find the ``commit =`` used.
      1. Open
         [ToyotaResearchInstitute/common_robotics_utilities](https://github.com/ToyotaResearchInstitute/common_robotics_utilities/releases)
         and check whether that commit already has an associated release tag.
      2. If not, then create a new release named ``v0.0.foo`` where ``foo`` is
         the 8-digit datestamp associated with the ``commit`` in question (i.e.,
         four digit year, two digit month, two digit day).
   2. Open ``models_internal/repository.bzl`` and find the ``commit =`` used.
      1. Open
         [RobotLocomotion/models](https://github.com/RobotLocomotion/models/releases)
         and check whether that commit already has an associated release tag.
      2. If not, then create a new release named ``v0.0.foo`` where ``foo`` is
         the 8-digit datestamp associated with the ``commit`` in question (i.e.,
         four digit year, two digit month, two digit day).
   3. Open ``optitrack_driver/repository.bzl`` and find the ``commit =`` used.
      1. Open
         [RobotLocomotion/optitrack-driver](https://github.com/RobotLocomotion/optitrack-driver/releases)
         and check whether that commit already has an associated release tag.
      2. If not, then create a new release named ``v0.0.foo`` where ``foo`` is
         the 8-digit datestamp associated with the ``commit`` in question (i.e.,
         four digit year, two digit month, two digit day).
   4. Open ``styleguide/repository.bzl`` and find the ``commit =`` used.
      1. Open [RobotLocomotion/styleguide](https://github.com/RobotLocomotion/styleguide/releases)
         and check whether that commit already has an associated release tag.
      2. If not, then create a new release named ``v0.0.foo`` where ``foo`` is
         the 8-digit datestamp associated with the ``commit`` in question (i.e.,
         four digit year, two digit month, two digit day).
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c

## Post-release tutorials updates

Upgrade our Deepnote-hosted tutorials to the latest release.  This requires
that you have "Edit" permission in the Deepnote project.  If you don't have
that yet, then ask for help on slack in the ``#releases`` channel.  Provide
the email address associated with your github account.

<<<<<<< HEAD
1. Post a new slack thread in ``#releases`` saying that you're beginning the
   tutorials deployment now (so that others are aware of the potentially-
   disruptive changes).
2. Open the tutorials [Dockerfile](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/%2FDockerfile):
=======
1. Open the tutorials [Dockerfile](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/%2FDockerfile):
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
   1. Edit the first line to refer to the YYYYMMDD for this release.
      1. For reference, the typical content is thus:
         ```
         FROM robotlocomotion/drake:jammy-20220929

         RUN apt-get -q update && apt-get -q install -y --no-install-recommends nginx-light xvfb && apt-get -q clean

         ENV DISPLAY=:1
         ```
      2. If the current content differs by more than just the date from the
         above template, ask for help on slack in the ``#releases`` channel.
   2. After editing the date, click the "Build" button in the upper right,
      and wait for the build to succeed.
      1. If the build fails due to an infrastructure flake, you'll need to
      tweak the Dockerfile before Deepnote will allow you to re-run the
      Build.  For example, add `&& true` to the end of a RUN line.
<<<<<<< HEAD
3. For reference (no action required), the
=======
2. For reference (no action required), the
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
   [requirements.txt](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/%2Frequirements.txt)
   file should have the following content:
   ```
   ipywidgets==7.7.0
   ```
<<<<<<< HEAD
4. For reference (no action required), the initialization notebook at
   [init.ipynb](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/Init%20notebook-5fcfe3fc0bd0403899baab3b6cf37a18)
   has this cell added the bottom, as a Drake-specific customization:
   ```
   %%bash
   /opt/drake/share/drake/setup/deepnote/install_nginx
   /opt/drake/share/drake/setup/deepnote/install_xvfb
   ```
   In case the display server is not working later on, this might be a good place to double-check.
   For Jammy we also needed to add ``cd /work`` atop the stanza that checks for
   ``requirements.txt`` to get it working again.
5. Copy the updated tutorials from the pinned Dockerfile release
   (in ``/opt/drake/share/drake/tutorials/...``) into the Deepnote project
   storage (``~/work/...``):
   1. Open [zzz_for_maintainers.ipynb](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/zzz_for_maintainers-fd55235184ab44289133abc40e94a5e0).
   2. Run each cell one by one, checking for errors as you go.
   3. Note the first cell will take 1-2 minutes to finish because Deepnote
      needs to boot the machine first.
6. Next you'll copy and run each notebook (in alphabetical order).
   Read all of these instructions before performing any of them.
   1. Caveats for specific notebook names:
      1. Do not run ``licensed_solvers_deepnote``; you don't have a license.
      2. Do not re-run ``zzz_for_maintainers``; you've already done that.
      3. The ``authoring_multibody_simulation`` notebook will appear to hang on
=======
3. For reference (no action required), the initialization notebook at
   [init.ipynb](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/%2Finit.ipynb)
   has this cell added the bottom, as a Drake-specific customization:
   ```
   %%bash
   /opt/drake/share/drake/setup/deepnote/install_xvfb
   ```
   In case the display server is not working later on, this might be a good place to double-check.
4. Copy the updated tutorials from the pinned Dockerfile release
   (in ``/opt/drake/share/drake/tutorials/...``) into the Deepnote project
   storage (``~/work/...``):
   1. Open [.for_maintainers.ipynb](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/%2F.for_maintainers.ipynb).
   2. Run each cell one by one, checking for errors as you go.
5. For almost all other notebooks (excluding the ``.for_maintainers`` notebook
   **and** excluding the ``licensed_solvers_deepnote`` notebook) one by one
   (probably in alphabetical order, for your sanity):
   1. Open the notebook and click "Run notebook".
      1. The ``authoring_multibody_simulation`` notebook will appear to hang on
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
         one of the middle cells where it uses JointSliders. It is _not_ hung,
         rather it is waiting for user input. Find the "Meshcat URL" link
         earlier in the notebook, click through to open Meshcat in a new tab,
         click "Open Controls", then click "Stop JointSliders" repeatedly until
         the option vanishes and the notebook completes.
<<<<<<< HEAD
      4. The ``rendering_multibody_plant`` sometimes crashes with an interrupted
         error. In that case, click through to the "Environment" gear in the
         right-hand panel, then into the ``init.ipynb`` notbook and re-run the
         initialization. Then go back to  ``rendering_multibody_plant`` and try
         again.
   2. To deploy run each of the ~2 dozen notebooks (i.e., do this step for
      ``authoring_leaf_system`` then ``authoring_multibody_simulation`` then
      ... etc.):
      1. In the right-hand panel of your screen, take note that each notebook
         appears in two places -- in "NOTEBOOKS" near the top and in "FILES"
         near the bottom. The "NOTBOOKS" is the old copy; the "FILES" is the new
         copy. Our goal is to replace the old copy with the new.
      2. Scroll down to the "FILES" and choose the top-most name. Right click on
         it and select "Move to notebooks".
      3. Because a notebook of that name already existed in "NOTEBOOKS" (the old
         copy), the moved notebook will be renamed with a ``-2`` suffix.
      4. Scroll up to "NOTEBOOKS". Right click on the old copy (without ``-2`)
         and select "Delete" and confirm. Right click on the new notebook (with
         ``-2``) and select "Rename" and remove the ``-2`` suffix.
      5. Open the (new) notebook and click "Run notebook". It should succeed.
      6. For all code cells, examine the output of each cell to check that no
         exceptions have snuck through (or any other unexpected error text).
         * The error "'%matplotlib notebook' is not supported in Deepnote" is
           expected and can be ignored.
      7. For all markdown cells, quickly skim over the rendered output to check
         that no markup errors have snuck through (e.g., LaTeX syntax errors).
      8. If you get an error like "Synchronization of file ... failed, your
         changes are not being saved. You might be running out of disk quota"
         you may ignore it.
      9. Leave the notebook output intact (do not clear the outputs). We want
         users to be able to read the outputs on their own, without necessarily
         running the notebook themselves.
      10. The moved notebook no longer appears in "FILES", so you can always
          use the top-most ``*.ipynb`` in "FILES" as your checklist for which
          one to tackle next.
6. On the right side, click "Environment" then "Stop Machine", as a
   courtesy. (It will time out on its own within the hour, but we might as
   well save a few nanograms of CO2 where we can.)
=======
      2. Do not try to run the ``licensed_solvers_deepnote`` notebook.
         (You do not have a suitable license key.)
      3. If you get an error like "Synchronization of file ... failed, your changes are not being saved. You might be running out of disk quota" you may ignore it.
   2. For all markdown cells, quickly skim over the rendered output to check
      that no markup errors have snuck through (e.g., LaTeX syntax errors).
   3. For all code cells, examine the output of each cell to check that no
      exceptions have snuck through (or any other unexpected error text).
      * The error "'%matplotlib notebook' is not supported in Deepnote" is
        expected and can be ignored.
   4. Leave the notebook output intact (do not clear the outputs). We want
      users to be able to read the outputs on their own, without necessarily
      running the notebook themselves.
6. On the right side, click "Environment" then "Stop Machine", as a
   courtesy. (It will time out on its own within the hour, but we might as
   well save a few nanograms of CO2 where we can.)

## Post-release wheel builds

After tagging the release, you must manually build and upload a PyPI release.

If you haven't done so already, follow Drake's PyPI
[account setup](https://docs.google.com/document/d/17D0yzyr0kGH44eWpiNY7E33A8hW1aiJRmADaoAlVISE/edit#)
instructions to obtain a username and password.

1. Use your Ubuntu 20.04 workstation for these steps.  (Do not use any other OS.)
2. Create some empty scratch folder to use for these steps, and then ``cd`` into it.
3. Run ``sudo apt install docker.io twine``
   1. If you get an "docker.sock connect" error, you might need to give yourself Docker permissions:
      1. Run ``sudo usermod -aG docker $USER``
      2. Run ``newgrp docker``
   2. If Docker still doesn't work, you might need to logout and/or reboot first.
4. Run ``git clone --filter=blob:none https://github.com/RobotLocomotion/drake.git``
5. Run ``cd drake``
6. Run ``git checkout v1.N.0``
7. To remove any cached images:
   1. Run ``docker rmi $(docker image ls --filter=reference='pip-drake:*' -q)``
   1. Run ``docker builder prune -f``
8. Run ``./setup/ubuntu/source_distribution/install_prereqs_user_environment.sh``
9. Run ``bazel run //tools/wheel:builder -- --output-dir=${PWD} --test 1.N.0``
10. Wait a long time for it to finish (around 30 minutes on a beefy workstation). It will take over all of your computer's resources, so don't plan to do much else concurrently.
11. There should have been exactly two whl files created. Run ``twine upload <...> <...>``, replacing the ``<...>`` placeholders with the paths to the two wheels to be uploaded (e.g., ``drake-0.35.0b1-cp36-cp36m-manylinux_2_27_x86_64.whl``, etc.)
    1. You will need your PyPI username and password for this. (Do not use drake-robot.)
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
